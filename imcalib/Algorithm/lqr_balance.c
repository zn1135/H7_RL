#include "lqr_balance.h"
#include "lqr_gain_table.h"
#include "machine_config.h"

#include <math.h>
#include <string.h>

#define LQR_GRAVITY         9.81f
#define LQR_K_RECALC_THRESH 0.0005f

/*
 * IMU 轴索引 — 台架第一步必须确认
 * 手法: 手把机头缓慢抬起/压下, 看 pitch 与角速度哪个分量响应最大、符号是否符合
 * 若 pitch 实际落在滚转槽, 只改这两行
 */
#define LQR_IMU_PITCH_IDX    ATTITUDE_ROLL
#define LQR_IMU_ROLL_IDX     ATTITUDE_PITCH
#define LQR_IMU_YAW_IDX      ATTITUDE_YAW
#define LQR_IMU_GYRO_PITCH   1u
#define LQR_IMU_GYRO_YAW     2u

lqr_debug_t lqr_debug;
/* 角度环绕 [-π, π] */
static float LQR_Wrap_Pi(float angle)
{
    while (angle > LEG_PI)  { angle -= LEG_2PI; }
    while (angle < -LEG_PI) { angle += LEG_2PI; }
    return angle;
}

/* 腿长目标区间 */
static void LQR_Len_Range(float *len_min, float *len_max)
{
    *len_min = machine->leg_len_min;
    *len_max = machine->leg_len_max;
    *len_min = fmaxf(*len_min, LQR_Gain_Info()->len_min);
    *len_max = fminf(*len_max, LQR_Gain_Info()->len_max);
}

/* 初始化 */
void LQR_Init(lqr_state_t *st)
{
    memset(st, 0, sizeof(*st));
    lqr_debug.vel_leg_comp_sign = -1.0f;
    lqr_debug.legacy_gain = 0u;
    lqr_debug.vel_src = machine->lqr.vel_src;
    lqr_debug.yaw_hold = machine->lqr.yaw_hold;
    lqr_debug.yaw_rate_hold = machine->lqr.yaw_rate_hold;
    lqr_debug.pos_hold = machine->lqr.pos_hold;
    lqr_debug.acc_fwd_sign = 1.0f;
    lqr_debug.pitch_comp_sign = -1.0f;  /* 现行公式 */
    lqr_debug.pos_arm_vel = machine->lqr.pos_arm_vel;
    lqr_debug.wheel_enable = machine->lqr.wheel_enable;
    lqr_debug.hip_enable = machine->lqr.hip_enable;
    lqr_debug.len_pid_enable = machine->lqr.len_pid_enable;
    lqr_debug.trq_max_wheel = machine->dji_trq_clamp;
    lqr_debug.trq_max_hip = machine->dm_trq_clamp;
    st->len_eval[0] = -1.0f;
    st->len_eval[1] = -1.0f;
    Lowpass_Init(&st->lpf_vel, machine->lqr.lpf_alpha[0]);
    Lowpass_Init(&st->lpf_omg_pitch, machine->lqr.lpf_alpha[1]);
    Lowpass_Init(&st->lpf_omg_yaw, machine->lqr.lpf_alpha[2]);
    Kalman_Accel_Init(&st->kf_vel, 0.0f, machine->lqr.kf_p0,
        machine->lqr.kf_q, machine->lqr.kf_r, machine->lqr.kf_p_max);
}

/* 前向加速度: 四元数把机体加速度转到世界系, 去重力, 投影到车头水平方向 */
static float LQR_Accel_Forward(const imu_state_t *imu)
{
    float q0;
    float q1;
    float q2;
    float q3;
    float r00;
    float r01;
    float r02;
    float r10;
    float r11;
    float r12;
    float wx;
    float wy;
    float norm;

    q0 = imu->quat[0];
    q1 = imu->quat[1];
    q2 = imu->quat[2];
    q3 = imu->quat[3];
    r00 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    r01 = 2.0f * (q1 * q2 - q0 * q3);
    r02 = 2.0f * (q1 * q3 + q0 * q2);
    r10 = 2.0f * (q1 * q2 + q0 * q3);
    r11 = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
    r12 = 2.0f * (q2 * q3 - q0 * q1);

    /* 世界系水平分量 (重力只在 z, 水平不用减) */
    wx = r00 * imu->acc_g[0] + r01 * imu->acc_g[1] + r02 * imu->acc_g[2];
    wy = r10 * imu->acc_g[0] + r11 * imu->acc_g[1] + r12 * imu->acc_g[2];
    /* 车头方向 = 机体 x 轴在水平面的投影 */
    norm = sqrtf(r00 * r00 + r10 * r10);
    if (norm < 1.0e-3f)
    {
        return 0.0f;
    }
    return (wx * r00 + wy * r10) / norm * LQR_GRAVITY;
}

/* 使能边沿: 腿长目标锁默认值, 位移积分清零 (滤波器常跑不复位)
 * 同 Leg2: 不查实测腿长, 趴地也投入, 靠腿长 PID 撑起 (倒地自起状态机后做) */
uint8_t LQR_Enable_Latch(lqr_state_t *st, const leg_state_t *leg_l,
                         const leg_state_t *leg_r)
{
    (void)leg_l;
    (void)leg_r;
    st->leg_len_tgt[0] = machine->lqr.leg_len_init[0];
    st->leg_len_tgt[1] = machine->lqr.leg_len_init[1];
    st->yaw_tgt = st->x[LQR_X_PHI];    /* 朝向锁当前 */
    st->vel_tgt = 0.0f;
    /* 只清位移积分; 滤波器每拍都在跑, 已是热态, 不复位 */
    st->pos = 0.0f;
    st->pos_armed = 1u;
    st->x[LQR_X_S] = 0.0f;
    return 1u;
}

/* 指令 → 目标 */
uint8_t LQR_Target_Update(lqr_state_t *st, const rc_command_t *cmd, float dt)
{
    float len_min;
    float len_max;
    uint8_t i;

    if (cmd == NULL || !cmd->online || !isfinite(dt) || dt <= 0.0f
        || !isfinite(cmd->vel) || !isfinite(cmd->yaw) || !isfinite(cmd->len))
    {
        return 0u;
    }

    LQR_Len_Range(&len_min, &len_max);

    st->target[LQR_X_S]     = machine->lqr.pos_target;
    st->vel_tgt            = cmd->vel;
    st->target[LQR_X_DS]    = cmd->vel;
    /* 转向保持朝向 */
    if (cmd->yaw != 0.0f)
    {
        st->yaw_tgt = st->x[LQR_X_PHI];
    }
    st->target[LQR_X_PHI]   = st->yaw_tgt;
    st->target[LQR_X_DPHI]  = cmd->yaw;
    st->target[LQR_X_THL]   = machine->lqr.leg_trim[0];
    st->target[LQR_X_DTHL]  = 0.0f;
    st->target[LQR_X_THR]   = machine->lqr.leg_trim[1];
    st->target[LQR_X_DTHR]  = 0.0f;
    st->target[LQR_X_THB]   = machine->lqr.pitch_trim;
    st->target[LQR_X_DTHB]  = 0.0f;

    /* 腿长目标: 拨轮按速率积分, 限制在腿长工作区间 */
    for (i = 0u; i < 2u; i++)
    {
        st->leg_len_tgt[i] += cmd->len * machine->lqr.len_rate * dt;
        st->leg_len_tgt[i] = clampf(st->leg_len_tgt[i], len_min, len_max);
    }
    return 1u;
}

/* 消费速度结果 */
void LQR_Velocity_Apply(lqr_state_t *st, float velocity, float dt)
{
    if (!isfinite(velocity))
    {
        st->valid = 0u;
        return;
    }
    st->ds_kf = velocity;
    st->x[LQR_X_DS] = lqr_debug.vel_src ? velocity : st->ds_lpf;
    /* 位移积分: 有速度指令时清零并撤防; 目标回零后车速降到 pos_arm_vel 以下才开始积 (0 = 立即) */
    if (st->target[LQR_X_DS] != 0.0f)
    {
        st->pos = 0.0f;
        st->pos_armed = 0u;
    }
    else
    {
        if (!st->pos_armed
            && (lqr_debug.pos_arm_vel <= 0.0f
                || fabsf(st->x[LQR_X_DS]) < lqr_debug.pos_arm_vel))
        {
            st->pos_armed = 1u;
        }
        if (st->pos_armed)
        {
            st->pos += st->x[LQR_X_DS] * dt;
        }
    }
    st->x[LQR_X_S] = st->pos;
    if (!isfinite(st->pos) || !isfinite(st->x[LQR_X_DS]))
    {
        st->valid = 0u;
    }
}

/* 状态估计: 每拍必算, 有效性写 st->valid 并返回 */
uint8_t LQR_State_Update(lqr_state_t *st, const imu_state_t *imu,
                         const leg_state_t *leg_l, const leg_state_t *leg_r,
                         const float wheel_vel[2], float dt)
{
    float pitch;
    float omg_pitch;
    float whl[2];
    float vel[2];

    uint8_t i;

    st->valid = 0u;
    if (!isfinite(dt) || dt <= 0.0f)
    {
        return 0u;
    }
    if (imu == NULL || leg_l == NULL || leg_r == NULL || wheel_vel == NULL)
    {
        return 0u;
    }
    if (!imu->online || !leg_l->output.valid || !leg_r->output.valid)
    {
        return 0u;
    }

    if (!isfinite(wheel_vel[0]) || !isfinite(wheel_vel[1])
        || !isfinite(imu->euler_rad[LQR_IMU_PITCH_IDX])
        || !isfinite(imu->euler_rad[LQR_IMU_ROLL_IDX])
        || !isfinite(imu->euler_rad[LQR_IMU_YAW_IDX]))
    {
        return 0u;
    }
    for (i = 0u; i < 3u; i++)
    {
        if (!isfinite(imu->gyro_rad_s[i]) || !isfinite(imu->acc_g[i]))
        {
            return 0u;
        }
    }
    for (i = 0u; i < 4u; i++)
    {
        if (!isfinite(imu->quat[i]))
        {
            return 0u;
        }
    }
    if (!isfinite(leg_l->output.virtual_leg_length) || !isfinite(leg_r->output.virtual_leg_length)
        || !isfinite(leg_l->output.virtual_leg_angle) || !isfinite(leg_r->output.virtual_leg_angle)
        || !isfinite(leg_l->output.d_virtual_leg_angle) || !isfinite(leg_r->output.d_virtual_leg_angle)
        || !isfinite(leg_l->output.d_virtual_leg_length) || !isfinite(leg_r->output.d_virtual_leg_length))
    {
        return 0u;
    }
    st->len[0] = leg_l->output.virtual_leg_length;
    st->len[1] = leg_r->output.virtual_leg_length;

    pitch = imu->euler_rad[LQR_IMU_PITCH_IDX];
    omg_pitch = Lowpass_Update(&st->lpf_omg_pitch,
                               imu->gyro_rad_s[LQR_IMU_GYRO_PITCH]);
    st->roll = imu->euler_rad[LQR_IMU_ROLL_IDX];

    /* 前摆正，转世界系 */
    st->x[LQR_X_THL]  = leg_l->output.virtual_leg_angle - pitch;
    st->x[LQR_X_DTHL] = leg_l->output.d_virtual_leg_angle - omg_pitch;
    st->x[LQR_X_THR]  = leg_r->output.virtual_leg_angle - pitch;
    st->x[LQR_X_DTHR] = leg_r->output.d_virtual_leg_angle - omg_pitch;
    st->x[LQR_X_THB]  = pitch;
    st->x[LQR_X_DTHB] = omg_pitch;
    st->x[LQR_X_PHI]  = imu->euler_rad[LQR_IMU_YAW_IDX];
    st->x[LQR_X_DPHI] = Lowpass_Update(&st->lpf_omg_yaw,
                                       imu->gyro_rad_s[LQR_IMU_GYRO_YAW]);

    /* 轮子相对地面角速度: 反馈已扣减速比, 再补偿腿摆与俯仰 (俯仰项符号 A/B, 见 LQR_PLAN §六 ①e) */
    whl[0] = wheel_vel[0] + lqr_debug.vel_leg_comp_sign
             * leg_l->output.d_virtual_leg_angle
             + lqr_debug.pitch_comp_sign * omg_pitch;
    whl[1] = wheel_vel[1] + lqr_debug.vel_leg_comp_sign
             * leg_r->output.d_virtual_leg_angle
             + lqr_debug.pitch_comp_sign * omg_pitch;
    st->whl[0] = whl[0];
    st->whl[1] = whl[1];

    /* 机体水平速度: 轮心线速度 + 摆杆摆动 + 摆杆伸缩 */
    vel[0] = whl[0] * machine->wheel_r
           - leg_l->output.virtual_leg_length * st->x[LQR_X_DTHL]
             * cosf(st->x[LQR_X_THL])
           - leg_l->output.d_virtual_leg_length * sinf(st->x[LQR_X_THL]);
    vel[1] = whl[1] * machine->wheel_r
           - leg_r->output.virtual_leg_length * st->x[LQR_X_DTHR]
             * cosf(st->x[LQR_X_THR])
           - leg_r->output.d_virtual_leg_length * sinf(st->x[LQR_X_THR]);
    /* 速度估计两条并行: 低通 / 卡尔曼 (加速度预测 + 运动学观测), vel_src 选一条进 x[1] */
    st->ds_raw = (vel[0] + vel[1]) * 0.5f;
    st->ds_lpf = Lowpass_Update(&st->lpf_vel, st->ds_raw);
    st->a_fwd  = lqr_debug.acc_fwd_sign * LQR_Accel_Forward(imu);
    if (!isfinite(st->ds_raw) || !isfinite(st->a_fwd))
    {
        return 0u;
    }
    st->valid = 1u;
    st->ds_kf = Kalman_Accel_Update(&st->kf_vel, st->a_fwd, st->ds_raw, dt);
    LQR_Velocity_Apply(st, st->ds_kf, dt);
    return st->valid;
}

/* 增益求值 + 状态反馈求和 */
void LQR_Control_Update(lqr_state_t *st)
{
    float K_sym[40];
    float len_l;
    float len_r;
    float sum;
    float term;
    uint8_t i;
    uint8_t j;

    st->gain_valid = 0u;
    memset(st->u, 0, sizeof(st->u));
    if (!st->valid || !LQR_Gain_Compatible()
        || !isfinite(st->len[0]) || !isfinite(st->len[1]))
    {
        return;
    }
    /* 增益多项式只在生成网格内有效，实测腿长越界时取边界增益。 */
    len_l = clampf(st->len[0], LQR_Gain_Info()->len_min, LQR_Gain_Info()->len_max);
    len_r = clampf(st->len[1], LQR_Gain_Info()->len_min, LQR_Gain_Info()->len_max);
    if (fabsf(len_l - st->len_eval[0]) > LQR_K_RECALC_THRESH
        || fabsf(len_r - st->len_eval[1]) > LQR_K_RECALC_THRESH
        || st->gain_legacy != lqr_debug.legacy_gain)
    {
        if (!LQR_Gain_Eval(len_l, len_r, K_sym, lqr_debug.legacy_gain))
        {
            return;
        }
        st->gain_legacy = lqr_debug.legacy_gain;
        for (i = 0u; i < LQR_U_NUM; i++)
        {
            for (j = 0u; j < LQR_X_NUM; j++)
            {
                st->K[i][j] = K_sym[j * LQR_U_NUM + i];
            }
        }
        st->len_eval[0] = len_l;
        st->len_eval[1] = len_r;
    }

    for (i = 0u; i < LQR_U_NUM; i++)
    {
        sum = LQR_Gain_Info()->u_eq[i];
        for (j = 0u; j < LQR_X_NUM; j++)
        {
            if (j == LQR_X_PHI)
            {
                term = lqr_debug.yaw_hold
                     ? st->K[i][j] * LQR_Wrap_Pi(st->target[j] + LQR_Gain_Info()->x_eq[j] - st->x[j])
                     : 0.0f;                        /* 关: 偏航角不参与 */
            }
            else if ((j == LQR_X_DPHI && !lqr_debug.yaw_rate_hold)
                     || (j == LQR_X_S && !lqr_debug.pos_hold))
            {
                term = 0.0f;                        /* 关: 该列不参与 */
            }
            else
            {
                term = st->K[i][j] * (st->target[j] + LQR_Gain_Info()->x_eq[j] - st->x[j]);
            }
            if (i == LQR_U_WL)
            {
                st->u_col[j] = term;                /* 左轮分项 */
            }
            sum += term;
        }
        if (!isfinite(sum))
        {
            memset(st->u, 0, sizeof(st->u));
            return;
        }
        if (i == LQR_U_WL || i == LQR_U_WR)
        {
            st->u[i] = clampf(sum, -lqr_debug.trq_max_wheel,
                              lqr_debug.trq_max_wheel);
        }
        else
        {
            st->u[i] = clampf(sum, -lqr_debug.trq_max_hip,
                              lqr_debug.trq_max_hip);
        }
    }
    st->gain_valid = 1u;
}
