#include "lqr_balance.h"
#include "lqr_gain_table.h"
#include "machine_config.h"

#include <math.h>
#include <string.h>

/* 一阶低通系数 */
#define LQR_LPF_ALPHA       0.3f
/* 速度卡尔曼: 同 Leg2_v1 Body.h (P0 / Q / R / P 上限) */
#define LQR_KF_P0           0.1f
#define LQR_KF_Q            0.007f
#define LQR_KF_R            0.01f
#define LQR_KF_P_MAX        0.5f
#define LQR_GRAVITY         9.81f
/* 腿长变化超此阈值才重算增益 (m) */
#define LQR_K_RECALC_THRESH 0.0005f

/* 站立目标 */
#define LQR_POS_TARGET      (0.10f)
#define LQR_LEG_ANG_TARGET  (0.04f)
#define LQR_LEG_LEN_INIT    0.14f    /* 投入腿长目标 */


/*
 * IMU 轴索引 — 台架第一步必须确认
 * 手法: 手把机头缓慢抬起/压下, 看 pitch 与角速度哪个分量响应最大、符号是否符合
 * 若 pitch 实际落在滚转槽, 只改这两行
 */
#define LQR_IMU_PITCH_IDX    ATTITUDE_PITCH
#define LQR_IMU_ROLL_IDX     ATTITUDE_ROLL
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
    *len_min = fmaxf(*len_min, LQR_K_LEN_MIN);
    *len_max = fminf(*len_max, LQR_K_LEN_MAX);
}

/* 初始化 */
void LQR_Init(lqr_state_t *st)
{
    memset(st, 0, sizeof(*st));
    lqr_debug.vel_leg_comp_sign = -1.0f;
    lqr_debug.vel_src = 1u;
    lqr_debug.yaw_hold = 1u;
    lqr_debug.yaw_rate_hold = 1u;
    lqr_debug.pos_hold = 1u;
    lqr_debug.vel_ramp = 5.0f;      /* 同 Leg2 RAMP_VEL_RATE */
    lqr_debug.acc_fwd_sign = 1.0f;
    lqr_debug.pitch_comp_sign = -1.0f;  /* 现行公式 */
    lqr_debug.pos_arm_vel = 0.0f;
    lqr_debug.wheel_enable = 1u;
    lqr_debug.hip_enable = 1u;
    lqr_debug.len_pid_enable = 1u;
    lqr_debug.trq_max_wheel = machine->dji_trq_clamp;
    lqr_debug.trq_max_hip = machine->dm_trq_clamp;
    st->len_eval[0] = -1.0f;
    st->len_eval[1] = -1.0f;
    Lowpass_Init(&st->lpf_vel, LQR_LPF_ALPHA);
    Lowpass_Init(&st->lpf_omg_pitch, LQR_LPF_ALPHA);
    Lowpass_Init(&st->lpf_omg_yaw, LQR_LPF_ALPHA);
    Kalman_Accel_Init(&st->kf_vel, 0.0f, LQR_KF_P0, LQR_KF_Q, LQR_KF_R,
                      LQR_KF_P_MAX);
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
    st->leg_len_tgt[0] = LQR_LEG_LEN_INIT;
    st->leg_len_tgt[1] = LQR_LEG_LEN_INIT;
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

    if (cmd == NULL || !cmd->online)
    {
        return 0u;
    }

    LQR_Len_Range(&len_min, &len_max);

    st->target[LQR_X_S]     = LQR_POS_TARGET;
    /* 速度目标斜坡 (同 Leg2): 松杆时目标按 vel_ramp 降到 0, 减速段仍算"有指令"不积位移, 车停稳才开始积 */
    {
        float vel_cmd = cmd->vel * LQR_RC_VEL_MAX;
        if (lqr_debug.vel_ramp > 0.0f)
        {
            float step = lqr_debug.vel_ramp * dt;
            st->vel_tgt = clampf(vel_cmd, st->vel_tgt - step, st->vel_tgt + step);
            if (fabsf(st->vel_tgt) < step) { st->vel_tgt = 0.0f; }   /* 收口到精确 0 */
        }
        else
        {
            st->vel_tgt = vel_cmd;
        }
    }
    st->target[LQR_X_DS]    = st->vel_tgt;
    /* 偏航: 摇杆有输入时目标跟随当前角 (不回正), 回中后锁住; 转向通道取负 (同 Leg2, 作者台架定) */
    if (cmd->yaw != 0.0f)
    {
        st->yaw_tgt = st->x[LQR_X_PHI];
    }
    st->target[LQR_X_PHI]   = st->yaw_tgt;
    st->target[LQR_X_DPHI]  = -cmd->yaw * LQR_RC_YAW_MAX;
    st->target[LQR_X_THL]   = LQR_LEG_ANG_TARGET;
    st->target[LQR_X_DTHL]  = 0.0f;
    st->target[LQR_X_THR]   = LQR_LEG_ANG_TARGET;
    st->target[LQR_X_DTHR]  = 0.0f;
    st->target[LQR_X_THB]   = 0.0f;
    st->target[LQR_X_DTHB]  = 0.0f;

    /* 腿长目标: 拨轮按速率积分, 限制在腿长工作区间 */
    for (i = 0u; i < 2u; i++)
    {
        st->leg_len_tgt[i] += cmd->len * LQR_RC_LEN_RATE * dt;
        st->leg_len_tgt[i] = clampf(st->leg_len_tgt[i], len_min, len_max);
    }
    return 1u;
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

    st->valid = 0u;
    if (imu == NULL || leg_l == NULL || leg_r == NULL || wheel_vel == NULL)
    {
        return 0u;
    }
    if (!imu->online || !leg_l->output.valid || !leg_r->output.valid)
    {
        return 0u;
    }

    st->len[0] = leg_l->output.virtual_leg_length;
    st->len[1] = leg_r->output.virtual_leg_length;

    pitch = imu->euler_rad[LQR_IMU_PITCH_IDX];
    omg_pitch = Lowpass_Update(&st->lpf_omg_pitch,
                               imu->gyro_rad_s[LQR_IMU_GYRO_PITCH]);
    st->roll = imu->euler_rad[LQR_IMU_ROLL_IDX];

    /* 腿摆角/角速度世界系 = 解算输出 + 机体俯仰
     * 本工程 virtual_leg_angle 前摆为正, 模型 θ_ll 前摆为负, 故取反后再加 pitch */
    st->x[LQR_X_THL]  = -leg_l->output.virtual_leg_angle + pitch;
    st->x[LQR_X_DTHL] = -leg_l->output.d_virtual_leg_angle + omg_pitch;
    st->x[LQR_X_THR]  = -leg_r->output.virtual_leg_angle + pitch;
    st->x[LQR_X_DTHR] = -leg_r->output.d_virtual_leg_angle + omg_pitch;
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
           + leg_l->output.virtual_leg_length * st->x[LQR_X_DTHL]
             * cosf(st->x[LQR_X_THL])
           + leg_l->output.d_virtual_leg_length * sinf(st->x[LQR_X_THL]);
    vel[1] = whl[1] * machine->wheel_r
           + leg_r->output.virtual_leg_length * st->x[LQR_X_DTHR]
             * cosf(st->x[LQR_X_THR])
           + leg_r->output.d_virtual_leg_length * sinf(st->x[LQR_X_THR]);
    /* 速度估计两条并行: 低通 / 卡尔曼 (加速度预测 + 运动学观测), vel_src 选一条进 x[1] */
    st->ds_raw = (vel[0] + vel[1]) * 0.5f;
    st->ds_lpf = Lowpass_Update(&st->lpf_vel, st->ds_raw);
    st->a_fwd  = lqr_debug.acc_fwd_sign * LQR_Accel_Forward(imu);
    st->ds_kf  = Kalman_Accel_Update(&st->kf_vel, st->a_fwd, st->ds_raw, dt);
    st->x[LQR_X_DS] = lqr_debug.vel_src ? st->ds_kf : st->ds_lpf;

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
    st->valid = 1u;
    return 1u;
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

    /* 增益多项式只在生成网格内有效，实测腿长越界时取边界增益。 */
    len_l = clampf(st->len[0], LQR_K_LEN_MIN, LQR_K_LEN_MAX);
    len_r = clampf(st->len[1], LQR_K_LEN_MIN, LQR_K_LEN_MAX);
    if (fabsf(len_l - st->len_eval[0]) > LQR_K_RECALC_THRESH
        || fabsf(len_r - st->len_eval[1]) > LQR_K_RECALC_THRESH)
    {
        LQR_K_WBR(len_l, len_r, K_sym);
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
        sum = 0.0f;
        for (j = 0u; j < LQR_X_NUM; j++)
        {
            if (j == LQR_X_PHI)
            {
                term = lqr_debug.yaw_hold
                     ? st->K[i][j] * LQR_Wrap_Pi(st->target[j] - st->x[j])
                     : 0.0f;                        /* 关: 偏航角不参与 */
            }
            else if ((j == LQR_X_DPHI && !lqr_debug.yaw_rate_hold)
                     || (j == LQR_X_S && !lqr_debug.pos_hold))
            {
                term = 0.0f;                        /* 关: 该列不参与 */
            }
            else
            {
                term = st->K[i][j] * (st->target[j] - st->x[j]);
            }
            if (i == LQR_U_WL)
            {
                st->u_col[j] = term;                /* 左轮分项 */
            }
            sum += term;
        }
        if (!isfinite(sum))
        {
            sum = 0.0f;
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
}
