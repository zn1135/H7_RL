#include "rl_torque.h"
#include "machine_config.h"
#include "robot_control.h"

#include <math.h>
#include <string.h>

#define RL_TQ_POS_SCALE        0.5f     /* 训练侧: 腿目标 = act × 0.5 + 默认角 */
#define RL_TQ_WHEEL_VEL_SCALE  10.0f    /* 训练侧: 轮目标速度 = act × 10 */
#define RL_TQ_WHEEL_VEL_MAX    20.0f    /* 轮目标速度限幅 */
#define RL_TQ_LEG_TRQ_MAX      40.0f    /* 训练侧: 虚拟腿关节力矩上限 (映射前裁) */
#define RL_TQ_WHEEL_TRQ_MAX    3.9f     /* 训练侧: 轮力矩上限 */
#define RL_TQ_VSHANK_MIN       2.277f
#define RL_TQ_VSHANK_MAX       3.133f

/* 虚拟关节索引 (仅限本文件内部) */
enum {
    VJ_L_THIGH = 0,
    VJ_L_SHANK = 1,
    VJ_L_WHEEL = 2,
    VJ_R_THIGH = 3,
    VJ_R_SHANK = 4,
    VJ_R_WHEEL = 5,
    VJ_NUM     = 6,
};

typedef struct {
    float upper[3];
    float hinge[3];
    float lower[3];
    float axis_y;
} gas_spring_geom_t;

/* chuanliantui.xml 的气弹簧端点 */
static const gas_spring_geom_t gas_spring_geom[2] = {
    {
        {0.02560606f, 0.00350000f, -0.03710530f},
        {-0.16528873f, -0.01150000f, -0.12953623f},
        {0.01969256f, -0.01000000f, -0.04446437f},
        -1.0f,
    },
    {
        {0.02560606f, -0.00350000f, -0.03710530f},
        {-0.16528873f, 0.01150000f, -0.12953623f},
        {0.01969256f, 0.01000000f, -0.04446437f},
        1.0f,
    },
};

/* 端点连线推力转虚拟小腿力矩 */
static float RL_Gas_Spring_Shank_Torque(uint8_t side, float q, float force_n)
{
    const gas_spring_geom_t *geom;
    float angle;
    float x_rot;
    float z_rot;
    float dx;
    float dy;
    float dz;
    float length;

    if (side >= 2u || !isfinite(q) || !isfinite(force_n) || force_n <= 0.0f)
    {
        return 0.0f;
    }
    geom = &gas_spring_geom[side];
    angle = geom->axis_y * q;
    x_rot = cosf(angle) * geom->lower[0] + sinf(angle) * geom->lower[2];
    z_rot = -sinf(angle) * geom->lower[0] + cosf(angle) * geom->lower[2];
    dx = geom->hinge[0] + x_rot - geom->upper[0];
    dy = geom->hinge[1] + geom->lower[1] - geom->upper[1];
    dz = geom->hinge[2] + z_rot - geom->upper[2];
    length = sqrtf(dx * dx + dy * dy + dz * dz);
    if (!isfinite(length) || length < 0.001f)
    {
        return 0.0f;
    }
    return force_n * geom->axis_y * (dx * z_rot - dz * x_rot) / length;
}

/* 检查数组 */
static uint8_t RL_Torque_Array_Finite(const float *data, uint32_t count)
{
    if (data == NULL)
    {
        return 0u;
    }
    for (uint32_t i = 0u; i < count; i++)
    {
        if (!isfinite(data[i]))
        {
            return 0u;
        }
    }
    return 1u;
}

/* 初始化参数: chuanliantui 起立策略 (networkzn1)
 * dof_pos = 训练默认角映射到固件关节 (zero + sign × 默认角, 见机器表 .rl)
 * PD 来自训练仓库 chuanliantui_config.py control: stiffness f0/f1 10, damping f0/f1 1.0, 轮 damping 0.1 */
void RL_Torque_Param_Init(rl_torque_param_t *param, rl_model_t model)
{
    const rl_map_t *map = &machine->rl;
    const float dof_train[6] = {RL_OBS_DOF_POS_L_THIGH, RL_OBS_DOF_POS_L_SHANK, 0.0f,
                                RL_OBS_DOF_POS_R_THIGH, RL_OBS_DOF_POS_R_SHANK, 0.0f};
    const float p_gains[6] = {10.0f, 10.0f, 0.0f, 10.0f, 10.0f, 0.0f};   /* 训练 Kp */
    const float d_gains[6] = {1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 0.0f};       /* 训练 Kd */

    (void)model;
    if (param == NULL)
    {
        return;
    }
    memset(param, 0, sizeof(*param));
    param->dof_pos[VJ_L_THIGH] = map->zero[0] + (float)map->sign[VJ_L_THIGH] * dof_train[VJ_L_THIGH];
    param->dof_pos[VJ_L_SHANK] = map->zero[1] + (float)map->sign[VJ_L_SHANK] * dof_train[VJ_L_SHANK];
    param->dof_pos[VJ_R_THIGH] = map->zero[2] + (float)map->sign[VJ_R_THIGH] * dof_train[VJ_R_THIGH];
    param->dof_pos[VJ_R_SHANK] = map->zero[3] + (float)map->sign[VJ_R_SHANK] * dof_train[VJ_R_SHANK];
    memcpy(param->p_gains, p_gains, sizeof(p_gains));
    memcpy(param->d_gains, d_gains, sizeof(d_gains));
    param->wheel_pid[0][0] = 0.1f;   /* 轮速度增益 = 训练 damping */
    param->wheel_pid[1][0] = 0.1f;
}

/* 初始化 PID */
void RL_Torque_State_Init(rl_torque_state_t *state,
                          const rl_torque_param_t *param)
{
    if (state == NULL || param == NULL)
    {
        return;
    }
    memset(state, 0, sizeof(*state));
    /* 腿 PID 只放 P (D 项在 Compute 用关节速度算, 同仿真) */
    PID_struct_init(&state->controller[VJ_L_THIGH], POSITION_PID, 1000.0f,
        0.0f, param->p_gains[VJ_L_THIGH], 0.0f, 0.0f, 0.0f, 0.0f);
    PID_struct_init(&state->controller[VJ_L_SHANK], POSITION_PID, 1000.0f,
        0.0f, param->p_gains[VJ_L_SHANK], 0.0f, 0.0f, 0.0f, 0.0f);
    PID_struct_init(&state->controller[VJ_L_WHEEL], POSITION_PID, 1000.0f,
        0.0f, param->wheel_pid[0][0], param->wheel_pid[0][1], param->wheel_pid[0][2], 0.0f, 0.0f);
    PID_struct_init(&state->controller[VJ_R_THIGH], POSITION_PID, 1000.0f,
        0.0f, param->p_gains[VJ_R_THIGH], 0.0f, 0.0f, 0.0f, 0.0f);
    PID_struct_init(&state->controller[VJ_R_SHANK], POSITION_PID, 1000.0f,
        0.0f, param->p_gains[VJ_R_SHANK], 0.0f, 0.0f, 0.0f, 0.0f);
    PID_struct_init(&state->controller[VJ_R_WHEEL], POSITION_PID, 1000.0f,
        0.0f, param->wheel_pid[1][0], param->wheel_pid[1][1], param->wheel_pid[1][2], 0.0f, 0.0f);
    /* 关节 PID 开启角度环绕 */
    state->controller[VJ_L_THIGH].angle_wrap = 1u;
    state->controller[VJ_L_SHANK].angle_wrap = 1u;
    state->controller[VJ_R_THIGH].angle_wrap = 1u;
    state->controller[VJ_R_SHANK].angle_wrap = 1u;

    /* pos_target 初始化为静息位 */
    state->pos_target[VJ_L_THIGH] = param->dof_pos[VJ_L_THIGH];
    state->pos_target[VJ_L_SHANK] = param->dof_pos[VJ_L_SHANK];
    state->pos_target[VJ_R_THIGH] = param->dof_pos[VJ_R_THIGH];
    state->pos_target[VJ_R_SHANK] = param->dof_pos[VJ_R_SHANK];
}

/* 计算力矩 */
uint8_t RL_Torque_Compute(const leg_state_t *leg_l, const leg_state_t *leg_r,
                          const rl_torque_param_t *param,
                          const float wheel_vel[2],
                          const float action[RL_ACTION_SIZE],
                          rl_torque_state_t *state,
                          torque_output_t *torque)
{
    float pos_ref[VJ_NUM];
    float vel_ref[VJ_NUM];
    float q[VJ_NUM];
    float qd[VJ_NUM];
    float tau_v[VJ_NUM];
    float tau_f[2];
    float tau_b[2];
    float shank_tau[2];
    float q_train[2];
    const rl_map_t *map;
    float leg_limit;
    float wheel_limit;

    torque->dm[DM_MOTOR_LEG_F_LFT] = 0.0f;
    torque->dm[DM_MOTOR_LEG_B_LFT] = 0.0f;
    torque->dm[DM_MOTOR_LEG_F_RGT] = 0.0f;
    torque->dm[DM_MOTOR_LEG_B_RGT] = 0.0f;
    torque->dji[DJI_MOTOR_WHEEL_LFT] = 0.0f;
    torque->dji[DJI_MOTOR_WHEEL_RGT] = 0.0f;

    if (leg_l == NULL || leg_r == NULL || param == NULL || state == NULL
        || wheel_vel == NULL || action == NULL || torque == NULL)
    {
        return 0u;
    }
    if (!leg_l->output.valid || !leg_r->output.valid)
    {
        return 0u;
    }
    if (!RL_Torque_Array_Finite(action, RL_ACTION_SIZE)
        || !RL_Torque_Array_Finite(wheel_vel, 2u))
    {
        return 0u;
    }

    /* 虚拟关节状态 */
    q[VJ_L_THIGH] = leg_l->output.thigh_angle;
    q[VJ_L_SHANK] = leg_l->output.virtual_shank_angle;
    q[VJ_L_WHEEL] = 0.0f;
    q[VJ_R_THIGH] = leg_r->output.thigh_angle;
    q[VJ_R_SHANK] = leg_r->output.virtual_shank_angle;
    q[VJ_R_WHEEL] = 0.0f;
    qd[VJ_L_THIGH] = leg_l->input.d_hip_f;
    qd[VJ_L_SHANK] = leg_l->output.d_virtual_shank_angle;
    qd[VJ_L_WHEEL] = wheel_vel[0];
    qd[VJ_R_THIGH] = leg_r->input.d_hip_f;
    qd[VJ_R_SHANK] = leg_r->output.d_virtual_shank_angle;
    qd[VJ_R_WHEEL] = wheel_vel[1];

    /* 位置/速度目标 */
    for (uint32_t i = 0u; i < VJ_NUM; i++)
    {
        pos_ref[i] = 0.0f;
        vel_ref[i] = 0.0f;
    }
    pos_ref[VJ_L_THIGH] = action[VJ_L_THIGH] * RL_TQ_POS_SCALE;
    pos_ref[VJ_L_SHANK] = action[VJ_L_SHANK] * RL_TQ_POS_SCALE;
    pos_ref[VJ_R_THIGH] = action[VJ_R_THIGH] * RL_TQ_POS_SCALE;
    pos_ref[VJ_R_SHANK] = action[VJ_R_SHANK] * RL_TQ_POS_SCALE;
    vel_ref[VJ_L_WHEEL] = clampf(action[VJ_L_WHEEL] * RL_TQ_WHEEL_VEL_SCALE,
        -RL_TQ_WHEEL_VEL_MAX, RL_TQ_WHEEL_VEL_MAX);
    vel_ref[VJ_R_WHEEL] = clampf(action[VJ_R_WHEEL] * RL_TQ_WHEEL_VEL_SCALE,
        -RL_TQ_WHEEL_VEL_MAX, RL_TQ_WHEEL_VEL_MAX);

    /* 虚拟关节 PD: 腿 Kp·wrap(目标 − q) − Kd·q̇, 轮 Kp·(v_ref − v) */
    for (uint32_t i = 0u; i < VJ_NUM; i++)
    {
        if (i == VJ_L_WHEEL || i == VJ_R_WHEEL)
        {
            state->pos_target[i] = vel_ref[i];
            tau_v[i] = pid_calc(&state->controller[i], qd[i], vel_ref[i],
                CTRL_DT);
        }
        else
        {
            float target = pos_ref[i] + param->dof_pos[i];
            state->pos_target[i] = target;
            tau_v[i] = pid_calc(&state->controller[i], q[i], target,
                CTRL_DT) - param->d_gains[i] * qd[i];
        }
    }
    /* 训练侧虚拟关节力矩上限: Isaac 在映射前裁 */
    for (uint32_t i = 0u; i < VJ_NUM; i++)
    {
        if (i == VJ_L_WHEEL || i == VJ_R_WHEEL)
        {
            tau_v[i] = clampf(tau_v[i], -RL_TQ_WHEEL_TRQ_MAX, RL_TQ_WHEEL_TRQ_MAX);
        }
        else
        {
            tau_v[i] = clampf(tau_v[i], -RL_TQ_LEG_TRQ_MAX, RL_TQ_LEG_TRQ_MAX);
        }
    }
    memcpy(state->virtual_torque, tau_v, sizeof(state->virtual_torque));

    /* 补偿符号按机器表，0 时关闭 */
    map = &machine->rl;
    shank_tau[0] = tau_v[VJ_L_SHANK];
    shank_tau[1] = tau_v[VJ_R_SHANK];
    if (map->configured && machine->gas_comp_sign[0] != 0)
    {
        q_train[0] = (float)map->sign[VJ_L_SHANK]
                   * Angle_Wrap_180(q[VJ_L_SHANK] - map->zero[1]);
        shank_tau[0] += (float)(map->sign[VJ_L_SHANK] * machine->gas_comp_sign[0])
                      * RL_Gas_Spring_Shank_Torque(0u, q_train[0], machine->gas_spring_force_n[0]);
    }
    if (map->configured && machine->gas_comp_sign[1] != 0)
    {
        q_train[1] = (float)map->sign[VJ_R_SHANK]
                   * Angle_Wrap_180(q[VJ_R_SHANK] - map->zero[3]);
        shank_tau[1] += (float)(map->sign[VJ_R_SHANK] * machine->gas_comp_sign[1])
                      * RL_Gas_Spring_Shank_Torque(1u, q_train[1], machine->gas_spring_force_n[1]);
    }

    /* 虚拟力矩映射: vshank_jac[0]→后髋, [1]→前髋 */
    tau_f[0] = tau_v[VJ_L_THIGH] + shank_tau[0] * leg_l->output.vshank_jac[1];
    tau_b[0] = shank_tau[0] * leg_l->output.vshank_jac[0];
    tau_f[1] = tau_v[VJ_R_THIGH] + shank_tau[1] * leg_r->output.vshank_jac[1];
    tau_b[1] = shank_tau[1] * leg_r->output.vshank_jac[0];

    /* DM 输出 (满限幅) */
    leg_limit = machine->dm_trq_clamp;
    torque->dm[DM_MOTOR_LEG_F_LFT] = clampf(tau_f[0], -leg_limit, leg_limit);
    torque->dm[DM_MOTOR_LEG_B_LFT] = clampf(tau_b[0], -leg_limit, leg_limit);
    torque->dm[DM_MOTOR_LEG_F_RGT] = clampf(tau_f[1], -leg_limit, leg_limit);
    torque->dm[DM_MOTOR_LEG_B_RGT] = clampf(tau_b[1], -leg_limit, leg_limit);

    /* DJI 输出 (满限幅) */
    wheel_limit = machine->dji_trq_clamp;
    torque->dji[DJI_MOTOR_WHEEL_LFT] = clampf(tau_v[VJ_L_WHEEL], -wheel_limit, wheel_limit);
    torque->dji[DJI_MOTOR_WHEEL_RGT] = clampf(tau_v[VJ_R_WHEEL], -wheel_limit, wheel_limit);

    memcpy(&state->last_torque, torque, sizeof(state->last_torque));
    return 1u;
}
