#include "rl_torque.h"
#include "gas_spring.h"
#include "machine_config.h"
#include "robot_control.h"

#include <math.h>
#include <string.h>

#define RL_TQ_POS_SCALE        0.5f     /* 训练侧: 腿目标 = act × 0.5 + 默认角 */
#define RL_TQ_WHEEL_VEL_SCALE  10.0f    /* 训练侧: 轮目标速度 = act × 10 */
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
    float kp = 10.0f,kd = 1.0f;
    const float p_gains[6] = {kp,kp, 0.0f, kp,kp, 0.0f};   /* 训练 Kp */
    const float d_gains[6] = {kd, kd, 0.0f, kd, kd, 0.0f};       /* 训练 Kd */

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
    float base_dm[4];
    float raw_dm[4];
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
    vel_ref[VJ_L_WHEEL] = action[VJ_L_WHEEL] * RL_TQ_WHEEL_VEL_SCALE;
    vel_ref[VJ_R_WHEEL] = action[VJ_R_WHEEL] * RL_TQ_WHEEL_VEL_SCALE;

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
    /* 机器力矩上限 */
    leg_limit = machine->dm_trq_clamp;
    wheel_limit = machine->dji_trq_clamp;
    for (uint32_t i = 0u; i < VJ_NUM; i++)
    {
        if (i == VJ_L_WHEEL || i == VJ_R_WHEEL)
        {
            tau_v[i] = clampf(tau_v[i], -wheel_limit, wheel_limit);
        }
        else
        {
            tau_v[i] = clampf(tau_v[i], -leg_limit, leg_limit);
        }
    }
    memcpy(state->virtual_torque, tau_v, sizeof(state->virtual_torque));

    /* 补偿前力矩 */
    base_dm[0] = tau_v[VJ_L_THIGH] + tau_v[VJ_L_SHANK] * leg_l->output.vshank_jac[1];
    base_dm[1] = tau_v[VJ_L_SHANK] * leg_l->output.vshank_jac[0];
    base_dm[2] = tau_v[VJ_R_THIGH] + tau_v[VJ_R_SHANK] * leg_r->output.vshank_jac[1];
    base_dm[3] = tau_v[VJ_R_SHANK] * leg_r->output.vshank_jac[0];

    /* 独立补偿叠加 */
    if (!Gas_Spring_Apply(leg_l, leg_r, base_dm, raw_dm))
    {
        return 0u;
    }

    /* DM 输出 (满限幅) */
    torque->dm[DM_MOTOR_LEG_F_LFT] = clampf(raw_dm[0], -leg_limit, leg_limit);
    torque->dm[DM_MOTOR_LEG_B_LFT] = clampf(raw_dm[1], -leg_limit, leg_limit);
    torque->dm[DM_MOTOR_LEG_F_RGT] = clampf(raw_dm[2], -leg_limit, leg_limit);
    torque->dm[DM_MOTOR_LEG_B_RGT] = clampf(raw_dm[3], -leg_limit, leg_limit);

    /* DJI 输出 (满限幅) */
    torque->dji[DJI_MOTOR_WHEEL_LFT] = clampf(tau_v[VJ_L_WHEEL], -wheel_limit, wheel_limit);
    torque->dji[DJI_MOTOR_WHEEL_RGT] = clampf(tau_v[VJ_R_WHEEL], -wheel_limit, wheel_limit);

    memcpy(&state->last_torque, torque, sizeof(state->last_torque));
    return 1u;
}
