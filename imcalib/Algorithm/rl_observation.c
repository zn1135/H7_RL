#include "rl_observation.h"

#include <math.h>
#include <string.h>

#define RL_QUAT_EPS 1.0e-6f

/* 检查数组 */
static uint8_t RL_Observation_Array_Finite(const float *data, uint32_t count)
{
    uint32_t i;

    if (data == NULL) return 0u;
    for (i = 0u; i < count; i++)
    {
        if (!isfinite(data[i])) return 0u;
    }
    return 1u;
}

/* 检查参数 */
static uint8_t RL_Observation_Param_Valid(const rl_observation_param_t *param)
{
    if (param == NULL || !param->configured) return 0u;
    if (!RL_Observation_Array_Finite(param->obs_dof_pos, 4u)) return 0u;
    if (!RL_Observation_Array_Finite(param->command_scale, 3u)) return 0u;
    if (!RL_Observation_Array_Finite(param->gyro_scale, 3u)) return 0u;
    if (!RL_Observation_Array_Finite(param->joint_vel_scale, 6u)) return 0u;
    return 1u;
}

/* 初始化观测 */
void RL_Observation_Init(rl_observation_state_t *state)
{
    RL_Observation_Reset(state);
}

/* 初始化参数: 训练侧观测缩放 + 默认角 (见 rl_observation.h) */
void RL_Observation_Param_Init(rl_observation_param_t *param)
{
    uint32_t i;

    if (param == NULL) return;
    memset(param, 0, sizeof(*param));
    param->obs_dof_pos[0] = RL_OBS_DOF_POS_L_THIGH;
    param->obs_dof_pos[1] = RL_OBS_DOF_POS_L_SHANK;
    param->obs_dof_pos[2] = RL_OBS_DOF_POS_R_THIGH;
    param->obs_dof_pos[3] = RL_OBS_DOF_POS_R_SHANK;
    param->command_scale[0] = RL_OBS_CMD_VX_SCALE;
    param->command_scale[1] = RL_OBS_CMD_YAW_SCALE;
    param->command_scale[2] = RL_OBS_CMD_HEIGHT_SCALE;
    for (i = 0u; i < 3u; i++)
    {
        param->gyro_scale[i] = RL_OBS_GYRO_SCALE;
    }
    for (i = 0u; i < 6u; i++)
    {
        param->joint_vel_scale[i] = RL_OBS_JOINT_VEL_SCALE;
    }
    param->configured = 1u;
}

/* 清空观测 */
void RL_Observation_Reset(rl_observation_state_t *state)
{
    if (state == NULL) return;
    memset(state, 0, sizeof(*state));
}

/* 计算重力 */
static void RL_Observation_Project_Gravity_Unit(const float quat[4], float gravity[3])
{
    float qx, qy, qz;  /* 四元虚部 */
    float qw;          /* 四元实部 */
    float tx, ty, tz;  /* 首次叉乘 */
    float cx, cy, cz;  /* 二次叉乘 */

    /* 对齐策略 */
    qx = -quat[1];
    qy = -quat[2];
    qz = -quat[3];
    qw = quat[0];

    /* 旋转重力 */
    tx = -2.0f * qy;
    ty =  2.0f * qx;
    tz =  0.0f;
    cx = qy * tz - qz * ty;
    cy = qz * tx - qx * tz;
    cz = qx * ty - qy * tx;

    gravity[0] = qw * tx + cx;
    gravity[1] = qw * ty + cy;
    gravity[2] = -1.0f + qw * tz + cz;
}

/* 投影重力接口 */
uint8_t RL_Observation_Project_Gravity(const float quat[4], float gravity[3])
{
    float norm;
    float quat_unit[4];
    uint32_t i;

    if (quat == NULL || gravity == NULL)
    {
        return 0u;
    }
    norm = sqrtf(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
    if (!isfinite(norm) || norm < RL_QUAT_EPS)
    {
        return 0u;
    }
    for (i = 0u; i < 4u; i++)
    {
        quat_unit[i] = quat[i] / norm;
    }
    RL_Observation_Project_Gravity_Unit(quat_unit, gravity);
    return RL_Observation_Array_Finite(gravity, 3u);
}

/* 构建观测 */
uint8_t RL_Observation_Build(rl_observation_state_t *state,
                             const rl_observation_param_t *param,
                             const float gyro_rad_s[3],
                             const float quat[4],
                             const float command[3],
                             const float joint_pos[4],
                             const float joint_vel[6],
                             uint8_t source_valid)
{
    float gravity[3];    /* 投影重力 */
    float quat_norm;     /* 四元模长 */
    float quat_unit[4];  /* 单位四元 */
    uint32_t i;          /* 循环索引 */

    if (state == NULL) return 0u;
    if (!source_valid || !RL_Observation_Param_Valid(param)
        || !RL_Observation_Array_Finite(gyro_rad_s, 3u)
        || !RL_Observation_Array_Finite(quat, 4u)
        || !RL_Observation_Array_Finite(command, 3u)
        || !RL_Observation_Array_Finite(joint_pos, 4u)
        || !RL_Observation_Array_Finite(joint_vel, 6u))
    {
        RL_Observation_Reset(state);
        return 0u;
    }

    quat_norm = sqrtf(quat[0] * quat[0] + quat[1] * quat[1]
                    + quat[2] * quat[2] + quat[3] * quat[3]);
    if (!isfinite(quat_norm) || quat_norm < RL_QUAT_EPS)
    {
        RL_Observation_Reset(state);
        return 0u;
    }

    for (i = 0u; i < 4u; i++) quat_unit[i] = quat[i] / quat_norm;
    RL_Observation_Project_Gravity_Unit(quat_unit, gravity);
    if (!RL_Observation_Array_Finite(gravity, 3u))
    {
        RL_Observation_Reset(state);
        return 0u;
    }

    /* imu_state 已由 machine.imu 完成轴映射与极性校正；策略按训练 XYZ 直接读取。 */
    for (i = 0u; i < 3u; i++)
        state->obs[RL_OBS_GYRO_X + i] = gyro_rad_s[i] * param->gyro_scale[i];
    for (i = 0u; i < 3u; i++)
        state->obs[RL_OBS_GRAV_X + i] = gravity[i];
    for (i = 0u; i < 3u; i++)
        state->obs[RL_OBS_CMD_VX + i] = command[i] * param->command_scale[i];

    state->obs[RL_OBS_L_THIGH] = joint_pos[0] - param->obs_dof_pos[0];
    state->obs[RL_OBS_L_SHANK] = joint_pos[1] - param->obs_dof_pos[1];
    state->obs[RL_OBS_R_THIGH] = joint_pos[2] - param->obs_dof_pos[2];
    state->obs[RL_OBS_R_SHANK] = joint_pos[3] - param->obs_dof_pos[3];

    for (i = 0u; i < 6u; i++)
        state->obs[RL_OBS_L_THIGH_VEL + i] = joint_vel[i] * param->joint_vel_scale[i];
    for (i = 0u; i < RL_ACTION_SIZE; i++)
        state->obs[RL_OBS_LAST_ACTION + i] = state->last_action[i];

    /* 整体裁剪 */
    for (i = 0u; i < RL_OBS_SIZE; i++)
    {
        if (state->obs[i] > RL_OBS_CLIP)
        {
            state->obs[i] = RL_OBS_CLIP;
        }
        else if (state->obs[i] < -RL_OBS_CLIP)
        {
            state->obs[i] = -RL_OBS_CLIP;
        }
    }

    if (!RL_Observation_Array_Finite(state->obs, RL_OBS_SIZE))
    {
        RL_Observation_Reset(state);
        return 0u;
    }

    state->valid = 1u;
    return 1u;
}

/* 更新历史 */
void RL_Observation_Update_History(rl_observation_state_t *state)
{
    uint32_t i;  /* 循环索引 */

    if (state == NULL || !state->valid)
    {
        RL_Observation_Reset(state);
        return;
    }

    if (!state->history_ready)
    {
        for (i = 0u; i < RL_OBS_HISTORY_FRAMES; i++)
            memcpy(&state->history[i * RL_OBS_SIZE], state->obs,
                   RL_OBS_SIZE * sizeof(float));
        state->history_ready = 1u;
        return;
    }

    memmove(&state->history[0], &state->history[RL_OBS_SIZE],
            (RL_OBS_HISTORY_SIZE - RL_OBS_SIZE) * sizeof(float));
    memcpy(&state->history[RL_OBS_HISTORY_SIZE - RL_OBS_SIZE], state->obs,
           RL_OBS_SIZE * sizeof(float));
}

/* 保存动作 */
void RL_Observation_Set_Last_Action(rl_observation_state_t *state,
                                    const float action[RL_ACTION_SIZE])
{
    if (state == NULL || action == NULL) return;

    memcpy(state->last_action, action, RL_ACTION_SIZE * sizeof(float));
}
