#ifndef RL_OBSERVATION_H
#define RL_OBSERVATION_H

#include <stdint.h>

#define RL_OBS_SIZE           25u
#define RL_OBS_HISTORY_FRAMES 5u
#define RL_OBS_HISTORY_SIZE   (RL_OBS_SIZE * RL_OBS_HISTORY_FRAMES)
#define RL_ACTION_SIZE        6u

/* 训练侧观测参数 (chuanliantui_standup, 训练侧说明 2026-09-22) */
#define RL_OBS_GYRO_SCALE        0.25f
#define RL_OBS_CMD_VX_SCALE      2.0f
#define RL_OBS_CMD_YAW_SCALE     0.25f
#define RL_OBS_CMD_HEIGHT_SCALE  5.0f
#define RL_OBS_JOINT_VEL_SCALE   0.05f
#define RL_OBS_DOF_POS_L_THIGH   (-0.06f)   /* 训练默认角 lf0 */
#define RL_OBS_DOF_POS_L_SHANK   (0.10f)    /* lf1 */
#define RL_OBS_DOF_POS_R_THIGH   (0.06f)    /* rf0 */
#define RL_OBS_DOF_POS_R_SHANK   (-0.10f)   /* rf1 */
#define RL_OBS_CLIP              100.0f     /* 观测裁剪 */

/* 观测索引 */
typedef enum {
    RL_OBS_GYRO_X = 0,       /* 角速度 */
    RL_OBS_GYRO_Y,
    RL_OBS_GYRO_Z,
    RL_OBS_GRAV_X,           /* 投影重力 */
    RL_OBS_GRAV_Y,
    RL_OBS_GRAV_Z,
    RL_OBS_CMD_VX,           /* 控制指令 */
    RL_OBS_CMD_YAW_RATE,
    RL_OBS_CMD_HEIGHT,
    RL_OBS_L_THIGH,          /* 关节角度 */
    RL_OBS_L_SHANK,
    RL_OBS_R_THIGH,
    RL_OBS_R_SHANK,
    RL_OBS_L_THIGH_VEL,      /* 关节速度 */
    RL_OBS_L_SHANK_VEL,
    RL_OBS_L_WHEEL_VEL,
    RL_OBS_R_THIGH_VEL,
    RL_OBS_R_SHANK_VEL,
    RL_OBS_R_WHEEL_VEL,
    RL_OBS_LAST_ACTION = 19  /* 上次动作 */
} rl_obs_index_t;

typedef struct {
    float obs_dof_pos[4];        /* 观测中位 */
    float command_scale[3];      /* 指令缩放 */
    float gyro_scale[3];         /* 陀螺缩放 */
    float joint_vel_scale[6];    /* 速度缩放 */
    uint8_t configured;          /* 参数有效 */
} rl_observation_param_t;

typedef struct {
    float obs[RL_OBS_SIZE];              /* 当前观测 */
    float history[RL_OBS_HISTORY_SIZE];  /* 五帧历史 */
    float last_action[RL_ACTION_SIZE];   /* 上次动作 */
    uint8_t valid;                       /* 观测有效 */
    uint8_t history_ready;               /* 历史有效 */
} rl_observation_state_t;

void RL_Observation_Init(rl_observation_state_t *state);
void RL_Observation_Param_Init(rl_observation_param_t *param);
void RL_Observation_Reset(rl_observation_state_t *state);
uint8_t RL_Observation_Project_Gravity(const float quat[4], float gravity[3]);
uint8_t RL_Observation_Build(rl_observation_state_t *state,
                             const rl_observation_param_t *param,
                             const float gyro_rad_s[3],
                             const float quat[4],
                             const float command[3],
                             const float joint_pos[4],
                             const float joint_vel[6],
                             uint8_t source_valid);
void RL_Observation_Update_History(rl_observation_state_t *state);
void RL_Observation_Set_Last_Action(rl_observation_state_t *state,
                                    const float action[RL_ACTION_SIZE]);

#endif
