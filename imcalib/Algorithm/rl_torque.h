#ifndef RL_TORQUE_H
#define RL_TORQUE_H

#include <stdint.h>

#include "dm.h"
#include "dji.h"
#include "leg_solver.h"
#include "pid.h"
#include "rl_policy.h"
#include "torque_output.h"

typedef struct {
    float dof_pos[6];
    float p_gains[6];
    float d_gains[6];
    float wheel_pid[2][3]; /* [左/右][kp, ki, kd] */
} rl_torque_param_t;

typedef struct {
    torque_output_t last_torque;
    float virtual_torque[RL_ACTION_SIZE];
    float pos_target[RL_ACTION_SIZE];
    pid_t controller[RL_ACTION_SIZE];
} rl_torque_state_t;

void RL_Torque_Param_Init(rl_torque_param_t *param, rl_model_t model);
void RL_Torque_State_Init(rl_torque_state_t *state,
                          const rl_torque_param_t *param);
uint8_t RL_Torque_Compute(const leg_state_t *leg_l, const leg_state_t *leg_r,
                          const rl_torque_param_t *param,
                          const float wheel_vel[2],
                          const float action[RL_ACTION_SIZE],
                          rl_torque_state_t *state,
                          torque_output_t *torque);

#endif
