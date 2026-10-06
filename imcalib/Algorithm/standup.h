#ifndef STANDUP_H
#define STANDUP_H

#include "leg_solver.h"
#include "imu_state.h"
#include "torque_output.h"
#include "pid.h"

enum {
    STANDUP_IDLE = 0,
    STANDUP_RETRACT,
    STANDUP_SWING,
    STANDUP_DONE,
    STANDUP_FAILED,
    STANDUP_REAR,
    STANDUP_EXTEND,
};

enum {
    STANDUP_OK = 0,
    STANDUP_BAD_INPUT,
    STANDUP_TIMEOUT,
    STANDUP_BAD_POSE,
    STANDUP_GATED,
    STANDUP_BAD_CONFIG,
};

enum {
    STANDUP_ROUTE_BALANCE = 0,
    STANDUP_ROUTE_PREPARE,
    STANDUP_ROUTE_STOP,
};

typedef struct {
    float extend_len;
    float extend_tol;
    float extend_timeout;
    float rear_angle;
    float rear_tol;
    float rear_rate;
    float rear_timeout;
    float retract_len;
    float retract_ready_len;
    float angle_pos_kp;
    float angle_pos_kd;
    float angle_speed_kp;
    float angle_speed_kd;
    float angle_speed_max;
    float tp_max;
    float trigger_angle;
    float trigger_pitch;
    float angle_tol;
    float length_tol;
    float pitch_max;
    float roll_max;
    float stable_time;
    float support_time;
    float blend_time;
    float trigger_time;
    float timeout[2];
} standup_param_t;

typedef struct {
    uint8_t enabled;
    uint8_t phase;
    uint8_t fault;
    uint8_t need;
    uint8_t len_history_ready;
    uint8_t angle_history_ready;
    uint8_t rear_path_ready;
    float elapsed;
    float stable;
    float trigger_elapsed;
    float blend;
    float support;
    float length_cmd[2];
    float angle_cmd[2];
    float angle_speed_cmd[2];
    float rear_position[2];   /* 展开角 */
    float rear_goal[2];
    float rear_last[2];
    float force[2];
    float tp[2];
    pid_t length_pid[2];
    pid_t angle_pos_pid[2];
    pid_t angle_speed_pid[2];
    torque_output_t prepare;
    float raw_dm[DM_MOTOR_NUM];   /* 限幅前 */
} standup_ctx_t;

extern standup_ctx_t standup_control;
extern standup_param_t standup_param;

void Standup_Init(standup_ctx_t *st);
void Standup_Reset(standup_ctx_t *st);
void Standup_Fail(standup_ctx_t *st, uint8_t fault);
/* DONE输入平衡候选 */
uint8_t Standup_Update(standup_ctx_t *st, const imu_state_t *imu,
                       const leg_state_t *left, const leg_state_t *right,
                       uint8_t permit, uint8_t allow_restart,
                       float dt, torque_output_t *torque);

#endif
