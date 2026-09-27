#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "main.h"
#include "cmsis_os.h"
#include "imu_state.h"
#include "dm.h"
#include "dji.h"
#include "rc_command.h"
#include "leg_solver.h"
#include "rl_observation.h"
#include "rl_policy.h"
#include "rl_torque.h"
#include "lqr_balance.h"
#include "leg_balance.h"
#include "machine_config.h"

#define CTRL_DT MACHINE_CTRL_DT

/* 控制策略 */
typedef enum {
    CTRL_STRATEGY_RL = 0,       /* RL 推理 */
    CTRL_STRATEGY_LQR,          /* LQR 平衡 */
    CTRL_STRATEGY_DISABLE,      /* 左下 / 离线 */
} ctrl_strategy_t;

typedef struct {
    float pos_rad[DM_MOTOR_NUM];        /* 解码角 */
    float pos_zero_rad[DM_MOTOR_NUM];   /* 加零点 */
    float vel_rad_s[DM_MOTOR_NUM];
    float trq_nm[DM_MOTOR_NUM];
    uint32_t last_rx_tick[DM_MOTOR_NUM];
    uint8_t online[DM_MOTOR_NUM];
} dm_motor_state_t;

typedef struct {
    float angle_rad[DJI_MOTOR_NUM];
    float angle_total_rad[DJI_MOTOR_NUM];
    float vel_rad_s[DJI_MOTOR_NUM];
    int16_t current_raw[DJI_MOTOR_NUM];
    uint32_t last_rx_tick[DJI_MOTOR_NUM];
    uint8_t online[DJI_MOTOR_NUM];
} dji_motor_state_t;

typedef struct {
    dm_motor_state_t dm;
    dji_motor_state_t dji;
    uint32_t timestamp_ms;
    uint8_t updated;
} motor_state_t;

typedef struct {
    float a[RL_ACTION_SIZE];
    uint32_t last_ok_tick;
    uint8_t updated;
    uint8_t rl_ready;               /* 推理动作可用 */
} action_state_t;

typedef struct {
    int8_t dm_front;
    int8_t dm_rear;
    uint8_t configured;
} leg_map_t;

typedef struct {
    uint8_t rc_enable;
    uint8_t motor_enabled;
    uint8_t fallen;
} robot_state_t;

typedef struct {
    rl_observation_state_t observation;
    rl_observation_param_t param;
    rl_policy_t policy;
    rl_torque_param_t torque_param[RL_MODEL_COUNT];
    rl_torque_state_t torque_state;
} rl_control_state_t;

typedef struct {
    float vx_cmd;
    float yaw_cmd;
    float height_cmd;
    float thigh_delta_cmd[2];
    float shin_delta[2];
    uint8_t mode;
} input_command_t;

#define FAULT_NONE    0u
#define FAULT_IMU     0x01u
#define FAULT_RC      0x02u
#define FAULT_MOTOR   0x04u
#define FAULT_CAN     0x08u
#define FAULT_ACTION  0x10u

extern imu_state_t imu_state;
extern motor_state_t motor_state;
extern leg_state_t leg_l;
extern leg_state_t leg_r;
extern action_state_t action_state;
extern input_command_t input_command;
extern rc_command_t rc_command;
extern leg_map_t leg_map_l;
extern leg_map_t leg_map_r;
extern robot_state_t robot_state;
extern rl_control_state_t rl_control;
extern lqr_state_t lqr_state;
extern leg_balance_t leg_balance;
extern volatile ctrl_strategy_t ctrl_strategy;
extern volatile uint32_t ctrl_fault;
extern volatile uint8_t output_debug_dm_sent;
extern volatile uint8_t output_debug_dji_sent;
extern volatile float rl_output_dm_cmd_nm[DM_MOTOR_NUM];
extern volatile float rl_output_wheel_cmd_nm[DJI_MOTOR_NUM];
extern uint8_t torque_output_enabled;
extern osSemaphoreId ctrl_tick_sem_handle;
extern osSemaphoreId policy_tick_sem_handle;

void Robot_Control_Init(void);
void Action_State_Clear(void);
/* TIM6 节拍分频: 每 MACHINE_POLICY_DIV 拍释放策略节拍 (ISR 调用) */
void Policy_Tick_Div(void);
uint8_t RL_Control_Select_Model(rl_model_t model);
void imu_task_init(void);
void imu_task_body(void);
void ctrl_task_init(void);
void ctrl_task_body(void);
void output_task_init(void);
void output_task_body(void);
uint8_t output_task_lqr_engaged(void);
uint8_t output_task_rl_engaged(void);
/* 遥控使能唯一判定 */
uint8_t strategy_rc_enable(const rc_command_t *cmd);
void comm_task_body(void);

#endif
