#include "robot_control.h"
#include "machine_config.h"

#include <string.h>

imu_state_t imu_state;
motor_state_t motor_state;
leg_state_t leg_l;
leg_state_t leg_r;
action_state_t action_state;
input_command_t input_command;
rc_command_t rc_command;
leg_map_t leg_map_l;
leg_map_t leg_map_r;
robot_state_t robot_state;
rl_control_state_t rl_control;
lqr_state_t lqr_state;
leg_balance_t leg_balance;
volatile ctrl_strategy_t ctrl_strategy;
volatile uint32_t ctrl_fault;
volatile uint8_t output_debug_dm_sent;
volatile uint8_t output_debug_dji_sent;
uint8_t torque_output_enabled;

osSemaphoreDef(ctrl_tick_sem);
osSemaphoreId ctrl_tick_sem_handle = NULL;
osSemaphoreDef(policy_tick_sem);
osSemaphoreId policy_tick_sem_handle = NULL;

/* TIM6 节拍分频: 每 MACHINE_POLICY_DIV 拍释放策略节拍 (ISR 调用) */
void Policy_Tick_Div(void)
{
    static uint8_t div_cnt;   /* 分频计数 */

    if (++div_cnt >= MACHINE_POLICY_DIV)
    {
        div_cnt = 0u;
        osSemaphoreRelease(policy_tick_sem_handle);
    }
}

/* 清空动作 */
void Action_State_Clear(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memset(action_state.a, 0, sizeof(action_state.a));
    action_state.last_ok_tick = 0u;
    action_state.updated = 0u;
    action_state.rl_ready = 0u;
    __set_PRIMASK(primask);
}

/* 控制初始化 */
void Robot_Control_Init(void)
{
    ctrl_tick_sem_handle = osSemaphoreCreate(osSemaphore(ctrl_tick_sem), 1);
    if (ctrl_tick_sem_handle == NULL)
    {
        Error_Handler();
    }
    policy_tick_sem_handle = osSemaphoreCreate(osSemaphore(policy_tick_sem), 1);
    if (policy_tick_sem_handle == NULL)
    {
        Error_Handler();
    }
    torque_output_enabled = 1u;

    Leg_Init(&leg_l);
    Leg_Init(&leg_r);
    leg_l.config.lu = machine->leg_lu;
    leg_l.config.lg = machine->leg_lg;
    leg_l.config.offset_phi0 = machine->leg_off_phi0[0];
    leg_l.config.configured = 1u;
    leg_r.config.lu = machine->leg_lu;
    leg_r.config.lg = machine->leg_lg;
    leg_r.config.offset_phi0 = machine->leg_off_phi0[1];
    leg_r.config.configured = 1u;

    leg_map_l.dm_front = DM_MOTOR_LEG_F_LFT;
    leg_map_l.dm_rear = DM_MOTOR_LEG_B_LFT;
    leg_map_l.configured = 1u;
    leg_map_r.dm_front = DM_MOTOR_LEG_F_RGT;
    leg_map_r.dm_rear = DM_MOTOR_LEG_B_RGT;
    leg_map_r.configured = 1u;

    if (machine->rl.configured)
    {
        RL_Observation_Init(&rl_control.observation);
        RL_Observation_Param_Init(&rl_control.param);
        RL_Policy_Reset(&rl_control.policy);
        RL_Torque_Param_Init(&rl_control.torque_param[RL_MODEL_STANDUP], RL_MODEL_STANDUP);
        RL_Torque_State_Init(&rl_control.torque_state, &rl_control.torque_param[RL_MODEL_STANDUP]);
    }
    Action_State_Clear();

    if (machine->lqr_configured)
    {
        LQR_Init(&lqr_state);
        Leg_Balance_Init(&leg_balance);
    }
    ctrl_strategy = CTRL_STRATEGY_DISABLE;
}

/* 切换模型 */
uint8_t RL_Control_Select_Model(rl_model_t model)
{
    if (!machine->rl.configured)
    {
        return 0u;
    }
    if (!RL_Policy_Select(&rl_control.policy, model))
    {
        return 0u;
    }
    RL_Observation_Reset(&rl_control.observation);
    RL_Torque_State_Init(&rl_control.torque_state, &rl_control.torque_param[model]);
    Action_State_Clear();
    return 1u;
}
