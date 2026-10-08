#include "robot_control.h"
#include "Attitude_Algorithm.h"
#include "dr16.h"
#include "dm.h"
#include "dji.h"
#include "can_bus.h"
#include "machine_config.h"
#include "standup.h"
#include "Vofa_send.h"
#include "uart_idle.h"
#include "ws2812.h"
#include "mono_ns.h"
#include "task.h"
#include "../Telemetry/vofa_trace.h"
#include "../Telemetry/joint_usb.h"
#include "usbd_cdc_if.h"

#include <math.h>
#include <string.h>

/* 更新电机状态 */
static void Motor_State_Update(motor_state_t *next)
{
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        const dm_motor_feedback_t *feedback = &dm_motor_feedback[i];

        next->dm.pos_rad[i] = feedback->pos_rad;
        next->dm.pos_zero_rad[i] = feedback->pos_zero_rad;
        next->dm.vel_rad_s[i] = feedback->vel_rad_s;
        next->dm.trq_nm[i] = feedback->trq_nm;
        next->dm.last_rx_tick[i] = feedback->last_rx_tick;
        next->dm.parsed_rx_ns[i] = feedback->parsed_rx_ns;
        next->dm.online[i] = (uint8_t)Dm_Is_Online(i);
    }
    for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        const dji_motor_feedback_t *feedback = &dji_motor_feedback[i];

        next->dji.angle_rad[i] = feedback->angle_rad;
        next->dji.angle_total_rad[i] = feedback->angle_total_rad;
        next->dji.vel_rad_s[i] = feedback->vel_rad_s;
        next->dji.current_raw[i] = feedback->current_raw;
        next->dji.last_rx_tick[i] = feedback->last_rx_tick;
        next->dji.online[i] = (uint8_t)Dji_Is_Online(i);
    }
    next->timestamp_ms = HAL_GetTick();
    next->updated = 1u;
}

/* 更新腿部状态 */
static void Leg_State_Update(const motor_state_t *motor, leg_state_t *left, leg_state_t *right)
{
    if (leg_map_l.configured
        && (uint8_t)leg_map_l.dm_front < DM_MOTOR_NUM && (uint8_t)leg_map_l.dm_rear < DM_MOTOR_NUM)
    {
        left->input.hip_f = motor->dm.pos_zero_rad[leg_map_l.dm_front] + LEG_PI;
        left->input.hip_b = motor->dm.pos_zero_rad[leg_map_l.dm_rear];
        left->input.d_hip_f = motor->dm.vel_rad_s[leg_map_l.dm_front];
        left->input.d_hip_b = motor->dm.vel_rad_s[leg_map_l.dm_rear];
        (void)Leg_Solve(left);
    }
    else
    {
        memset(&left->output, 0, sizeof(left->output));
    }
    if (leg_map_r.configured
        && (uint8_t)leg_map_r.dm_front < DM_MOTOR_NUM && (uint8_t)leg_map_r.dm_rear < DM_MOTOR_NUM)
    {
        right->input.hip_f = motor->dm.pos_zero_rad[leg_map_r.dm_front] + LEG_PI;
        right->input.hip_b = motor->dm.pos_zero_rad[leg_map_r.dm_rear];
        right->input.d_hip_f = motor->dm.vel_rad_s[leg_map_r.dm_front];
        right->input.d_hip_b = motor->dm.vel_rad_s[leg_map_r.dm_rear];
        (void)Leg_Solve(right);
    }
    else
    {
        memset(&right->output, 0, sizeof(right->output));
    }
}

/* 更新遥控: 指令解算 + 使能 */
static void Remote_Control_Update(void)
{
    dr16_t remote;
    rc_command_t next;
    uint8_t rc_enable;

    DR16_Process();
    remote = DR16_Snapshot();
    next = rc_command;
    Rc_Command_Update(&next, &remote);
    rc_enable = strategy_rc_enable(&next);
    taskENTER_CRITICAL();
    rc_command = next;
    robot_state.rc_enable = rc_enable;
    input_command.mode = remote.online ? remote.s1 : 0u;
    taskEXIT_CRITICAL();
}

/* 更新故障状态 */
static void Robot_Fault_Update(void)
{
    static uint32_t rl_ready_lost_tick;
    uint32_t fault;
    uint8_t motors_ok;

    fault = FAULT_NONE;
    motors_ok = 1u;
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (!motor_state.dm.online[i] || Dm_Has_Fault(i))
        {
            motors_ok = 0u;
        }
    }
    for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        if (!motor_state.dji.online[i])
        {
            motors_ok = 0u;
        }
    }
    if (!imu_state.online || !imu_state.pitch_world_valid || !isfinite(imu_state.pitch_world))
    {
        fault |= FAULT_IMU;
    }
    if (!DR16_Online())
    {
        fault |= FAULT_RC;
    }
    if (!Can_Bus_Online(true))
    {
        fault |= FAULT_CAN;
    }
    if (!motors_ok)
    {
        fault |= FAULT_MOTOR;
    }
    if (ctrl_strategy == CTRL_STRATEGY_RL
        && (!action_state.updated || HAL_GetTick() - action_state.last_ok_tick >= 100u))
    {
        fault |= FAULT_ACTION;
    }
    /* 动作持续不可用 */
    if (output_task_rl_engaged() && action_state.rl_ready == 0)
    {
        if (rl_ready_lost_tick == 0u)
        {
            rl_ready_lost_tick = HAL_GetTick();
        }
        else if (HAL_GetTick() - rl_ready_lost_tick >= 100u)
        {
            fault |= FAULT_ACTION;
        }
    }
    else
    {
        rl_ready_lost_tick = 0u;
    }
    ctrl_fault = fault;
}

/* 更新翻倒状态 */
static void Robot_Fallen_Update(void)
{
    float pitch_abs;

    if (ctrl_fault & FAULT_IMU)
    {
        return;
    }
    taskENTER_CRITICAL();
    pitch_abs = fabsf(imu_state.pitch_world);
    taskEXIT_CRITICAL();
    if (pitch_abs > 1.4f)
    {
        robot_state.fallen = 1u;
    }
    else if (pitch_abs < 1.0f)
    {
        robot_state.fallen = 0u;
    }
}

/* 更新使能状态: 使能沿发使能, 失能沿发失能; 两个方向都有看门狗兜底 */
static void Robot_Enable_Update(void)
{
    uint8_t enable_request;

    enable_request = Robot_Control_Enable_Allowed();
    if (enable_request && !robot_state.motor_enabled)
    {
        robot_state.motor_enabled = 1u;
        (void)Dm_All_Enable();
    }
    else if (!enable_request && robot_state.motor_enabled)
    {
        robot_state.motor_enabled = 0u;
        (void)Dji_All_Stop();
        (void)Dm_All_Disable();
    }

    if (robot_state.motor_enabled)
    {
        Dm_Enable_Watchdog();
    }
    else
    {
        Dm_Disable_Watchdog();
    }
}

static void Robot_Control_Send_Vofa(void)
{
    static float dbg[VOFA_MAX_CH];
    uint8_t online_mask;
    uint16_t state_bits;

    memset(dbg, 0, sizeof(dbg));
    /* ch0 在线掩码 */
    online_mask  = imu_state.online ? 0x01u : 0x00u;
    online_mask |= DR16_Online() ? 0x02u : 0x00u;
    online_mask |= motor_state.dm.online[0] ? 0x04u : 0x00u;
    online_mask |= motor_state.dm.online[1] ? 0x08u : 0x00u;
    online_mask |= motor_state.dm.online[2] ? 0x10u : 0x00u;
    online_mask |= motor_state.dm.online[3] ? 0x20u : 0x00u;
    online_mask |= motor_state.dji.online[0] ? 0x40u : 0x00u;
    online_mask |= motor_state.dji.online[1] ? 0x80u : 0x00u;
    dbg[0] = (float)online_mask;

    /* ch1 状态位: 使能/跌倒/左腿有效/右腿有效/四髋使能/已投入 */
    state_bits  = robot_state.motor_enabled ? 0x01u : 0x00u;
    state_bits |= robot_state.fallen ? 0x02u : 0x00u;
    state_bits |= leg_l.output.valid ? 0x04u : 0x00u;
    state_bits |= leg_r.output.valid ? 0x08u : 0x00u;
    state_bits |= Dm_Is_Enabled(DM_MOTOR_LEG_F_LFT) ? 0x10u : 0x00u;
    state_bits |= Dm_Is_Enabled(DM_MOTOR_LEG_B_LFT) ? 0x20u : 0x00u;
    state_bits |= Dm_Is_Enabled(DM_MOTOR_LEG_F_RGT) ? 0x40u : 0x00u;
    state_bits |= Dm_Is_Enabled(DM_MOTOR_LEG_B_RGT) ? 0x80u : 0x00u;
    state_bits |= output_task_lqr_engaged() ? 0x100u : 0x00u;
    state_bits |= output_task_rl_engaged() ? 0x200u : 0x00u;
    dbg[1] = (float)state_bits;
    dbg[2] = (float)ctrl_fault;


    for(int i = 0;i<10;i++)
    {
        dbg[i + 3] = lqr_state.x[i];
    }
    for(int i = 0;i<10;i++)
    {
        dbg[i + 13] = lqr_state.target[i];
    }
    dbg[23] = lqr_state.a_fwd;
    (void)Vofa_Send(dbg, VOFA_MAX_CH);
}

/* 通信单周期 */
void comm_task_body(void)
{
    motor_state_t next_motor;
    leg_state_t next_left;
    leg_state_t next_right;

    Dm_Parse();
    Dji_Parse();
    next_motor = motor_state;
    next_left = leg_l;
    next_right = leg_r;
    Motor_State_Update(&next_motor);
    Leg_State_Update(&next_motor, &next_left, &next_right);
    taskENTER_CRITICAL();
    motor_state = next_motor;
    leg_l = next_left;
    leg_r = next_right;
    taskEXIT_CRITICAL();

    WS2812_RainbowBlink();
    Remote_Control_Update();
    JointUsb_Process();
    Robot_Fault_Update();
    Robot_Fallen_Update();
    Robot_Enable_Update();
    JointUsb_Pump();
    /* 策略追踪与普通 VOFA 同口互斥 */
    if (Vofa_Transport_Update((uint8_t)(!robot_state.motor_enabled
        && !JointUsb_ModeLock() && !JointUsb_StreamRequested())))
    {
        Vofa_Trace_Discard();
    }
    if (!Vofa_Trace_Pump())
    {
        Robot_Control_Send_Vofa();
    }
    
    // Dm_Send_Command(0, DM_CMD_SET_ZERO);
    // Dm_Send_Command(1, DM_CMD_SET_ZERO); 
    // Dm_Send_Command(2, DM_CMD_SET_ZERO);
    // Dm_Send_Command(3, DM_CMD_SET_ZERO); 
}
