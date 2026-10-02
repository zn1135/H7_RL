#include "robot_control.h"
#include "Attitude_Algorithm.h"
#include "dr16.h"
#include "dm.h"
#include "dji.h"
#include "can_bus.h"
#include "machine_config.h"
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
static void Motor_State_Update(void)
{
    vTaskSuspendAll();
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        const dm_motor_feedback_t *feedback = &dm_motor_feedback[i];

        motor_state.dm.pos_rad[i] = feedback->pos_rad;
        motor_state.dm.pos_zero_rad[i] = feedback->pos_zero_rad;
        motor_state.dm.vel_rad_s[i] = feedback->vel_rad_s;
        motor_state.dm.trq_nm[i] = feedback->trq_nm;
        motor_state.dm.last_rx_tick[i] = feedback->last_rx_tick;
        motor_state.dm.parsed_rx_ns[i] = feedback->parsed_rx_ns;
        motor_state.dm.online[i] = (uint8_t)Dm_Is_Online(i);
    }
    (void)xTaskResumeAll();
    for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        const dji_motor_feedback_t *feedback = &dji_motor_feedback[i];

        motor_state.dji.angle_rad[i] = feedback->angle_rad;
        motor_state.dji.angle_total_rad[i] = feedback->angle_total_rad;
        motor_state.dji.vel_rad_s[i] = feedback->vel_rad_s;
        motor_state.dji.current_raw[i] = feedback->current_raw;
        motor_state.dji.last_rx_tick[i] = feedback->last_rx_tick;
        motor_state.dji.online[i] = (uint8_t)Dji_Is_Online(i);
    }
    motor_state.timestamp_ms = HAL_GetTick();
    motor_state.updated = 1u;
}

/* 更新腿部状态 */
static void Leg_State_Update(void)
{
    if (leg_map_l.configured)
    {
        leg_l.input.hip_f = motor_state.dm.pos_zero_rad[leg_map_l.dm_front] + LEG_PI;
        leg_l.input.hip_b = motor_state.dm.pos_zero_rad[leg_map_l.dm_rear];
        leg_l.input.d_hip_f = motor_state.dm.vel_rad_s[leg_map_l.dm_front];
        leg_l.input.d_hip_b = motor_state.dm.vel_rad_s[leg_map_l.dm_rear];
    }
    if (leg_map_r.configured)
    {
        leg_r.input.hip_f = motor_state.dm.pos_zero_rad[leg_map_r.dm_front] + LEG_PI;
        leg_r.input.hip_b = motor_state.dm.pos_zero_rad[leg_map_r.dm_rear];
        leg_r.input.d_hip_f = motor_state.dm.vel_rad_s[leg_map_r.dm_front];
        leg_r.input.d_hip_b = motor_state.dm.vel_rad_s[leg_map_r.dm_rear];
    }
    vTaskSuspendAll();
    (void)Leg_Solve(&leg_l);
    (void)Leg_Solve(&leg_r);
    (void)xTaskResumeAll();
}

/* 更新遥控: 指令解算 + 使能 */
static void Remote_Control_Update(void)
{
    dr16_t remote;

    DR16_Process();
    remote = DR16_Snapshot();
    Rc_Command_Update(&rc_command, &remote);
    robot_state.rc_enable = strategy_rc_enable(&rc_command);
    input_command.mode = remote.online ? remote.s1 : 0u;
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
    if (!imu_state.online)
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

    pitch_abs = fabsf(imu_state.euler_rad[ATTITUDE_PITCH]);
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

    enable_request = (uint8_t)(robot_state.rc_enable
        && ctrl_fault == FAULT_NONE && !robot_state.fallen);
    if (JointUsb_ModeLock())
    {
        enable_request = JointUsb_EnableAllowed();
    }
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

/* 普通 VOFA: 状态3 + 观测25 + 大腿角2 + 腿长2 */
static void Robot_Control_Send_Vofa(void)
{
    static float dbg[32];
    uint8_t online_mask;
    uint16_t state_bits;
    uint32_t rl_bits;

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

    rl_bits  = rl_control.policy.ready ? 0x01u : 0x00u;
    rl_bits |= rl_control.observation.history_ready ? 0x02u : 0x00u;
    rl_bits |= action_state.rl_ready ? 0x04u : 0x00u;
    rl_bits |= output_task_rl_engaged() ? 0x08u : 0x00u;
    rl_bits |= rl_control.observation.valid ? 0x40u : 0x00u;
    rl_bits |= machine->rl.configured ? 0x80u : 0x00u;
    rl_bits |= (rl_control.policy.run_fail & 0xFFu) << 8;
    rl_bits |= output_debug_dm_sent ? 0x00010000u : 0x00u;
    rl_bits |= output_debug_dji_sent ? 0x00020000u : 0x00u;
    dbg[2] = (float)rl_bits;

    for(int i = 0;i<25;i++)
    {
        dbg[i + 3] = rl_control.observation.obs[i];
    }
    // dbg[28] = leg_l.output.virtual_leg_length;
    // dbg[29] = leg_r.output.virtual_leg_length;
    // dbg[30] = leg_l.output.virtual_leg_angle;
    // dbg[31] = leg_r.output.virtual_leg_angle;
    dbg[28] = leg_l.output.thigh_angle;
    dbg[29] = leg_r.output.thigh_angle;
    dbg[30] = leg_l.output.virtual_leg_length;
    dbg[31] = leg_r.output.virtual_leg_length;

    (void)Vofa_Send(dbg, 32u);
}

/* 通信单周期 */
void comm_task_body(void)
{
    Dm_Parse();
    Dji_Parse();
    Motor_State_Update();
    Leg_State_Update();
    WS2812_RainbowBlink();
    Remote_Control_Update();
    JointUsb_Process();
    Robot_Fallen_Update();
    Robot_Fault_Update();
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
}
