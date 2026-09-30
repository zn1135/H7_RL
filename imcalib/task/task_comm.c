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
#include "task.h"
#include "../Telemetry/s2r_telemetry.h"
#include "../Telemetry/vofa_trace.h"
#include "../Telemetry/joint_usb.h"
#include "../Telemetry/host_policy_usb.h"
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
    if (HostPolicy_ModeLock())
    {
        enable_request = HostPolicy_EnableAllowed();
    }
    else if (JointUsb_ModeLock())
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

/* 普通 VOFA: 25 维观测与状态。 */
static void Robot_Control_Send_Vofa(void)
{
    static float dbg[VOFA_MAX_CH];
    static uint8_t send_div;
    static uint32_t send_seq;

    if (++send_div < 20u)
    {
        return;
    }
    send_div = 0u;

    taskENTER_CRITICAL();
    memcpy(dbg, rl_control.observation.obs, RL_OBS_SIZE * sizeof(float));
    dbg[25] = (float)rl_control.observation.valid;
    dbg[26] = (float)rl_control.observation.history_ready;
    dbg[27] = (float)robot_state.motor_enabled;
    dbg[28] = (float)output_task_rl_engaged();
    dbg[29] = (float)ctrl_fault;
    dbg[30] = (float)HostPolicy_ModeLock();
    taskEXIT_CRITICAL();
    dbg[31] = (float)((++send_seq) & 0x00FFFFFFu);
    (void)Vofa_Send(dbg, VOFA_MAX_CH);
}

/* 通信单周期 */
void comm_task_body(void)
{
    uint8_t s2r_active;

    Dm_Parse();
    Dji_Parse();
    Motor_State_Update();
    Leg_State_Update();
    WS2812_RainbowBlink();
    Remote_Control_Update();
    JointUsb_Process();
    HostPolicy_Process();
    Robot_Fallen_Update();
    Robot_Fault_Update();
    Robot_Enable_Update();
    HostPolicy_Pump();
    if (!HostPolicy_ModeLock())
    {
        JointUsb_Pump();
    }
    /* 同口互斥：S2R > 策略 VOFA > 普通 VOFA；切换条件见 md/vofa_policy_trace.md。 */
    s2r_active = S2R_Pump();
    if (s2r_active)
    {
        Vofa_Trace_Discard();
    }
    else
    {
        if (Vofa_Transport_Update((uint8_t)(!robot_state.motor_enabled
            && !JointUsb_ModeLock() && !JointUsb_StreamRequested()
            && !HostPolicy_ModeLock())))
        {
            Vofa_Trace_Discard();
        }
        if (!Vofa_Trace_Pump())
        {
            Robot_Control_Send_Vofa();
        }
    }
}
