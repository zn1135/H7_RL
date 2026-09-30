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
#include "../Telemetry/s2r_telemetry.h"
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

/*
 * 普通 VOFA：单侧腿诊断，角度均为固件坐标。
 * ch0 时间ms，ch1 侧，ch2~4 观测指令vx/yaw/height；
 * ch5~8 大腿目标/实际、小腿目标/实际；ch9 腿长，ch10 摆角；
 * ch11~14 前/后髋下发力矩、前/后髋反馈力矩；ch15 有效位。
 * ch16~19 前/后髋位置、前/后髋速度；ch20~21 前/后髋雅可比；
 * ch22~23 前/后反馈年龄ms，无可信时间戳为-1。
 */
static void Robot_Control_Send_Vofa(void)
{
    static float dbg[24];
    static uint8_t send_div;
    const leg_map_t *map;
    const leg_state_t *leg;
    uint8_t side;
    uint8_t front;
    uint8_t rear;
    uint8_t i;
    uint8_t command_valid;
    uint16_t valid = 0u;
    uint64_t sample_ns;
    float command[3] = {0.0f, 0.0f, 0.0f};
    float scale;

    /* 二分频 */
    if (++send_div < 2u)
    {
        return;
    }
    send_div = 0u;

    memset(dbg, 0, sizeof(dbg));
    /* 一致快照 */
    vTaskSuspendAll();
    side = output_leg_test_side();
    map = side == 0u ? &leg_map_l : &leg_map_r;
    leg = side == 0u ? &leg_l : &leg_r;
    sample_ns = Mono_Ns_Get();
    dbg[0] = (float)(sample_ns / 1000000u);
    dbg[1] = (float)side;
    dbg[22] = -1.0f;
    dbg[23] = -1.0f;
    valid |= output_debug_dm_sent ? 0x020u : 0u;
    valid |= robot_state.motor_enabled ? 0x040u : 0u;
    valid |= output_task_rl_engaged() ? 0x080u : 0u;
    valid |= torque_output_enabled ? 0x100u : 0u;
    valid |= leg_response_test.active ? 0x200u : 0u;
    valid |= leg_response_test.inhibited ? 0x400u : 0u;
    valid |= ctrl_fault != 0u ? 0x800u : 0u;

    /* 观测指令 */
    command_valid = (uint8_t)(rl_control.observation.valid && rl_control.param.configured);
    if (command_valid)
    {
        for (i = 0u; i < 3u; i++)
        {
            scale = rl_control.param.command_scale[i];
            if (!isfinite(scale) || scale == 0.0f
                || !isfinite(rl_control.observation.obs[RL_OBS_CMD_VX + i]))
            {
                command_valid = 0u;
                break;
            }
            command[i] = rl_control.observation.obs[RL_OBS_CMD_VX + i] / scale;
            if (!isfinite(command[i]))
            {
                command_valid = 0u;
                break;
            }
        }
    }
    if (command_valid)
    {
        for (i = 0u; i < 3u; i++)
        {
            dbg[2u + i] = command[i];
        }
        valid |= 0x010u;
    }

    if (side <= 1u && map->configured && map->dm_front >= 0 && map->dm_rear >= 0
        && map->dm_front < DM_MOTOR_NUM && map->dm_rear < DM_MOTOR_NUM
        && map->dm_front != map->dm_rear)
    {
        front = (uint8_t)map->dm_front;
        rear = (uint8_t)map->dm_rear;
        dbg[11] = isfinite(rl_output_dm_cmd_nm[front]) ? rl_output_dm_cmd_nm[front] : 0.0f;
        dbg[12] = isfinite(rl_output_dm_cmd_nm[rear]) ? rl_output_dm_cmd_nm[rear] : 0.0f;
        if (motor_state.dm.parsed_rx_ns[front] != 0u
            && motor_state.dm.parsed_rx_ns[front] <= sample_ns)
        {
            dbg[22] = (float)(sample_ns - motor_state.dm.parsed_rx_ns[front]) * 1.0e-6f;
        }
        if (motor_state.dm.parsed_rx_ns[rear] != 0u
            && motor_state.dm.parsed_rx_ns[rear] <= sample_ns)
        {
            dbg[23] = (float)(sample_ns - motor_state.dm.parsed_rx_ns[rear]) * 1.0e-6f;
        }
        if (motor_state.dm.online[front] && !Dm_Has_Fault(front)
            && isfinite(motor_state.dm.pos_zero_rad[front])
            && isfinite(motor_state.dm.trq_nm[front])
            && isfinite(motor_state.dm.vel_rad_s[front])
            && motor_state.dm.parsed_rx_ns[front] != 0u
            && motor_state.dm.parsed_rx_ns[front] <= sample_ns
            && sample_ns - motor_state.dm.parsed_rx_ns[front] <= 10000000u)
        {
            dbg[13] = motor_state.dm.trq_nm[front];
            dbg[16] = motor_state.dm.pos_zero_rad[front];
            dbg[18] = motor_state.dm.vel_rad_s[front];
            valid |= 0x001u;
        }
        if (motor_state.dm.online[rear] && !Dm_Has_Fault(rear)
            && isfinite(motor_state.dm.pos_zero_rad[rear])
            && isfinite(motor_state.dm.trq_nm[rear])
            && isfinite(motor_state.dm.vel_rad_s[rear])
            && motor_state.dm.parsed_rx_ns[rear] != 0u
            && motor_state.dm.parsed_rx_ns[rear] <= sample_ns
            && sample_ns - motor_state.dm.parsed_rx_ns[rear] <= 10000000u)
        {
            dbg[14] = motor_state.dm.trq_nm[rear];
            dbg[17] = motor_state.dm.pos_zero_rad[rear];
            dbg[19] = motor_state.dm.vel_rad_s[rear];
            valid |= 0x002u;
        }

        if ((valid & 0x003u) == 0x003u && leg->output.valid
            && isfinite(leg->output.thigh_angle)
            && isfinite(leg->output.virtual_shank_angle)
            && isfinite(leg->output.virtual_leg_length)
            && isfinite(leg->output.virtual_leg_angle)
            && isfinite(leg->output.vshank_jac[0])
            && isfinite(leg->output.vshank_jac[1]))
        {
            dbg[6] = leg->output.thigh_angle;
            dbg[8] = leg->output.virtual_shank_angle;
            dbg[9] = leg->output.virtual_leg_length;
            dbg[10] = leg->output.virtual_leg_angle;
            dbg[20] = leg->output.vshank_jac[1];
            dbg[21] = leg->output.vshank_jac[0];
            valid |= 0x004u;
        }

        /* 真实目标 */
        if (rl_output_diag.valid && rl_output_diag.updated_ns <= sample_ns
            && sample_ns - rl_output_diag.updated_ns <= 10000000u
            && isfinite(rl_output_diag.joint_target[side * 2u])
            && isfinite(rl_output_diag.joint_target[side * 2u + 1u]))
        {
            dbg[5] = rl_output_diag.joint_target[side * 2u];
            dbg[7] = rl_output_diag.joint_target[side * 2u + 1u];
            valid |= 0x008u;
        }
    }
    dbg[15] = (float)valid;
    (void)xTaskResumeAll();
    (void)Vofa_Send(dbg, 24u);
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
    Robot_Fallen_Update();
    Robot_Fault_Update();
    Robot_Enable_Update();
    JointUsb_Pump();
    /* 同口互斥：S2R > 策略 VOFA > 普通 VOFA；普通布局现为单侧腿响应。 */
    s2r_active = S2R_Pump();
    if (s2r_active)
    {
        Vofa_Trace_Discard();
    }
    else
    {
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
}
