#include "robot_control.h"
#include "machine_config.h"
#include "dm.h"
#include "dji.h"
#include "tim.h"
#include "mono_ns.h"
#include "joint_usb.h"
#include "gas_spring.h"
#include "standup.h"
#include "slip.h"
#include <math.h>
#include <string.h>

/*
 * 输出任务三层结构 (作者 2026-09-22 定: 解算只算, 分发唯一):
 *   1 估计层  LQR_State_Update()            每拍必算, 不看挡位
 *   2 求解层  solve_*()                      按策略算 torque_output_t, 只写 torque 不碰驱动
 *   3 分发层  output_dispatch()              全文件唯一的 Dm_Send / Dji_Send 调用点
 * 改输出行为 (总开关 / 限幅 / 斜坡 / 极性) 只动 output_dispatch(); 改控制律只动 solve_*()
 */

static slip_state_t slip_control;
static uint8_t lqr_running;     /* 已投入 */
static uint8_t rl_engaged;      /* RL 已投入 */
static uint8_t rl_wait_ticks;   /* 执行分频 */
typedef struct {
    imu_state_t imu;
    leg_state_t left;
    leg_state_t right;
    rc_command_t rc;
    rc_control_mode_t mode;
    float wheel_vel[2];
    uint8_t drive;
    uint8_t normal;
    uint8_t recovery;
} control_frame_t;

static control_frame_t control_frame;
static compensation_debug_t pending_compensation;
volatile float rl_output_dm_cmd_nm[DM_MOTOR_NUM];
volatile float rl_output_wheel_cmd_nm[DJI_MOTOR_NUM];

/* ================= 拨杆模式: 唯一解码 ================= */
rc_control_mode_t Control_Mode_Decode(const rc_command_t *cmd)
{
    rc_control_mode_t mode;

    memset(&mode, 0, sizeof(mode));
    mode.strategy = CTRL_STRATEGY_DISABLE;
    if (cmd == NULL)
    {
        return mode;
    }
    mode.usb_reset = (uint8_t)(cmd->s1 == DR16_SW_DOWN);
    if (!cmd->online)
    {
        return mode;
    }
    switch (cmd->s1)
    {
    case DR16_SW_MID:
        if (cmd->s2 == DR16_SW_UP)
        {
            mode.rc_enable = (uint8_t)(GAS_SPRING_COMP_ENABLE
                && machine->spring != NULL && machine->gravity != NULL);
            mode.strategy = mode.rc_enable
                ? CTRL_STRATEGY_COMPENSATION : CTRL_STRATEGY_DISABLE;
            mode.engage = 1u;
        }
        else
        {
            mode.strategy = CTRL_STRATEGY_LQR;
            mode.rc_enable = LQR_Ready();
            mode.engage = (uint8_t)(cmd->s2 == DR16_SW_MID);
        }
        break;

    case DR16_SW_UP:
        mode.strategy = CTRL_STRATEGY_RL;
        mode.rc_enable = machine->rl.configured ? 1u : 0u;
        mode.engage = (uint8_t)(cmd->s2 == DR16_SW_MID);
        mode.usb_permit = (uint8_t)(cmd->s2 == DR16_SW_DOWN);
        break;

    case DR16_SW_DOWN:
    default:
        break;
    }
    return mode;
}

uint8_t strategy_rc_enable(const rc_command_t *cmd)
{
    return Control_Mode_Decode(cmd).rc_enable;
}

/* USB锁优先 */
static ctrl_strategy_t strategy_from_mode(const rc_control_mode_t *mode)
{
    if (JointUsb_ModeLock())
    {
        return mode->usb_permit
            ? CTRL_STRATEGY_JOINT_USB : CTRL_STRATEGY_DISABLE;
    }
    if (!robot_state.rc_enable)
    {
        return CTRL_STRATEGY_DISABLE;
    }
    return mode->strategy;
}

/* 统一使能许可 */
uint8_t Robot_Control_Enable_Allowed(void)
{
    uint8_t recovery;
    rc_control_mode_t mode;

    if (JointUsb_ModeLock())
    {
        return JointUsb_EnableAllowed();
    }
    mode = Control_Mode_Decode(&rc_command);
    recovery = (uint8_t)(standup_control.enabled && standup_control.recovery_enabled
        && mode.strategy == CTRL_STRATEGY_LQR && mode.engage);
    return (uint8_t)(robot_state.rc_enable && rc_command.online
        && ctrl_fault == FAULT_NONE && (!robot_state.fallen || recovery));
}

/* 本拍快照与许可 */
static void Control_Frame_Read(void)
{
    taskENTER_CRITICAL();
    control_frame.imu = imu_state;
    control_frame.left = leg_l;
    control_frame.right = leg_r;
    control_frame.rc = rc_command;
    control_frame.mode = Control_Mode_Decode(&control_frame.rc);
    control_frame.wheel_vel[0] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    control_frame.wheel_vel[1] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
    control_frame.drive = (uint8_t)(Robot_Control_Enable_Allowed()
        && robot_state.motor_enabled && torque_output_enabled);
    control_frame.normal = (uint8_t)(control_frame.drive && !robot_state.fallen);
    control_frame.recovery = (uint8_t)(control_frame.drive && standup_control.enabled
        && control_frame.mode.strategy == CTRL_STRATEGY_LQR && control_frame.mode.engage);
    taskEXIT_CRITICAL();
}

/* 输出初始化 */
void output_task_init(void)
{
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
}

/* LQR 是否已投入 */
uint8_t output_task_lqr_engaged(void)
{
    return lqr_running;
}

/* RL 是否已投入: 左上 + 右中 + 电机使能 (供策略任务预热计时与 VOFA) */
uint8_t output_task_rl_engaged(void)
{
    return rl_engaged;
}

/* ================= 3 分发层: 唯一下发点 ================= */
/* valid=0 或总输出关 → 零力矩; 否则原样下发 (各路限幅已在控制器内做, 极性在驱动边界做) */
static void output_dispatch(const torque_output_t *torque)
{
    torque_output_t applied;
    compensation_debug_t debug;
    uint8_t i;

    applied = *torque;
    debug = pending_compensation;
    debug.bench = (uint8_t)(ctrl_strategy == CTRL_STRATEGY_COMPENSATION);
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (!isfinite(applied.dm[i]))
        {
            applied.valid = 0u;
        }
    }
    for (i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        if (!isfinite(applied.dji[i]))
        {
            applied.valid = 0u;
        }
    }
    if (!applied.valid || !control_frame.drive || ctrl_fault != FAULT_NONE
        || !torque_output_enabled)
    {
        memset(&debug, 0, sizeof(debug));
        debug.bench = (uint8_t)(ctrl_strategy == CTRL_STRATEGY_COMPENSATION);
        taskENTER_CRITICAL();
        compensation_debug = debug;
        for (i = 0u; i < DM_MOTOR_NUM; i++)
        {
            rl_output_dm_cmd_nm[i] = 0.0f;
        }
        for (i = 0u; i < DJI_MOTOR_NUM; i++)
        {
            rl_output_wheel_cmd_nm[i] = 0.0f;
        }
        taskEXIT_CRITICAL();
        output_debug_dm_sent = (uint8_t)(Dm_Send_Zero() == HAL_OK);
        output_debug_dji_sent = (uint8_t)(Dji_All_Stop() == HAL_OK);
        return;
    }
    taskENTER_CRITICAL();
    compensation_debug = debug;
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        rl_output_dm_cmd_nm[i] = applied.dm[i];
    }
    rl_output_wheel_cmd_nm[DJI_MOTOR_WHEEL_LFT] = applied.dji[DJI_MOTOR_WHEEL_LFT];
    rl_output_wheel_cmd_nm[DJI_MOTOR_WHEEL_RGT] = applied.dji[DJI_MOTOR_WHEEL_RGT];
    taskEXIT_CRITICAL();

    output_debug_dm_sent = (uint8_t)(Dm_Send_Torque(applied.dm) == HAL_OK);
    output_debug_dji_sent = (uint8_t)(Dji_Send_Wheel_Torque(
        applied.dji[DJI_MOTOR_WHEEL_LFT], applied.dji[DJI_MOTOR_WHEEL_RGT]) == HAL_OK);
        // (void)Dm_Send_Zero();
        // (void)Dji_All_Stop();
}

/* 组装观测、速度补偿、位置积分 */
static void Control_State_Update(uint8_t reset_velocity)
{
    if (!LQR_State_Update(&lqr_state, &control_frame.imu,
        &control_frame.left, &control_frame.right,
        control_frame.wheel_vel, MACHINE_LQR_DT))
    {
        Slip_Reset(&slip_control);
        return;
    }
    if (reset_velocity || !slip_control.initialized)
    {
        Slip_Init(&slip_control, lqr_state.ds_raw, lqr_state.a_fwd);
    }
    else
    {
        (void)Slip_Update(&slip_control, lqr_state.ds_raw,
            lqr_state.a_fwd, MACHINE_LQR_DT);
    }
    if (!slip_control.valid)
    {
        lqr_state.valid = 0u;
        Slip_Reset(&slip_control);
        return;
    }
    LQR_Velocity_Apply(&lqr_state, slip_control.velocity, MACHINE_LQR_DT);
}

/* LQR 投入锁存 */
static uint8_t lqr_engage_update(void)
{
    uint8_t ready;
    lqr_debug_t saved_debug;

    ready = (uint8_t)(control_frame.normal && lqr_state.valid);
    if (!ready)
    {
        lqr_running = 0u;
    }
    else if (!lqr_running)
    {
        if (standup_control.recovered)
        {
            saved_debug = lqr_debug;
            LQR_Init(&lqr_state);
            lqr_debug = saved_debug;
            Control_State_Update(1u);
            standup_control.recovered = 0u;
        }
        /* 使能沿: 锁腿长目标 (不查实测腿长, 同 Leg2) */
        lqr_running = LQR_Enable_Latch(&lqr_state, &control_frame.left, &control_frame.right);
        if (lqr_running)
        {
            Leg_Balance_Reset(&leg_balance);
        }
    }
    return lqr_running;
}

/* LQR 退出 / 未投入: 清观测值 */
static void lqr_idle(void)
{
    lqr_running = 0u;
    Leg_Balance_Reset(&leg_balance);
}

/* ================= 2 求解层: 只写 torque, 不下发 ================= */
/* 仅弹簧与重力 */
static void solve_compensation(torque_output_t *torque)
{
    const float base_dm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float raw_dm[4];
    float limit;
    float world_angle[2];
    uint8_t i;

    if (!control_frame.normal || !control_frame.imu.pitch_world_valid
        || !isfinite(control_frame.imu.pitch_world)
        || !Gas_Spring_Apply(&control_frame.left, &control_frame.right, base_dm, raw_dm))
    {
        return;
    }
    for (i = 0u; i < 4u; i++)
    {
        pending_compensation.spring_dm[i] = raw_dm[i];
    }
    world_angle[0] = remainderf(control_frame.left.output.virtual_leg_angle
        - control_frame.imu.pitch_world, LEG_2PI);
    world_angle[1] = remainderf(control_frame.right.output.virtual_leg_angle
        - control_frame.imu.pitch_world, LEG_2PI);
    if (!Gravity_Comp_Apply(&control_frame.left, &control_frame.right,
        world_angle, raw_dm, raw_dm, &pending_compensation.gravity))
    {
        memset(&pending_compensation, 0, sizeof(pending_compensation));
        return;
    }
    memcpy(pending_compensation.raw_dm, raw_dm, sizeof(raw_dm));
    limit = machine->dm_trq_clamp;
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        torque->dm[i] = clampf(raw_dm[i], -limit, limit);
        pending_compensation.saturated |= (uint8_t)(torque->dm[i] != raw_dm[i]);
    }
    torque->valid = 1u;
}

/* LQR 平衡: 目标 → 状态反馈 → 腿部力控 */
static void solve_lqr(torque_output_t *torque)
{
    if (!LQR_Target_Update(&lqr_state, &control_frame.rc, MACHINE_LQR_DT))
    {
        return;
    }
    if (!lqr_state.valid)
    {
        return;
    }
    LQR_Control_Update(&lqr_state);
    if (!lqr_state.gain_valid)
    {
        return;
    }
    torque->valid = Leg_Balance_Compute(&leg_balance, &lqr_state, &control_frame.left, &control_frame.right,
                                        MACHINE_LQR_DT, torque);
    if (torque->valid)
    {
        pending_compensation = leg_balance.compensation;
    }
}

/* 独立自起选路 */
static void solve_lqr_standup(torque_output_t *torque)
{
    uint8_t permit;
    uint8_t route;
    uint8_t was_done;

    permit = control_frame.recovery;
    was_done = (uint8_t)(standup_control.phase == STANDUP_DONE);
    if (was_done && permit && lqr_engage_update())
    {
        solve_lqr(torque);
    }
    route = Standup_Update(&standup_control, &control_frame.imu, &control_frame.left, &control_frame.right,
        permit, 1u, MACHINE_LQR_DT, torque);
    if (route == STANDUP_ROUTE_BALANCE)
    {
        if (!was_done)
        {
            if (lqr_engage_update())
            {
                solve_lqr(torque);
            }
            else
            {
                Standup_Fail(&standup_control, STANDUP_GATED);
            }
        }
        if (!torque->valid)
        {
            Standup_Fail(&standup_control, STANDUP_BAD_INPUT);
            Torque_Output_Clear(torque);
            lqr_idle();
        }
    }
    else
    {
        lqr_idle();
        if (route == STANDUP_ROUTE_PREPARE && torque->valid)
        {
            pending_compensation = standup_control.compensation;
        }
    }
}

/* RL: 动作 → 力矩; 前提: 遥控使能 + 电机使能 + 两腿有效 + 推理动作可用 */
static void solve_rl(const float wheel_vel[2], torque_output_t *torque)
{
    if (RL_Torque_Compute(&control_frame.left, &control_frame.right,
        &rl_control.torque_param[rl_control.policy.selected_model],
        wheel_vel, action_state.a, &rl_control.torque_state, torque) == 0u)
    {
        /* 失败: 零力矩 */
        torque->valid = 0u;
        return;
    }
    torque->valid = 1u;
}

/* ================= 主体: 估计 → 求解 → 分发 ================= */
void output_task_body(void)
{
    float wheel_vel[2];
    ctrl_strategy_t strategy;
    torque_output_t torque;
    torque_output_t sent_torque;
    uint8_t i;
    uint8_t dispatch;
#if CONTROL_TIME_VOFA_ENABLE
    static uint32_t previous_start_us;
    static uint32_t previous_output_us;
    uint32_t start_us;
    uint32_t period_us;
    uint32_t run_us;

    start_us = (uint32_t)(Mono_Ns_Get() / 1000u);
    period_us = control_time_debug.sequence != 0u
        ? start_us - previous_start_us : 0u;
    previous_start_us = start_us;
#endif

    Control_Frame_Read();
    wheel_vel[0] = control_frame.wheel_vel[0];
    wheel_vel[1] = control_frame.wheel_vel[1];

    /* 1 估计与融合 */
    if (LQR_Gain_Compatible())
    {
        Control_State_Update(0u);
    }

    /* 2 求解: torque 默认全零 valid=0, 只有走通的分支才置 valid */
    Torque_Output_Clear(&torque);
    memset(&pending_compensation, 0, sizeof(pending_compensation));
    strategy = strategy_from_mode(&control_frame.mode);
    ctrl_strategy = strategy;
    if (!standup_control.enabled || strategy == CTRL_STRATEGY_COMPENSATION || JointUsb_ModeLock()
        || (control_frame.rc.online && !(control_frame.mode.strategy == CTRL_STRATEGY_LQR
            && control_frame.mode.engage)))
    {
        Standup_Reset(&standup_control);
    }
    else if (strategy != CTRL_STRATEGY_LQR && standup_control.phase != STANDUP_IDLE)
    {
        Standup_Fail(&standup_control, STANDUP_GATED);
    }
    rl_engaged = 0u;
    dispatch = 1u;
    if (strategy != CTRL_STRATEGY_RL)
    {
        rl_wait_ticks = 0u;
    }

    switch (strategy)
    {
    case CTRL_STRATEGY_COMPENSATION:
        lqr_idle();
        solve_compensation(&torque);
        break;

    case CTRL_STRATEGY_LQR:
    {
        if (standup_control.enabled && control_frame.mode.engage)
        {
            solve_lqr_standup(&torque);
        }
        else if (control_frame.mode.engage && lqr_engage_update())
        {
            solve_lqr(&torque);
        }
        else
        {
            lqr_idle();
        }
        break;
    }

    case CTRL_STRATEGY_RL:
        lqr_running = 0u;
        rl_engaged = (uint8_t)(control_frame.normal && control_frame.mode.engage);
        if (rl_engaged && action_state.rl_ready
            && control_frame.left.output.valid && control_frame.right.output.valid)
        {
            if (rl_wait_ticks == 0u)
            {
                solve_rl(wheel_vel, &torque);
                if (torque.valid)
                {
                    rl_wait_ticks = MACHINE_RL_CTRL_DIV - 1u;
                }
            }
            else
            {
                rl_wait_ticks--;
                dispatch = 0u;
            }
        }
        else
        {
            rl_wait_ticks = 0u;
        }
        break;

    case CTRL_STRATEGY_JOINT_USB:
        lqr_idle();
        JointUsb_Compute(&torque);
        break;

    case CTRL_STRATEGY_DISABLE:
    default:
        lqr_idle();
        break;
    }

    /* 3 分发: 唯一出口 */
    if (dispatch)
    {
        uint64_t queue_ns = Mono_Ns_Get();

        output_dispatch(&torque);
#if CONTROL_TIME_VOFA_ENABLE
        control_time_debug.output_period_us = control_time_debug.output_sequence != 0u
            ? start_us - previous_output_us : 0u;
        previous_output_us = start_us;
        control_time_debug.output_sequence++;
#endif
        if (JointUsb_ModeLock() || JointUsb_StreamRequested())
        {
            /* 记录最终命令 */
            Torque_Output_Clear(&sent_torque);
            sent_torque.valid = 1u;
            for (i = 0u; i < DM_MOTOR_NUM; i++)
            {
                sent_torque.dm[i] = rl_output_dm_cmd_nm[i];
            }
            for (i = 0u; i < DJI_MOTOR_NUM; i++)
            {
                sent_torque.dji[i] = rl_output_wheel_cmd_nm[i];
            }
            JointUsb_ActuationTick(&sent_torque, queue_ns, output_debug_dm_sent);
        }
    }
#if CONTROL_TIME_VOFA_ENABLE
    run_us = (uint32_t)(Mono_Ns_Get() / 1000u) - start_us;
    if (control_time_debug.sequence != 0u)
    {
        if (control_time_debug.min_period_us == 0u
            || period_us < control_time_debug.min_period_us)
        {
            control_time_debug.min_period_us = period_us;
        }
        if (period_us > control_time_debug.max_period_us)
        {
            control_time_debug.max_period_us = period_us;
        }
    }
    control_time_debug.period_us = period_us;
    control_time_debug.run_us = run_us;
    control_time_debug.sequence++;
#endif
}
