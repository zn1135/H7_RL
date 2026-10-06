#include "robot_control.h"
#include "machine_config.h"
#include "dm.h"
#include "dji.h"
#include "tim.h"
#include "mono_ns.h"
#include "joint_usb.h"
#include "gas_spring.h"
#include "standup.h"

/*
 * 输出任务三层结构 (作者 2026-09-22 定: 解算只算, 分发唯一):
 *   1 估计层  LQR_State_Update()            每拍必算, 不看挡位
 *   2 求解层  solve_*()                      按策略算 torque_output_t, 只写 torque 不碰驱动
 *   3 分发层  output_dispatch()              全文件唯一的 Dm_Send / Dji_Send 调用点
 * 改输出行为 (总开关 / 限幅 / 斜坡 / 极性) 只动 output_dispatch(); 改控制律只动 solve_*()
 */

static uint8_t lqr_running;     /* 已投入 */
static uint8_t rl_engaged;      /* RL 已投入 */
static uint8_t rl_wait_ticks;   /* 执行分频 */
volatile float rl_output_dm_cmd_nm[DM_MOTOR_NUM];
volatile float rl_output_wheel_cmd_nm[DJI_MOTOR_NUM];

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
    uint8_t i;

    applied = *torque;
    if (!applied.valid || !torque_output_enabled)
    {
        for (i = 0u; i < DM_MOTOR_NUM; i++)
        {
            rl_output_dm_cmd_nm[i] = 0.0f;
        }
        for (i = 0u; i < DJI_MOTOR_NUM; i++)
        {
            rl_output_wheel_cmd_nm[i] = 0.0f;
        }
        output_debug_dm_sent = (uint8_t)(Dm_Send_Zero() == HAL_OK);
        output_debug_dji_sent = (uint8_t)(Dji_All_Stop() == HAL_OK);
        return;
    }
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        rl_output_dm_cmd_nm[i] = applied.dm[i];
    }
    rl_output_wheel_cmd_nm[DJI_MOTOR_WHEEL_LFT] = applied.dji[DJI_MOTOR_WHEEL_LFT];
    rl_output_wheel_cmd_nm[DJI_MOTOR_WHEEL_RGT] = applied.dji[DJI_MOTOR_WHEEL_RGT];

    output_debug_dm_sent = (uint8_t)(Dm_Send_Torque(applied.dm) == HAL_OK);
    output_debug_dji_sent = (uint8_t)(Dji_Send_Wheel_Torque(
        applied.dji[DJI_MOTOR_WHEEL_LFT], applied.dji[DJI_MOTOR_WHEEL_RGT]) == HAL_OK);
        // (void)Dm_Send_Zero();
        // (void)Dji_All_Stop();
}

/* ================= 模式与投入 ================= */
/* 遥控使能判定 (全机唯一): online + 左中(需 LQR 表) / 左上(需 RL 表) */
uint8_t strategy_rc_enable(const rc_command_t *cmd)
{
    if (cmd == NULL || !cmd->online)
    {
        return 0u; 
    }
    if (gas_spring_only_enabled)
    {
        return (uint8_t)(GAS_SPRING_COMP_ENABLE
            && machine->spring != NULL
            && cmd->s1 == DR16_SW_UP);
    }
    if (cmd->s1 == DR16_SW_MID)
    {
        return LQR_Ready();
    }
    if (cmd->s1 == DR16_SW_UP)
    {
        return machine->rl.configured ? 1u : 0u;
    }
    return 0u;
}

/* 左拨杆选模式: 先看 rc_enable(唯一判定), 再按挡位给策略 */
static ctrl_strategy_t strategy_from_remote(const rc_command_t *cmd)
{
    if (gas_spring_only_enabled)
    {
        return strategy_rc_enable(cmd) && robot_state.rc_enable && !JointUsb_ModeLock()
            ? CTRL_STRATEGY_GAS_SPRING : CTRL_STRATEGY_DISABLE;
    }
    if (JointUsb_ModeLock())
    {
        return JointUsb_PhysicalPermit()
            ? CTRL_STRATEGY_JOINT_USB : CTRL_STRATEGY_DISABLE;
    }
    if (!robot_state.rc_enable)
    {
        return CTRL_STRATEGY_DISABLE;
    }
    if (cmd->s1 == DR16_SW_MID)
    {
        return CTRL_STRATEGY_LQR;
    }
    if (cmd->s1 == DR16_SW_UP)
    {
        return CTRL_STRATEGY_RL;
    }
    return CTRL_STRATEGY_DISABLE;
}

/* LQR 投入锁存 */
static uint8_t lqr_engage_update(void)
{
    uint8_t ready;

    ready = (uint8_t)(robot_state.motor_enabled
                      && ctrl_fault == FAULT_NONE && !robot_state.fallen
                      && robot_state.rc_enable && rc_command.online
                      && imu_state.online
                      && leg_l.output.valid && leg_r.output.valid);
    if (!ready)
    {
        lqr_running = 0u;
    }
    else if (!lqr_running)
    {
        /* 使能沿: 锁腿长目标 (不查实测腿长, 同 Leg2) */
        lqr_running = LQR_Enable_Latch(&lqr_state, &leg_l, &leg_r);
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
/* 仅弹簧补偿 */
static void solve_gas_spring(torque_output_t *torque)
{
    const float base_dm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float raw_dm[4];
    float limit;
    uint8_t i;

    if (!(robot_state.rc_enable && robot_state.motor_enabled
          && rc_command.online && rc_command.s1 == DR16_SW_UP
          && rc_command.s2 == DR16_SW_MID)
        || !Gas_Spring_Apply(&leg_l, &leg_r, base_dm, raw_dm))
    {
        return;
    }
    limit = machine->dm_trq_clamp;
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        torque->dm[i] = clampf(raw_dm[i], -limit, limit);
    }
    torque->valid = 1u;
}

/* LQR 平衡: 目标 → 状态反馈 → 腿部力控 */
static void solve_lqr(torque_output_t *torque)
{
    if (!LQR_Target_Update(&lqr_state, &rc_command, MACHINE_LQR_DT))
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
    torque->valid = Leg_Balance_Compute(&leg_balance, &lqr_state, &leg_l, &leg_r,
                                        MACHINE_LQR_DT, torque);
}

/* 独立自起选路 */
static void solve_lqr_standup(torque_output_t *torque)
{
    uint8_t permit;
    uint8_t allow_restart;
    uint8_t route;
    uint8_t was_done;

    permit = (uint8_t)(robot_state.motor_enabled && robot_state.rc_enable
        && rc_command.online && ctrl_fault == FAULT_NONE && !robot_state.fallen
        && imu_state.online && torque_output_enabled);
    allow_restart = (uint8_t)(rc_command.vel == 0.0f && rc_command.yaw == 0.0f
        && rc_command.len == 0.0f);
    was_done = (uint8_t)(standup_control.phase == STANDUP_DONE);
    if (was_done && permit && lqr_engage_update())
    {
        solve_lqr(torque);
    }
    route = Standup_Update(&standup_control, &imu_state, &leg_l, &leg_r,
        permit, allow_restart, MACHINE_LQR_DT, torque);
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
    }
}

/* RL: 动作 → 力矩; 前提: 遥控使能 + 电机使能 + 两腿有效 + 推理动作可用 */
static void solve_rl(const float wheel_vel[2], torque_output_t *torque)
{
    if (!(robot_state.rc_enable && robot_state.motor_enabled
          && leg_l.output.valid && leg_r.output.valid
          && action_state.rl_ready))
    {
        return;
    }
    if (RL_Torque_Compute(&leg_l, &leg_r,
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

    wheel_vel[0] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    wheel_vel[1] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];

    /* 1 估计: 每拍必算 (同 RL 观测) */
    if (LQR_Gain_Compatible())
    {
        (void)LQR_State_Update(&lqr_state, &imu_state, &leg_l, &leg_r, wheel_vel, MACHINE_LQR_DT);
    }

    /* 2 求解: torque 默认全零 valid=0, 只有走通的分支才置 valid */
    Torque_Output_Clear(&torque);
    strategy = strategy_from_remote(&rc_command);
    ctrl_strategy = strategy;
    if (!standup_control.enabled || gas_spring_only_enabled || JointUsb_ModeLock()
        || (rc_command.online && (rc_command.s1 != DR16_SW_MID
            || rc_command.s2 != DR16_SW_MID)))
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
    case CTRL_STRATEGY_GAS_SPRING:
        lqr_idle();
        solve_gas_spring(&torque);
        break;

    case CTRL_STRATEGY_LQR:
    {
        if (standup_control.enabled && rc_command.s2 == DR16_SW_MID)
        {
            solve_lqr_standup(&torque);
        }
        else if (rc_command.s2 == DR16_SW_MID && lqr_engage_update())
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
        rl_engaged = (uint8_t)(rc_command.s2 == DR16_SW_MID && robot_state.motor_enabled
                               && robot_state.rc_enable && rc_command.online
                               && ctrl_fault == FAULT_NONE && !robot_state.fallen);
        if (rl_engaged && action_state.rl_ready
            && leg_l.output.valid && leg_r.output.valid && torque_output_enabled)
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
