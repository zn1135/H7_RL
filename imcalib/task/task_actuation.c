#include "robot_control.h"
#include "machine_config.h"
#include "dm.h"
#include "dji.h"
#include "tim.h"
#include "mono_ns.h"
#include "joint_usb.h"

/*
 * 输出任务三层结构 (作者 2026-09-22 定: 解算只算, 分发唯一):
 *   1 估计层  LQR_State_Update()            每拍必算, 不看挡位
 *   2 求解层  solve_*()                      按策略算 torque_output_t, 只写 torque 不碰驱动
 *   3 分发层  output_dispatch()              全文件唯一的 Dm_Send / Dji_Send 调用点
 * 改输出行为 (总开关 / 限幅 / 斜坡 / 极性) 只动 output_dispatch(); 改控制律只动 solve_*()
 */

static uint8_t lqr_running;     /* 已投入 */
static uint8_t rl_engaged;      /* RL 已投入 */
volatile float rl_output_dm_cmd_nm[DM_MOTOR_NUM];
volatile float rl_output_wheel_cmd_nm[DJI_MOTOR_NUM];
volatile leg_response_test_t leg_response_test = {0u, 0u, 0u, 0u};
volatile rl_output_diag_t rl_output_diag;
volatile uint8_t vofa_leg_response_side = 0u;

/* 失能锁存测试配置 */
static void output_leg_test_update(void)
{
    uint8_t requested;
    uint8_t side;

    requested = leg_response_test.requested;
    side = vofa_leg_response_side;
    if (!robot_state.motor_enabled)
    {
        leg_response_test.active = (uint8_t)(requested == 1u);
        leg_response_test.side = side;
        leg_response_test.inhibited = (uint8_t)(requested > 1u || side > 1u);
    }
    else if (requested != leg_response_test.active
        || ((leg_response_test.active || requested)
            && side != leg_response_test.side))
    {
        leg_response_test.inhibited = 1u;
    }
}

/* 遥测跟随锁存侧 */
uint8_t output_leg_test_side(void)
{
    return leg_response_test.side;
}

/* 记录本拍关节目标 */
static void output_rl_diag_update(const torque_output_t *torque, uint64_t queue_ns)
{
    rl_output_diag.valid = 0u;
    if (ctrl_strategy != CTRL_STRATEGY_RL || !torque->valid)
    {
        return;
    }
    rl_output_diag.joint_target[0] = rl_control.torque_state.pos_target[0];
    rl_output_diag.joint_target[1] = rl_control.torque_state.pos_target[1];
    rl_output_diag.joint_target[2] = rl_control.torque_state.pos_target[3];
    rl_output_diag.joint_target[3] = rl_control.torque_state.pos_target[4];
    rl_output_diag.updated_ns = queue_ns;
    rl_output_diag.valid = 1u;
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
    const leg_map_t *map;
    uint8_t i;

    applied = *torque;
    if (!JointUsb_ModeLock())
    {
        if (leg_response_test.inhibited
            || (leg_response_test.active && ctrl_strategy != CTRL_STRATEGY_RL))
        {
            applied.valid = 0u;
        }
        else if (leg_response_test.active)
        {
            map = leg_response_test.side == 0u ? &leg_map_l : &leg_map_r;
            if (leg_response_test.side > 1u || !map->configured
                || map->dm_front < 0 || map->dm_front >= DM_MOTOR_NUM
                || map->dm_rear < 0 || map->dm_rear >= DM_MOTOR_NUM
                || map->dm_front == map->dm_rear)
            {
                applied.valid = 0u;
            }
            else
            {
                for (i = 0u; i < DM_MOTOR_NUM; i++)
                {
                    if (i != (uint8_t)map->dm_front && i != (uint8_t)map->dm_rear)
                    {
                        applied.dm[i] = 0.0f;
                    }
                }
                for (i = 0u; i < DJI_MOTOR_NUM; i++)
                {
                    applied.dji[i] = 0.0f;
                }
            }
        }
    }
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
    if (cmd->s1 == DR16_SW_MID)
    {
        return machine->lqr_configured ? 1u : 0u;
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
/* LQR 平衡: 目标 → 状态反馈 → 腿部力控 */
static void solve_lqr(torque_output_t *torque)
{
    (void)LQR_Target_Update(&lqr_state, &rc_command, CTRL_DT);
    if (!lqr_state.valid)
    {
        return;
    }
    LQR_Control_Update(&lqr_state);
    torque->valid = Leg_Balance_Compute(&leg_balance, &lqr_state, &leg_l, &leg_r,
                                        CTRL_DT, torque);
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

    output_leg_test_update();
    wheel_vel[0] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    wheel_vel[1] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];

    /* 1 估计: 每拍必算 (同 RL 观测) */
    if (machine->lqr_configured)
    {
        (void)LQR_State_Update(&lqr_state, &imu_state, &leg_l, &leg_r, wheel_vel, CTRL_DT);
    }

    /* 2 求解: torque 默认全零 valid=0, 只有走通的分支才置 valid */
    Torque_Output_Clear(&torque);
    strategy = strategy_from_remote(&rc_command);
    ctrl_strategy = strategy;
    rl_engaged = 0u;

    switch (strategy)
    {
    case CTRL_STRATEGY_LQR:
    {
        if (rc_command.s2 == DR16_SW_MID && lqr_engage_update())
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
        rl_engaged = (uint8_t)(rc_command.s2 == DR16_SW_MID && robot_state.motor_enabled);
        if (rl_engaged)
        {
            solve_rl(wheel_vel, &torque);
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
    {
        uint64_t queue_ns = Mono_Ns_Get();

        output_dispatch(&torque);
        output_rl_diag_update(&torque, queue_ns);
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
}
