#include "s2r_source.h"
#include "robot_control.h"
#include "machine_config.h"
#include "hi229.h"
#include "dm.h"
#include "dji.h"

#include <string.h>
#include <math.h>

/* ============================================================================
 * 采样适配层 (gap 测试)
 * 只读现有固件的全局状态 (imu_state / motor_state / leg_l / leg_r / rl_control /
 * action_state / hi229_data ...), 用变化检测推断事件; 不修改任何现有结构体,
 * 也不改动现有任务逻辑。序号与时间戳全部由本模块自行维护。
 * ==========================================================================*/

/* 轮序: 协议/RL 电机槽 [4]=左轮←物理 RGT, [5]=右轮←物理 LFT (同 RL 输入交叉) */
static const uint8_t wheel_source[2] = {DJI_MOTOR_WHEEL_RGT, DJI_MOTOR_WHEEL_LFT};

/* 采样状态: 变化检测基线 + 序号 */
static struct {
    uint32_t state_seq;                     /* 采样拍数 */
    uint32_t imu_seq;                       /* IMU 有效帧数 */
    uint32_t last_imu_ts;                   /* 上次 IMU 传感器时间戳 */
    uint32_t last_imu_tick;                 /* 上次 IMU 接收 tick */
    uint32_t last_run_ok;                   /* 上次推理成功计数 */
    uint32_t last_run_fail;                 /* 上次推理失败计数 */
    uint32_t last_dm_tick[DM_MOTOR_NUM];
    uint32_t last_dji_tick[DJI_MOTOR_NUM];
    uint64_t motor_rx_us[S2R_MOTOR_NUM];
} src;

void S2R_Source_Reset(void)
{
    src.last_run_ok = 0u;
    src.last_run_fail = 0u;
    memset(src.motor_rx_us, 0, sizeof(src.motor_rx_us));
}

/* 电机反馈到达时刻: 接收 tick 变化 → 新帧, 打模块时间戳 */
static void sample_motor_rx(void)
{
    uint8_t i;

    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (dm_motor_feedback[i].last_rx_tick != src.last_dm_tick[i])
        {
            src.last_dm_tick[i] = dm_motor_feedback[i].last_rx_tick;
            if (dm_motor_feedback[i].last_rx_tick != 0u)
            {
                src.motor_rx_us[i] = S2R_Now_Us();
            }
        }
    }
    for (i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        if (dji_motor_feedback[i].last_rx_tick != src.last_dji_tick[i])
        {
            src.last_dji_tick[i] = dji_motor_feedback[i].last_rx_tick;
            if (dji_motor_feedback[i].last_rx_tick != 0u)
            {
                src.motor_rx_us[DM_MOTOR_NUM + i] = S2R_Now_Us();
            }
        }
    }
}

/* IMU: 传感器 ms 时间戳或接收 tick 变化 → 新帧 */
static void sample_imu(void)
{
    if (!hi229_data.online)
    {
        return;
    }
    if (hi229_data.ts == src.last_imu_ts && hi229_data.last_rx_tick == src.last_imu_tick)
    {
        return;
    }
    src.last_imu_ts = hi229_data.ts;
    src.last_imu_tick = hi229_data.last_rx_tick;
    src.imu_seq++;
    S2R_Imu_Record(src.imu_seq, S2R_Now_Us());
}

/* 策略: 推理计数变化 → 一次推理; 观测/历史直接读策略任务留下的状态 */
static void sample_policy(void)
{
    const rl_map_t *map = &machine->rl;
    float published[S2R_ACTION_NUM];
    float command[3];
    uint32_t total;
    uint32_t i;
    uint8_t ok;
    uint8_t engaged;

    engaged = output_task_rl_engaged();
    S2R_Session_Update(engaged);
    if (engaged && !rl_control.observation.valid)
    {
        S2R_History_Reset();
    }
    total = rl_control.policy.run_ok + rl_control.policy.run_fail;
    if (total == (src.last_run_ok + src.last_run_fail))
    {
        return;
    }
    ok = (uint8_t)(rl_control.policy.run_ok != src.last_run_ok);
    src.last_run_ok = rl_control.policy.run_ok;
    src.last_run_fail = rl_control.policy.run_fail;
    for (i = 0u; i < S2R_ACTION_NUM; i++)
    {
        published[i] = (float)map->sign[i] * action_state.a[i];   /* 固件 → 训练空间 */
    }
    command[0] = input_command.vx_cmd;
    command[1] = input_command.yaw_cmd;
    command[2] = input_command.height_cmd;
    S2R_Observation((uint8_t)(rl_control.observation.valid && ok),
                    (uint8_t)(rl_control.observation.history_ready && ok),
                    S2R_Now_Us());
    S2R_Policy_Begin(rl_control.observation.obs, rl_control.observation.history, command);
    S2R_Policy_End(ok, published, rl_control.policy.latent);
}

/* 控制器追踪: 只读 rl_control.torque_state 与两腿解算输出 */
static void sample_trace(s2r_control_sample_t *sample)
{
    const leg_state_t *legs[2] = {&leg_l, &leg_r};
    const pid_t *pid;
    float q[6];
    float target;
    float error;
    float request;
    uint32_t i;
    uint32_t leg_index;
    uint32_t vj;

    for (i = 0u; i < S2R_TRACE_NUM; i++)
    {
        sample->values[i] = NAN;
    }
    q[0] = leg_l.output.thigh_angle;
    q[1] = leg_l.output.virtual_shank_angle;
    q[2] = 0.0f;
    q[3] = leg_r.output.thigh_angle;
    q[4] = leg_r.output.virtual_shank_angle;
    q[5] = 0.0f;

    /* 0~5 虚拟关节角 (轮角不参与, 同执行层) */
    memcpy(sample->values, q, sizeof(q));
    /* 6~11 虚拟关节速度, 轮速按 RL 输入交叉来源 */
    sample->values[6] = leg_l.input.d_hip_f;
    sample->values[7] = leg_l.output.d_virtual_shank_angle;
    sample->values[8] = motor_state.dji.vel_rad_s[wheel_source[0]];
    sample->values[9] = leg_r.input.d_hip_f;
    sample->values[10] = leg_r.output.d_virtual_shank_angle;
    sample->values[11] = motor_state.dji.vel_rad_s[wheel_source[1]];
    /* 16~17 轮目标速度 */
    sample->values[16] = rl_control.torque_state.pos_target[2];
    sample->values[17] = rl_control.torque_state.pos_target[5];

    for (i = 0u; i < 4u; i++)
    {
        vj = (i < 2u) ? i : i + 1u;         /* VJ: 0,1 左腿; 3,4 右腿 */
        leg_index = i;
        pid = &rl_control.torque_state.controller[vj];
        target = rl_control.torque_state.pos_target[vj];
        error = target - q[vj];
        sample->values[12u + leg_index] = target;      /* 腿目标角 */
        sample->values[18u + leg_index] = error;       /* wrap 前误差 */
        sample->values[22u + leg_index] = pid->err[NOW];   /* wrap 后误差 (PID 内) */
        if (pid->angle_wrap && (error > 3.14159265f || error < -3.14159265f))
        {
            sample->wrap_mask |= 1u << leg_index;
        }
    }
    for (i = 0u; i < 6u; i++)
    {
        sample->values[32u + i] = rl_control.torque_state.virtual_torque[i];   /* 限幅后虚拟力矩 */
    }
    /* 40~43 小腿雅可比 (前髋/后髋序, 同执行层映射) */
    sample->values[40] = legs[0]->output.vshank_jac[1];
    sample->values[41] = legs[0]->output.vshank_jac[0];
    sample->values[42] = legs[1]->output.vshank_jac[1];
    sample->values[43] = legs[1]->output.vshank_jac[0];

    /* 请求饱和 → 限幅位 (推导量, 不是驱动内部限幅标志) */
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        request = rl_output_dm_cmd_nm[i];
        if (request >= machine->dm_trq_clamp * 0.999f
            || request <= -machine->dm_trq_clamp * 0.999f)
        {
            sample->motor_clamp |= 1u << i;
        }
    }
    for (i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        request = rl_output_wheel_cmd_nm[i];
        if (request >= machine->dji_trq_clamp * 0.999f
            || request <= -machine->dji_trq_clamp * 0.999f)
        {
            sample->motor_clamp |= 1u << (DM_MOTOR_NUM + i);
        }
    }
    sample->virtual_clamp = 0u;
}

/* 执行层采样: 反馈 + 追踪 + 请求 (遥测层决定是否成帧) */
static void sample_control(s2r_control_sample_t *sample)
{
    uint32_t i;

    sample->now_us = S2R_Now_Us();
    src.state_seq++;
    sample->state_seq = src.state_seq;
    sample->imu_seq = src.imu_seq;
    sample->send_mask = 0u;     /* 驱动提交状态未暴露, 见 META */
    sample->rl_valid = (uint8_t)(output_task_rl_engaged()
        && torque_output_enabled
        && robot_state.motor_enabled
        && action_state.rl_ready);
    sample->used_seq = (sample->rl_valid && action_state.updated)
        ? S2R_Published_Seq() : 0u;
    memcpy(sample->motor_rx_us, src.motor_rx_us, sizeof(sample->motor_rx_us));

    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        sample->q_motor[i] = motor_state.dm.pos_rad[i];
        sample->dq_motor[i] = motor_state.dm.vel_rad_s[i];
        sample->tau_feedback[i] = motor_state.dm.trq_nm[i];
        sample->tau_request[i] = rl_output_dm_cmd_nm[i];
    }
    for (i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        sample->q_motor[DM_MOTOR_NUM + i] = motor_state.dji.angle_total_rad[wheel_source[i]];
        sample->dq_motor[DM_MOTOR_NUM + i] = motor_state.dji.vel_rad_s[wheel_source[i]];
        sample->tau_feedback[DM_MOTOR_NUM + i] = NAN;      /* 轮力矩无反馈 */
        sample->tau_request[DM_MOTOR_NUM + i] = rl_output_wheel_cmd_nm[i];
    }
    sample_trace(sample);
}

/* 通信任务每拍: 采样 + 交给遥测层成帧 */
void S2R_Source_Tick(void)
{
    static s2r_control_sample_t sample;     /* 唯一调用者是 S2R_Pump, 不占 commTask 栈 */

    memset(&sample, 0, sizeof(sample));
    sample_motor_rx();
    sample_imu();
    sample_policy();
    sample_control(&sample);
    S2R_Control(&sample);
}
