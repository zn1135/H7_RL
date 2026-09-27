#include "robot_control.h"
#include "rl_observation.h"
#include "rl_policy.h"
#include "machine_config.h"
#include "pid.h"

#include <string.h>

/* 选中 RL 后预热，再运行 networkzn1 推理；观测无效时发布零动作。 */
static uint16_t warmup_cnt;     /* 预热计数 */

static imu_state_t RL_IMU_Snapshot(void)
{
    imu_state_t snapshot;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    snapshot = imu_state;
    __set_PRIMASK(primask);
    return snapshot;
}

/* 检查电机 */
static uint8_t RL_Motors_Online(void)
{
    for (uint8_t i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (!motor_state.dm.online[i])
        {
            return 0u;
        }
    }
    for (uint8_t i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        if (!motor_state.dji.online[i])
        {
            return 0u;
        }
    }
    return 1u;
}

/* 固件关节 → 训练关节: q_t = sign × wrap(q − zero), 速度同符号; 映射未配置返回 0 */
static uint8_t RL_Joint_Map(float joint_pos[4], float joint_vel[6])
{
    const rl_map_t *map = &machine->rl;
    float qd[6];

    if (!map->configured)
    {
        return 0u;
    }
    joint_pos[0] = (float)map->sign[0] * Angle_Wrap_180(leg_l.output.thigh_angle - map->zero[0]);
    joint_pos[1] = (float)map->sign[1] * Angle_Wrap_180(leg_l.output.virtual_shank_angle - map->zero[1]);
    joint_pos[2] = (float)map->sign[3] * Angle_Wrap_180(leg_r.output.thigh_angle - map->zero[2]);
    joint_pos[3] = (float)map->sign[4] * Angle_Wrap_180(leg_r.output.virtual_shank_angle - map->zero[3]);
    qd[0] = leg_l.input.d_hip_f;
    qd[1] = leg_l.output.d_virtual_shank_angle;
    /* 台架反馈核对: 物理左轮目前从 DJI RGT 索引进入, RL 左轮槽在这里交换来源 */
    qd[2] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_RGT];
    qd[3] = leg_r.input.d_hip_f;
    qd[4] = leg_r.output.d_virtual_shank_angle;
    qd[5] = motor_state.dji.vel_rad_s[DJI_MOTOR_WHEEL_LFT];
    for (uint8_t i = 0u; i < 6u; i++)
    {
        joint_vel[i] = (float)map->sign[i] * qd[i];
    }
    return 1u;
}

/* 构建观测 + 推历史; 源无效或映射未配置 → 观测清零返回 0 */
static uint8_t RL_Control_Update_Observation(const float command[3])
{
    imu_state_t imu = RL_IMU_Snapshot();
    float joint_pos[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float joint_vel[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t source_valid;

    source_valid = (uint8_t)(imu.online && leg_l.output.valid
        && leg_r.output.valid && RL_Motors_Online()
        && RL_Joint_Map(joint_pos, joint_vel));
    if (!RL_Observation_Build(&rl_control.observation, &rl_control.param,
        imu.gyro_rad_s, imu.quat, command,
        joint_pos, joint_vel, source_valid))
    {
        return 0u;
    }
    RL_Observation_Update_History(&rl_control.observation);
    return rl_control.observation.history_ready;
}

/* 未投入时的观测预览: 只供 VOFA, 不推历史 */
static void RL_Observation_Preview(const float command[3])
{
    imu_state_t imu = RL_IMU_Snapshot();
    float joint_pos[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float joint_vel[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t source_valid;

    source_valid = (uint8_t)(imu.online && leg_l.output.valid
        && leg_r.output.valid && RL_Joint_Map(joint_pos, joint_vel));
    RL_Observation_Reset(&rl_control.observation);
    (void)RL_Observation_Build(&rl_control.observation, &rl_control.param,
        imu.gyro_rad_s, imu.quat, command,
        joint_pos, joint_vel, source_valid);
}

/* 发布动作 (固件关节空间) */
static void RL_Action_Publish(const float action[RL_ACTION_SIZE], uint8_t rl_ready)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(action_state.a, action, sizeof(action_state.a));
    action_state.updated = 1u;
    action_state.last_ok_tick = HAL_GetTick();
    action_state.rl_ready = rl_ready;
    __set_PRIMASK(primask);
}

/* 遥控 → 策略指令: 前进 / 转向 / 高度 (范围 RL_CMD_*; 转向右推为负, 同 LQR) */
static void RL_Command_From_Rc(float command[3])
{
    command[0] = rc_command.vel * RL_CMD_VX_MAX;
    command[1] = -rc_command.yaw * RL_CMD_YAW_MAX;
    command[2] = RL_CMD_HEIGHT_MIN
               + (rc_command.len + 1.0f) * 0.5f * (RL_CMD_HEIGHT_MAX - RL_CMD_HEIGHT_MIN);
    input_command.vx_cmd = command[0];
    input_command.yaw_cmd = command[1];
    input_command.height_cmd = command[2];
}

/* 动作裁剪 (RL_ACTION_CLIP = 0 不裁) */
static void RL_Action_Clip(float action[RL_ACTION_SIZE])
{
    float limit = RL_ACTION_CLIP;

    if (limit <= 0.0f)
    {
        return;
    }
    for (uint8_t i = 0u; i < RL_ACTION_SIZE; i++)
    {
        action[i] = clampf(action[i], -limit, limit);
    }
}

/* 推理路径单周期 */
static void RL_Infer_Body(void)
{
    const rl_map_t *map = &machine->rl;
    float command[3];
    float action_t[RL_ACTION_SIZE] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};   /* 训练空间 */
    float action[RL_ACTION_SIZE] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};     /* 固件空间 */

    RL_Command_From_Rc(command);

    /* 未投入: 清历史 (预览观测照算供 VOFA), 发零动作保持新鲜 (LQR 挡也走这里) */
    if (!output_task_rl_engaged())
    {
        warmup_cnt = 0u;
        RL_Observation_Preview(command);
        RL_Action_Publish(action, 0u);
        return;
    }

    if (!RL_Control_Update_Observation(command))
    {
        /* 观测无效: 重新预热, 零力矩 */
        warmup_cnt = 0u;
        RL_Action_Publish(action, 0u);
        return;
    }

    if (warmup_cnt < RL_WARMUP_STEPS)
    {
        warmup_cnt++;
    }
    else
    {
        if (!RL_Policy_Run(&rl_control.policy, &rl_control.observation, action_t))
        {
            RL_Observation_Set_Last_Action(&rl_control.observation, action_t);   /* 失败: 记录零动作 */
            RL_Action_Publish(action, 0u);   /* 推理失败 */
            return;
        }
        RL_Action_Clip(action_t);
    }

    RL_Observation_Set_Last_Action(&rl_control.observation, action_t);
    for (uint8_t i = 0u; i < RL_ACTION_SIZE; i++)
    {
        action[i] = (float)map->sign[i] * action_t[i];   /* 训练 → 固件 */
    }
    RL_Action_Publish(action, 1u);
}

/* 策略初始化 */
void ctrl_task_init(void)
{
    if (machine->rl.configured)
    {
        (void)RL_Policy_Init(&rl_control.policy);
    }
}

/* 策略单周期 */
void ctrl_task_body(void)
{
    if (machine->rl.configured)
    {
        RL_Infer_Body();
    }
}
