#include "standup.h"
#include "machine_config.h"
#include "gas_spring.h"

#include <math.h>
#include <string.h>

#define STANDUP_SUPPORT_ANGLE_RANGE 0.6f

standup_ctx_t standup_control;
standup_param_t standup_param = {
    /* 恢复触发 */
    .trigger_angle           = 50.0f * LEG_PI / 180.0f,         /* 腿角 rad */
    .trigger_pitch           = 40.0f * LEG_PI / 180.0f,         /* 俯仰 rad */
    .trigger_roll            = 40.0f * LEG_PI / 180.0f,         /* 横滚 rad */
    .trigger_time            = 0.2f,                            /* 持续 s */
    .pitch_max               = 1.0f,                            /* 翻身 rad */
    .roll_max                = 1.0f,                            /* 侧偏 rad */

    /* 伸腿与后摆 */
    .extend_len              = 0.30f,                           /* 目标 m */
    .extend_tol              = 0.01f,                           /* 容差 m */
    .extend_timeout          = 4.0f,                            /* 超时 s */
    .rear_angle              = -1.5f,                           /* 后点 rad */
    .rear_tol                = 0.3f,                            /* 容差 rad */
    .rear_rate               = 5.1f,                            /* 目标 rad/s */
    .rear_timeout            = 4.0f,                            /* 超时 s */

    /* 收腿与站立 */
    .retract_len             = 0.15f,                           /* 目标 m */
    .retract_ready_len       = 0.16f,                           /* 转摆 m */
    .length_tol              = 0.03f,                           /* 容差 m */
    .angle_tol               = 0.3f,                            /* 容差 rad */
    .roll_ready              = 0.3f,                            /* 到位 rad */
    .timeout                 = {3.0f, 4.0f},                    /* 收/摆 s */

    /* 摆角串级PD */
    .angle_pos_kp            = 25.0f,                           /* 位置 P */
    .angle_pos_kd            = 20.0f,                           /* 位置 D */
    .angle_speed_kp          = 5.0f,                            /* 速度 P */
    .angle_speed_kd          = 5.0f,                            /* 速度 D */
    .angle_speed_max         = 10.0f,                           /* 目标 rad/s */

    /* 出力与纠偏 */
    .tp_max                  = 40.0f,                           /* 摆矩 Nm */
    .roll_force_max          = 40.0f,                           /* 差动力 N */
    .roll_force_rate         = 10.0f,                           /* 渐变 N/s */
    .recovery_force_max      = 150.0f,                          /* 恢复力 N */

    /* 翻倒恢复 */
    .recovery_settle_time    = 0.05f,                            /* 缓冲 s */
    .recovery_tuck_time      = 0.05f,                            /* 伸腿 s */
    .recovery_len            = 0.28f,                           /* 扫腿长 m */
    .recovery_angle_rate     = 3.0f,                            /* 目标 rad/s */
    .recovery_ready_pitch    = 0.8f,                           /* 回正 rad */
    .recovery_timeout        = 4.0f,                            /* 单阶段 s */
    .recovery_stall_time     = 0.3f,                            /* 卡住 s */

    /* 持稳与交接 */
    .stable_time             = 0.05f,                           /* 持稳 s */
    .support_time            = 0.4f,                            /* 支撑渐入 s */
    .blend_time              = 0.25f,                           /* LQR渐入 s */

    /* 失败重试 */
    .retry_wait              = 0.1f,                            /* 撤力等待 s */
    .recovery_retry_max      = 3u,                              /* 重试次数 */
};

static float Standup_Wrap(float angle)
{
    angle = fmodf(angle, LEG_2PI);
    if (angle > LEG_PI)
    {
        angle -= LEG_2PI;
    }
    if (angle < -LEG_PI)
    {
        angle += LEG_2PI;
    }
    return angle;
}

static void Standup_Enter(standup_ctx_t *st, uint8_t phase)
{
    st->phase = phase;
    st->need = (uint8_t)(phase == STANDUP_RETRACT
        || phase == STANDUP_SWING || phase == STANDUP_FAILED || phase == STANDUP_REAR
        || phase == STANDUP_EXTEND || phase == STANDUP_SETTLE
        || phase == STANDUP_TUCK || phase == STANDUP_FLIP);
    st->elapsed = 0.0f;
    st->stable = 0.0f;
    st->trigger_elapsed = 0.0f;
}

/* 初始化自起PD */
static void Standup_PID_Init(standup_ctx_t *st)
{
    uint8_t i;

    PID_struct_init(&st->roll_pid, POSITION_PID,
        fminf(standup_param.roll_force_max, machine->lqr.roll.max_output), 0.0f,
        machine->lqr.roll.kp, 0.0f, machine->lqr.roll.kd, 0.0f, 0.0f);
    for (i = 0u; i < 2u; i++)
    {
        PID_struct_init(&st->length_pid[i], POSITION_PID,
            machine->lqr.leg_len[i].max_output, 0.0f,
            machine->lqr.leg_len[i].kp, 0.0f,
            machine->lqr.leg_len[i].kd, 0.0f, 0.0f);
        PID_struct_init(&st->angle_pos_pid[i], POSITION_PID,
            standup_param.angle_speed_max, 0.0f,
            standup_param.angle_pos_kp, 0.0f,
            standup_param.angle_pos_kd, 0.0f, 0.0f);
        st->angle_pos_pid[i].angle_wrap = 1u;
        PID_struct_init(&st->angle_speed_pid[i], POSITION_PID,
            fminf(standup_param.tp_max, machine->dm_trq_clamp), 0.0f,
            standup_param.angle_speed_kp, 0.0f,
            standup_param.angle_speed_kd, 0.0f, 0.0f);
    }
}

void Standup_Init(standup_ctx_t *st)
{
    memset(st, 0, sizeof(*st));
    st->enabled = 1u;
    st->recovery_enabled = 1u;
    Standup_PID_Init(st);
}

void Standup_Reset(standup_ctx_t *st)
{
    uint8_t enabled;
    uint8_t recovery_enabled;

    enabled = st->enabled;
    recovery_enabled = st->recovery_enabled;
    memset(st, 0, sizeof(*st));
    st->enabled = enabled;
    st->recovery_enabled = recovery_enabled;
}

void Standup_Fail(standup_ctx_t *st, uint8_t fault)
{
    if (st->phase != STANDUP_FAILED)
    {
        st->phase = STANDUP_FAILED;
        st->need = 1u;
        st->fault = fault;
        st->retry_elapsed = 0.0f;
    }
    Torque_Output_Clear(&st->prepare);
}

/* 公共反馈标志 */
static uint8_t Standup_Input_Valid(const imu_state_t *imu,
                                  const leg_state_t *const leg[2])
{
    uint8_t i;

    if (imu == NULL || !imu->online || !imu->pitch_world_valid)
    {
        return 0u;
    }
    for (i = 0u; i < 2u; i++)
    {
        if (leg[i] == NULL || !leg[i]->output.valid
            || !leg[i]->output.force_valid)
        {
            return 0u;
        }
    }
    return 1u;
}

static uint8_t Standup_Need(const imu_state_t *imu,
                           const leg_state_t *const leg[2])
{
    float left_error;
    float right_error;
    float pitch_error;

    left_error = Standup_Wrap(leg[0]->output.virtual_leg_angle
        - machine->lqr.leg_trim[0]);
    right_error = Standup_Wrap(leg[1]->output.virtual_leg_angle
        - machine->lqr.leg_trim[1]);
    pitch_error = Standup_Wrap(imu->pitch_world - machine->lqr.pitch_trim);
    return (uint8_t)(fabsf(left_error) > standup_param.trigger_angle
        || fabsf(right_error) > standup_param.trigger_angle
        || fabsf(pitch_error) > standup_param.trigger_pitch
        || fabsf(imu->euler_rad[0u]) > standup_param.trigger_roll);
}

/* 伸腿目标 */
static float Standup_Extend_Length(void)
{
    return clampf(standup_param.extend_len, machine->leg_len_min, machine->leg_len_max);
}

static uint8_t Standup_Extended(const leg_state_t *const leg[2])
{
    float minimum;

    minimum = Standup_Extend_Length() - standup_param.extend_tol;
    return (uint8_t)(leg[0]->output.virtual_leg_length >= minimum
        && leg[1]->output.virtual_leg_length >= minimum);
}

/* 后摆到位 */
static uint8_t Standup_Rear_Ready(const leg_state_t *const leg[2])
{
    uint8_t i;

    for (i = 0u; i < 2u; i++)
    {
        if (fabsf(Standup_Wrap(leg[i]->output.virtual_leg_angle
            - standup_param.rear_angle)) > standup_param.rear_tol)
        {
            return 0u;
        }
    }
    return 1u;
}

/* 锁定上绕路径 */
static void Standup_Rear_Path_Init(standup_ctx_t *st, const imu_state_t *imu,
                                   const leg_state_t *const leg[2])
{
    float angle;
    float start;
    float goal;
    uint8_t i;

    for (i = 0u; i < 2u; i++)
    {
        angle = Standup_Wrap(leg[i]->output.virtual_leg_angle);
        start = Standup_Wrap(angle - imu->pitch_world);
        goal = Standup_Wrap(standup_param.rear_angle - imu->pitch_world);
        if (start < 0.0f)
        {
            start += LEG_2PI;
        }
        if (goal < 0.0f)
        {
            goal += LEG_2PI;
        }
        st->rear_position[i] = angle;
        st->rear_last[i] = angle;
        st->rear_goal[i] = angle + goal - start;
        st->angle_cmd[i] = angle;
        st->angle_pos_pid[i].angle_wrap = 0u;
    }
    st->rear_path_ready = 1u;
}

/* 连续角度反馈 */
static void Standup_Rear_Path_Update(standup_ctx_t *st,
                                     const leg_state_t *const leg[2])
{
    float angle;
    uint8_t i;

    for (i = 0u; i < 2u; i++)
    {
        angle = Standup_Wrap(leg[i]->output.virtual_leg_angle);
        st->rear_position[i] += Standup_Wrap(angle - st->rear_last[i]);
        st->rear_last[i] = angle;
    }
}

static uint8_t Standup_Ready(const standup_ctx_t *st,
                            const imu_state_t *imu,
                            const leg_state_t *const leg[2])
{
    float length_goal;
    float angle_error;
    float command_error;
    uint8_t i;

    if (st->phase == STANDUP_EXTEND)
    {
        return Standup_Extended(leg);
    }
    if (st->phase == STANDUP_REAR)
    {
        if (!st->rear_path_ready)
        {
            return 0u;
        }
        for (i = 0u; i < 2u; i++)
        {
            if (fabsf(st->rear_goal[i] - st->rear_position[i]) > standup_param.rear_tol
                || fabsf(st->rear_goal[i] - st->angle_cmd[i]) > standup_param.rear_tol)
            {
                return 0u;
            }
        }
        return 1u;
    }
    if (st->phase == STANDUP_RETRACT)
    {
        return (uint8_t)(leg[0]->output.virtual_leg_length <= standup_param.retract_ready_len
            && leg[1]->output.virtual_leg_length <= standup_param.retract_ready_len);
    }
    if (st->phase == STANDUP_SWING
        && fabsf(imu->euler_rad[0u]) > standup_param.roll_ready)
    {
        return 0u;
    }
    for (i = 0u; i < 2u; i++)
    {
        length_goal = standup_param.retract_len;
        if (fabsf(leg[i]->output.virtual_leg_length - length_goal) > standup_param.length_tol
            || fabsf(st->length_cmd[i] - length_goal) > standup_param.length_tol)
        {
            return 0u;
        }
        if (st->phase == STANDUP_SWING)
        {
            angle_error = Standup_Wrap(leg[i]->output.virtual_leg_angle
                - machine->lqr.leg_trim[i]);
            command_error = Standup_Wrap(st->angle_cmd[i] - machine->lqr.leg_trim[i]);
            if (fabsf(angle_error) > standup_param.angle_tol
                || fabsf(command_error) > standup_param.angle_tol)
            {
                return 0u;
            }
        }
    }
    return 1u;
}

/* 预置PD历史 */
static void Standup_Prime(pid_t *pid, float measured, float target)
{
    float error;
    uint8_t i;

    error = target - measured;
    if (pid->angle_wrap)
    {
        error = Standup_Wrap(error);
    }
    for (i = 0u; i < 3u; i++)
    {
        pid->get[i] = measured;
        pid->set[i] = target;
        pid->err[i] = error;
    }
}

/* 恢复阶段 */
static uint8_t Standup_Recovering(const standup_ctx_t *st)
{
    return (uint8_t)(st->phase >= STANDUP_SETTLE && st->phase <= STANDUP_FLIP);
}

/* 重力姿态分类 */
static uint8_t Standup_Pose_Update(standup_ctx_t *st, const imu_state_t *imu)
{
    float norm;
    float q0;
    float q1;
    float q2;
    float q3;

    q0 = imu->quat[0]; q1 = imu->quat[1];
    q2 = imu->quat[2]; q3 = imu->quat[3];
    norm = q0*q0 + q1*q1 + q2*q2 + q3*q3;
    if (!isfinite(norm) || norm <= 1e-6f)
    {
        st->pose = STANDUP_POSE_INVALID;
        return st->pose;
    }
    st->upright = 1.0f - 2.0f * (q1*q1 + q2*q2) / norm;
    st->side = 2.0f * fabsf(q0*q1 + q2*q3) / norm;
    if (st->side > sinf(standup_param.roll_max)
        || (st->upright < 0.0f && st->side > 0.5f))
    {
        st->pose = STANDUP_POSE_SIDE;
    }
    else if (st->upright < 0.0f || fabsf(imu->pitch_world) > standup_param.pitch_max)
    {
        st->pose = STANDUP_POSE_INVERTED;
    }
    else if (fabsf(imu->euler_rad[0u]) > standup_param.roll_max)
    {
        st->pose = STANDUP_POSE_SIDE;
    }
    else
    {
        st->pose = STANDUP_POSE_NORMAL;
    }
    return st->pose;
}

/* 恢复转段与预置 */
static void Standup_Recovery_Enter(standup_ctx_t *st, uint8_t phase,
                                    const leg_state_t *const leg[2])
{
    float angle;
    uint8_t i;

    Standup_Enter(st, phase);
    st->len_history_ready = 0u;
    st->angle_history_ready = 0u;
    st->roll_history_ready = 0u;
    st->roll_force = 0.0f;
    st->roll_applied = 0.0f;
    st->support = 0.0f;
    st->stall_elapsed = 0.0f;
    for (i = 0u; i < 2u; i++)
    {
        angle = Standup_Wrap(leg[i]->output.virtual_leg_angle);
        st->recovery_position[i] = angle;
        st->recovery_last[i] = angle;
        st->angle_cmd[i] = angle;
        st->length_cmd[i] = leg[i]->output.virtual_leg_length;
        st->angle_pos_pid[i].angle_wrap = 0u;
    }
    if (phase == STANDUP_FLIP)
    {
        st->recovery_sweep = 0.0f;
    }
}

/* 回正窗口 */
static uint8_t Standup_Recovery_Ready(const standup_ctx_t *st,
                                     const imu_state_t *imu, float pitch_limit)
{
    return (uint8_t)(st->upright > 0.0f
        && fabsf(imu->pitch_world) <= pitch_limit
        && fabsf(imu->euler_rad[0u]) <= standup_param.roll_ready);
}

static uint8_t Standup_Recovery_Retry(standup_ctx_t *st,
                                     const leg_state_t *const leg[2])
{
    if (st->retry >= standup_param.recovery_retry_max)
    {
        Standup_Fail(st, STANDUP_TIMEOUT);
        return STANDUP_ROUTE_STOP;
    }
    st->retry++;
    Standup_Recovery_Enter(st, STANDUP_TUCK, leg);
    return STANDUP_ROUTE_PREPARE;
}

/* 翻倒恢复阶段机 */
static uint8_t Standup_Recovery_Stage(standup_ctx_t *st, const imu_state_t *imu,
                                     const leg_state_t *const leg[2], float dt)
{
    float angle;
    float error;
    float speed;
    uint8_t i;

    for (i = 0u; i < 2u; i++)
    {
        angle = Standup_Wrap(leg[i]->output.virtual_leg_angle);
        st->recovery_position[i] += Standup_Wrap(angle - st->recovery_last[i]);
        st->recovery_last[i] = angle;
    }
    st->elapsed += dt;
    if (st->elapsed > standup_param.recovery_timeout)
    {
        return Standup_Recovery_Retry(st, leg);
    }
    switch (st->phase)
    {
    case STANDUP_SETTLE:
        if (st->elapsed >= standup_param.recovery_settle_time)
        {
            Standup_Recovery_Enter(st, STANDUP_TUCK, leg);
        }
        break;
    case STANDUP_TUCK:
        if (st->elapsed >= standup_param.recovery_tuck_time)
        {
            Standup_Recovery_Enter(st, STANDUP_FLIP, leg);
        }
        break;
    case STANDUP_FLIP:
        if (Standup_Recovery_Ready(st, imu, standup_param.recovery_ready_pitch))
        {
            st->stable += dt;
            if (st->stable >= standup_param.stable_time)
            {
                st->recovered = 1u;
                st->len_history_ready = 0u;
                st->angle_history_ready = 0u;
                st->support = 0.0f;
                for (i = 0u; i < 2u; i++)
                {
                    st->angle_pos_pid[i].angle_wrap = 1u;
                    st->length_cmd[i] = leg[i]->output.virtual_leg_length;
                }
                if (!Standup_Extended(leg))
                {
                    Standup_Enter(st, STANDUP_EXTEND);
                }
                else if (!Standup_Rear_Ready(leg))
                {
                    Standup_Enter(st, STANDUP_REAR);
                }
                else if (leg[0]->output.virtual_leg_length <= standup_param.retract_ready_len
                    && leg[1]->output.virtual_leg_length <= standup_param.retract_ready_len)
                {
                    Standup_Enter(st, STANDUP_SWING);
                }
                else
                {
                    Standup_Enter(st, STANDUP_RETRACT);
                }
                break;
            }
        }
        else
        {
            st->stable = 0.0f;
        }
        error = fmaxf(fabsf(st->angle_cmd[0] - st->recovery_position[0]),
                      fabsf(st->angle_cmd[1] - st->recovery_position[1]));
        speed = fmaxf(fabsf(leg[0]->output.d_virtual_leg_angle),
                      fabsf(leg[1]->output.d_virtual_leg_angle));
        if (error > 0.5f && speed < 0.1f)
        {
            if (st->stall_elapsed == 0.0f)
            {
                st->stall_upright = st->upright;
            }
            st->stall_elapsed += dt;
            if (st->upright > st->stall_upright + 0.05f)
            {
                st->stall_elapsed = 0.0f;
            }
        }
        else
        {
            st->stall_elapsed = 0.0f;
        }
        if (st->stall_elapsed >= standup_param.recovery_stall_time
            || st->recovery_sweep >= LEG_2PI)
        {
            return Standup_Recovery_Retry(st, leg);
        }
        break;
    default:
        Standup_Fail(st, STANDUP_BAD_CONFIG);
        return STANDUP_ROUTE_STOP;
    }
    return STANDUP_ROUTE_PREPARE;
}

/* 恢复目标限速 */
static void Standup_Recovery_Target(standup_ctx_t *st,
                                     const leg_state_t *const leg[2], float dt)
{
    float goal;
    uint8_t i;

    goal = standup_param.recovery_len;
    goal = clampf(goal, machine->leg_len_min, machine->leg_len_max);
    for (i = 0u; i < 2u; i++)
    {
        if (st->phase == STANDUP_SETTLE)
        {
            st->length_cmd[i] = leg[i]->output.virtual_leg_length;
            st->angle_cmd[i] = st->recovery_position[i];
        }
        else
        {
            st->length_cmd[i] = goal;
        }
        if (st->phase == STANDUP_FLIP && st->stable == 0.0f)
        {
            st->angle_cmd[i] += standup_param.recovery_angle_rate * dt;
        }

    }
    if (st->phase == STANDUP_FLIP && st->stable == 0.0f)
    {
        st->recovery_sweep += standup_param.recovery_angle_rate * dt;
    }

}

/* 阶段判断与切换 */
static uint8_t Standup_Stage_Update(standup_ctx_t *st,
                                   const imu_state_t *imu,
                                   const leg_state_t *const leg[2],
                                   uint8_t permit, uint8_t allow_restart,
                                   float dt)
{
    float timeout;
    uint8_t retry;
    uint8_t i;

    (void)allow_restart;
    if (st->phase == STANDUP_FAILED)
    {
        if (!permit)
        {
            st->retry_elapsed = 0.0f;
            return STANDUP_ROUTE_STOP;
        }
        if (st->fault == STANDUP_BAD_CONFIG
            || st->retry >= standup_param.recovery_retry_max)
        {
            return STANDUP_ROUTE_STOP;
        }
        st->retry_elapsed += dt;
        if (st->retry_elapsed < standup_param.retry_wait)
        {
            return STANDUP_ROUTE_STOP;
        }
        retry = st->retry + 1u;
        Standup_Reset(st);
        st->retry = retry;
    }
    if (!permit)
    {
        st->need = 1u;
        if (st->phase != STANDUP_IDLE)
        {
            Standup_Fail(st, STANDUP_GATED);
        }
        return STANDUP_ROUTE_STOP;
    }
    if (st->recovery_enabled)
    {
        Standup_Pose_Update(st, imu);
        if (st->pose == STANDUP_POSE_INVALID)
        {
            Standup_Fail(st, STANDUP_BAD_INPUT);
            return STANDUP_ROUTE_STOP;
        }
        if ((st->pose == STANDUP_POSE_INVERTED || st->upright < 0.0f)
            && !Standup_Recovering(st))
        {
            retry = st->retry;
            Standup_Reset(st);
            st->retry = retry;
            Standup_PID_Init(st);
            Standup_Pose_Update(st, imu);
            Standup_Recovery_Enter(st, STANDUP_SETTLE, leg);
        }
    }
    if (Standup_Recovering(st))
    {
        return Standup_Recovery_Stage(st, imu, leg, dt);
    }
    if (st->phase == STANDUP_DONE)
    {
        if (Standup_Need(imu, leg))
        {
            st->trigger_elapsed += dt;
        }
        else
        {
            st->trigger_elapsed = 0.0f;
            if (st->blend >= 1.0f)
            {
                st->retry = 0u;
            }
        }
        if (st->trigger_elapsed < standup_param.trigger_time)
        {
            return STANDUP_ROUTE_BALANCE;
        }
        retry = st->retry;
        Standup_Reset(st);
        st->retry = retry;
    }
    if (st->phase == STANDUP_IDLE)
    {
        retry = st->retry;
        Standup_Reset(st);
        st->retry = retry;
        st->need = 1u;
        Standup_PID_Init(st);
        if (st->recovery_enabled)
        {
            Standup_Pose_Update(st, imu);
        }
        if (!Standup_Extended(leg))
        {
            Standup_Enter(st, STANDUP_EXTEND);
        }
        else if (!Standup_Rear_Ready(leg))
        {
            Standup_Enter(st, STANDUP_REAR);
        }
        else if (leg[0]->output.virtual_leg_length <= standup_param.retract_ready_len
            && leg[1]->output.virtual_leg_length <= standup_param.retract_ready_len)
        {
            Standup_Enter(st, STANDUP_SWING);
        }
        else
        {
            Standup_Enter(st, STANDUP_RETRACT);
        }
    }
    if (st->phase != STANDUP_RETRACT && st->phase != STANDUP_SWING
        && st->phase != STANDUP_REAR && st->phase != STANDUP_EXTEND)
    {
        Standup_Fail(st, STANDUP_BAD_CONFIG);
        return STANDUP_ROUTE_STOP;
    }
    if (st->phase == STANDUP_REAR && st->rear_path_ready)
    {
        Standup_Rear_Path_Update(st, leg);
    }
    if (!isfinite(imu->euler_rad[0u])
        || fabsf(imu->pitch_world) > standup_param.pitch_max
        || (!st->recovery_enabled && fabsf(imu->euler_rad[0u]) > standup_param.roll_max))
    {
        Standup_Fail(st, STANDUP_BAD_POSE);
        return STANDUP_ROUTE_STOP;
    }
    st->elapsed += dt;
    if (st->phase == STANDUP_EXTEND)
    {
        timeout = standup_param.extend_timeout;
    }
    else if (st->phase == STANDUP_REAR)
    {
        timeout = standup_param.rear_timeout;
    }
    else
    {
        timeout = standup_param.timeout[st->phase - STANDUP_RETRACT];
    }
    if (st->elapsed > timeout)
    {
        Standup_Fail(st, STANDUP_TIMEOUT);
        return STANDUP_ROUTE_STOP;
    }
    if (Standup_Ready(st, imu, leg))
    {
        if (st->phase == STANDUP_RETRACT)
        {
            Standup_Enter(st, STANDUP_SWING);
            return STANDUP_ROUTE_PREPARE;
        }
        st->stable += dt;
    }
    else
    {
        st->stable = 0.0f;
    }
    if (st->stable >= standup_param.stable_time)
    {
        if (st->phase == STANDUP_REAR || st->phase == STANDUP_EXTEND)
        {
            st->len_history_ready = 0u;
            st->angle_history_ready = 0u;
            for (i = 0u; i < 2u; i++)
            {
                st->angle_pos_pid[i].angle_wrap = 1u;
            }
            if (st->phase == STANDUP_EXTEND && !Standup_Rear_Ready(leg))
            {
                Standup_Enter(st, STANDUP_REAR);
            }
            else if (leg[0]->output.virtual_leg_length <= standup_param.retract_ready_len
                && leg[1]->output.virtual_leg_length <= standup_param.retract_ready_len)
            {
                Standup_Enter(st, STANDUP_SWING);
            }
            else
            {
                Standup_Enter(st, STANDUP_RETRACT);
            }
        }
        else
        {
            Standup_Enter(st, STANDUP_DONE);
            st->blend = 0.0f;
        }
    }
    return STANDUP_ROUTE_PREPARE;
}

/* 按阶段赋目标 */
static void Standup_Target_Update(standup_ctx_t *st,
                                  const imu_state_t *imu,
                                  const leg_state_t *const leg[2],
                                  float dt)
{
    float step;
    uint8_t i;

    if (Standup_Recovering(st))
    {
        Standup_Recovery_Target(st, leg, dt);
        return;
    }
    if (!st->len_history_ready)
    {
        for (i = 0u; i < 2u; i++)
        {
            if (st->phase == STANDUP_REAR)
            {
                st->length_cmd[i] = leg[i]->output.virtual_leg_length;
            }
            st->angle_cmd[i] = Standup_Wrap(leg[i]->output.virtual_leg_angle);
        }
    }
    if (st->phase == STANDUP_DONE)
    {
        return;
    }
    if (st->phase == STANDUP_REAR && !st->rear_path_ready)
    {
        Standup_Rear_Path_Init(st, imu, leg);
    }
    if (st->phase == STANDUP_SWING)
    {
        st->support = clampf(st->support + dt / standup_param.support_time, 0.0f, 1.0f);
    }
    for (i = 0u; i < 2u; i++)
    {
        if (st->phase == STANDUP_EXTEND)
        {
            st->length_cmd[i] = Standup_Extend_Length();
        }
        else if (st->phase == STANDUP_REAR)
        {
            step = standup_param.rear_rate * dt;
            st->angle_cmd[i] = clampf(st->rear_goal[i],
                st->angle_cmd[i] - step, st->angle_cmd[i] + step);
        }
        else
        {
            st->length_cmd[i] = standup_param.retract_len;
            if (st->phase == STANDUP_SWING)
            {
                st->angle_cmd[i] = Standup_Wrap(machine->lqr.leg_trim[i]);
            }
        }
    }
}

/* 支撑方向差动力 */
static void Standup_Roll_Calculate(standup_ctx_t *st, const imu_state_t *imu,
                                   const leg_state_t *const leg[2], float dt)
{
    float request;
    float weight;
    float step;
    uint8_t i;

    if (!st->roll_history_ready)
    {
        Standup_Prime(&st->roll_pid, imu->euler_rad[0u], 0.0f);
        st->roll_history_ready = 1u;
    }
    request = pid_calc(&st->roll_pid, imu->euler_rad[0u], 0.0f, dt);
    request = clampf(request, -standup_param.roll_force_max, standup_param.roll_force_max);
    step = standup_param.roll_force_rate * dt;
    st->roll_force = clampf(request, st->roll_force - step, st->roll_force + step);
    st->roll_weight = 1.0f;
    for (i = 0u; i < 2u; i++)
    {
        weight = clampf(cosf(leg[i]->output.virtual_leg_angle - imu->pitch_world), 0.0f, 1.0f);
        st->roll_weight = fminf(st->roll_weight, weight);
    }
    st->roll_applied = st->roll_force * st->roll_weight;
    st->force[0] += st->roll_applied;
    st->force[1] -= st->roll_applied;
}

/* 计算腿端F与Tp */
static uint8_t Standup_PID_Calculate(standup_ctx_t *st,
                                    const leg_state_t *const leg[2],
                                    float dt)
{
    float angle;
    float rate;
    float support_weight;
    uint8_t i;

    for (i = 0u; i < 2u; i++)
    {
        if (Standup_Recovering(st))
        {
            angle = st->recovery_position[i];
        }
        else
        {
            angle = st->phase == STANDUP_REAR ? st->rear_position[i]
                : Standup_Wrap(leg[i]->output.virtual_leg_angle);
        }
        rate = leg[i]->output.d_virtual_leg_angle;
        if (!isfinite(angle) || !isfinite(rate))
        {
            return 0u;
        }
        if (!st->len_history_ready)
        {
            Standup_Prime(&st->length_pid[i], leg[i]->output.virtual_leg_length, st->length_cmd[i]);
        }
        st->force[i] = pid_calc(&st->length_pid[i], leg[i]->output.virtual_leg_length, st->length_cmd[i], dt);
        st->tp[i] = 0.0f;
        if (st->phase != STANDUP_RETRACT)
        {
            if (!st->angle_history_ready)
            {
                Standup_Prime(&st->angle_pos_pid[i], angle, st->angle_cmd[i]);
            }
            st->angle_speed_cmd[i] = pid_calc(&st->angle_pos_pid[i], angle, st->angle_cmd[i], dt);
            if (!st->angle_history_ready)
            {
                Standup_Prime(&st->angle_speed_pid[i], rate, st->angle_speed_cmd[i]);
            }
            st->tp[i] = pid_calc(&st->angle_speed_pid[i], rate, st->angle_speed_cmd[i], dt);
            st->tp[i] = clampf(st->tp[i], -standup_param.tp_max, standup_param.tp_max);
        }
        support_weight = clampf(1.0f - fabsf(Standup_Wrap(angle - machine->lqr.leg_trim[i]))
            / STANDUP_SUPPORT_ANGLE_RANGE, 0.0f, 1.0f);
        st->force[i] += st->support * support_weight * machine->lqr.support_force[i];
        if (!isfinite(st->force[i]) || !isfinite(st->tp[i]))
        {
            return 0u;
        }
    }
    st->len_history_ready = 1u;
    if (st->phase != STANDUP_RETRACT)
    {
        st->angle_history_ready = 1u;
    }
    return 1u;
}

/* 公共映射与补偿 */
static uint8_t Standup_Torque_Output(standup_ctx_t *st,
                                    const leg_state_t *const leg[2])
{
    float base[DM_MOTOR_NUM];
    float raw[DM_MOTOR_NUM];
    float limit;
    uint8_t i;

    Torque_Output_Clear(&st->prepare);
    for (i = 0u; i < 2u; i++)
    {
        if (!Leg_Force_Map_Forward(leg[i], st->force[i], st->tp[i], &base[2u * i]))
        {
            return 0u;
        }
    }
    if (!Gas_Spring_Apply(leg[0], leg[1], base, raw))
    {
        return 0u;
    }
    limit = machine->dm_trq_clamp;
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        st->raw_dm[i] = raw[i];
        st->prepare.dm[i] = clampf(raw[i], -limit, limit);
    }
    st->prepare.valid = 1u;
    return 1u;
}

/* 完成后的输出渐变 */
static uint8_t Standup_Blend(standup_ctx_t *st, float dt,
                            torque_output_t *torque)
{
    float value;
    uint8_t i;

    if (!torque->valid || !st->prepare.valid)
    {
        return 0u;
    }
    st->blend = clampf(st->blend + dt / standup_param.blend_time, 0.0f, 1.0f);
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        value = (1.0f - st->blend) * st->prepare.dm[i] + st->blend * torque->dm[i];
        if (!isfinite(value))
        {
            return 0u;
        }
        torque->dm[i] = clampf(value, -machine->dm_trq_clamp, machine->dm_trq_clamp);
    }
    for (i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        value = st->blend * torque->dji[i];
        if (!isfinite(value))
        {
            return 0u;
        }
        torque->dji[i] = clampf(value, -machine->dji_trq_clamp, machine->dji_trq_clamp);
    }
    return 1u;
}

uint8_t Standup_Update(standup_ctx_t *st, const imu_state_t *imu,
                       const leg_state_t *left, const leg_state_t *right,
                       uint8_t permit, uint8_t allow_restart, float dt,
                       torque_output_t *torque)
{
    const leg_state_t *leg[2] = {left, right};
    uint8_t previous_phase;
    uint8_t route;
    uint8_t i;

    if (!st->enabled)
    {
        return STANDUP_ROUTE_BALANCE;
    }
    if (!isfinite(dt) || dt <= 0.0f || !Standup_Input_Valid(imu, leg))
    {
        Standup_Fail(st, STANDUP_BAD_INPUT);
        st->retry_elapsed = 0.0f;
        Torque_Output_Clear(torque);
        return STANDUP_ROUTE_STOP;
    }
    previous_phase = st->phase;
    route = Standup_Stage_Update(st, imu, leg, permit, allow_restart, dt);
    if (route == STANDUP_ROUTE_STOP)
    {
        Torque_Output_Clear(torque);
        return route;
    }
    if (route == STANDUP_ROUTE_BALANCE && st->blend >= 1.0f)
    {
        if (previous_phase != STANDUP_IDLE && !torque->valid)
        {
            Standup_Fail(st, STANDUP_BAD_INPUT);
            Torque_Output_Clear(torque);
            return STANDUP_ROUTE_STOP;
        }
        return route;
    }

    Standup_Target_Update(st, imu, leg, dt);
    if (!Standup_PID_Calculate(st, leg, dt))
    {
        Standup_Fail(st, STANDUP_BAD_INPUT);
        Torque_Output_Clear(torque);
        return STANDUP_ROUTE_STOP;
    }
    if (!st->recovery_enabled || st->upright > 0.0f)
    {
        Standup_Roll_Calculate(st, imu, leg, dt);
    }
    else
    {
        st->roll_force = 0.0f;
        st->roll_applied = 0.0f;
        st->roll_weight = 0.0f;
    }
    if (Standup_Recovering(st))
    {
        for (i = 0u; i < 2u; i++)
        {
            st->force[i] = clampf(st->force[i], -standup_param.recovery_force_max,
                standup_param.recovery_force_max);
        }
    }
    if (!Standup_Torque_Output(st, leg))
    {
        Standup_Fail(st, STANDUP_BAD_INPUT);
        Torque_Output_Clear(torque);
        return STANDUP_ROUTE_STOP;
    }
    if (route == STANDUP_ROUTE_PREPARE)
    {
        *torque = st->prepare;
    }
    else if (!Standup_Blend(st, dt, torque))
    {
        Standup_Fail(st, STANDUP_BAD_INPUT);
        Torque_Output_Clear(torque);
        return STANDUP_ROUTE_STOP;
    }
    return route;
}
