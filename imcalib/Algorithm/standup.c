#include "standup.h"
#include "machine_config.h"
#include "gas_spring.h"

#include <math.h>
#include <string.h>

#define STANDUP_SUPPORT_ANGLE_RANGE 0.6f

standup_ctx_t standup_control;
standup_param_t standup_param = {
    .extend_len = 0.30f, .extend_tol = 0.01f, .extend_timeout = 4.0f,
    .rear_angle = -1.5f, .rear_tol = 0.3f, .rear_timeout = 4.0f,
    .rear_rate = 4.8f,
    .retract_len = 0.15f, .retract_ready_len = 0.19f,
    .angle_pos_kp = 25.0f, .angle_pos_kd = 20.0f,
    .angle_speed_kp = 5.0f, .angle_speed_kd = 5.0f,
    .angle_speed_max = 10.0f,
    .tp_max = 40.0f,
    .trigger_angle = 50.0f * LEG_PI / 180.0f,
    .trigger_pitch = 40.0f * LEG_PI / 180.0f,
    .angle_tol = 0.3f, .length_tol = 0.04f,
    .pitch_max = 1.0f, .roll_max = 0.75f,
    .stable_time = 0.1f, .support_time = 0.3f, 
    .blend_time = 0.35f, .trigger_time = 0.1f,
    .timeout = {3.0f, 4.0f},
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
        || phase == STANDUP_EXTEND);
    st->elapsed = 0.0f;
    st->stable = 0.0f;
    st->trigger_elapsed = 0.0f;
}

/* 初始化自起PD */
static void Standup_PID_Init(standup_ctx_t *st)
{
    uint8_t i;

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
    Standup_PID_Init(st);
}

void Standup_Reset(standup_ctx_t *st)
{
    uint8_t enabled;

    enabled = st->enabled;
    memset(st, 0, sizeof(*st));
    st->enabled = enabled;
}

void Standup_Fail(standup_ctx_t *st, uint8_t fault)
{
    if (st->phase != STANDUP_FAILED)
    {
        st->phase = STANDUP_FAILED;
        st->need = 1u;
        st->fault = fault;
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
        || fabsf(pitch_error) > standup_param.trigger_pitch);
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

/* 阶段判断与切换 */
static uint8_t Standup_Stage_Update(standup_ctx_t *st,
                                   const imu_state_t *imu,
                                   const leg_state_t *const leg[2],
                                   uint8_t permit, uint8_t allow_restart,
                                   float dt)
{
    float timeout;
    uint8_t i;

    if (st->phase == STANDUP_FAILED)
    {
        return STANDUP_ROUTE_STOP;
    }
    if (!permit && st->phase != STANDUP_IDLE)
    {
        Standup_Fail(st, STANDUP_GATED);
        return STANDUP_ROUTE_STOP;
    }
    if (st->phase == STANDUP_DONE)
    {
        if (st->blend < 1.0f)
        {
            return STANDUP_ROUTE_BALANCE;
        }
        if (allow_restart && Standup_Need(imu, leg))
        {
            st->trigger_elapsed += dt;
        }
        else
        {
            st->trigger_elapsed = 0.0f;
        }
        if (st->trigger_elapsed < standup_param.trigger_time)
        {
            return STANDUP_ROUTE_BALANCE;
        }
        Standup_Reset(st);
    }
    if (st->phase == STANDUP_IDLE)
    {
        st->need = 1u;
        if (!permit)
        {
            return STANDUP_ROUTE_STOP;
        }
        Standup_Reset(st);
        Standup_PID_Init(st);
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
        || fabsf(imu->euler_rad[0u]) > standup_param.roll_max)
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
    if (Standup_Ready(st, leg))
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
        angle = st->phase == STANDUP_REAR ? st->rear_position[i]
            : Standup_Wrap(leg[i]->output.virtual_leg_angle);
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

    if (!st->enabled)
    {
        return STANDUP_ROUTE_BALANCE;
    }
    if (!isfinite(dt) || dt <= 0.0f || !Standup_Input_Valid(imu, leg))
    {
        Standup_Fail(st, STANDUP_BAD_INPUT);
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
    if (!Standup_PID_Calculate(st, leg, dt)
        || !Standup_Torque_Output(st, leg))
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
