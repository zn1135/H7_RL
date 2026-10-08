#include "leg_balance.h"
#include "gas_spring.h"
#include "machine_config.h"

#include <math.h>
#include <string.h>

/* 初始化 — PID 初始化不覆盖 max_err/deadband/angle_wrap, 必须先清零 */
void Leg_Balance_Init(leg_balance_t *lb)
{
    memset(lb, 0, sizeof(*lb));
    lb->len_prime_enable = 1u;

    PID_struct_init(&lb->leg_len[0], POSITION_PID, machine->lqr.leg_len[0].max_output,
                    machine->lqr.leg_len[0].integral_limit, machine->lqr.leg_len[0].kp,
                    machine->lqr.leg_len[0].ki, machine->lqr.leg_len[0].kd, 0.0f, 0.0f);
    PID_struct_init(&lb->leg_len[1], POSITION_PID, machine->lqr.leg_len[1].max_output,
                    machine->lqr.leg_len[1].integral_limit, machine->lqr.leg_len[1].kp,
                    machine->lqr.leg_len[1].ki, machine->lqr.leg_len[1].kd, 0.0f, 0.0f);
    PID_struct_init(&lb->roll, POSITION_PID, machine->lqr.roll.max_output,
                    machine->lqr.roll.integral_limit, machine->lqr.roll.kp,
                    machine->lqr.roll.ki, machine->lqr.roll.kd, 0.0f, 0.0f);
}

/* 清控制器历史与调试观测值 */
void Leg_Balance_Reset(leg_balance_t *lb)
{
    pid_t *pid[3];
    uint8_t i;

    if (lb == NULL)
    {
        return;
    }
    pid[0] = &lb->leg_len[0];
    pid[1] = &lb->leg_len[1];
    pid[2] = &lb->roll;
    for (i = 0u; i < 3u; i++)
    {
        memset(pid[i]->err, 0, sizeof(pid[i]->err));
        memset(pid[i]->set, 0, sizeof(pid[i]->set));
        memset(pid[i]->get, 0, sizeof(pid[i]->get));
        pid[i]->pout = 0.0f;
        pid[i]->iout = 0.0f;
        pid[i]->dout = 0.0f;
        pid[i]->pos_out = 0.0f;
        pid[i]->last_pos_out = 0.0f;
    }
    memset(lb->F, 0, sizeof(lb->F));
    memset(lb->Tp, 0, sizeof(lb->Tp));
    memset(&lb->cmd, 0, sizeof(lb->cmd));
    lb->len_history_ready = 0u;
}

/* 预置腿长历史 */
static void Leg_Balance_Prime_Length(pid_t *pid, float measured, float target)
{
    float error;
    uint8_t i;

    error = target - measured;
    for (i = 0u; i < 3u; i++)
    {
        pid->get[i] = measured;
        pid->set[i] = target;
        pid->err[i] = error;
    }
}

/* 力域映射 + 限幅: (足端力, 髋扭矩) → 前后髋电机力矩 (虚功原理), 轮扭矩直接限幅 */
static uint8_t Leg_Balance_Output(leg_balance_t *lb, const leg_state_t *leg_l,
                                  const leg_state_t *leg_r, const float F[2],
                                  const float Tp[2], const float wheel[2],
                                  torque_output_t *torque)
{
    float base_dm[4];
    float raw_dm[4];
    uint8_t i;

    memset(&lb->cmd, 0, sizeof(lb->cmd));   /* 失败时与零力矩一致 */
    for (i = 0u; i < 2u; i++)
    {
        if (!isfinite(F[i]) || !isfinite(Tp[i]))
        {
            return 0u;
        }
    }
    if (!Leg_Force_Map_Forward(leg_l, F[0], Tp[0], &base_dm[0])
        || !Leg_Force_Map_Forward(leg_r, F[1], Tp[1], &base_dm[2]))
    {
        return 0u;
    }
    /* 独立补偿叠加 */
    if (!Gas_Spring_Apply(leg_l, leg_r, base_dm, raw_dm))
    {
        return 0u;
    }
    lb->F[0] = F[0];
    lb->F[1] = F[1];
    lb->Tp[0] = Tp[0];
    lb->Tp[1] = Tp[1];
    for (i = 0u; i < 4u; i++)
    {
        torque->dm[i] = clampf(raw_dm[i], -lqr_debug.trq_max_hip,
                              lqr_debug.trq_max_hip);
    }

    /* 轮扭矩 (输出极性在 dji.c 驱动边界统一处理) */
    torque->dji[DJI_MOTOR_WHEEL_LFT] = clampf(wheel[0], -lqr_debug.trq_max_wheel,
                                              lqr_debug.trq_max_wheel);
    torque->dji[DJI_MOTOR_WHEEL_RGT] = clampf(wheel[1], -lqr_debug.trq_max_wheel,
                                              lqr_debug.trq_max_wheel);
    lb->cmd = *torque;
    lb->cmd.valid = 1u;
    return 1u;
}

/* 腿长/横滚 PID + 力向量 + 雅可比映射 → 电机力矩 */
uint8_t Leg_Balance_Compute(leg_balance_t *lb, const lqr_state_t *st,
                            const leg_state_t *leg_l, const leg_state_t *leg_r,
                            float dt, torque_output_t *torque)
{
    float Tp[2];
    float F[2];
    float wheel[2];
    float length[2];
    float target[2];

    if (lb == NULL || st == NULL || leg_l == NULL || leg_r == NULL
        || torque == NULL)
    {
        return 0u;
    }
    if (!leg_l->output.valid || !leg_r->output.valid)
    {
        return 0u;
    }

    length[0] = leg_l->output.virtual_leg_length;
    length[1] = leg_r->output.virtual_leg_length;
    target[0] = st->leg_len_tgt[0];
    target[1] = st->leg_len_tgt[1];

    /* 首拍消除突变 */
    if (lb->len_prime_enable && !lb->len_history_ready)
    {
        if (!isfinite(length[0]) || !isfinite(length[1])
            || !isfinite(target[0]) || !isfinite(target[1]))
        {
            return 0u;
        }
        Leg_Balance_Prime_Length(&lb->leg_len[0], length[0], target[0]);
        Leg_Balance_Prime_Length(&lb->leg_len[1], length[1], target[1]);
        lb->len_history_ready = 1u;
    }

    /* 1. 辅助 PID */
    (void)pid_calc(&lb->leg_len[0], length[0], target[0], dt);
    (void)pid_calc(&lb->leg_len[1], length[1], target[1], dt);
    (void)pid_calc(&lb->roll, st->roll, 0.0f, dt);

    /* 模型力矩转力域 */
    Tp[0] = -st->u[LQR_U_BL];
    Tp[1] = -st->u[LQR_U_BR];
    F[0] = lb->leg_len[0].pos_out + lb->roll.pos_out + machine->lqr.support_force[0];
    F[1] = lb->leg_len[1].pos_out - lb->roll.pos_out + machine->lqr.support_force[1];
    if (!lqr_debug.hip_enable)
    {
        Tp[0] = 0.0f;
        Tp[1] = 0.0f;
    }
    if (!lqr_debug.len_pid_enable)
    {
        F[0] = machine->lqr.support_force[0];
        F[1] = machine->lqr.support_force[1];
    }
    wheel[0] = lqr_debug.wheel_enable ? st->u[LQR_U_WL] : 0.0f;
    wheel[1] = lqr_debug.wheel_enable ? st->u[LQR_U_WR] : 0.0f;

    /* 3. 力域映射 + 限幅 */
    return Leg_Balance_Output(lb, leg_l, leg_r, F, Tp, wheel, torque);
}
