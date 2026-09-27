#include "leg_balance.h"

#include <math.h>
#include <string.h>

/* 初始化 — PID 初始化不覆盖 max_err/deadband/angle_wrap, 必须先清零 */
void Leg_Balance_Init(leg_balance_t *lb)
{
    memset(lb, 0, sizeof(*lb));

    PID_struct_init(&lb->leg_len[0], POSITION_PID, LEG_BALANCE_OUT_MAX, 0.0f,
                    LEG_BALANCE_LEN_KP, 0.0f, LEG_BALANCE_LEN_KD, 0.0f, 0.0f);
    PID_struct_init(&lb->leg_len[1], POSITION_PID, LEG_BALANCE_OUT_MAX, 0.0f,
                    LEG_BALANCE_LEN_KP, 0.0f, LEG_BALANCE_LEN_KD, 0.0f, 0.0f);
    PID_struct_init(&lb->roll, POSITION_PID, LEG_BALANCE_OUT_MAX, 0.0f,
                    LEG_BALANCE_ROLL_KP, 0.0f, LEG_BALANCE_ROLL_KD, 0.0f, 0.0f);
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
}

/* 力域映射 + 限幅: (足端力, 髋扭矩) → 前后髋电机力矩 (虚功原理), 轮扭矩直接限幅 */
static uint8_t Leg_Balance_Output(leg_balance_t *lb, const leg_state_t *leg_l,
                                  const leg_state_t *leg_r, const float F[2],
                                  const float Tp[2], const float wheel[2],
                                  torque_output_t *torque)
{
    float tau[2];
    uint8_t i;

    memset(&lb->cmd, 0, sizeof(lb->cmd));   /* 失败时与零力矩一致 */
    for (i = 0u; i < 2u; i++)
    {
        if (!isfinite(F[i]) || !isfinite(Tp[i]))
        {
            return 0u;
        }
    }
    lb->F[0] = F[0];
    lb->F[1] = F[1];
    lb->Tp[0] = Tp[0];
    lb->Tp[1] = Tp[1];

    if (!Leg_Force_Map_Forward(leg_l, F[0], Tp[0], tau))
    {
        return 0u;
    }
    torque->dm[DM_MOTOR_LEG_F_LFT] = clampf(tau[0], -lqr_debug.trq_max_hip,
                                            lqr_debug.trq_max_hip);
    torque->dm[DM_MOTOR_LEG_B_LFT] = clampf(tau[1], -lqr_debug.trq_max_hip,
                                            lqr_debug.trq_max_hip);

    if (!Leg_Force_Map_Forward(leg_r, F[1], Tp[1], tau))
    {
        return 0u;
    }
    torque->dm[DM_MOTOR_LEG_F_RGT] = clampf(tau[0], -lqr_debug.trq_max_hip,
                                            lqr_debug.trq_max_hip);
    torque->dm[DM_MOTOR_LEG_B_RGT] = clampf(tau[1], -lqr_debug.trq_max_hip,
                                            lqr_debug.trq_max_hip);

    /* 轮扭矩 (输出极性在 dji.c 驱动边界统一处理) */
    torque->dji[DJI_MOTOR_WHEEL_LFT] = clampf(wheel[0], -lqr_debug.trq_max_wheel,
                                              lqr_debug.trq_max_wheel);
    torque->dji[DJI_MOTOR_WHEEL_RGT] = clampf(wheel[1], -lqr_debug.trq_max_wheel,
                                              lqr_debug.trq_max_wheel);
    lb->cmd = *torque;
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

    if (lb == NULL || st == NULL || leg_l == NULL || leg_r == NULL
        || torque == NULL)
    {
        return 0u;
    }
    if (!leg_l->output.valid || !leg_r->output.valid)
    {
        return 0u;
    }

    /* 1. 辅助 PID */
    (void)pid_calc(&lb->leg_len[0], leg_l->output.virtual_leg_length,
                   st->leg_len_tgt[0], dt);
    (void)pid_calc(&lb->leg_len[1], leg_r->output.virtual_leg_length,
                   st->leg_len_tgt[1], dt);
    (void)pid_calc(&lb->roll, st->roll, 0.0f, dt);

    /* 2. 力向量: 轮扭矩/虚拟髋扭矩来自 LQR, 足端力来自腿长+横滚+前馈
     * Tp 取反: 本工程腿摆角与模型 θ_ll 反号, 广义力随之反号
     * (Leg_Force_Map_Forward(F,Tp) ≡ Leg_Tougue(F,-Tp)) */
    Tp[0] = -st->u[LQR_U_BL];
    Tp[1] = -st->u[LQR_U_BR];
    F[0] = lb->leg_len[0].pos_out + lb->roll.pos_out + LEG_BALANCE_F_FEEDFORWARD;
    F[1] = lb->leg_len[1].pos_out - lb->roll.pos_out + LEG_BALANCE_F_FEEDFORWARD;
    if (!lqr_debug.hip_enable)
    {
        Tp[0] = 0.0f;
        Tp[1] = 0.0f;
    }
    if (!lqr_debug.len_pid_enable)
    {
        F[0] = LEG_BALANCE_F_FEEDFORWARD;
        F[1] = LEG_BALANCE_F_FEEDFORWARD;
    }
    wheel[0] = lqr_debug.wheel_enable ? st->u[LQR_U_WL] : 0.0f;
    wheel[1] = lqr_debug.wheel_enable ? st->u[LQR_U_WR] : 0.0f;

    /* 3. 力域映射 + 限幅 */
    return Leg_Balance_Output(lb, leg_l, leg_r, F, Tp, wheel, torque);
}
