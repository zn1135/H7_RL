#ifndef LEG_BALANCE_H
#define LEG_BALANCE_H

#include <stdint.h>

#include "leg_solver.h"
#include "lqr_balance.h"
#include "pid.h"
#include "torque_output.h"

/* PID 输出上限 — 正常量级远达不到, 仅作护栏 */
#define LEG_BALANCE_OUT_MAX        5000.0f
/* 足端支持力恒定前馈 (N) — 参考值, 台架对账后标定 */
#define LEG_BALANCE_F_FEEDFORWARD  8.0f

/* 辅助 PID 参数: 同 Leg2_v1 balance.h (同为 1kHz, kd 直接照抄) */
#define LEG_BALANCE_LEN_KP         2500.0f
#define LEG_BALANCE_LEN_KD         10000.0f
#define LEG_BALANCE_ROLL_KP        200.0f
#define LEG_BALANCE_ROLL_KD        50.0f
typedef struct {
    pid_t leg_len[2];   /* 腿长 */
    pid_t roll;         /* 横滚补偿 */
    float F[2];         /* 足端力 (调试) */
    float Tp[2];        /* 虚拟髋扭矩 (调试) */
    torque_output_t cmd;     /* 力矩命令 (调试) */
} leg_balance_t;

void    Leg_Balance_Init(leg_balance_t *lb);
void    Leg_Balance_Reset(leg_balance_t *lb);
uint8_t Leg_Balance_Compute(leg_balance_t *lb, const lqr_state_t *st,
                            const leg_state_t *leg_l, const leg_state_t *leg_r,
                            float dt, torque_output_t *torque);

#endif
