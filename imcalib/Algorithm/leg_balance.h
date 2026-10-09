#ifndef LEG_BALANCE_H
#define LEG_BALANCE_H

#include <stdint.h>

#include "leg_solver.h"
#include "lqr_balance.h"
#include "pid.h"
#include "simple-function.h"
#include "torque_output.h"
#include "gravity_comp.h"

typedef struct {
    pid_t leg_len[2];   /* 腿长 */
    ramp_t length_ramp[2]; /* 腿长目标 */
    pid_t roll;         /* 横滚补偿 */
    float F[2];         /* 足端力 (调试) */
    float Tp[2];        /* 虚拟髋扭矩 (调试) */
    torque_output_t cmd;     /* 力矩命令 (调试) */
    compensation_debug_t compensation;
    uint8_t len_prime_enable; /* 首拍预置 */
    uint8_t len_history_ready; /* 历史就绪 */
} leg_balance_t;

void    Leg_Balance_Init(leg_balance_t *lb);
void    Leg_Balance_Reset(leg_balance_t *lb);
uint8_t Leg_Balance_Compute(leg_balance_t *lb, const lqr_state_t *st,
                            const leg_state_t *leg_l, const leg_state_t *leg_r,
                            float dt, torque_output_t *torque);

#endif
