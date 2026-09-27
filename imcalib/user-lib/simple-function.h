#ifndef SIMPLE_FUNCTION_H
#define SIMPLE_FUNCTION_H

/* ================= 简单函数库 (一阶低通 / 斜坡等) ================= */

/* 一阶低通 */
typedef struct {
    float out;
    float alpha;
} lowpass1d_t;

void  Lowpass_Init(lowpass1d_t *lp, float alpha);
float Lowpass_Update(lowpass1d_t *lp, float in);
void  Lowpass_Reset(lowpass1d_t *lp);

/* 斜坡 (斜率限制): 按最大斜率线性逼近目标, 到位后直接跟随 */
typedef struct {
    float out;      /* 当前输出 */
    float rate;     /* 最大斜率, 单位/秒 */
} ramp_t;

void  Ramp_Init(ramp_t *r, float rate);
float Ramp_Update(ramp_t *r, float target, float dt);
float Ramp_Reset(ramp_t *r, float value);

#endif
