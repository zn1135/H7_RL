#include "simple-function.h"

/* ================= 一阶低通 ================= */

/* 初始化 */
void Lowpass_Init(lowpass1d_t *lp, float alpha)
{
    lp->out = 0.0f;
    lp->alpha = alpha;
}

/* 单周期: y = a*x + (1-a)*y */
float Lowpass_Update(lowpass1d_t *lp, float in)
{
    lp->out = lp->alpha * in + (1.0f - lp->alpha) * lp->out;
    return lp->out;
}

/* 清零 */
void Lowpass_Reset(lowpass1d_t *lp)
{
    lp->out = 0.0f;
}

/* ================= 斜坡 (斜率限制) ================= */

/* 初始化 */
void Ramp_Init(ramp_t *r, float rate)
{
    r->out = 0.0f;
    if (rate < 0.0f)
    {
        rate = -rate;
    }
    r->rate = rate;
}

/* 单周期: 每个 dt 最多走 rate*dt, 到目标即等于目标 */
float Ramp_Update(ramp_t *r, float target, float dt)
{
    float step;

    step = r->rate * dt;
    if ((target - r->out) > step)
    {
        r->out += step;
    }
    else if ((target - r->out) < -step)
    {
        r->out -= step;
    }
    else
    {
        r->out = target;
    }
    return r->out;
}

/* 直接置值 (初始化/复位用, 也用于第一次上电对齐实测值) */
float Ramp_Reset(ramp_t *r, float value)
{
    r->out = value;
    return r->out;
}
