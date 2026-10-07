#include "slip.h"
#include <math.h>
#include <string.h>

slip_param_t slip_param = {
    .gate = 3.0f,              /* σ倍数 */
    .gate_min = 0.4f,          /* 门限 m/s */
    .confirm_time = 0.04f,     /* 确认 s */
    .clear_time = 0.15f,       /* 恢复 s */
    .clear_ratio = 0.6f,       /* 迟滞比例 */
    .blank_time = 0.1f,        /* 屏蔽 s */
};

void Slip_Reset(slip_state_t *st)
{
    memset(st, 0, sizeof(*st));
}

/* 标志确认 */
static void Slip_Detect(slip_state_t *st, float dt)
{
    if (st->blank > 0.0f)
    {
        st->blank = fmaxf(0.0f, st->blank - dt);
        return;
    }
    if (slip_param.gate <= 0.0f)
    {
        st->suspected = 0u;
        st->confirm = 0.0f;
        st->clear = 0.0f;
        return;
    }
    if (st->noise_scale > 1.0f)
    {
        st->confirm += dt;
        st->clear = 0.0f;
        if (st->confirm >= slip_param.confirm_time)
        {
            st->suspected = 1u;
        }
    }
    else
    {
        st->confirm = 0.0f;
        if (fabsf(st->innovation) < st->limit * slip_param.clear_ratio)
        {
            st->clear += dt;
            if (st->clear >= slip_param.clear_time)
            {
                st->suspected = 0u;
            }
        }
        else
        {
            st->clear = 0.0f;
        }
    }
}

/* 预测与降权融合 */
float Slip_Update(slip_state_t *st, float wheel_velocity, float acceleration,
    float dt, float p0, float q, float r, float p_max)
{
    float ratio;
    float gain;

    if (!st->active)
    {
        Slip_Reset(st);
        st->covariance = p0;
        st->blank = slip_param.blank_time;
        st->active = 1u;
    }
    st->velocity += acceleration * dt;
    st->covariance = fminf(st->covariance + q, p_max);
    st->innovation = wheel_velocity - st->velocity;
    st->noise_scale = 1.0f;
    st->limit = fmaxf(slip_param.gate_min,
        slip_param.gate * sqrtf(st->covariance + r));
    if (slip_param.gate > 0.0f)
    {
        ratio = fabsf(st->innovation) / st->limit;
        if (ratio > 1.0f)
        {
            st->noise_scale = ratio * ratio;
        }
    }
    gain = st->covariance / (st->covariance + r * st->noise_scale);
    st->velocity += gain * st->innovation;
    st->covariance *= 1.0f - gain;
    Slip_Detect(st, dt);
    return st->velocity;
}
