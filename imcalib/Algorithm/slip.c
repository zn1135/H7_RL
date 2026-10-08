/* 两状态速度补偿：加速度观测、轮速异常降权，不判打滑状态。 */
#include "slip.h"
#include <math.h>
#include <string.h>

#define SLIP_NOISE_DT 0.001f

slip_param_t slip_param = {
    .p0_velocity     = 0.1f,   /* (m/s)² */
    .p0_acceleration = 0.25f,  /* (m/s²)² */
    .q_velocity      = 0.007f, /* 每1ms */
    .q_acceleration  = 0.01f,  /* 每1ms */
    .r_velocity      = 0.01f,  /* (m/s)² */
    .r_acceleration  = 0.25f,  /* (m/s²)² */
    .gate            = 3.0f,   /* σ倍数 */
    .gate_min        = 0.4f,   /* m/s */
};

/* 重新等待有效先验 */
void Slip_Reset(slip_state_t *st)
{
    if (st != NULL)
    {
        memset(st, 0, sizeof(*st));
    }
}

/* 当前运动学量建先验 */
void Slip_Init(slip_state_t *st, float v0, float a0)
{
    if (st == NULL)
    {
        return;
    }
    Slip_Reset(st);
    if (!isfinite(v0) || !isfinite(a0)
        || !(slip_param.p0_velocity >= 0.0f)
        || !(slip_param.p0_acceleration >= 0.0f))
    {
        return;
    }
    st->velocity = v0;
    st->acceleration = a0;
    st->covariance[0][0] = slip_param.p0_velocity;
    st->covariance[1][1] = slip_param.p0_acceleration;
    st->noise_scale = 1.0f;
    st->limit = fmaxf(slip_param.gate_min,
        slip_param.gate * sqrtf(st->covariance[0][0] + slip_param.r_velocity));
    st->initialized = 1u;
    st->valid = (uint8_t)(isfinite(st->limit) && st->limit > 0.0f);
}

/* 单观测Joseph修正 */
static uint8_t Slip_Correct(slip_state_t *st, uint8_t axis,
                            float innovation, float noise)
{
    float gain[2];
    float transform[2][2];
    float result[2][2];
    float variance;
    float cross;
    uint8_t i;
    uint8_t j;
    uint8_t m;
    uint8_t n;

    variance = st->covariance[axis][axis] + noise;
    if (!(noise > 0.0f) || !isfinite(variance) || variance <= 0.0f)
    {
        return 0u;
    }
    for (i = 0u; i < 2u; i++)
    {
        gain[i] = st->covariance[i][axis] / variance;
        for (j = 0u; j < 2u; j++)
        {
            transform[i][j] = (i == j ? 1.0f : 0.0f)
                - (j == axis ? gain[i] : 0.0f);
        }
    }
    st->velocity += gain[0] * innovation;
    st->acceleration += gain[1] * innovation;
    for (i = 0u; i < 2u; i++)
    {
        for (j = 0u; j < 2u; j++)
        {
            result[i][j] = gain[i] * noise * gain[j];
            for (m = 0u; m < 2u; m++)
            {
                for (n = 0u; n < 2u; n++)
                {
                    result[i][j] += transform[i][m] * st->covariance[m][n]
                        * transform[j][n];
                }
            }
        }
    }
    cross = 0.5f * (result[0][1] + result[1][0]);
    result[0][1] = cross;
    result[1][0] = cross;
    memcpy(st->covariance, result, sizeof(result));
    return (uint8_t)(isfinite(st->velocity) && isfinite(st->acceleration)
        && isfinite(result[0][0]) && isfinite(result[1][1]) && isfinite(cross)
        && result[0][0] >= 0.0f && result[1][1] >= 0.0f);
}

/* 预测、加速度修正、轮速降权修正 */
uint8_t Slip_Update(slip_state_t *st, float wheel_v, float imu_acc, float dt)
{
    float p00;
    float p01;
    float p10;
    float p11;
    float scale;
    float ratio;
    float noise;

    if (st == NULL)
    {
        return 0u;
    }
    st->valid = 0u;
    if (!isfinite(wheel_v) || !isfinite(imu_acc) || !isfinite(dt) || dt <= 0.0f
        || !(slip_param.q_velocity >= 0.0f) || !(slip_param.q_acceleration >= 0.0f)
        || !(slip_param.r_velocity > 0.0f) || !(slip_param.r_acceleration > 0.0f)
        || !(slip_param.gate > 0.0f) || !(slip_param.gate_min > 0.0f))
    {
        Slip_Reset(st);
        return 0u;
    }
    if (!st->initialized)
    {
        Slip_Init(st, wheel_v, imu_acc);
        return st->valid;
    }
    scale = dt / SLIP_NOISE_DT;
    p00 = st->covariance[0][0] + dt * (st->covariance[0][1] + st->covariance[1][0])
        + dt * dt * st->covariance[1][1] + slip_param.q_velocity * scale;
    p01 = st->covariance[0][1] + dt * st->covariance[1][1];
    p10 = st->covariance[1][0] + dt * st->covariance[1][1];
    p11 = st->covariance[1][1] + slip_param.q_acceleration * scale;
    st->velocity += st->acceleration * dt;
    st->covariance[0][0] = p00;
    st->covariance[0][1] = p01;
    st->covariance[1][0] = p10;
    st->covariance[1][1] = p11;
    if (!Slip_Correct(st, 1u, imu_acc - st->acceleration, slip_param.r_acceleration))
    {
        Slip_Reset(st);
        return 0u;
    }
    st->innovation = wheel_v - st->velocity;
    st->limit = fmaxf(slip_param.gate_min,
        slip_param.gate * sqrtf(st->covariance[0][0] + slip_param.r_velocity));
    ratio = fabsf(st->innovation) / st->limit;
    st->noise_scale = fmaxf(1.0f, ratio * ratio);
    noise = slip_param.r_velocity * st->noise_scale;
    if (!isfinite(st->limit) || st->limit <= 0.0f
        || !Slip_Correct(st, 0u, st->innovation, noise))
    {
        Slip_Reset(st);
        return 0u;
    }
    st->valid = 1u;
    return 1u;
}
