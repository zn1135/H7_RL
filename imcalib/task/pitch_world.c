// 版本：v1.2  日期：2026-10-05
#include "pitch_world.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define PITCH_WORLD_PI                 3.14159265358979323846f
#define PITCH_WORLD_2PI                6.28318530717958647692f
#define PITCH_WORLD_HALF_PI            1.57079632679489661923f

/* 重力判向补角 */
bool Pitch_World_Calc(float pitch, const float quat[4], float *pitch_world)
{
    float q[4];
    float scale;
    float norm;
    float gravity_z;
    float angle;
    uint8_t i;

    if (quat == NULL || pitch_world == NULL || !isfinite(pitch)
        || fabsf(pitch) > PITCH_WORLD_HALF_PI)
    {
        return false;
    }

    scale = 0.0f;
    for (i = 0u; i < 4u; i++)
    {
        if (!isfinite(quat[i]))
        {
            return false;
        }
        scale = fmaxf(scale, fabsf(quat[i]));
    }
    if (scale == 0.0f)
    {
        return false;
    }

    /* 缩放防溢出 */
    norm = 0.0f;
    for (i = 0u; i < 4u; i++)
    {
        q[i] = quat[i] / scale;
        norm += q[i] * q[i];
    }
    norm = sqrtf(norm);
    for (i = 0u; i < 4u; i++)
    {
        q[i] /= norm;
    }

    /* 重力转机体 */
    gravity_z = 2.0f * (q[1] * q[1] + q[2] * q[2]) - 1.0f;
    angle = pitch;
    if (gravity_z > 0.0f)
    {
        if (pitch >= 0.0f)
        {
            angle = PITCH_WORLD_PI - pitch;
        }
        else
        {
            angle = -PITCH_WORLD_PI - pitch;
        }
    }
    if (angle >= PITCH_WORLD_PI)
    {
        angle -= PITCH_WORLD_2PI;
    }
    *pitch_world = angle;
    return true;
}
