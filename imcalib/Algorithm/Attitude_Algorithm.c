#include "Attitude_Algorithm.h"
#include "machine_config.h"
#include <math.h>
#include <string.h>

#define ATTITUDE_DEG_TO_RAD  0.01745329251994f
#define ATTITUDE_QUAT_NORM_MIN  0.001f

/* 世界竖直加速度 */
bool Attitude_Accel_Vertical(const imu_state_t *state, float *acceleration)
{
    float r20;
    float r21;
    float r22;
    float specific;
    float norm;
    const float *q;

    if (acceleration == NULL)
    {
        return false;
    }
    *acceleration = 0.0f;
    if (state == NULL || !state->online)
    {
        return false;
    }
    q = state->quat;
    norm = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (!isfinite(norm) || fabsf(norm - 1.0f) > 0.01f)
    {
        return false;
    }
    r20 = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    r21 = 2.0f * (q[2] * q[3] + q[0] * q[1]);
    r22 = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
#if MACHINE_DEFAULT == MACHINE_ID_BIG_WHEELLEG
    specific = r20 * state->acc_g[0] - r21 * state->acc_g[1] - r22 * state->acc_g[2];
    *acceleration = (specific - 1.0f) * 9.81f;
#else
    specific = r20 * state->acc_g[0] + r21 * state->acc_g[1] + r22 * state->acc_g[2];
    *acceleration = -(specific + 1.0f) * 9.81f;
#endif
    return isfinite(*acceleration);
}

/* 初始化姿态 */
void Attitude_Init(imu_state_t *state)
{
    if (state == NULL) return;
    memset(state, 0, sizeof(*state));
    state->quat[0] = 1.0f;
}

/* 四元数归一化 + 欧拉角转 rad */
bool Attitude_Update(imu_state_t *state)
{
    uint8_t axis;
    float q_norm;

    if (state == NULL) return false;

    /* 归一化四元数 */
    q_norm = sqrtf(state->quat[0] * state->quat[0]
        + state->quat[1] * state->quat[1]
        + state->quat[2] * state->quat[2]
        + state->quat[3] * state->quat[3]);
    if (!isfinite(q_norm) || q_norm < ATTITUDE_QUAT_NORM_MIN)
        return false;

    for (axis = 0u; axis < 4u; axis++)
        state->quat[axis] /= q_norm;

    /* 欧拉角 deg → rad */
    for (axis = 0u; axis < 3u; axis++)
        state->euler_rad[axis] = state->euler_deg[axis] * ATTITUDE_DEG_TO_RAD;

    return true;
}
