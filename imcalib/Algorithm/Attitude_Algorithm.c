#include "Attitude_Algorithm.h"
#include <math.h>
#include <string.h>

#define ATTITUDE_DEG_TO_RAD  0.01745329251994f
#define ATTITUDE_QUAT_NORM_MIN  0.001f

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
