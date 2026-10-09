#ifndef ATTITUDE_ALGORITHM_H
#define ATTITUDE_ALGORITHM_H

#include <stdbool.h>
#include "imu_state.h"

void Attitude_Init(imu_state_t *state);
bool Attitude_Update(imu_state_t *state);
bool Attitude_Accel_Vertical(const imu_state_t *state, float *acceleration);

#endif
