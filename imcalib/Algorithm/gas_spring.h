#ifndef GAS_SPRING_H
#define GAS_SPRING_H

#include <stdint.h>
#include "leg_solver.h"

#ifndef GAS_SPRING_COMP_ENABLE
#define GAS_SPRING_COMP_ENABLE 1
#endif

float Leg_SpringF(float L0);
uint8_t Gas_Spring_Apply(const leg_state_t *left, const leg_state_t *right,
                         const float base_dm[4], float raw_dm[4]);

#endif
