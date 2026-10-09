#ifndef GRAVITY_COMP_H
#define GRAVITY_COMP_H

#include "leg_solver.h"
#include "machine_config.h"

typedef struct {
    float world_angle;
    float offset;
    float radius;
    float torque;
    uint8_t clamped;
    uint8_t valid;
} gravity_leg_result_t;

typedef struct {
    gravity_leg_result_t leg[2];
    float delta_dm[4];
    uint8_t valid;
} gravity_comp_result_t;

typedef struct {
    gravity_comp_result_t gravity;
    float spring_dm[4];
    float raw_dm[4];
    uint8_t saturated;
    uint8_t bench;
} compensation_debug_t;

uint8_t Gravity_Comp_Compute(const machine_gravity_cfg_t *cfg,
    float length, float world_angle, gravity_leg_result_t *result);
uint8_t Gravity_Comp_Apply(const leg_state_t *left, const leg_state_t *right,
    const float world_angle[2], const float base_dm[4], float raw_dm[4],
    gravity_comp_result_t *result);

#endif
