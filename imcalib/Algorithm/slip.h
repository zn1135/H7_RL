#ifndef SLIP_H
#define SLIP_H
#include <stdint.h>

typedef struct {
    float gate;
    float gate_min;
    float confirm_time;
    float clear_time;
    float clear_ratio;
    float blank_time;
} slip_param_t;

typedef struct {
    float velocity;
    float covariance;
    float innovation;
    float limit;
    float noise_scale;
    float blank;
    float confirm;
    float clear;
    uint8_t active;
    uint8_t suspected;
} slip_state_t;

extern slip_param_t slip_param;
void Slip_Reset(slip_state_t *st);
float Slip_Update(slip_state_t *st, float wheel_velocity, float acceleration,
    float dt, float p0, float q, float r, float p_max);
#endif
