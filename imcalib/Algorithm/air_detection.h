#ifndef AIR_DETECTION_H
#define AIR_DETECTION_H

#include "leg_solver.h"
#include "imu_state.h"

typedef struct {
    uint8_t enabled;
    uint8_t control_enabled;
    uint8_t vofa_page;
    float wheel_mass;
    float force_off;
    float force_on;
    float time_off;
    float time_on;
    float sample_timeout;
    float sample_skew;
    float derivative_tau;
    float force_tau;
    float hip_max;
} air_param_t;

typedef struct {
    const imu_state_t *imu;
    const leg_state_t *leg[2];
    float torque[2][2];
    uint64_t rx_ns[2][2];
    uint64_t now_ns;
    uint8_t permit;
} air_input_t;

typedef struct {
    float motor_force;
    float motor_torque;
    float spring_force;
    float gravity_torque;
    float pressure;
    float wheel_acceleration;
    float force_raw;
    float force;
    float height_velocity;
    float height_acceleration;
    uint64_t sample_ns;
} air_leg_t;

typedef struct {
    air_leg_t leg[2];
    float mean_force;
    float elapsed;
    uint64_t imu_seen_ns;
    uint64_t valid_ns;
    uint32_t imu_sequence;
    uint8_t flight;
    uint8_t armed;
    uint8_t valid;
    uint8_t fault;
    uint8_t applied;
} air_detection_t;

extern air_param_t air_param;
extern air_detection_t air_detection;
extern volatile air_detection_t air_debug;
void Air_Detection_Reset(air_detection_t *state);
void Air_Detection_Update(air_detection_t *state, const air_input_t *input);
uint8_t Air_Detection_Active(const air_detection_t *state);

#endif
