#ifndef PID_H
#define PID_H

#include <stdint.h>

enum {
    POSITION_PID,
    DELTA_PID,
};

enum {
    NOW,
    LAST,
    LLAST,
};

typedef struct __FFC_t {
    float FFCTar;
    float rin;
    float lastRin;
    float perRin;
    float FFC_pos_out;
    float set_point;
    float num1;
    float num2;
} FFC_t;

typedef struct __pid_t {
    float p;
    float i;
    float d;
    uint8_t pid_mode;
    float MaxOutput;
    float IntegralLimit;
    FFC_t xFeedForward;
    float set[3];
    float get[3];
    float err[3];
    float pout;
    float iout;
    float dout;
    float pos_out;
    float last_pos_out;
    float delta_u;
    float delta_out;
    float last_delta_out;
    float max_err;
    float deadband;
    float vel_err;
    uint8_t angle_wrap;
    void (*f_param_init)(struct __pid_t *pid, uint8_t pid_mode,
                         float maxOutput, float integralLimit,
                         float p, float i, float d);
} pid_t;

void PID_struct_init(pid_t *pid, uint8_t mode, float maxout,
                     float intergral_limit, float kp, float ki, float kd,
                     float ff_param1, float ff_param2);
void FeedForwardParamInit(FFC_t *ffc, float param1, float param2);
float pid_calc(pid_t *pid, float get, float set, float delta_time);
float Deadband_Soften(float err, float deadband);
float Angle_Wrap_180(float deg);
float FeedForwardController(FFC_t *ffc, float target, float num1, float num2);
void abs_limit(float *value, float abs_max);

#endif
