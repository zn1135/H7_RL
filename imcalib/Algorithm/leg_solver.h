#ifndef LEG_SOLVER_H
#define LEG_SOLVER_H

#include <stdint.h>

/* 数学常量 */
#define LEG_PI       3.14159265358979f
#define LEG_2PI      6.28318530717959f
#define LEG_HALF_PI  1.57079632679490f

/* 通用限幅 */
static inline float clampf(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/* 几何参数 */
typedef struct {
    float lu;           /* 上杆长 */
    float lg;           /* 下杆长 */
    float offset_phi0;  /* 摆角零位偏置 */
    uint8_t configured;
} leg_config_t;

/* 髋关节输入 */
typedef struct {
    float hip_f;        /* 前髋角 */
    float hip_b;        /* 后髋角 */
    float d_hip_f;      /* 前髋速度 */
    float d_hip_b;      /* 后髋速度 */
} leg_input_t;

/* 求解输出 */
typedef struct {
    float thigh_angle;              /* 大腿角 (电机+偏置, 不经解算) */
    float virtual_leg_length;       /* 虚拟腿长 |OP| */
    float virtual_leg_angle;        /* 虚拟腿摆角 (相对竖直) */
    float virtual_shank_angle;      /* 虚拟小腿角 (小腿相对大腿) */
    float d_virtual_leg_length;     /* 虚拟腿长速度 */
    float d_virtual_leg_angle;      /* 虚拟腿摆角速度 */
    float d_virtual_shank_angle;    /* 虚拟小腿角速度 */
    float vshank_jac[2];            /* 虚拟小腿雅可比 [0]=b, [1]=a */
    float point_jac[2][2];          /* P点雅可比 */
    float leg_jac[2][2];            /* 腿雅可比 */
    float force_map[2][2];          /* 力矩映射 */
    float force_det;
    uint8_t force_valid;
    uint8_t valid;
} leg_output_t;

/* 求解中间量 */
typedef struct {
    float qf;           /* 前髋角 */
    float qb;           /* 后髋角 */
    float vf;           /* 前髋速度 */
    float vb;           /* 后髋速度 */
    float lu;
    float lg;
    float phi_a;        /* 前杆角度 */
    float phi_b;        /* 后杆角度 */
    float virtual_leg_angle_abs;    /* 虚拟腿摆角绝对值 */
} leg_solver_cache_t;

/* 腿部状态 */
typedef struct {
    leg_config_t config;
    leg_input_t  input;
    leg_output_t output;
} leg_state_t;

void Leg_Init(leg_state_t *leg);
uint8_t Leg_Solve(leg_state_t *leg);
uint8_t Leg_Force_Map_Forward(const leg_state_t *leg, float force,
                              float torque, float output[2]);

#endif
