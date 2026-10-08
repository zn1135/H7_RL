#ifndef LQR_BALANCE_H
#define LQR_BALANCE_H

#include <stdint.h>

#include "rc_command.h"
#include "imu_state.h"
#include "leg_solver.h"
#include "simple-function.h"
#include "lqr_gain_table.h"

/* 状态序 — 与 MATLAB 模型一致 */
enum {
    LQR_X_S = 0,    /* 前进位移 m */
    LQR_X_DS,       /* 前进速度 m/s */
    LQR_X_PHI,      /* 偏航角 rad */
    LQR_X_DPHI,     /* 偏航角速度 rad/s */
    LQR_X_THL,      /* 左腿前摆角 rad */
    LQR_X_DTHL,     /* 左腿摆角速度 rad/s */
    LQR_X_THR,      /* 右腿前摆角 rad */
    LQR_X_DTHR,     /* 右腿摆角速度 rad/s */
    LQR_X_THB,      /* 机体俯仰角 rad */
    LQR_X_DTHB,     /* 机体俯仰角速度 rad/s */
    LQR_X_NUM,
};

/* 输出序 */
enum {
    LQR_U_WL = 0,   /* 左轮扭矩 N·m */
    LQR_U_WR,       /* 右轮扭矩 N·m */
    LQR_U_BL,       /* 左髋虚拟扭矩 N·m */
    LQR_U_BR,       /* 右髋虚拟扭矩 N·m */
    LQR_U_NUM,
};

typedef struct {
    float   vel_leg_comp_sign; /* 速度补偿 */
    uint8_t yaw_hold;          /* 偏航角环 */
    uint8_t yaw_rate_hold;     /* 偏航角速度环 */
    uint8_t pos_hold;          /* 位移环 (0 = 位移列不进控制) */
    float   acc_fwd_sign;      /* 前向加速度符号 */
    float   pitch_comp_sign;   /* 轮速补偿俯仰项符号 */
    float   pos_arm_vel;       /* 首次积分速度门槛m/s；0不检查速度 */
    uint8_t wheel_enable;      /* 轮通道 */
    uint8_t hip_enable;        /* 髋通道 */
    uint8_t len_pid_enable;    /* 腿长PID */
    uint8_t legacy_gain;       /* 小机旧表对照 */
    float   trq_max_wheel;     /* 轮限幅 */
    float   trq_max_hip;       /* 髋限幅 */
} lqr_debug_t;

typedef struct {
    float x[LQR_X_NUM];             /* 状态 */
    float target[LQR_X_NUM];        /* 目标 */
    float u[LQR_U_NUM];             /* LQR 输出 */
    float u_col[LQR_X_NUM];         /* 左轮各列贡献 (调试) */
    float K[LQR_U_NUM][LQR_X_NUM];  /* 增益 */
    float len[2];                   /* 实测腿长 */
    float len_eval[2];              /* 上次增益求值腿长 */
    float whl[2];                   /* 轮对地角速度 (调试) */
    uint8_t gain_valid;
    uint8_t gain_legacy;
    uint8_t valid;                  /* 状态估计有效 */
    float ds_raw;                   /* 运动学速度 (未滤波) */
    float ds_kf;                    /* 融合速度 */
    float a_fwd;                    /* 前向加速度 m/s² */
    float leg_len_tgt[2];           /* 腿长目标 */
    float pos;                      /* 位移积分 */
    uint8_t pos_armed;              /* 位置基准已锁 */
    float yaw_tgt;                  /* 偏航角目标 */
    float vel_tgt;                  /* 速度目标 */
    float roll;                     /* 机体横滚角 */
    lowpass1d_t lpf_omg_pitch;      /* 俯仰角速度低通 */
    lowpass1d_t lpf_omg_yaw;        /* 偏航角速度低通 */
} lqr_state_t;

extern lqr_debug_t lqr_debug;

void    LQR_Init(lqr_state_t *st);
void    LQR_Velocity_Apply(lqr_state_t *st, float velocity, float dt);
uint8_t LQR_Enable_Latch(lqr_state_t *st, const leg_state_t *leg_l,
                         const leg_state_t *leg_r);
uint8_t LQR_Target_Update(lqr_state_t *st, const rc_command_t *cmd, float dt);
uint8_t LQR_State_Update(lqr_state_t *st, const imu_state_t *imu,
                         const leg_state_t *leg_l, const leg_state_t *leg_r,
                         const float wheel_vel[2], float dt);
void    LQR_Control_Update(lqr_state_t *st);

#endif
