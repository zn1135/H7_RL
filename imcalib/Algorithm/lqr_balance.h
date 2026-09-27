#ifndef LQR_BALANCE_H
#define LQR_BALANCE_H

#include <stdint.h>

#include "rc_command.h"
#include "imu_state.h"
#include "leg_solver.h"
#include "simple-function.h"
#include "kalman.h"

/* 状态序 — 与 MATLAB 模型一致 */
enum {
    LQR_X_S = 0,    /* 前进位移 m */
    LQR_X_DS,       /* 前进速度 m/s */
    LQR_X_PHI,      /* 偏航角 rad (不参与控制) */
    LQR_X_DPHI,     /* 偏航角速度 rad/s */
    LQR_X_THL,      /* 左腿摆角-世界系 rad */
    LQR_X_DTHL,     /* 左腿摆角速度 rad/s */
    LQR_X_THR,      /* 右腿摆角-世界系 rad */
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

/* K 表拟合域 */
#define LQR_K_LEN_MIN       0.13f
#define LQR_K_LEN_MAX       0.23f

/* 遥控量程 */
#define LQR_RC_VEL_MAX      1.2f    /* m/s */
#define LQR_RC_YAW_MAX      5.0f    /* rad/s */
#define LQR_RC_LEN_RATE     0.3f    /* m/s */

typedef struct {
    float   vel_leg_comp_sign; /* 速度补偿 */
    uint8_t vel_src;           /* 0 低通 1 卡尔曼 */
    uint8_t yaw_hold;          /* 偏航角环 */
    uint8_t yaw_rate_hold;     /* 偏航角速度环 */
    uint8_t pos_hold;          /* 位移环 (0 = 位移列不进控制) */
    float   vel_ramp;          /* 速度目标斜坡 m/s^2 (0 = 不斜坡, 阶跃) */
    float   acc_fwd_sign;      /* 前向加速度符号 */
    float   pitch_comp_sign;   /* 轮速补偿俯仰项符号 */
    float   pos_arm_vel;       /* 松杆后车速低于此才积位移 m/s (0 = 立即) */
    uint8_t wheel_enable;      /* 轮通道 */
    uint8_t hip_enable;        /* 髋通道 */
    uint8_t len_pid_enable;    /* 腿长PID */
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
    uint8_t valid;                  /* 状态估计有效 */
    float ds_raw;                   /* 运动学速度 (未滤波) */
    float ds_lpf;                   /* 低通速度 */
    float ds_kf;                    /* 卡尔曼速度 */
    float a_fwd;                    /* 前向加速度 m/s² */
    float leg_len_tgt[2];           /* 腿长目标 */
    float pos;                      /* 位移积分 */
    uint8_t pos_armed;              /* 位移积分已启动 */
    float yaw_tgt;                  /* 偏航角目标 */
    float vel_tgt;                  /* 速度目标 (斜坡后) */
    float roll;                     /* 机体横滚角 */
    lowpass1d_t lpf_vel;            /* 速度低通 */
    lowpass1d_t lpf_omg_pitch;      /* 俯仰角速度低通 */
    lowpass1d_t lpf_omg_yaw;        /* 偏航角速度低通 */
    kalman_accel_t kf_vel;          /* 速度卡尔曼 */
} lqr_state_t;

extern lqr_debug_t lqr_debug;

void    LQR_Init(lqr_state_t *st);
uint8_t LQR_Enable_Latch(lqr_state_t *st, const leg_state_t *leg_l,
                         const leg_state_t *leg_r);
uint8_t LQR_Target_Update(lqr_state_t *st, const rc_command_t *cmd, float dt);
uint8_t LQR_State_Update(lqr_state_t *st, const imu_state_t *imu,
                         const leg_state_t *leg_l, const leg_state_t *leg_r,
                         const float wheel_vel[2], float dt);
void    LQR_Control_Update(lqr_state_t *st);

#endif
