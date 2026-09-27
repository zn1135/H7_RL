#ifndef __MACHINE_CONFIG_H
#define __MACHINE_CONFIG_H

#include <stdint.h>

/* 机器编号 */
#define MACHINE_ID_BIG_WHEELLEG    0u
#define MACHINE_ID_SMALL_WHEELLEG  1u
#define MACHINE_NUM               2u

/* 电机数量 (须与 dm.c / dji.c 的枚举一致) */
#define MACHINE_LEG_NUM           4u
#define MACHINE_WHEEL_NUM         2u

/* 上电默认机器: 底层机型参数只改这一行。
 * LQR 增益表仅针对小轮腿，RL 模型仅针对大轮腿；控制器不能仅靠本宏互换。
 * FDCAN1/3 的数据阶段时序随机器切换；FDCAN2 保持 CubeMX 设置。 */
#ifndef MACHINE_DEFAULT
#define MACHINE_DEFAULT           MACHINE_ID_BIG_WHEELLEG
#endif

#if MACHINE_DEFAULT == MACHINE_ID_BIG_WHEELLEG
#define MACHINE_TIM6_PERIOD       1999u   /* 500 Hz */
#define MACHINE_CTRL_DT           0.002f
#define MACHINE_POLICY_DIV        5u      /* 策略节拍 500/5 = 100 Hz */
#define MACHINE_VOFA_PORT         1u      /* USART1 */
#define MACHINE_FDCAN13_DATA_PRESCALER  1u
#define MACHINE_FDCAN13_DATA_SEG1       4u
#define MACHINE_FDCAN13_DATA_SEG2       1u
#elif MACHINE_DEFAULT == MACHINE_ID_SMALL_WHEELLEG
#define MACHINE_TIM6_PERIOD       999u    /* 1 kHz */
#define MACHINE_CTRL_DT           0.001f
#define MACHINE_POLICY_DIV        10u     /* 策略节拍 1000/10 = 100 Hz */
#define MACHINE_VOFA_PORT         8u      /* UART8 */
#define MACHINE_FDCAN13_DATA_PRESCALER  3u
#define MACHINE_FDCAN13_DATA_SEG1       5u
#define MACHINE_FDCAN13_DATA_SEG2       2u
#else
#error "Unsupported MACHINE_DEFAULT"
#endif

/* 一路电机的极性 */
typedef struct {
    int8_t fb;      /* 反馈极性 */
    int8_t out;     /* 输出极性 */
} motor_sign_t;

/* IMU 安装: 机体三轴各取模块哪一路、乘什么符号 (输出 = 原始值 × 符号) */
typedef struct {
    uint8_t eul_src[3];   /* 欧拉角来源, 按机体 俯仰/横滚/偏航 序; 值 = 模块 0 Roll / 1 Pitch / 2 Yaw */
    int8_t  eul_sign[3];  /* 欧拉角符号, 同序 */
    int8_t  gyr_sign[3];  /* 角速度符号, 模块 X/Y/Z */
    int8_t  acc_sign[3];  /* 加速度符号, 模块 X/Y/Z */
    uint8_t quat_src[3];  /* 四元数输出 X/Y/Z 分别取模块 X/Y/Z 哪一路 */
    int8_t  quat_sign[3]; /* 四元数输出 X/Y/Z 极性, 独立于 quat_src */
} imu_cfg_t;

/* RL 关节映射: 训练关节 = sign × wrap(固件角 − zero); 速度、动作、力矩乘同一 sign
 * sign 序 [左大腿 左小腿 左轮 右大腿 右小腿 右轮]; zero 只有四个腿关节 (同序去掉轮) */
typedef struct {
    int8_t  sign[6];      /* 符号 */
    float   zero[4];      /* 零位 */
    uint8_t configured;   /* 训练侧定义 + 台架核对后置 1 */
} rl_map_t;

/* 一台机器的全部参数 */
typedef struct {
    const char *name;
    uint8_t     lqr_configured;     /* 此机器有匹配的 LQR 增益表 */
    /* 轮: 型号(0=M2006, 1=M3508) + 总传动比 + 满限幅力矩(Nm) */
    uint8_t     dji_type;
    float       dji_gear_ratio;
    float       dji_trq_clamp;
    float       wheel_r;
    /* 腿: MIT 三个满量程 + 满限幅力矩(Nm) */
    float       dm_pos_max;
    float       dm_vel_max;
    float       dm_trq_max;
    float       dm_trq_clamp;
    /* 极性: 腿 4 台 (前左/后左/前右/后右), 轮 2 个 (左/右) */
    motor_sign_t dm_sign[MACHINE_LEG_NUM];
    motor_sign_t dji_sign[MACHINE_WHEEL_NUM];
    /* 总线号: 1=FDCAN1 2=FDCAN2 3=FDCAN3 */
    uint8_t     dm_bus[MACHINE_LEG_NUM];
    uint8_t     dji_bus;
    /* 电机零点: 4 台腿 (前左/后左/前右/后右), 在 dm.c 解码时叠加 */
    float       dm_zero[MACHINE_LEG_NUM];
    /* 腿几何: 杆长 + 腿长工作区间 + 腿摆角零位 (左/右) */
    float       leg_lu;
    float       leg_lg;
    float       leg_len_min;
    float       leg_len_max;
    float       leg_off_phi0[2];
    float       gas_spring_force_n[2]; /* 左右轴向力 */
    int8_t      gas_comp_sign[2];      /* -1/0/+1 */
    /* IMU 安装极性 */
    imu_cfg_t   imu;
    /* RL 关节映射 */
    rl_map_t    rl;
} machine_cfg_t;

extern const machine_cfg_t machine_table[MACHINE_NUM];
extern const machine_cfg_t *const machine; /* 编译期选定的机器 */

uint8_t Machine_Id(void);

#endif
