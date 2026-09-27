#include "machine_config.h"

/* 两份电机配置表: 换机器改 machine_config.h 的 MACHINE_DEFAULT */
const machine_cfg_t machine_table[MACHINE_NUM] = {
    [MACHINE_ID_BIG_WHEELLEG] = {
        .name           = "big_wheelleg",
        /* TODO: 为大轮腿生成并接入专用 LQR 增益表，当前增益只针对小轮腿。 */
        .lqr_configured = 0u,
        .dji_type       = 1u,                          /* M3508 + C620 */
        .dji_gear_ratio = 15.5f,                       /* 转子→轮子总减速比 */
        .dji_trq_clamp  = 3.9f,                        /* 对齐训练侧轮关节力矩限幅 */
        .wheel_r        = 0.04f,                       /* 占位，待实测 */
        .dm_pos_max     = 3.14159f,                    /* DM-J8009P: 上位机 ±π */
        .dm_vel_max     = 45.0f,
        .dm_trq_max     = 54.0f,                       /* MIT 刻度, 勿改 */
        .dm_trq_clamp   = 40.0f,                       /* 对齐训练侧腿关节力矩限幅 */
        /* 极性: 前左/后左/前右/后右 */
        .dm_sign        = {{1, 1}, {1, 1}, {-1, -1}, {-1, -1}},
        .dji_sign       = {{1, 1}, {-1, -1}},
        /* 总线: 腿 4 台全在 FDCAN1, 轮在 FDCAN3 */
        .dm_bus         = {1, 1, 1, 1},
        .dji_bus        = 3,
        /* 电机零点: 前左/后左/前右/后右 (作者标定) */
        .dm_zero        = {0.476998f, 1.974491f,0.476998f, 1.974491f },
        /* 腿几何: 杆长 0.21/0.25; 腿长区间为实测工作区间 */
        .leg_lu         = 0.21f,
        .leg_lg         = 0.25f,
        .leg_len_min    = 0.14f,
        .leg_len_max    = 0.34f,
        .leg_off_phi0   = {-0.13f, -0.07f},
        .gas_spring_force_n = {150.0f, 150.0f},
        .gas_comp_sign = {0, 0},                     /* 左右符号待台架 */
        /* IMU: 照抄原 hi229.h 全局宏, 大机器待实测 */
        .imu = {
            .eul_src   = {1, 0, 2}, 
            .eul_sign  = {1, 1, -1},
            .gyr_sign  = {1, 1, -1}, /* RL 训练轴: 低头 +pitch、左滚 +roll、左偏航 -yaw */
            .acc_sign  = {-1, 1, -1},
            .quat_src  = {0, 1 , 2},    /* 四元数 X/Y 通道交换; 这里只选通道 */
            .quat_sign = {-1, 1, -1},  /* quat_src 后独立修正训练极性 */
        },
        /* RL 关节映射候选 A (作者 2026-09-23 定, 依据变更 97 URDF 推导):
         * lf0 = −(thigh − 2.476872), lf1 = −(vs − 3.086386), 右腿反号; 轮 sign 按训练轴/台架后退方向校正
         * 默认站姿固件应读 thigh ≈ 2.54 / vs ≈ 2.99 (VOFA ch11/ch13), 不符则回看候选 B */
        .rl = {
            .sign       = {-1, -1, -1, 1, 1, 1},
            .zero       = {2.476872f, 3.086386f, 2.476872f, 3.086386f},
            .configured = 1u,
        },
    },
    [MACHINE_ID_SMALL_WHEELLEG] = {
        .name           = "small_wheelleg",
        /* 小轮腿目前只使用 LQR。TODO: 训练小轮腿 RL 模型后再配置关节映射并启用。 */
        .lqr_configured = 1u,
        .dji_type       = 0u,                          /* M2006 */
        .dji_gear_ratio = 36.0f,
        .dji_trq_clamp  = 1.8f,                        /* 满限幅 = 0.18 Nm/A × 10A */
        .wheel_r        = 0.04f,                       /* Leg2_v1 WHEEL_R */
        .dm_pos_max     = 3.14159f,                    /* DM-J4310 */
        .dm_vel_max     = 30.0f,
        .dm_trq_max     = 10.0f,
        .dm_trq_clamp   = 10.0f,
        .dm_sign        = {{1, 1}, {1, 1}, {-1, -1}, {-1, -1}},
        .dji_sign       = {{1, 1}, {-1, -1}},
        /* 总线: 左腿 FDCAN1, 右腿 FDCAN3, 轮 FDCAN2 */
        .dm_bus         = {1, 1, 3, 3},
        .dji_bus        = 2,
        /* 电机零点: 前左/后左/前右/后右 (本机原值) */
        .dm_zero        = {-0.056f, 0.296f, 0.024f, 0.316f},
        /* 腿几何 (本机原值) */
        .leg_lu         = 0.13087f,
        .leg_lg         = 0.15240f,
        .leg_len_min    = 0.13f,
        .leg_len_max    = 0.23f,
        .leg_off_phi0   = {-0.0f, -0.0f},
        .gas_spring_force_n = {0.0f, 0.0f},
        .gas_comp_sign = {0, 0},
        /* IMU */
        .imu = {
            .eul_src   = {1, 0, 2},
            .eul_sign  = {-1,-1, -1},
            .gyr_sign  = {-1,-1, -1},
            .acc_sign  = {-1, 1, -1},
            .quat_src  = {0, 1, 2},
            .quat_sign = {-1, 1, -1},
        },
        /* RL 关节映射: 模型对应大机器, 小机器未配置 */
        .rl = {
            .sign       = {0, 0, 0, 0, 0, 0},
            .zero       = {0.0f, 0.0f, 0.0f, 0.0f},
            .configured = 0u,
        },
    },
};

const machine_cfg_t *const machine = &machine_table[MACHINE_DEFAULT];

/* 当前机器号 */
uint8_t Machine_Id(void)
{
    return (uint8_t)(machine - machine_table);
}
