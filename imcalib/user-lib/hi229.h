#ifndef __HI229_H
#define __HI229_H

#include "main.h"
#include <stdbool.h>

/* ============================================================================
 * 协议参数
 * ==========================================================================*/
#define HI229_HEADER_0        0x5A
#define HI229_HEADER_1        0xA5
#define HI229_FRAME_LEN       82u     /* 头(2) + 长度(2) + CRC(2) + payload(76) */
#define HI229_PAYLOAD_LEN     76u     /* hi229_frame_t 大小 */
#define HI229_OFFLINE_MS      100u

/* 轴向符号已移到 machine_config.c 的 imu 表, 驱动只出原始值 */

/* ============================================================================
 * 协议帧结构 — HI229 0x91 包 payload（76字节）
 * ==========================================================================*/
typedef struct __attribute__((packed)) {
    uint8_t  tag;       /* 包标识：0x91 */
    uint8_t  id;        /* 设备 ID */
    uint8_t  rev[2];    /* 保留 */
    float    prs;       /* 气压 (Pa) */
    uint32_t ts;        /* 时间戳 (ms) */
    float    acc[3];    /* 加速度 (G) */
    float    gyr[3];    /* 角速度 (deg/s) */
    float    mag[3];    /* 磁场 (uT) */
    float    eul[3];    /* 欧拉角 (deg) Roll/Pitch/Yaw */
    float    quat[4];   /* 四元数 W/X/Y/Z */
} hi229_frame_t;

typedef char hi229_frame_size_must_match_payload_len[
    (sizeof(hi229_frame_t) == HI229_PAYLOAD_LEN) ? 1 : -1];

/* ============================================================================
 * 数组索引枚举
 * ==========================================================================*/
typedef enum {
    HI229_ROLL  = 0,
    HI229_PITCH = 1,
    HI229_YAW   = 2,
} hi229_eul_idx_t;

typedef enum {
    HI229_ACC_X = 0,
    HI229_ACC_Y = 1,
    HI229_ACC_Z = 2,
} hi229_acc_idx_t;

typedef enum {
    HI229_QUAT_W = 0,
    HI229_QUAT_X = 1,
    HI229_QUAT_Y = 2,
    HI229_QUAT_Z = 3,
} hi229_quat_idx_t;

/* ============================================================================
 * HI229 输出数据
 * ==========================================================================*/
typedef struct {
    float    eul[3];    /* 欧拉角 deg */
    float    acc[3];    /* 加速度 G */
    float    gyr[3];    /* 角速度 deg/s */
    float    quat[4];   /* 四元数 */
    uint32_t ts;        /* IMU 时间戳 */
    bool     online;
    uint32_t last_rx_tick;
} hi229_data_t;

extern hi229_data_t hi229_data;

/* ============================================================================
 * 对外接口
 * ==========================================================================*/
void       HI229_Init(void);
void       HI229_Process(void);
bool       HI229_Online(void);
float      HI229_Deadline(float input, float deadline);
hi229_data_t HI229_Snapshot(void);

#endif
