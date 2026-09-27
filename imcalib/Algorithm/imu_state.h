#ifndef IMU_STATE_H
#define IMU_STATE_H

#include <stdint.h>

/* 欧拉角索引 */
enum {
    ATTITUDE_PITCH = 0,
    ATTITUDE_ROLL  = 1,
    ATTITUDE_YAW   = 2,
};

/* IMU 状态 */
typedef struct {
    float quat[4];          /* 四元数 */
    float euler_deg[3];     /* 欧拉角 */
    float euler_rad[3];     /* 弧度 */
    float gyro_rad_s[3];    /* 角速度 */
    float acc_g[3];         /* 加速度 G */
    uint8_t online;         /* 在线 */
    uint32_t last_timestamp_ms;
} imu_state_t;

#endif
