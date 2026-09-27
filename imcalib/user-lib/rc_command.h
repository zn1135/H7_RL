#ifndef RC_COMMAND_H
#define RC_COMMAND_H

#include "dr16.h"

/* 摇杆死区 (raw) */
#define RC_DEADBAND_VEL     10u
#define RC_DEADBAND_YAW     20u
#define RC_DEADBAND_LEN     20u
#define RC_DEADBAND_ANG     20u

/* 遥控指令: 去死区归一化 [-1,1] + 拨杆 */
typedef struct {
    float vel;          /* 右摇杆Y 前进 */
    float yaw;          /* 右摇杆X 转向 */
    float len;          /* 拨轮 腿长 */
    float ang;          /* 左摇杆Y 摆角 */
    uint8_t s1;         /* 左拨杆 */
    uint8_t s2;         /* 右拨杆 */
    uint8_t online;
} rc_command_t;

void Rc_Command_Update(rc_command_t *cmd, const dr16_t *rc);

#endif
