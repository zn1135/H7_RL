#ifndef __DM_H
#define __DM_H

#include "main.h"
#include "can_bus.h"
#include <stdbool.h>

/* 电机序 */
typedef enum {
    DM_MOTOR_LEG_F_LFT = 0,
    DM_MOTOR_LEG_B_LFT,
    DM_MOTOR_LEG_F_RGT,
    DM_MOTOR_LEG_B_RGT,
    DM_MOTOR_NUM,
} dm_motor_idx_t;

/* 命令码 */
typedef enum {
    DM_CMD_CLEAR_ERROR = 0xFBu,
    DM_CMD_ENABLE      = 0xFCu,
    DM_CMD_DISABLE     = 0xFDu,
    DM_CMD_SET_ZERO    = 0xFEu,
} dm_cmd_t;

/* 编码值 */
#define DM_ANGLE_CPR        65536L
#define DM_MIT_FIELD_MAX    0x0FFFu

/* MIT量程: 见 machine_config.h (dm.c 内引用) */

/* 超时值 */
#define DM_OFFLINE_MS       10u

/* 固定参 (与 dji 同结构) */
typedef motor_cfg_t dm_motor_config_t;

/* 反馈值 */
typedef struct {
    volatile uint8_t  raw_data[8];
    uint16_t          angle_raw;
    uint16_t          vel_raw;
    uint16_t          trq_raw;
    uint8_t           err_raw;
    uint8_t           motor_id;
    uint8_t           temp_mos;
    uint8_t           temp_rotor;
    int32_t           angle_total;
    float             pos_rad;          /* 解码角 */
    float             pos_zero_rad;     /* 加零点 */
    float             vel_rad_s;
    float             trq_nm;
    volatile uint8_t  raw_pending;
    volatile uint8_t  rx_seen;
    volatile uint32_t last_rx_tick;
} dm_motor_feedback_t;

extern const dm_motor_config_t dm_motor_config[DM_MOTOR_NUM];
extern dm_motor_feedback_t dm_motor_feedback[DM_MOTOR_NUM];

void Dm_Init(void);
void Dm_Parse(void);
bool Dm_Is_Online(uint8_t index);
bool Dm_Is_Enabled(uint8_t index);
bool Dm_Has_Fault(uint8_t index);
void Dm_Enable_Watchdog(void);
void Dm_Disable_Watchdog(void);
float Dm_Uint_To_Float(uint16_t value, float min, float max, uint8_t bits);
uint16_t Dm_Float_To_Uint(float value, float min, float max, uint8_t bits);
HAL_StatusTypeDef Dm_Mit_Control(uint8_t index, uint16_t angle_raw,
                                 uint16_t vel_raw, uint16_t kp_raw,
                                 uint16_t kd_raw, uint16_t trq_raw);
HAL_StatusTypeDef Dm_Send_Command(uint8_t index, uint8_t command);
HAL_StatusTypeDef Dm_All_Enable(void);
HAL_StatusTypeDef Dm_All_Disable(void);
HAL_StatusTypeDef Dm_Send_Zero(void);
HAL_StatusTypeDef Dm_Send_Torque(const float torque[DM_MOTOR_NUM]);

#endif
