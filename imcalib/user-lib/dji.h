#ifndef __DJI_H
#define __DJI_H

#include "main.h"
#include "can_bus.h"
#include <stdbool.h>

/* 电机型 */
typedef enum {
    DJI_M2006 = 0,
    DJI_M3508 = 1,
} dji_motor_type_t;

#define DJI_ANGLE_CPR          8192L
#define DJI_OFFLINE_MS         10u

/* 电机固有参数 (按型号, 与机器无关) */
#define DJI_CURRENT_MAX_M2006  10000
#define DJI_CURRENT_MAX_M3508  16384
/* 满电流时输出轴堵转力矩 (在"标准减速比"下) */
#define DJI_NM_FULL_M2006      (0.18f * 10.0f)
#define DJI_NM_FULL_M3508      (0.30f * 20.0f)
/* 标准减速比 (实机总比见机器表 dji_gear_ratio, 按比例缩放) */
#define DJI_RATIO_STD_M2006    36.0f
#define DJI_RATIO_STD_M3508    19.2f

/* 型号与减速比: 见 machine_config.c (按机器选择) */

/* 电机序 */
#define DJI_MOTOR_WHEEL_LFT    0u
#define DJI_MOTOR_WHEEL_RGT    1u
#define DJI_MOTOR_NUM          2u

/* 固定参 (与 dm 同结构) */
typedef motor_cfg_t dji_motor_config_t;

/* 反馈值 */
typedef struct {
    volatile uint8_t raw_data[8];
    uint16_t         angle_raw;
    int16_t          vel_raw;
    int16_t          current_raw;
    int8_t           temp_raw;
    int32_t          angle_total;
    float            angle_rad;
    float            angle_total_rad;
    float            vel_rad_s;
    volatile uint8_t raw_pending;
    volatile uint8_t rx_seen;
    uint8_t          angle_pending;
    volatile uint32_t last_rx_tick;
} dji_motor_feedback_t;

extern const dji_motor_config_t dji_motor_config[DJI_MOTOR_NUM];
extern dji_motor_feedback_t dji_motor_feedback[DJI_MOTOR_NUM];
extern volatile int16_t wheel_current[4]; /* 最后一次左右轮电流指令 raw, 供发送与 VOFA 观测 */

void Dji_Init(void);
void Dji_Parse(void);
void Dji_Circle_Calculate(void);
float Dji_Encoder_To_Rad(int32_t encoder_count);
float Dji_Rpm_To_Rad_S(int16_t rpm);
int16_t Dji_Torque_To_Current(uint8_t index, float torque_nm);
HAL_StatusTypeDef Dji_Send_Wheel_Torque(float left_torque_nm,
                                        float right_torque_nm);
HAL_StatusTypeDef Dji_Send_Current(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id,
                                   const volatile int16_t current_raw[4]);
HAL_StatusTypeDef Dji_All_Stop(void);
bool Dji_Is_Online(uint8_t index);

#endif
