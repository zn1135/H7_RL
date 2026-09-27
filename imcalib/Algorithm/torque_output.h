#ifndef TORQUE_OUTPUT_H
#define TORQUE_OUTPUT_H

#include <string.h>
#include <stdint.h>
#include "dm.h"
#include "dji.h"

/* 力矩命令: 各控制器 (LQR / 手动腿测 / RL) 的统一产物, 只由 task_actuation.c 的 output_dispatch() 下发
 *   dm[]   四髋 (前左/后左/前右/后右, DM 驱动索引), N·m
 *   dji[]  两轮 (左/右, DJI 驱动索引), N·m
 *   valid  0 = 本拍无有效命令 (未投入 / 解算失败 / 未使能), 分发层发零力矩 */
typedef struct {
    float   dm[DM_MOTOR_NUM];
    float   dji[DJI_MOTOR_NUM];
    uint8_t valid;
} torque_output_t;

static inline void Torque_Output_Clear(torque_output_t *t)
{
    memset(t, 0, sizeof(*t));
}

#endif
