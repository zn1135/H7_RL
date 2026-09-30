#ifndef VOFA_TRACE_H
#define VOFA_TRACE_H

#include <stdint.h>
#include "imu_state.h"
#include "rl_observation.h"

/*
 * 策略 VOFA 追踪：默认占用机器表中的调试串口，1152000 8N1、32 通道 JustFloat。
 * 每次 100 Hz 策略采样记录同源 IMU、25 维网络观测和发布动作；首次投入及
 * 每秒补发 5 × 25 维历史。帧类型、时间戳、带宽和解码见
 * md/vofa_policy_trace.md。仅采样和异步发送，不参与电机控制。
 * 当前上电请求位为 0；写 1 可在电机失能且发送完成时切到策略追踪布局。
 */
extern volatile uint8_t vofa_trace_requested;

void Vofa_Trace_Record(const imu_state_t *imu,
                       const rl_observation_state_t *observation,
                       const float action[RL_ACTION_SIZE],
                       uint64_t obs_time_us,
                       uint8_t engaged,
                       uint8_t rl_ready,
                       float inference_us);
uint8_t Vofa_Trace_Pump(void);
void Vofa_Trace_Discard(void);

#endif
