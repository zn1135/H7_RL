#include "vofa_trace.h"
#include "robot_control.h"
#include "Vofa_send.h"
#include "mono_ns.h"

#include <string.h>

#define VOFA_TRACE_QUEUE_SIZE 32u
#define VOFA_TRACE_CH 32u
#define VOFA_TRACE_QUEUE_MASK (VOFA_TRACE_QUEUE_SIZE - 1u)
#define VOFA_TRACE_SYNC_US 1000000u
#define VOFA_TRACE_SEQ_MASK 0x00FFFFFFu

#if VOFA_MAX_CH < VOFA_TRACE_CH || RL_OBS_SIZE != 25u || RL_OBS_HISTORY_FRAMES != 5u
#error "VOFA trace layout requires 32 channels, 25 observations and 5 history frames"
#endif

/* 策略写，通信读 */
static struct {
    float frames[VOFA_TRACE_QUEUE_SIZE][VOFA_TRACE_CH];
    volatile uint8_t head;
    volatile uint8_t tail;
    uint32_t sequence;
    uint32_t dropped;
    uint64_t last_sync_us;
    uint8_t last_engaged;
} trace;

volatile uint8_t vofa_trace_requested = 0u;
static volatile uint8_t trace_enabled = 0u;
static volatile uint8_t sync_requested = 1u;

/* 帧首 */
static void frame_begin(float frame[VOFA_TRACE_CH], uint8_t kind,
                        uint32_t sequence, uint64_t time_us, uint32_t flags)
{
    memset(frame, 0, VOFA_TRACE_CH * sizeof(float));
    frame[0] = (float)kind;
    frame[1] = (float)(sequence & VOFA_TRACE_SEQ_MASK);
    frame[2] = (float)(time_us & 0xFFFFu);
    frame[3] = (float)((time_us >> 16) & 0xFFFFu);
    frame[4] = (float)((time_us >> 32) & 0xFFFFu);
    frame[5] = (float)flags;
}

/* 策略入队 */
static uint8_t frame_enqueue(const float frame[VOFA_TRACE_CH])
{
    uint8_t next = (uint8_t)((trace.head + 1u) & VOFA_TRACE_QUEUE_MASK);

    if (next == trace.tail)
    {
        trace.dropped++;
        return 0u;
    }
    memcpy(trace.frames[trace.head], frame, VOFA_TRACE_CH * sizeof(float));
    __DMB();
    trace.head = next;
    return 1u;
}

/* 队列空槽 */
static uint8_t queue_free(void)
{
    return (uint8_t)((trace.tail - trace.head - 1u) & VOFA_TRACE_QUEUE_MASK);
}

/* 状态位 */
static uint32_t frame_flags(const imu_state_t *imu,
                            const rl_observation_state_t *observation,
                            uint8_t engaged, uint8_t rl_ready)
{
    uint32_t flags;
    uint8_t i;

    flags = engaged ? 1u : 0u;
    flags |= observation->valid ? 1u << 1 : 0u;
    flags |= observation->history_ready ? 1u << 2 : 0u;
    flags |= rl_ready ? 1u << 3 : 0u;
    flags |= imu->online ? 1u << 4 : 0u;
    flags |= robot_state.motor_enabled ? 1u << 5 : 0u;
    flags |= robot_state.fallen ? 1u << 6 : 0u;
    flags |= rl_control.policy.ready ? 1u << 7 : 0u;
    flags |= (ctrl_fault & 0x1Fu) << 8;
    flags |= output_debug_dm_sent ? 1u << 13 : 0u;
    flags |= output_debug_dji_sent ? 1u << 14 : 0u;
    flags |= rc_command.online ? 1u << 15 : 0u;
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        flags |= motor_state.dm.online[i] ? 1u << (16u + i) : 0u;
    }
    for (i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        flags |= motor_state.dji.online[i] ? 1u << (22u + i) : 0u;
    }
    return flags;
}

/* 策略快照 */
void Vofa_Trace_Record(const imu_state_t *imu,
                       const rl_observation_state_t *observation,
                       const float action[RL_ACTION_SIZE],
                       uint64_t obs_time_us,
                       uint8_t engaged,
                       uint8_t rl_ready,
                       float inference_us)
{
    float frame[VOFA_TRACE_CH];
    uint32_t flags;
    uint32_t sequence;
    uint64_t output_time_us;
    uint8_t i;
    uint8_t sync_due;

    if (!trace_enabled || imu == NULL || observation == NULL || action == NULL)
    {
        return;
    }
    sequence = (++trace.sequence) & VOFA_TRACE_SEQ_MASK;
    flags = frame_flags(imu, observation, engaged, rl_ready);
    output_time_us = Mono_Ns_Get() / 1000u;
    sync_due = (uint8_t)(engaged && observation->history_ready
        && (sync_requested || !trace.last_engaged
            || obs_time_us - trace.last_sync_us >= VOFA_TRACE_SYNC_US));
    trace.last_engaged = engaged;
    if (queue_free() < 2u)
    {
        trace.dropped += 2u;
        return;
    }

    frame_begin(frame, 0u, sequence, obs_time_us, flags);
    memcpy(&frame[6], observation->obs, RL_OBS_SIZE * sizeof(float));
    frame[31] = (float)(imu->last_timestamp_ms & VOFA_TRACE_SEQ_MASK);
    (void)frame_enqueue(frame);

    frame_begin(frame, 1u, sequence, output_time_us, flags);
    for (i = 0u; i < 3u; i++)
    {
        frame[6u + i] = imu->gyro_rad_s[i];
        frame[13u + i] = imu->acc_g[i];
        frame[16u + i] = imu->euler_rad[i];
    }
    for (i = 0u; i < 4u; i++)
    {
        frame[9u + i] = imu->quat[i];
        frame[25u + i] = rl_output_dm_cmd_nm[i];
    }
    for (i = 0u; i < RL_ACTION_SIZE; i++)
    {
        frame[19u + i] = action[i];
    }
    for (i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        frame[29u + i] = rl_output_wheel_cmd_nm[i];
    }
    frame[31] = inference_us;
    (void)frame_enqueue(frame);

    if (sync_due && queue_free() >= RL_OBS_HISTORY_FRAMES)
    {
        for (i = 0u; i < RL_OBS_HISTORY_FRAMES; i++)
        {
            frame_begin(frame, (uint8_t)(2u + i), sequence, obs_time_us, flags);
            memcpy(&frame[6], &observation->history[i * RL_OBS_SIZE],
                   RL_OBS_SIZE * sizeof(float));
            frame[31] = (float)(trace.dropped & VOFA_TRACE_SEQ_MASK);
            (void)frame_enqueue(frame);
        }
        trace.last_sync_us = obs_time_us;
        sync_requested = 0u;
    }
}

/* 清待发帧 */
void Vofa_Trace_Discard(void)
{
    trace.tail = trace.head;
    sync_requested = 1u;
}

/* DMA 发帧 */
uint8_t Vofa_Trace_Pump(void)
{
    uint8_t desired;
    uint8_t tail;

    desired = vofa_trace_requested ? 1u : 0u;
    if (desired != trace_enabled && !robot_state.motor_enabled
        && Vofa_Transport_Idle())
    {
        trace_enabled = desired;
        Vofa_Trace_Discard();
    }
    if (!trace_enabled)
    {
        return 0u;
    }
    tail = trace.tail;
    if (tail == trace.head)
    {
        return 1u;
    }
    __DMB();
    if (Vofa_Send(trace.frames[tail], VOFA_TRACE_CH))
    {
        trace.tail = (uint8_t)((tail + 1u) & VOFA_TRACE_QUEUE_MASK);
    }
    return 1u;
}
