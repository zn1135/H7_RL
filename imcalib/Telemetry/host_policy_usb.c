#include "host_policy_usb.h"
#include "robot_control.h"
#include "joint_usb.h"
#include "mono_ns.h"
#include "usbd_cdc_if.h"
#include "Vofa_send.h"
#include "dma_cache.h"
#include "firmware_build_info.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <string.h>

#define HPI_RX_SIZE 512u
#define HPI_RX_MASK (HPI_RX_SIZE - 1u)
#define HPI_RX_PAYLOAD_MAX 32u
#define HPI_TX_PAYLOAD_MAX 616u
#define HPI_RX_FRAME_MAX (HPI_RX_PAYLOAD_MAX + 16u)
#define HPI_TX_FRAME_MAX (HPI_TX_PAYLOAD_MAX + 16u)
#define HPI_TIMEOUT_NS 30000000ULL

enum { HPI_HELLO = 1u, HPI_ARM = 2u, HPI_ACTION = 3u, HPI_STOP = 4u };
enum { HPI_STATUS = 0x81u, HPI_INPUT = 0x82u, HPI_ACK = 0x83u };
enum { HPI_OK = 0u, HPI_BAD_STATE, HPI_BAD_SESSION, HPI_STALE,
       HPI_BAD_ACTION, HPI_BAD_PAYLOAD };

static uint8_t rx_ring[HPI_RX_SIZE];
static uint64_t rx_time_ns[HPI_RX_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint8_t rx_overflow;
static uint8_t parse_buf[HPI_RX_FRAME_MAX];
static uint16_t parse_len;
static volatile uint8_t mode_lock;
static volatile uint8_t armed;
static volatile uint8_t latched;
static uint32_t session_id;
static uint32_t last_host_seq;
static uint32_t last_input_seq;
static uint32_t last_action_seq;
static uint32_t last_sent_input_seq;
static uint32_t dropped_inputs;
static volatile uint64_t last_action_ns;
static volatile uint64_t last_tx_ns;
static volatile float last_training_action[RL_ACTION_SIZE];
static uint8_t input_payload[HPI_TX_PAYLOAD_MAX];
typedef struct {
    uint32_t seq;
    uint64_t sample_us;
    float obs[RL_OBS_SIZE];
    float history[RL_OBS_HISTORY_SIZE];
    float command[3];
} host_input_snapshot_t;
static host_input_snapshot_t input_snapshots[2];
static host_input_snapshot_t report_snapshot;
static volatile uint32_t input_version;
static volatile uint8_t input_pending;
static uint8_t reply_payload[96];
static uint16_t reply_len;
static uint8_t reply_type;
static uint32_t reply_seq;
static uint8_t reply_pending;
static uint8_t tx_buf[HPI_TX_FRAME_MAX]
    __attribute__((aligned(DMA_CACHE_LINE_SIZE)));

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
        | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void put_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static void put_u64(uint8_t *p, uint64_t value)
{
    put_u32(p, (uint32_t)value);
    put_u32(p + 4u, (uint32_t)(value >> 32));
}

static void put_f32(uint8_t *p, float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    put_u32(p, bits);
}

static uint32_t crc32(const uint8_t *bytes, uint16_t count)
{
    uint32_t crc = 0xffffffffu;
    uint16_t i;
    uint8_t bit;

    for (i = 0u; i < count; i++)
    {
        crc ^= bytes[i];
        for (bit = 0u; bit < 8u; bit++)
        {
            crc = (crc >> 1) ^ ((0u - (crc & 1u)) & 0xedb88320u);
        }
    }
    return crc ^ 0xffffffffu;
}

static void stop_output(void)
{
    uint8_t i;

    armed = 0u;
    latched = 1u;
    taskENTER_CRITICAL();
    action_state.rl_ready = 0u;
    for (i = 0u; i < RL_ACTION_SIZE; i++)
    {
        action_state.a[i] = 0.0f;
        last_training_action[i] = 0.0f;
    }
    taskEXIT_CRITICAL();
}

uint8_t HostPolicy_ModeLock(void)
{
    return (uint8_t)(mode_lock || latched);
}

uint8_t HostPolicy_Armed(void)
{
    return armed;
}

uint32_t HostPolicy_AppliedSeq(void)
{
    return last_action_seq;
}

uint8_t HostPolicy_EnableAllowed(void)
{
    uint8_t i;
    uint64_t now = Mono_Ns_Get();
    uint64_t action_ns;
    uint64_t tx_ns;

    taskENTER_CRITICAL();
    action_ns = last_action_ns;
    tx_ns = last_tx_ns;
    taskEXIT_CRITICAL();

    if (!mode_lock || !armed || latched || !CDC_Configured_HS()
        || vofa_transport.active == VOFA_TRANSPORT_USB
        || !rc_command.online || rc_command.s1 != DR16_SW_UP
        || rc_command.s2 != DR16_SW_MID || !robot_state.rc_enable
        || !imu_state.online || !leg_l.output.valid || !leg_r.output.valid
        || ctrl_fault != FAULT_NONE || robot_state.fallen
        || !torque_output_enabled || now - action_ns >= HPI_TIMEOUT_NS
        || now - tx_ns >= HPI_TIMEOUT_NS)
    {
        return 0u;
    }
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (!motor_state.dm.online[i])
        {
            return 0u;
        }
    }
    for (i = 0u; i < DJI_MOTOR_NUM; i++)
    {
        if (!motor_state.dji.online[i])
        {
            return 0u;
        }
    }
    return 1u;
}

void HostPolicy_LastTrainingAction(float action[RL_ACTION_SIZE])
{
    uint8_t i;

    taskENTER_CRITICAL();
    for (i = 0u; i < RL_ACTION_SIZE; i++)
    {
        action[i] = last_training_action[i];
    }
    taskEXIT_CRITICAL();
}

void HostPolicy_SubmitInput(const float obs[25], const float history[125],
                            const float command[3], uint64_t sample_us)
{
    uint16_t i;
    host_input_snapshot_t *saved;

    if (!mode_lock || !armed || latched)
    {
        return;
    }
    taskENTER_CRITICAL();
    if (input_pending)
    {
        dropped_inputs++;
    }
    input_version++;
    last_input_seq++;
    saved = &input_snapshots[last_input_seq & 1u];
    saved->seq = last_input_seq;
    saved->sample_us = sample_us;
    memcpy(saved->obs, obs, sizeof(saved->obs));
    memcpy(saved->history, history, sizeof(saved->history));
    memcpy(saved->command, command, sizeof(saved->command));
    put_u32(input_payload, session_id);
    put_u32(input_payload + 4u, last_input_seq);
    put_u64(input_payload + 8u, sample_us);
    for (i = 0u; i < RL_OBS_SIZE; i++)
    {
        put_f32(input_payload + 16u + 4u*i, obs[i]);
    }
    for (i = 0u; i < RL_OBS_HISTORY_SIZE; i++)
    {
        put_f32(input_payload + 116u + 4u*i, history[i]);
    }
    input_pending = 1u;
    input_version++;
    taskEXIT_CRITICAL();
}

void HostPolicy_RxIsr(const uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint16_t next;
    uint64_t packet_ns;

    if (vofa_transport.active == VOFA_TRANSPORT_USB)
    {
        return;
    }
    packet_ns = Mono_Ns_Get();
    for (i = 0u; i < len; i++)
    {
        next = (uint16_t)((rx_head + 1u) & HPI_RX_MASK);
        if (next == rx_tail)
        {
            rx_overflow = 1u;
            return;
        }
        rx_ring[rx_head] = data[i];
        rx_time_ns[rx_head] = packet_ns;
        __DMB();
        rx_head = next;
    }
}

void HostPolicy_RxOverflowIsr(void)
{
    rx_overflow = 1u;
}

static void reply(uint8_t type, uint32_t seq, const uint8_t *payload,
                  uint16_t len)
{
    if (reply_pending)
    {
        if (mode_lock)
        {
            stop_output();
        }
        return;
    }
    memcpy(reply_payload, payload, len);
    reply_type = type;
    reply_seq = seq;
    reply_len = len;
    reply_pending = 1u;
}

static void ack(uint32_t seq, uint8_t code)
{
    uint8_t payload[9];

    payload[0] = code;
    put_u32(payload + 1u, session_id);
    put_u32(payload + 5u, last_action_seq);
    reply(HPI_ACK, seq, payload, sizeof(payload));
}

static void status(uint32_t seq)
{
    uint8_t payload[80] = {0};

    put_u32(payload, session_id);
    payload[4] = mode_lock;
    payload[5] = armed;
    put_u16(payload + 6u, RL_OBS_SIZE);
    put_u16(payload + 8u, RL_OBS_HISTORY_SIZE);
    put_u16(payload + 10u, RL_ACTION_SIZE);
    put_u32(payload + 12u, ctrl_fault);
    memcpy(payload + 16u, FIRMWARE_SOURCE_SHA256, 64u);
    reply(HPI_STATUS, seq, payload, sizeof(payload));
}

static void command(uint8_t type, uint32_t seq, const uint8_t *payload,
                    uint16_t len, uint64_t rx_ns)
{
    uint32_t requested_session;
    uint32_t input_seq;
    uint8_t i;
    float training[RL_ACTION_SIZE];

    if (type == HPI_HELLO && len == 0u)
    {
        if (!mode_lock)
        {
            if (JointUsb_ModeLock() || robot_state.motor_enabled
                || !rc_command.online || rc_command.s1 != DR16_SW_DOWN)
            {
                ack(seq, HPI_BAD_STATE);
                return;
            }
            mode_lock = 1u;
            latched = 0u;
            armed = 0u;
            session_id = (uint32_t)rx_ns ^ (uint32_t)(rx_ns >> 32)
                ^ HAL_GetTick();
            if (session_id == 0u)
            {
                session_id = 1u;
            }
            last_host_seq = 0u;
            last_input_seq = 0u;
            last_action_seq = 0u;
            last_sent_input_seq = 0u;
            input_pending = 0u;
            dropped_inputs = 0u;
            for (i = 0u; i < RL_ACTION_SIZE; i++)
            {
                last_training_action[i] = 0.0f;
            }
        }
        status(seq);
        return;
    }
    if (type == HPI_STOP && len == 0u)
    {
        stop_output();
        if (rc_command.s1 == DR16_SW_DOWN && !robot_state.motor_enabled)
        {
            mode_lock = 0u;
            latched = 0u;
        }
        ack(seq, HPI_OK);
        return;
    }
    if (!mode_lock || seq <= last_host_seq)
    {
        ack(seq, mode_lock ? HPI_STALE : HPI_BAD_STATE);
        return;
    }
    last_host_seq = seq;
    if (type == HPI_ARM && len == 4u)
    {
        requested_session = get_u32(payload);
        if (requested_session != session_id)
        {
            ack(seq, HPI_BAD_SESSION);
        }
        else if (armed || latched || JointUsb_ModeLock()
                 || !rc_command.online || rc_command.s1 != DR16_SW_UP
                 || rc_command.s2 != DR16_SW_MID || ctrl_fault != FAULT_NONE
                 || robot_state.fallen || !CDC_Configured_HS())
        {
            ack(seq, HPI_BAD_STATE);
        }
        else
        {
            taskENTER_CRITICAL();
            last_action_ns = rx_ns;
            last_tx_ns = rx_ns;
            taskEXIT_CRITICAL();
            armed = 1u;
            if (!HostPolicy_EnableAllowed())
            {
                armed = 0u;
                ack(seq, HPI_BAD_STATE);
            }
            else
            {
                ack(seq, HPI_OK);
            }
        }
        return;
    }
    if (type == HPI_ACTION && len == 32u)
    {
        requested_session = get_u32(payload);
        input_seq = get_u32(payload + 4u);
        if (requested_session != session_id)
        {
            ack(seq, HPI_BAD_SESSION);
            return;
        }
        if (!armed || !HostPolicy_EnableAllowed())
        {
            stop_output();
            ack(seq, HPI_BAD_STATE);
            return;
        }
        if (input_seq <= last_action_seq || input_seq > last_input_seq
            || last_input_seq - input_seq > 1u)
        {
            ack(seq, HPI_STALE);
            return;
        }
        taskENTER_CRITICAL();
        if (input_snapshots[input_seq & 1u].seq != input_seq)
        {
            taskEXIT_CRITICAL();
            ack(seq, HPI_STALE);
            return;
        }
        memcpy(&report_snapshot, &input_snapshots[input_seq & 1u],
               sizeof(report_snapshot));
        taskEXIT_CRITICAL();
        for (i = 0u; i < RL_ACTION_SIZE; i++)
        {
            memcpy(&training[i], payload + 8u + 4u*i, sizeof(float));
            if (!isfinite(training[i])
                || (RL_ACTION_CLIP > 0.0f && fabsf(training[i]) > RL_ACTION_CLIP))
            {
                stop_output();
                ack(seq, HPI_BAD_ACTION);
                return;
            }
        }
        taskENTER_CRITICAL();
        for (i = 0u; i < RL_ACTION_SIZE; i++)
        {
            last_training_action[i] = training[i];
            action_state.a[i] = (float)machine->rl.sign[i] * training[i];
        }
        action_state.updated = 1u;
        action_state.last_ok_tick = HAL_GetTick();
        action_state.rl_ready = 1u;
        last_action_seq = input_seq;
        last_action_ns = rx_ns;
        taskEXIT_CRITICAL();
        ack(seq, HPI_OK);
        return;
    }
    ack(seq, HPI_BAD_PAYLOAD);
}

static void parse_byte(uint8_t byte, uint64_t byte_ns)
{
    uint16_t payload_len;
    uint16_t frame_len;

    if (parse_len >= sizeof(parse_buf))
    {
        parse_len = 0u;
    }
    parse_buf[parse_len++] = byte;
    while (parse_len >= 4u)
    {
        if (memcmp(parse_buf, "HPI1", 4u) != 0)
        {
            memmove(parse_buf, parse_buf + 1u, --parse_len);
            continue;
        }
        if (parse_len < 12u)
        {
            return;
        }
        payload_len = get_u16(parse_buf + 6u);
        frame_len = (uint16_t)(16u + payload_len);
        if (parse_buf[4] != 1u || payload_len > HPI_RX_PAYLOAD_MAX)
        {
            memmove(parse_buf, parse_buf + 1u, --parse_len);
            continue;
        }
        if (parse_len < frame_len)
        {
            return;
        }
        if (crc32(parse_buf, (uint16_t)(frame_len - 4u))
            != get_u32(parse_buf + frame_len - 4u))
        {
            memmove(parse_buf, parse_buf + 1u, --parse_len);
            continue;
        }
        command(parse_buf[5], get_u32(parse_buf + 8u), parse_buf + 12u,
                payload_len, byte_ns);
        parse_len = (uint16_t)(parse_len - frame_len);
        memmove(parse_buf, parse_buf + frame_len, parse_len);
    }
}

void HostPolicy_Process(void)
{
    uint16_t tail;
    uint8_t byte;
    uint64_t byte_ns;

    if (vofa_transport.active == VOFA_TRANSPORT_USB)
    {
        if (armed)
        {
            stop_output();
        }
        rx_tail = rx_head;
        parse_len = 0u;
        return;
    }
    if (armed && !HostPolicy_EnableAllowed())
    {
        stop_output();
    }
    if (rx_overflow)
    {
        if (mode_lock)
        {
            stop_output();
        }
        rx_tail = rx_head;
        parse_len = 0u;
        rx_overflow = 0u;
    }
    while (rx_tail != rx_head)
    {
        tail = rx_tail;
        byte = rx_ring[tail];
        byte_ns = rx_time_ns[tail];
        __DMB();
        rx_tail = (uint16_t)((tail + 1u) & HPI_RX_MASK);
        parse_byte(byte, byte_ns);
    }
}

static uint8_t transmit(uint8_t type, uint32_t seq, const uint8_t *payload,
                        uint16_t len)
{
    uint16_t total;

    if (len > HPI_TX_PAYLOAD_MAX || !CDC_Transmit_Ready_HS())
    {
        return 0u;
    }
    memcpy(tx_buf, "HPI1", 4u);
    tx_buf[4] = 1u;
    tx_buf[5] = type;
    put_u16(tx_buf + 6u, len);
    put_u32(tx_buf + 8u, seq);
    memcpy(tx_buf + 12u, payload, len);
    total = (uint16_t)(12u + len);
    put_u32(tx_buf + total, crc32(tx_buf, total));
    total += 4u;
    Dma_Cache_Clean_Tx(tx_buf, total);
    if (CDC_Transmit_HS(tx_buf, total) != USBD_OK)
    {
        return 0u;
    }
    taskENTER_CRITICAL();
    last_tx_ns = Mono_Ns_Get();
    taskEXIT_CRITICAL();
    return 1u;
}

void HostPolicy_Pump(void)
{
    static uint8_t snapshot[HPI_TX_PAYLOAD_MAX];
    uint32_t before;
    uint32_t after;

    if (!CDC_Configured_HS() || vofa_transport.active == VOFA_TRANSPORT_USB)
    {
        return;
    }
    if (reply_pending)
    {
        if (transmit(reply_type, reply_seq, reply_payload, reply_len))
        {
            reply_pending = 0u;
        }
        return;
    }
    if (!mode_lock || !armed || !input_pending)
    {
        return;
    }
    before = input_version;
    if (before & 1u)
    {
        return;
    }
    __DMB();
    memcpy(snapshot, input_payload, HPI_TX_PAYLOAD_MAX);
    __DMB();
    after = input_version;
    if (before != after)
    {
        return;
    }
    if (transmit(HPI_INPUT, get_u32(snapshot + 4u), snapshot,
                 HPI_TX_PAYLOAD_MAX))
    {
        last_sent_input_seq = get_u32(snapshot + 4u);
        if (input_version == after)
        {
            input_pending = 0u;
        }
    }
}
