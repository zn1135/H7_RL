#include "joint_usb.h"
#include "joint_usb_limits.h"
#include "robot_control.h"
#include "mono_ns.h"
#include "usbd_cdc_if.h"
#include "Vofa_send.h"
#include "dma_cache.h"
#include "firmware_build_info.h"
#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <string.h>

#define JOINT_USB_RX_RING 512u
#define JOINT_USB_RX_MASK (JOINT_USB_RX_RING - 1u)
#define JOINT_USB_PAYLOAD_MAX 192u
#define JOINT_USB_FRAME_MAX (JOINT_USB_PAYLOAD_MAX + 16u)
#define JOINT_USB_TIMEOUT_NS 30000000ULL

enum { JUSB_STATUS = 1u, JUSB_ARM = 2u, JUSB_SET = 3u,
       JUSB_STOP = 4u, JUSB_ECHO = 5u, JUSB_STREAM = 6u };
enum { JUSB_REPLY = 0x81u, JUSB_STATE = 0x82u,
       JUSB_SAMPLE = 0x83u, JUSB_ECHO_REPLY = 0x85u };
enum { JUSB_OK = 0u, JUSB_BAD_STATE, JUSB_NO_LIMITS,
       JUSB_FAULT, JUSB_RANGE, JUSB_STALE, JUSB_BAD_PAYLOAD };

typedef struct {
    uint64_t sample_ns;
    uint64_t rx_ns;
    uint64_t queue_ns;
    uint64_t motor_rx_ns[DM_MOTOR_NUM];
    uint32_t host_seq;
    uint32_t faults;
    uint32_t flags;
    float applied[DM_MOTOR_NUM];
    float angle[DM_MOTOR_NUM];
    float zero_angle[DM_MOTOR_NUM];
    float speed[DM_MOTOR_NUM];
    float feedback_torque[DM_MOTOR_NUM];
    float leg[8];
    uint8_t selected;
} joint_sample_t;

static const float torque_max[DM_MOTOR_NUM] = JOINT_USB_TORQUE_MAX_NM;
static const float speed_max[DM_MOTOR_NUM] = JOINT_USB_SPEED_MAX_RAD_S;
static const float pos_min[DM_MOTOR_NUM] = JOINT_USB_POS_MIN_RAD;
static const float pos_max[DM_MOTOR_NUM] = JOINT_USB_POS_MAX_RAD;

static uint8_t rx_ring[JOINT_USB_RX_RING];
static uint64_t rx_time_ns[JOINT_USB_RX_RING];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint32_t rx_overflow;
static volatile uint32_t rx_overflow_total;
static uint8_t parse_buf[JOINT_USB_FRAME_MAX];
static uint16_t parse_len;
static uint32_t rx_bad_frame;
static uint32_t last_host_seq;
static uint32_t tx_seq;
static volatile uint8_t armed;
static volatile uint8_t latched;
static volatile uint8_t stream_requested;
static volatile uint8_t selected = 0xffu;
static volatile float commanded_nm;
static volatile uint64_t last_command_ns;
static volatile uint64_t last_command_rx_ns;
static volatile uint64_t last_tx_accept_ns;
static volatile uint32_t command_seq;
static volatile uint32_t sample_version;
static joint_sample_t latest_sample;
static uint32_t sent_sample_version;
static uint32_t dropped_samples;
static uint32_t reply_overwrites;
static uint8_t reply_buf[JOINT_USB_PAYLOAD_MAX];
static uint16_t reply_len;
static uint8_t reply_type;
static uint32_t reply_seq;
static uint8_t reply_pending;
static uint8_t tx_buf[JOINT_USB_FRAME_MAX]
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

static uint32_t crc32(const uint8_t *data, uint16_t len)
{
    uint32_t crc;
    uint16_t i;
    uint8_t bit;

    crc = 0xffffffffu;
    for (i = 0u; i < len; i++)
    {
        crc ^= data[i];
        for (bit = 0u; bit < 8u; bit++)
        {
            crc = (crc >> 1) ^ ((0u - (crc & 1u)) & 0xedb88320u);
        }
    }
    return crc ^ 0xffffffffu;
}

static uint8_t limits_valid(void)
{
    uint8_t i;

    if (JOINT_USB_LIMITS_APPROVED != 1u)
    {
        return 0u;
    }
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (!isfinite(torque_max[i]) || torque_max[i] <= 0.0f
            || torque_max[i] > machine->dm_trq_max
            || !isfinite(speed_max[i]) || speed_max[i] <= 0.0f
            || !isfinite(pos_min[i]) || !isfinite(pos_max[i])
            || pos_min[i] >= pos_max[i])
        {
            return 0u;
        }
    }
    return 1u;
}

uint8_t JointUsb_PhysicalPermit(void)
{
    return Control_Mode_Decode(&rc_command).usb_permit;
}

uint8_t JointUsb_ModeLock(void)
{
    return (uint8_t)(JointUsb_PhysicalPermit() || armed || latched);
}

uint8_t JointUsb_StreamRequested(void)
{
    return (uint8_t)(vofa_transport.active != VOFA_TRANSPORT_USB
        && stream_requested);
}

static void abort_run(void)
{
    armed = 0u;
    commanded_nm = 0.0f;
    selected = 0xffu;
    latched = 1u;
}

uint8_t JointUsb_EnableAllowed(void)
{
    uint64_t now;

    now = Mono_Ns_Get();
    return (uint8_t)(vofa_transport.active != VOFA_TRANSPORT_USB
        && armed && !latched && stream_requested && limits_valid()
        && JointUsb_PhysicalPermit() && CDC_Configured_HS()
        && ctrl_fault == FAULT_NONE && !robot_state.fallen
        && torque_output_enabled
        && now - last_command_ns < JOINT_USB_TIMEOUT_NS
        && now - last_tx_accept_ns < JOINT_USB_TIMEOUT_NS);
}

void JointUsb_Compute(torque_output_t *torque)
{
    uint8_t i;

    if (!armed)
    {
        return;
    }
    if (!JointUsb_EnableAllowed() || !robot_state.motor_enabled)
    {
        abort_run();
        return;
    }
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        if (!motor_state.dm.online[i]
            || !isfinite(motor_state.dm.pos_zero_rad[i])
            || !isfinite(motor_state.dm.vel_rad_s[i])
            || motor_state.dm.pos_zero_rad[i] < pos_min[i]
            || motor_state.dm.pos_zero_rad[i] > pos_max[i]
            || fabsf(motor_state.dm.vel_rad_s[i]) > speed_max[i])
        {
            abort_run();
            return;
        }
    }
    if (selected < DM_MOTOR_NUM)
    {
        torque->dm[selected] = commanded_nm;
    }
    torque->valid = 1u;
}

void JointUsb_RxIsr(const uint8_t *data, uint32_t len)
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
        next = (uint16_t)((rx_head + 1u) & JOINT_USB_RX_MASK);
        if (next == rx_tail)
        {
            rx_overflow++;
            rx_overflow_total++;
            return;
        }
        rx_ring[rx_head] = data[i];
        rx_time_ns[rx_head] = packet_ns;
        __DMB();
        rx_head = next;
    }
}

void JointUsb_RxOverflowIsr(void)
{
    rx_overflow++;
    rx_overflow_total++;
}

static void make_reply(uint8_t type, uint32_t seq, const uint8_t *data,
                       uint16_t len)
{
    if (len > JOINT_USB_PAYLOAD_MAX)
    {
        return;
    }
    if (reply_pending)
    {
        reply_overwrites++;
        if (armed)
        {
            abort_run();
        }
    }
    memcpy(reply_buf, data, len);
    reply_len = len;
    reply_type = type;
    reply_seq = seq;
    reply_pending = 1u;
}

static void ack(uint32_t seq, uint8_t code, uint64_t rx_ns)
{
    uint8_t payload[9];

    payload[0] = code;
    put_u64(payload + 1u, rx_ns);
    make_reply(JUSB_REPLY, seq, payload, sizeof(payload));
}

static void send_state(uint32_t seq)
{
    uint8_t payload[148] = {0};
    uint8_t i;

    payload[0] = Machine_Id();
    payload[1] = limits_valid();
    payload[2] = JointUsb_PhysicalPermit();
    payload[3] = armed;
    payload[4] = latched;
    payload[5] = rc_command.s1;
    payload[6] = rc_command.s2;
    payload[7] = robot_state.motor_enabled;
    put_u32(payload + 8u, ctrl_fault);
    put_u32(payload + 12u, rx_overflow_total);
    put_u32(payload + 16u, rx_bad_frame);
    memcpy(payload + 20u, FIRMWARE_SOURCE_SHA256, 64u);
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        put_f32(payload + 84u + 4u*i, torque_max[i]);
        put_f32(payload + 100u + 4u*i, speed_max[i]);
        put_f32(payload + 116u + 4u*i, pos_min[i]);
        put_f32(payload + 132u + 4u*i, pos_max[i]);
    }
    make_reply(JUSB_STATE, seq, payload, sizeof(payload));
}

static void command(uint8_t type, uint32_t seq, const uint8_t *payload,
                    uint16_t len, uint64_t rx_ns)
{
    float tau;
    uint8_t motor;

    if (type == JUSB_STATUS && len == 0u)
    {
        if (!armed)
        {
            last_host_seq = seq;
        }
        send_state(seq);
        return;
    }
    if (type == JUSB_ECHO && len <= 32u)
    {
        make_reply(JUSB_ECHO_REPLY, seq, payload, len);
        return;
    }
    if (type == JUSB_STREAM && len == 1u && payload[0] <= 1u)
    {
        stream_requested = payload[0];
        sent_sample_version = 0u;
        ack(seq, JUSB_OK, rx_ns);
        return;
    }
    if (type == JUSB_STOP && len == 0u)
    {
        abort_run();
        last_host_seq = seq;
        ack(seq, JUSB_OK, rx_ns);
        return;
    }
    if (seq <= last_host_seq)
    {
        ack(seq, JUSB_STALE, rx_ns);
        return;
    }
    last_host_seq = seq;
    if (type == JUSB_ARM && len == 0u)
    {
        if (!limits_valid())
        {
            ack(seq, JUSB_NO_LIMITS, rx_ns);
        }
        else if (latched || !stream_requested || !JointUsb_PhysicalPermit()
                 || !CDC_Configured_HS() || vofa_transport.active == VOFA_TRANSPORT_USB)
        {
            ack(seq, JUSB_BAD_STATE, rx_ns);
        }
        else if (ctrl_fault != FAULT_NONE || robot_state.fallen)
        {
            ack(seq, JUSB_FAULT, rx_ns);
        }
        else
        {
            commanded_nm = 0.0f;
            selected = 0xffu;
            last_command_ns = rx_ns;
            last_tx_accept_ns = rx_ns;
            armed = 1u;
            ack(seq, JUSB_OK, rx_ns);
        }
        return;
    }
    if (type == JUSB_SET && len == 5u)
    {
        motor = payload[0];
        memcpy(&tau, payload + 1u, sizeof(tau));
        if (!armed || !JointUsb_EnableAllowed())
        {
            abort_run();
            ack(seq, JUSB_BAD_STATE, rx_ns);
        }
        else if (motor >= DM_MOTOR_NUM || !isfinite(tau)
                 || fabsf(tau) > torque_max[motor])
        {
            abort_run();
            ack(seq, JUSB_RANGE, rx_ns);
        }
        else
        {
            taskENTER_CRITICAL();
            selected = motor;
            commanded_nm = tau;
            command_seq = seq;
            last_command_rx_ns = rx_ns;
            last_command_ns = rx_ns;
            taskEXIT_CRITICAL();
            ack(seq, JUSB_OK, rx_ns);
        }
        return;
    }
    ack(seq, JUSB_BAD_PAYLOAD, rx_ns);
}

static void parse_byte(uint8_t byte, uint64_t byte_ns)
{
    uint16_t payload_len;
    uint16_t frame_len;

    if (parse_len >= sizeof(parse_buf))
    {
        parse_len = 0u;
        rx_bad_frame++;
    }
    parse_buf[parse_len++] = byte;
    while (parse_len >= 4u)
    {
        if (memcmp(parse_buf, "JID1", 4u) != 0)
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
        if (parse_buf[4] != 1u || payload_len > JOINT_USB_PAYLOAD_MAX)
        {
            memmove(parse_buf, parse_buf + 1u, --parse_len);
            rx_bad_frame++;
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
            rx_bad_frame++;
            continue;
        }
        command(parse_buf[5], get_u32(parse_buf + 8u), parse_buf + 12u,
                payload_len, byte_ns);
        parse_len = (uint16_t)(parse_len - frame_len);
        memmove(parse_buf, parse_buf + frame_len, parse_len);
    }
}

void JointUsb_Process(void)
{
    uint16_t tail;
    uint8_t byte;
    uint64_t byte_ns;
    rc_control_mode_t mode;

    if (vofa_transport.active == VOFA_TRANSPORT_USB)
    {
        if (armed)
        {
            abort_run();
        }
        stream_requested = 0u;
        reply_pending = 0u;
        rx_tail = rx_head;
        parse_len = 0u;
        rx_overflow = 0u;
        return;
    }
    mode = Control_Mode_Decode(&rc_command);
    if (mode.usb_reset)
    {
        armed = 0u;
        latched = 0u;
        commanded_nm = 0.0f;
    }
    if (armed && !JointUsb_EnableAllowed())
    {
        abort_run();
    }
    if (rx_overflow != 0u)
    {
        if (armed)
        {
            abort_run();
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
        rx_tail = (uint16_t)((tail + 1u) & JOINT_USB_RX_MASK);
        parse_byte(byte, byte_ns);
    }
}

void JointUsb_ActuationTick(const torque_output_t *torque, uint64_t queue_ns,
                            uint8_t dm_ok)
{
    uint8_t i;
    joint_sample_t *s;

    sample_version++;
    __DMB();
    s = &latest_sample;
    s->sample_ns = Mono_Ns_Get();
    s->rx_ns = last_command_rx_ns;
    s->queue_ns = queue_ns;
    s->host_seq = command_seq;
    s->faults = ctrl_fault;
    s->flags = (armed ? 1u : 0u) | (latched ? 2u : 0u)
        | (robot_state.motor_enabled ? 4u : 0u) | (dm_ok ? 8u : 0u)
        | (leg_l.output.valid ? 16u : 0u) | (leg_r.output.valid ? 32u : 0u);
    s->selected = selected;
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        s->applied[i] = torque->valid && dm_ok ? torque->dm[i] : 0.0f;
        s->angle[i] = motor_state.dm.pos_rad[i];
        s->zero_angle[i] = motor_state.dm.pos_zero_rad[i];
        s->speed[i] = motor_state.dm.vel_rad_s[i];
        s->feedback_torque[i] = motor_state.dm.trq_nm[i];
        s->motor_rx_ns[i] = motor_state.dm.parsed_rx_ns[i];
    }
    s->leg[0] = leg_l.output.virtual_leg_length;
    s->leg[1] = leg_l.output.virtual_leg_angle;
    s->leg[2] = leg_r.output.virtual_leg_length;
    s->leg[3] = leg_r.output.virtual_leg_angle;
    s->leg[4] = leg_l.output.thigh_angle;
    s->leg[5] = leg_l.output.virtual_shank_angle;
    s->leg[6] = leg_r.output.thigh_angle;
    s->leg[7] = leg_r.output.virtual_shank_angle;
    __DMB();
    sample_version++;
}

static uint16_t pack_sample(uint8_t *out, const joint_sample_t *s)
{
    uint8_t i;

    put_u64(out, s->sample_ns);
    put_u64(out + 8u, s->rx_ns);
    put_u64(out + 16u, s->queue_ns);
    put_u32(out + 24u, s->host_seq);
    put_u32(out + 28u, s->flags);
    put_u32(out + 32u, s->faults);
    put_u32(out + 36u, dropped_samples);
    out[40] = s->selected;
    out[41] = 0u;
    out[42] = 0u;
    out[43] = 0u;
    for (i = 0u; i < DM_MOTOR_NUM; i++)
    {
        put_f32(out + 44u + 4u*i, s->applied[i]);
        put_f32(out + 60u + 4u*i, s->angle[i]);
        put_f32(out + 76u + 4u*i, s->zero_angle[i]);
        put_f32(out + 92u + 4u*i, s->speed[i]);
        put_f32(out + 108u + 4u*i, s->feedback_torque[i]);
        put_u64(out + 124u + 8u*i, s->motor_rx_ns[i]);
    }
    for (i = 0u; i < 8u; i++)
    {
        put_f32(out + 156u + 4u*i, s->leg[i]);
    }
    put_u32(out + 188u, rx_bad_frame);
    return 192u;
}

static uint8_t transmit(uint8_t type, uint32_t seq, const uint8_t *payload,
                        uint16_t len)
{
    uint16_t total;

    if (len > JOINT_USB_PAYLOAD_MAX || !CDC_Transmit_Ready_HS())
    {
        return 0u;
    }
    memcpy(tx_buf, "JID1", 4u);
    tx_buf[4] = 1u;
    tx_buf[5] = type;
    tx_buf[6] = (uint8_t)len;
    tx_buf[7] = (uint8_t)(len >> 8);
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
    last_tx_accept_ns = Mono_Ns_Get();
    taskEXIT_CRITICAL();
    return 1u;
}

void JointUsb_Pump(void)
{
    static joint_sample_t snapshot;
    uint8_t payload[JOINT_USB_PAYLOAD_MAX];
    uint32_t before;
    uint32_t after;
    uint8_t tries;

    if (!CDC_Configured_HS())
    {
        stream_requested = 0u;
        return;
    }
    if (vofa_transport.active == VOFA_TRANSPORT_USB
        || !CDC_Transmit_Ready_HS())
    {
        return;
    }
    if (reply_pending)
    {
        if (transmit(reply_type, reply_seq, reply_buf, reply_len))
        {
            reply_pending = 0u;
        }
        return;
    }
    if (!stream_requested)
    {
        return;
    }
    for (tries = 0u; tries < 3u; tries++)
    {
        before = sample_version;
        if ((before & 1u) != 0u || before == sent_sample_version)
        {
            return;
        }
        __DMB();
        memcpy(&snapshot, &latest_sample, sizeof(snapshot));
        __DMB();
        after = sample_version;
        if (before == after)
        {
            break;
        }
    }
    if (tries == 3u)
    {
        return;
    }
    if (sent_sample_version != 0u && after - sent_sample_version > 2u)
    {
        dropped_samples += (after - sent_sample_version) / 2u - 1u;
    }
    if (transmit(JUSB_SAMPLE, ++tx_seq, payload,
                 pack_sample(payload, &snapshot)))
    {
        sent_sample_version = after;
    }
}
