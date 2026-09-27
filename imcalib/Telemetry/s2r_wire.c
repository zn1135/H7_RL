#include "s2r_wire.h"
#include <string.h>

void S2R_Put_U16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

void S2R_Put_U32(uint8_t *p, uint32_t value)
{
    uint32_t i;
    for (i = 0u; i < 4u; i++)
    {
        p[i] = (uint8_t)(value >> (8u * i));
    }
}

void S2R_Put_U64(uint8_t *p, uint64_t value)
{
    S2R_Put_U32(p, (uint32_t)value);
    S2R_Put_U32(p + 4, (uint32_t)(value >> 32));
}

void S2R_Put_F32(uint8_t *p, float value)
{
    uint32_t bits;
    typedef char float_must_be_32_bits[(sizeof(float) == 4) ? 1 : -1];
    (void)sizeof(float_must_be_32_bits);
    memcpy(&bits, &value, sizeof(bits));
    S2R_Put_U32(p, bits);
}

uint32_t S2R_Crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t bit;
    size_t i;
    for (i = 0u; i < len; i++)
    {
        crc ^= data[i];
        for (bit = 0u; bit < 8u; bit++)
        {
            crc = (crc >> 1) ^ ((0u - (crc & 1u)) & 0xEDB88320u);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

uint16_t S2R_Encode(uint8_t *out, uint64_t boot_id, const s2r_record_t *record)
{
    if (record->len > S2R_PAYLOAD_MAX)
    {
        return 0u;
    }
    memcpy(out, "S2R1", 4);
    out[4] = 1u;
    out[5] = record->type;
    S2R_Put_U16(out + 6, 40u);
    S2R_Put_U16(out + 8, record->len);
    S2R_Put_U16(out + 10, 0u);
    S2R_Put_U32(out + 12, record->seq);
    S2R_Put_U64(out + 16, boot_id);
    S2R_Put_U32(out + 24, record->session);
    S2R_Put_U64(out + 28, record->t_us);
    S2R_Put_U32(out + 36, record->flags);
    memcpy(out + 40, record->payload, record->len);
    S2R_Put_U32(out + 40 + record->len, S2R_Crc32(out, 40u + record->len));
    return (uint16_t)(44u + record->len);
}

uint8_t S2R_Queue_Push(s2r_queue_t *q, const s2r_record_t *r, uint8_t priority)
{
    uint32_t seq = q->generated++;
    s2r_record_t *slot;
    if (r->len > S2R_PAYLOAD_MAX || q->count >= S2R_QUEUE_CAPACITY
        || (!priority && q->count >= S2R_QUEUE_CAPACITY - S2R_QUEUE_RESERVED))
    {
        q->dropped++;
        return 0u;
    }
    slot = &q->slots[q->write_idx];
    slot->t_us = r->t_us;
    slot->session = r->session;
    slot->flags = r->flags;
    slot->seq = seq;
    slot->len = r->len;
    slot->type = r->type;
    memcpy(slot->payload, r->payload, r->len);
    q->write_idx = (uint8_t)((q->write_idx + 1u) % S2R_QUEUE_CAPACITY);
    q->count++;
    q->bytes += 44u + r->len;
    if (q->bytes > q->peak_bytes)
    {
        q->peak_bytes = q->bytes;
    }
    return 1u;
}

uint8_t S2R_Queue_Pop(s2r_queue_t *q, s2r_record_t *r)
{
    if (!q->count)
    {
        return 0u;
    }
    memcpy(r, &q->slots[q->read_idx], offsetof(s2r_record_t, payload)
        + q->slots[q->read_idx].len);
    q->read_idx = (uint8_t)((q->read_idx + 1u) % S2R_QUEUE_CAPACITY);
    q->count--;
    q->bytes -= 44u + r->len;
    return 1u;
}
