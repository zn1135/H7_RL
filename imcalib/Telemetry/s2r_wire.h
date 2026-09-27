#ifndef S2R_WIRE_H
#define S2R_WIRE_H

#include <stdint.h>
#include <stddef.h>

#define S2R_PAYLOAD_MAX 1024u
#define S2R_FRAME_MAX (S2R_PAYLOAD_MAX + 44u)
#define S2R_QUEUE_CAPACITY 24u
#define S2R_QUEUE_RESERVED 4u

enum {
    S2R_META = 1, S2R_EVENT, S2R_POLICY, S2R_CONTROL,
    S2R_IMU, S2R_HEALTH, S2R_HISTORY
};
enum {
    S2R_STARTED = 1u, S2R_START_EDGE = 2u, S2R_ACTIVE = 4u,
    S2R_OBS_VALID = 8u, S2R_HISTORY_VALID = 16u, S2R_INFER_OK = 32u,
    S2R_OUTPUT_ENABLED = 64u, S2R_ONLINE = 128u, S2R_FALLEN = 256u,
    S2R_FAULT = 512u, S2R_WARMUP = 1024u, S2R_SNAPSHOT_VALID = 2048u,
    S2R_CLAMP = 4096u,
    /* 板端短窗录制 (md §10a): 本帧是录制/回放段的产物 */
    S2R_RECORDING = 8192u, S2R_REPLAYING = 16384u
};

typedef struct {
    uint64_t t_us;
    uint32_t session;
    uint32_t flags;
    uint32_t seq;
    uint16_t len;
    uint8_t type;
    uint8_t payload[S2R_PAYLOAD_MAX];
} s2r_record_t;

typedef struct {
    s2r_record_t slots[S2R_QUEUE_CAPACITY];
    uint32_t generated;
    uint32_t dropped;
    uint32_t peak_bytes;
    uint32_t bytes;
    uint8_t read_idx;
    uint8_t write_idx;
    uint8_t count;
} s2r_queue_t;

void S2R_Put_U16(uint8_t *p, uint16_t value);
void S2R_Put_U32(uint8_t *p, uint32_t value);
void S2R_Put_U64(uint8_t *p, uint64_t value);
void S2R_Put_F32(uint8_t *p, float value);
uint32_t S2R_Crc32(const uint8_t *data, size_t len);
uint16_t S2R_Encode(uint8_t *out, uint64_t boot_id, const s2r_record_t *record);
/* 调用侧加锁 */
uint8_t S2R_Queue_Push(s2r_queue_t *queue, const s2r_record_t *record, uint8_t priority);
uint8_t S2R_Queue_Pop(s2r_queue_t *queue, s2r_record_t *record);

#endif
