#include "s2r_wire.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char **argv)
{
    static s2r_queue_t q;
    static s2r_record_t r, out;
    uint8_t bytes[S2R_FRAME_MAX];
    uint32_t i;
    FILE *file;
    assert(S2R_Crc32((const uint8_t *)"123456789", 9) == 0xCBF43926u);
    S2R_Put_U64(bytes, 0xFEDCBA9876543210ULL);
    assert(memcmp(bytes, "\x10\x32\x54\x76\x98\xba\xdc\xfe", 8) == 0);
    S2R_Put_F32(bytes, 1.0f);
    assert(memcmp(bytes, "\0\0\x80\x3f", 4) == 0);
    r.type = S2R_EVENT;
    r.len = 24;
    r.session = 7;
    r.t_us = 0x123456789ULL;
    r.flags = S2R_STARTED | S2R_START_EDGE;
    S2R_Put_U32(r.payload, 17);
    S2R_Put_U16(r.payload + 4, 2);
    for (i = 0; i < S2R_QUEUE_CAPACITY - S2R_QUEUE_RESERVED; i++)
    {
        assert(S2R_Queue_Push(&q, &r, 0));
    }
    assert(!S2R_Queue_Push(&q, &r, 0));
    for (i = 0; i < S2R_QUEUE_RESERVED; i++)
    {
        assert(S2R_Queue_Push(&q, &r, 1));
    }
    assert(!S2R_Queue_Push(&q, &r, 1));
    assert(q.dropped == 2 && q.generated == 26);
    assert(q.peak_bytes == 24 * 68);
    for (i = 0; i < S2R_QUEUE_CAPACITY; i++)
    {
        assert(S2R_Queue_Pop(&q, &out));
        assert(out.seq == (i < 20 ? i : i + 1));
        assert(out.session == 7 && out.t_us == r.t_us);
    }
    assert(!S2R_Queue_Pop(&q, &out) && q.bytes == 0);
    assert(S2R_Queue_Push(&q, &r, 0));
    memset(r.payload, 0xEE, sizeof(r.payload));
    assert(S2R_Queue_Pop(&q, &out));
    assert(out.seq == 26 && out.payload[0] == 17);
    assert(S2R_Encode(bytes, 0x0102030405060708ULL, &out) == 68);
    assert(bytes[12] == 26 && bytes[16] == 8 && bytes[23] == 1);
    if (argc == 2)
    {
        file = fopen(argv[1], "wb");
        assert(file);
        assert(fwrite(bytes, 1, 68, file) == 68);
        assert(fclose(file) == 0);
    }
    r.len = S2R_PAYLOAD_MAX + 1;
    assert(!S2R_Encode(bytes, 0, &r));
    assert(!S2R_Queue_Push(&q, &r, 1));
    puts("wire CRC/endian/queue/reserved/drop/copy: PASS");
    return 0;
}
