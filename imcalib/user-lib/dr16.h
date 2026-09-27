#ifndef __DR16_H
#define __DR16_H

#include "main.h"
#include <stdbool.h>

#define DR16_FRAME_LEN   18
#define DR16_OFFLINE_MS  50u
#define DR16_CH_LIMIT    660

#define DR16_SW_UP       1
#define DR16_SW_MID      3
#define DR16_SW_DOWN     2

typedef struct {
    int16_t  ch0;
    int16_t  ch1;
    int16_t  ch2;
    int16_t  ch3;
    int16_t  wheel;
    uint8_t  s1;
    uint8_t  s2;
    int16_t  mx;
    int16_t  my;
    int16_t  mz;
    uint8_t  ml;
    uint8_t  mr;
    uint16_t key;
    bool     online;
    uint32_t last_rx_tick;
} dr16_t;

extern dr16_t dr16;

void        DR16_Init(void);
void        DR16_Process(void);
bool        DR16_Online(void);
int16_t     DR16_Deadline(int16_t input, uint16_t deadline);
dr16_t      DR16_Snapshot(void);

#define DR16_SW_LEFT_UP     (dr16.s1 == DR16_SW_UP)
#define DR16_SW_LEFT_MID    (dr16.s1 == DR16_SW_MID)
#define DR16_SW_LEFT_DOWN   (dr16.s1 == DR16_SW_DOWN)
#define DR16_SW_RIGHT_UP    (dr16.s2 == DR16_SW_UP)
#define DR16_SW_RIGHT_MID   (dr16.s2 == DR16_SW_MID)
#define DR16_SW_RIGHT_DOWN  (dr16.s2 == DR16_SW_DOWN)

#endif
