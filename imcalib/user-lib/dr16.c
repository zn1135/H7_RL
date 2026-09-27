#include "dr16.h"
#include "uart_idle.h"
#include "usart.h"
#include <string.h>

dr16_t dr16 = {0};
static volatile bool dr16_parsing = false;

/* 数据校验 */
static bool DR16_Validate(const dr16_t *r)
{
    if (r->ch0 < -DR16_CH_LIMIT || r->ch0 > DR16_CH_LIMIT) return false;
    if (r->ch1 < -DR16_CH_LIMIT || r->ch1 > DR16_CH_LIMIT) return false;
    if (r->ch2 < -DR16_CH_LIMIT || r->ch2 > DR16_CH_LIMIT) return false;
    if (r->ch3 < -DR16_CH_LIMIT || r->ch3 > DR16_CH_LIMIT) return false;
    if (r->wheel < -DR16_CH_LIMIT || r->wheel > DR16_CH_LIMIT) return false;
    if (r->s1 < 1 || r->s1 > 3) return false;
    if (r->s2 < 1 || r->s2 > 3) return false;
    if (r->ml > 1 || r->mr > 1) return false;
    return true;
}

/* 解析 */
static void DR16_Parse(const uint8_t *buf)
{
    dr16_t parsed;

    parsed.ch0 = (int16_t)(((buf[0] | (buf[1] << 8)) & 0x07FF) - 1024);
    parsed.ch1 = (int16_t)((((buf[1] >> 3) | (buf[2] << 5)) & 0x07FF) - 1024);
    parsed.ch2 = (int16_t)((((buf[2] >> 6) | (buf[3] << 2) | (buf[4] << 10)) & 0x07FF) - 1024);
    parsed.ch3 = (int16_t)((((buf[4] >> 1) | (buf[5] << 7)) & 0x07FF) - 1024);

    parsed.s1 = (uint8_t)(((buf[5] >> 4) & 0x000C) >> 2);
    parsed.s2 = (uint8_t)((buf[5] >> 4) & 0x0003);

    parsed.mx  = (int16_t)(buf[6]  | (buf[7]  << 8));
    parsed.my  = (int16_t)(buf[8]  | (buf[9]  << 8));
    parsed.mz  = (int16_t)(buf[10] | (buf[11] << 8));
    parsed.ml  = buf[12];
    parsed.mr  = buf[13];

    parsed.key = (uint16_t)(buf[14] | (buf[15] << 8));

    parsed.wheel = (int16_t)(1024 -
        ((uint16_t)buf[16] | ((uint16_t)buf[17] << 8)));

    if (!DR16_Validate(&parsed))
        return;

    dr16_parsing       = true;
    parsed.online      = true;
    parsed.last_rx_tick = HAL_GetTick();
    dr16               = parsed;
    dr16_parsing       = false;
}

/* 回调桩 */
static void DR16_Rx_Cb(const uint8_t *data, uint16_t len)
{
    (void)data;
    (void)len;
}

/* 初始化 */
void DR16_Init(void)
{
    UART_Rx_Init(&dbus_rx, &huart9, &hdma_uart9_rx, DBUS_BUF_SIZE, DR16_Rx_Cb);
}

/* 接收处理 */
void DR16_Process(void)
{
    uint8_t buf[DBUS_BUF_SIZE];
    uint16_t len = UART_Rx_Take(&dbus_rx, buf, sizeof(buf));

    /* 允许尾部带下一帧的字节; 帧长不足才丢弃 */
    if (len >= DR16_FRAME_LEN)
    {
        DR16_Parse(buf);
    }
}

/* 在线检测 */
bool DR16_Online(void)
{
    if (!dr16.online)
        return false;

    if (HAL_GetTick() - dr16.last_rx_tick > DR16_OFFLINE_MS)
    {
        memset(&dr16, 0, sizeof(dr16));
        return false;
    }
    return true;
}

/* 死区 */
int16_t DR16_Deadline(int16_t input, uint16_t deadline)
{
    if (input > (int16_t)deadline || input < -(int16_t)deadline)
        return input;
    return 0;
}

/* 快照: 正在接收则跳过，避免跨帧混值 */
dr16_t DR16_Snapshot(void)
{
    if (dr16_parsing)
        return dr16;
    return dr16;
}
