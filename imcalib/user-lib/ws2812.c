#include "ws2812.h"

#define WS2812_LowLevel    0xC0     // 0��
#define WS2812_HighLevel   0xF0     // 1��

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];
    uint8_t res = 0;
    for (int i = 0; i < 8; i++)
    {
        txbuf[7-i]  = (((g>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[15-i] = (((r>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[23-i] = (((b>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
    }
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 0, 0xFFFF);
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 0xFFFF);
    for (int i = 0; i < 100; i++)
    {
        HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 1, 0xFFFF);
    }
}

/* 色相 0~360 -> RGB（全饱和全亮，彩虹色环） */
static void Hue_To_RGB(uint16_t hue, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t region = (uint8_t)(hue / 60u);                 /* 0~5 */
    uint8_t rem    = (uint8_t)((hue % 60u) * 255u / 60u); /* 0~255 */
    uint8_t up     = rem;                                  /* 上升分量 */
    uint8_t down   = 255u - rem;                           /* 下降分量 */

    switch (region)
    {
        case 0: *r = 255u; *g = up;   *b = 0u;    break;   /* 红 -> 黄 */
        case 1: *r = down; *g = 255u; *b = 0u;    break;   /* 黄 -> 绿 */
        case 2: *r = 0u;   *g = 255u; *b = up;    break;   /* 绿 -> 青 */
        case 3: *r = 0u;   *g = down; *b = 255u;  break;   /* 青 -> 蓝 */
        case 4: *r = up;   *g = 0u;   *b = 255u;  break;   /* 蓝 -> 紫 */
        default:*r = 255u; *g = 0u;   *b = down;  break;   /* 紫 -> 红 */
    }
}

/* 每 500Hz tick 调用一次：1秒闪2次 + 彩虹变色 */
void WS2812_RainbowBlink(void)
{
    static uint32_t tick      = 0u;
    static uint16_t last_hue  = 0xFFFFu;
    static uint8_t  last_on   = 0xFFu;

    uint8_t  on  = (uint8_t)(((tick / 250u) & 1u) == 0u); /* 亮半周期 */
    uint16_t hue = (uint16_t)((tick / 10u)  % 360u);      /* 色相步进 */

    if (on != last_on || (on && hue != last_hue))
    {
        uint8_t r, g, b;
        if (on) Hue_To_RGB(hue, &r, &g, &b);
        else    r = g = b = 0u;
        WS2812_Ctrl(r, g, b);
    }
    last_hue = hue;
    last_on  = on;
    tick++;
}