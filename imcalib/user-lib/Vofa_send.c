#include "Vofa_send.h"
#include "dma_cache.h"
#include <string.h>

/* 帧尾 */
static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

void Vofa_Send(const float *data, uint8_t n)
{
    if (data == NULL || n == 0 || n > VOFA_MAX_CH) return;

    static uint8_t buf[VOFA_MAX_CH * sizeof(float) + 4]
        __attribute__((aligned(DMA_CACHE_LINE_SIZE)));  /* DMA 异步读，不能放栈 */
    uint16_t data_len = n * sizeof(float);

    /* DMA 尚未发送完上一帧时，不能覆写它正在读取的静态缓冲区。 */
    if ((VOFA_UART)->gState != HAL_UART_STATE_READY) return;

    memcpy(buf, data, data_len);
    memcpy(buf + data_len, tail, 4);

    Dma_Cache_Clean_Tx(buf, data_len + sizeof(tail));
    HAL_UART_Transmit_DMA(VOFA_UART, buf, data_len + 4);
}
