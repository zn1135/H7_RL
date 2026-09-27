#include "uart_idle.h"
#include "dma_cache.h"
#include <string.h>

UART_Rx_t dbus_rx   = {0};
UART_Rx_t debug_rx  = {0};
UART_Rx_t hi229_rx  = {0};

void UART_Rx_Init(UART_Rx_t *rx, UART_HandleTypeDef *huart,
                  DMA_HandleTypeDef *hdma_rx, uint16_t size,
                  UART_Parse_cb parse)
{
    rx->huart    = huart;
    rx->hdma_rx  = hdma_rx;
    rx->buf_size = size;
    rx->dma_pos  = 0;
    rx->parse    = parse;

    Dma_Cache_Invalidate_Rx(rx->dma_buf, size);
    HAL_UART_Receive_DMA(huart, rx->dma_buf, size);

    __HAL_DMA_DISABLE_IT(hdma_rx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(hdma_rx, DMA_IT_TC);

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    __HAL_UART_ENABLE_IT(huart, UART_IT_PE);
    __HAL_UART_ENABLE_IT(huart, UART_IT_ERR);
}

void UART_Idle_Isr(UART_HandleTypeDef *huart, UART_Rx_t *rx)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))
        __HAL_UART_CLEAR_PEFLAG(huart);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))
        __HAL_UART_CLEAR_FEFLAG(huart);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))
        __HAL_UART_CLEAR_NEFLAG(huart);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
        __HAL_UART_CLEAR_OREFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    if (!__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
        return;
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    uint16_t pos = rx->buf_size - (uint16_t)__HAL_DMA_GET_COUNTER(rx->hdma_rx);
    uint16_t len = (pos >= rx->dma_pos) ? (pos - rx->dma_pos)
                                         : (rx->buf_size - rx->dma_pos + pos);
    if (len == 0)
        return;

    Dma_Cache_Invalidate_Rx(rx->dma_buf, rx->buf_size);

    uint16_t first_len = rx->buf_size - rx->dma_pos;
    if (first_len > len) first_len = len;
    memcpy(rx->isr_buf, &rx->dma_buf[rx->dma_pos], first_len);
    if (len > first_len)
        memcpy(&rx->isr_buf[first_len], rx->dma_buf, len - first_len);

    rx->dma_pos = pos;
    rx->isr_len = len;
    rx->flag    = 1;
}

/* 在短临界区内取完整快照，避免 ISR 覆盖任务正在解析的缓冲区。 */
uint16_t UART_Rx_Take(UART_Rx_t *rx, uint8_t *dst, uint16_t capacity)
{
    uint32_t primask;
    uint16_t len;

    if (rx == NULL || dst == NULL || capacity == 0u)
    {
        return 0u;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    len = rx->flag ? rx->isr_len : 0u;
    if (len > capacity)
    {
        len = 0u;
    }
    if (len > 0u)
    {
        memcpy(dst, rx->isr_buf, len);
    }
    rx->flag = 0u;
    __set_PRIMASK(primask);
    return len;
}
