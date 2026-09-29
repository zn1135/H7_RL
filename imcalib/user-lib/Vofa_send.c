#include "Vofa_send.h"
#include "dma_cache.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* 帧尾 */
static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};
vofa_transport_t vofa_transport = {VOFA_TRANSPORT_UART, VOFA_TRANSPORT_UART};

uint8_t Vofa_Transport_Ready(void)
{
    if (vofa_transport.active == VOFA_TRANSPORT_USB)
    {
        return CDC_Transmit_Ready_HS();
    }
    return (uint8_t)((VOFA_UART)->gState == HAL_UART_STATE_READY);
}

uint8_t Vofa_Transport_Idle(void)
{
    if (vofa_transport.active == VOFA_TRANSPORT_USB)
    {
        return CDC_Transmit_Idle_HS();
    }
    return (uint8_t)((VOFA_UART)->gState == HAL_UART_STATE_READY);
}

uint8_t Vofa_Transport_Update(uint8_t can_switch)
{
    uint8_t desired;

    desired = vofa_transport.requested;
    if (desired != VOFA_TRANSPORT_UART && desired != VOFA_TRANSPORT_USB)
    {
        return 0u;
    }
    if (desired == vofa_transport.active || !can_switch || !Vofa_Transport_Idle())
    {
        return 0u;
    }
    vofa_transport.active = desired;
    return 1u;
}

uint8_t Vofa_Transport_Send(uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0u || !Vofa_Transport_Ready())
    {
        return 0u;
    }
    Dma_Cache_Clean_Tx(data, len);
    if (vofa_transport.active == VOFA_TRANSPORT_USB)
    {
        return (uint8_t)(CDC_Transmit_HS(data, len) == USBD_OK);
    }
    return (uint8_t)(HAL_UART_Transmit_DMA(VOFA_UART, data, len) == HAL_OK);
}

uint8_t Vofa_Send(const float *data, uint8_t n)
{
    static uint8_t buf[VOFA_MAX_CH * sizeof(float) + 4]
        __attribute__((aligned(DMA_CACHE_LINE_SIZE)));  /* DMA 异步读，不能放栈 */
    uint16_t data_len;

    if (data == NULL || n == 0 || n > VOFA_MAX_CH)
    {
        return 0u;
    }
    data_len = n * sizeof(float);

    /* DMA 尚未发送完上一帧时，不能覆写它正在读取的静态缓冲区。 */
    if (!Vofa_Transport_Ready())
    {
        return 0u;
    }

    memcpy(buf, data, data_len);
    memcpy(buf + data_len, tail, 4);

    return Vofa_Transport_Send(buf, data_len + 4u);
}
