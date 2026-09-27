#include "hi229.h"
#include "uart_idle.h"
#include "usart.h"
#include <string.h>

/* ============================================================================
 * 运行时状态
 * ==========================================================================*/
hi229_data_t hi229_data = {0};

static uint16_t HI229_Crc16(uint16_t crc, const uint8_t *data, uint16_t len)
{
    while (len-- > 0u)
    {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0u; i < 8u; i++)
        {
            crc = (crc & 0x8000u)
                ? (uint16_t)((crc << 1) ^ 0x1021u)
                : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ============================================================================
 * 初始化 — UART7 + DMA + IDLE 接收
 * ==========================================================================*/
void HI229_Init(void)
{
    UART_Rx_Init(&hi229_rx, &huart7, &hdma_uart7_rx, HI229_BUF_SIZE, NULL);
}

/* ============================================================================
 * 任务侧解析 — 帧头 + 长度校验，直接写入 hi229_data
 * ==========================================================================*/
void HI229_Process(void)
{
    uint8_t buf[HI229_BUF_SIZE];
    uint16_t len = UART_Rx_Take(&hi229_rx, buf, sizeof(buf));
    const hi229_frame_t *raw = NULL;

    /* IDLE 边界不等于协议帧边界：搜索块内的完整 HI229 帧。 */
    for (uint16_t offset = 0; offset + HI229_FRAME_LEN <= len; offset++)
    {
        uint16_t data_len;

        if (buf[offset] != HI229_HEADER_0
            || buf[offset + 1u] != HI229_HEADER_1)
            continue;

        data_len = (uint16_t)buf[offset + 2u]
                 | ((uint16_t)buf[offset + 3u] << 8);
        if (data_len != HI229_PAYLOAD_LEN)
            continue;

        uint16_t crc = HI229_Crc16(0u, &buf[offset], 4u);
        crc = HI229_Crc16(crc, &buf[offset + 6u], HI229_PAYLOAD_LEN);
        uint16_t received_crc = (uint16_t)buf[offset + 4u]
                              | ((uint16_t)buf[offset + 5u] << 8);
        if (crc != received_crc)
            continue;

        raw = (const hi229_frame_t *)&buf[offset + 6u];
        break;
    }
    if (raw == NULL) return;

    /* 解析 payload，直接写入 hi229_data */

    hi229_data.eul[HI229_ROLL]  = raw->eul[HI229_ROLL];
    hi229_data.eul[HI229_PITCH] = raw->eul[HI229_PITCH];
    hi229_data.eul[HI229_YAW]   = raw->eul[HI229_YAW];

    hi229_data.gyr[HI229_ROLL]  = raw->gyr[HI229_ROLL];
    hi229_data.gyr[HI229_PITCH] = raw->gyr[HI229_PITCH];
    hi229_data.gyr[HI229_YAW]   = raw->gyr[HI229_YAW];

    hi229_data.acc[HI229_ACC_X] = raw->acc[HI229_ACC_X];
    hi229_data.acc[HI229_ACC_Y] = raw->acc[HI229_ACC_Y];
    hi229_data.acc[HI229_ACC_Z] = raw->acc[HI229_ACC_Z];

    hi229_data.quat[HI229_QUAT_W] = raw->quat[HI229_QUAT_W];
    hi229_data.quat[HI229_QUAT_X] = raw->quat[HI229_QUAT_X];
    hi229_data.quat[HI229_QUAT_Y] = raw->quat[HI229_QUAT_Y];
    hi229_data.quat[HI229_QUAT_Z] = raw->quat[HI229_QUAT_Z];

    hi229_data.ts           = raw->ts;
    hi229_data.online       = true;
    hi229_data.last_rx_tick = HAL_GetTick();
}

/* ============================================================================
 * 在线检测 — 100ms 无有效帧 → 清零
 * ==========================================================================*/
bool HI229_Online(void)
{
    if (hi229_data.online && HAL_GetTick() - hi229_data.last_rx_tick > HI229_OFFLINE_MS)
    {
        hi229_data.online = false;
        memset(&hi229_data, 0, sizeof(hi229_data));
    }
    return hi229_data.online;
}

/* ============================================================================
 * 死区滤波
 * ==========================================================================*/
float HI229_Deadline(float input, float deadline)
{
    if (input > deadline)       return input - deadline;
    else if (input < -deadline) return input + deadline;
    else                        return 0.0f;
}

/* ============================================================================
 * 快照
 * ==========================================================================*/
hi229_data_t HI229_Snapshot(void)
{
    return hi229_data;
}
