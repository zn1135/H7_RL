#ifndef DMA_CACHE_H
#define DMA_CACHE_H

#include "main.h"
#include <stdint.h>

#define DMA_CACHE_LINE_SIZE  32u

/* 清理发送缓存 */
static inline void Dma_Cache_Clean_Tx(const void *address, uint32_t length)
{
    uintptr_t first_line;
    uintptr_t after_last_line;

    if (address == NULL || length == 0u
        || (SCB->CCR & SCB_CCR_DC_Msk) == 0u)
        return;

    first_line = (uintptr_t)address & ~((uintptr_t)DMA_CACHE_LINE_SIZE - 1u);
    after_last_line = ((uintptr_t)address + length + DMA_CACHE_LINE_SIZE - 1u)
                    & ~((uintptr_t)DMA_CACHE_LINE_SIZE - 1u);
    SCB_CleanDCache_by_Addr((uint32_t *)first_line,
                            (int32_t)(after_last_line - first_line));
    __DSB();
}

/* 刷新接收缓存 */
static inline void Dma_Cache_Invalidate_Rx(const void *address, uint32_t length)
{
    uintptr_t first_line;
    uintptr_t after_last_line;

    if (address == NULL || length == 0u
        || (SCB->CCR & SCB_CCR_DC_Msk) == 0u)
        return;

    first_line = (uintptr_t)address & ~((uintptr_t)DMA_CACHE_LINE_SIZE - 1u);
    after_last_line = ((uintptr_t)address + length + DMA_CACHE_LINE_SIZE - 1u)
                    & ~((uintptr_t)DMA_CACHE_LINE_SIZE - 1u);
    SCB_InvalidateDCache_by_Addr((uint32_t *)first_line,
                                 (int32_t)(after_last_line - first_line));
    __DSB();
}

#endif
