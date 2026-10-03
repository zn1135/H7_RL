#ifndef VOFA_SEND_H
#define VOFA_SEND_H

#include "main.h"
#include "usart.h"
#include "machine_config.h"

#define VOFA_MAX_CH  38u  /* 普通38/追踪32 */
#define VOFA_TRANSPORT_UART 1u
#define VOFA_TRANSPORT_USB  0u

typedef struct {
    volatile uint8_t requested;
    volatile uint8_t active;
} vofa_transport_t;

extern vofa_transport_t vofa_transport;

/* 默认 UART8，USB CDC 交给 JID1；requested/active: 1=UART，0=USB CDC。 */
/* UART 端口随 MACHINE_DEFAULT 自动切换。 */
/*   8 = UART8  (1152000, TX DMA 正常)                */
/*   1 = USART1 (1152000, TX DMA 正常)                */
/* 注意: UART7 被 HI229 占用且 TX DMA 是 CIRCULAR 模式; */
/*       UART9 被 DR16 占用且没有 TX DMA, 都不能选。     */
/* 波特率必须与 Vofa+ 一致 (CubeMX 里对应口已是 1152000)。*/
#if MACHINE_VOFA_PORT == 8u
#define VOFA_UART   &huart8
#elif MACHINE_VOFA_PORT == 1u
#define VOFA_UART   &huart1
#else
#error "MACHINE_VOFA_PORT must be 8 (UART8) or 1 (USART1)"
#endif

uint8_t Vofa_Send(const float *data, uint8_t n);
uint8_t Vofa_Transport_Ready(void);
uint8_t Vofa_Transport_Idle(void);
uint8_t Vofa_Transport_Update(uint8_t can_switch);
uint8_t Vofa_Transport_Send(uint8_t *data, uint16_t len);

#endif
