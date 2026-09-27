#ifndef __CAN_BUS_H
#define __CAN_BUS_H

#include "main.h"
#include "fdcan.h"
#include <stdbool.h>

#define CAN_BUS_NUM         3
#define CAN_BUS_ROUTE_MAX   8
#define CAN_BUS_DEAD_MS     100
#define CAN_BUS_REINIT_MS   100

/* 回调类型 */
typedef void (*can_parse_fn_t)(void *ctx, uint32_t can_id,
                                const uint8_t *data, uint8_t len);

/* 总线状态 */
typedef enum {
    CAN_BUS_STATE_ACTIVE = 0,
    CAN_BUS_STATE_DEAD,
} can_bus_state_t;

/* 电机固定参 (DM / DJI 共用): 报文 ID (总线按机器选) */
typedef struct {
    uint16_t             feedback_id;
    uint16_t             control_id;
} motor_cfg_t;


/* 路由条目 */
typedef struct {
    uint32_t       can_id;
    can_parse_fn_t parse;
    void          *ctx;
} can_route_t;

/* 每条总线运行时 */
typedef struct {
    can_route_t           route[CAN_BUS_ROUTE_MAX];
    volatile uint32_t     route_cnt;
    FDCAN_TxHeaderTypeDef tx_template;
    volatile can_bus_state_t state;
    volatile uint32_t     alive_cnt;
    volatile uint32_t     last_rx_id;    /* 最近收到帧的 CAN ID */
    volatile uint32_t     last_tx_tick;
    uint32_t              alive_prev;
    uint32_t              dead_since;
    uint32_t              reinit_tick;
} can_bus_t;

void    Can_Bus_Init(void);
bool    Can_Bus_Register(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id,
                         can_parse_fn_t fn, void *ctx);
HAL_StatusTypeDef Can_Bus_Transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id,
                                   const uint8_t *data, uint8_t len);
bool    Can_Bus_Online(bool expect_traffic);
/* 按总线号取句柄 (1=FDCAN1 2=FDCAN2 3=FDCAN3) */
FDCAN_HandleTypeDef *Can_Bus_Handle(uint8_t bus);
/* 收帧计数 / 最近一帧 ID (调试用) */
uint32_t Can_Bus_Rx_Count(uint8_t bus);
uint32_t Can_Bus_Last_Rx_Id(uint8_t bus);


#endif
