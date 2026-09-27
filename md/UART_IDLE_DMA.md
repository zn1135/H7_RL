# UART IDLE + DMA 接收框架

> 配合文件：[`uart_idle.h`](../imcalib/user-lib/uart_idle.h)、[`uart_idle.c`](../imcalib/user-lib/uart_idle.c)

---

## 1. 架构

```
uart_idle.c/h    ← 底层框架 (ISR + 初始化)
dr16.c/h         ← 设备层 (遥控器: 判断+拷贝+解析)
hi229.c/h        ← 设备层 (IMU: 判断+扫描+解析)
```

---

## 2. 数据流

```
UART9 RX → DMA → dbus_rx.dma_buf (Circular)
UART7 RX → DMA → hi229_rx.dma_buf (Circular)
    ↓ IDLE 中断 (一帧结束)
UART_Idle_Isr
    ↓ 清错误 → 按 dma_pos 推进环形位置 → 快照拷入 isr_buf → 置 isr_len/flag
dbus_rx.flag = 1 / hi229_rx.flag = 1
    ↓ 任务层调用 XXX_Process()
UART_Rx_Take（PRIMASK 临界区）取走 isr_buf 并清 flag → 校验 → 解析
```

ISR 侧细节见 `uart_idle.c` 的 `UART_Idle_Isr()`：先清 PE/FE/NE/ORE 与 ErrorCode，再按 DMA 剩余计数算出本段长度（相对 `dma_pos` 环形推进，回绕时两段拷贝），拷贝前做 D-cache invalidate（`Dma_Cache_Invalidate_Rx`）。快照完成后只置 `isr_len`/`flag`，不在 ISR 里解析。

---

## 3. 底层 API (uart_idle.c/h)

```c
/* 结构体（与 uart_idle.h 对齐） */
typedef struct {
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma_rx;
    uint8_t  dma_buf[DEBUG_BUF_SIZE] __attribute__((aligned(DMA_CACHE_LINE_SIZE)));
    uint8_t  isr_buf[DEBUG_BUF_SIZE];
    volatile uint16_t isr_len;
    volatile uint8_t  flag;
    uint16_t buf_size;
    volatile uint16_t dma_pos;   /* 环形缓冲消费位置 */
    UART_Parse_cb parse;         /* 未使用：框架从不调用 */
} UART_Rx_t;

/* 实例：dbus_rx、hi229_rx 在用；debug_rx 未使用（仅定义，未 Init/未进 ISR） */

/* 初始化（第 5 参 parse 当前不会被调用，传 NULL 或桩函数） */
UART_Rx_Init(&dbus_rx, &huart9, &hdma_uart9_rx, DBUS_BUF_SIZE, DR16_Rx_Cb);
UART_Rx_Init(&hi229_rx, &huart7, &hdma_uart7_rx, HI229_BUF_SIZE, NULL);

/* ISR（stm32h7xx_it.c 的 UARTn_IRQHandler 里调用） */
UART_Idle_Isr(&huart9, &dbus_rx);
UART_Idle_Isr(&huart7, &hi229_rx);

/* 任务侧取快照：PRIMASK 临界区内拷贝 isr_buf → dst 并清 flag，返回长度 */
uint16_t UART_Rx_Take(UART_Rx_t *rx, uint8_t *dst, uint16_t capacity);
```

（历史：`UART_Parse_cb parse` 回调机制从未被调用，仅在 `UART_Rx_Init` 里保存；解析一律在任务侧 `XXX_Process` 内完成。早期文档初始化示例写作 `DBUS_Parse` + `&huart1`，已按当前代码更正为 UART9/UART7。）

---

## 4. 设备层 API（dr16.c/h / hi229.c/h）

```c
/* 任务里只需要调这个 */
DR16_Process();

/* 内部实现（dr16.c） */
void DR16_Process(void) {
    uint8_t buf[DBUS_BUF_SIZE];
    uint16_t len = UART_Rx_Take(&dbus_rx, buf, sizeof(buf));
                         ← PRIMASK 临界区取走 isr_buf 并清 flag，返回长度
    if (len >= DR16_FRAME_LEN) {
        DR16_Parse(buf); ← 解析（校验不过直接丢弃）
    }
}
```

HI229 同构：`HI229_Process()` 调 `UART_Rx_Take(&hi229_rx, ...)` 取快照后，在块内扫描完整帧（帧头 + 长度 + CRC16 校验）再解析，见 `hi229.c`。两者分别在 `task_comm.c` / `task_imu.c` 每拍调用。

---

## 5. 为什么用 IDLE 中断

DMA 全传输中断只在 buffer 收满时触发。如果一帧没有填满 buffer，中断不会来，数据就"卡"在 DMA buffer 里。UART IDLE 中断在线上无数据时触发，即一帧发完就通知 CPU。

---

## 6. 为什么关 DMA 中断

HAL 默认的 `HAL_UART_Receive_DMA()` 会开启 DMA 的半传输(HT)和全传输(TC)中断。这两个中断我们不需要（靠 IDLE 就够了），留着会产生无用中断。

---

## 7. ISR 中的错误处理

不诊断错误原因，只关心帧是否对齐。串口线上的干扰无法在软件层面修复，清掉标志等下一帧。

---

## 8. CubeMX 配置要求

| 配置项 | UART9 (DR16) | UART7 (HI229) |
|--------|--------------|-------------|
| Mode | 异步 | 异步 |
| DMA RX | Circular | Circular |
| NVIC | 全局中断使能 | 全局中断使能 |

---

## 9. 扩展新设备

1. 声明 `UART_Rx_t xxx_rx`（加进 `uart_idle.c/h` 的实例列表）+ 各自 BUF_SIZE 宏
2. `UART_Rx_Init(&xxx_rx, &huartN, &hdma_uartN_rx, XXX_BUF_SIZE, NULL)` 初始化（末参 parse 可传 NULL——该回调当前不被调用）
3. 对应 `UARTn_IRQHandler` 里加 `UART_Idle_Isr(&huartN, &xxx_rx)`（`Core/Src/stm32h7xx_it.c`）
4. 写 `XXX_Process()`：`UART_Rx_Take()` 临界区取快照 → 校验 → 解析
5. 任务里每拍调 `XXX_Process()`
