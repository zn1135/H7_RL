# USB CDC 遥测发送（MC02H7）

当前工程的 USB_OTG_HS 使用 PA11/PA12 和内置 Full Speed PHY，USB 链路为 12 Mbps；没有切到外部 ULPI High Speed PHY。VOFA 与 S2R 共用传输选择器，本轮 UART 直采上电默认走机器表指定的 UART8，USB CDC 保留为可选传输。保持原来的 S2R > 策略 VOFA > 旧 VOFA 互斥顺序。VOFA 帧和 S2R1 帧格式不变，两种格式不能混在同一次采集文件中。

## 发送与切换

`Vofa_send.c` 是共用的非阻塞发送入口。`vofa_transport.requested/active` 使用相同枚举，上电均为 1（UART）；失能、S2R 退出且上一帧发送完成后，将 `requested` 写 0 切到 USB CDC，写 1 返回机器表指定的 UART8/USART1。其他值不触发切换。切换后策略 VOFA 清队列并请求重发历史。USB 未枚举或 CDC 仍忙时，发送入口返回忙，调用方保留待发帧；USB 传输完成前不覆写发送缓冲。

板端 USB 数据口接电脑后使用其虚拟 COM 口。CDC line coding 默认报告 1152000、8N1，VOFA+ 和 `s2r_capture.py` 可沿用这一设置；该数值不控制 USB 线上速率。UART 回退仍为实际的 1152000、8N1。`S2R_Pump()` 沿用原先按 1152000 串口时间估算的保守帧间节流。

## DMA 与 DCache

USB 控制器的内部 DMA 在 `usbd_conf.c` 的 `USER CODE` MSP 初始化段、`USB_CoreInit()` 之前启用；CubeMX 生成的 `dma_enable = DISABLE` 赋值因此会被该段覆盖。两套链接配置都预留 `0x2404C000` 起的 16 KiB AXI SRAM；`main.c` 在开启 DCache 前将这块 MPU 区域设为非缓存。USB PCD、CDC、描述符及控制缓冲的可写数据放在此区，GCC 工程在启动后首次使用前复制已初始化数据并清零其余数据。USB DMA 不能直接读取 DTCM 中的缓冲。

VOFA 帧缓冲与 S2R 编码缓冲保留在 DMA 可访问的 AXI SRAM，且 32 字节对齐；每次提交 USB DMA 或 UART DMA 发送前先清理对应的 DCache 行。USB CDC 的 `TxState` 在整个异步传输期间占用该缓冲。不要通过关闭全局 DCache 代替上述内存安排。

## 上机验收

本次源码与内存布局尚待用户构建、烧录和台架验收。构建前运行 `py -3 tools/s2r_build_info.py`；检查 GCC/Keil map 中 USB 可写对象均落在 `0x2404C000`～`0x2404FFFF`，两个遥测帧缓冲落在 AXI SRAM。先验 USB 枚举与 COM 打开，再做失能状态下的 VOFA 5 秒连续采集，要求采样序号、辅助帧连续；随后分别验 VOFA 起立短窗与 S2R 的 CRC、序号缺口和回放。出现 USB 未配置或长时间忙时，先看接收端是否打开 COM，不改控制参数。
