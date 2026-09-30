# USB CDC 遥测发送（MC02H7）

> 当前新增独立的关节 USB 双向试验协议，详见 [joint-usb-sysid.md](joint-usb-sysid.md)。本页描述原 VOFA/S2R 遥测发送路径及历史上机结果。

当前工程的 USB_OTG_HS 使用 PA11/PA12 和内置 Full Speed PHY，USB 链路为 12 Mbps；没有切到外部 ULPI High Speed PHY。当前测试固件上电将普通 VOFA JustFloat 留在 UART8，板载 USB CDC 交给 `JID1` 双向通信。VOFA 与 S2R 仍共用遥测传输选择器，S2R、策略 VOFA、普通 VOFA 保持同口互斥，两种帧格式不能混在同一次采集文件中。

另有默认关闭的 `HPI1` 上位机策略模式，使用同一 USB CDC 端口；它与 `JID1`
以及 USB 端遥测互斥，详情见 [HPI1 说明](host-policy-usb.md)。

## 发送与切换

`Vofa_send.c` 是共用的非阻塞发送入口。`vofa_transport.requested/active` 使用相同枚举，上电均为 1（UART8）；失能、S2R 退出、关节 USB 未占用且上一帧发送完成后，将 `requested` 写 0 切到 USB CDC，写 1 返回机器表指定的 UART8/USART1。其他值不触发切换。切换后策略 VOFA 清队列并请求重发历史。USB 未枚举或 CDC 仍忙时，发送入口返回忙，调用方保留待发帧；USB 传输完成前不覆写发送缓冲。

USB CDC 由遥测和 `JID1` 关节试验二选一占用：当前 `vofa_transport.active=1`，USB CDC 可用于 `JID1`。切到 `active=0` 的 USB 遥测时，`JointUsb` 丢弃 USB 接收命令、不能使能关节输出且不发送 `JID1`；物理许可挡位仍保持原有电机失能门控。切换回 USB 遥测需先失能、退出关节 USB 模式及停止采样流。当前模式必须以烧录后的实机收包确认，不能只看枚举成功。

板端 USB 数据口接电脑后使用其虚拟 COM 口。设备管理器中应出现 `STM32 Virtual ComPort`（VID 0483、PID 5740）；不要选调试器或其他 USB 转串口的 COM 口。CDC line coding 默认报告 1152000、8N1，VOFA+ 和 `s2r_capture.py` 可沿用这一设置；该数值不控制 USB 线上速率。UART 回退仍为实际的 1152000、8N1。`S2R_Pump()` 沿用原先按 1152000 串口时间估算的保守帧间节流。

## DMA 与 DCache

当前固件将 USB 控制器内部 DMA 设为 `DISABLE`，改由 USB FIFO 中断发送；在普通 VOFA、USB 占用互斥已修正的条件下，实机持续收包通过。两套链接配置仍预留 `0x2404C000` 起的 16 KiB AXI SRAM；`main.c` 在开启 DCache 前将这块 MPU 区域设为非缓存。USB PCD、CDC、描述符及控制缓冲的可写数据放在此区，GCC 工程在启动后首次使用前复制已初始化数据并清零其余数据。

VOFA 帧缓冲与 S2R 编码缓冲保留在 DMA 可访问的 AXI SRAM，且 32 字节对齐；发送入口仍执行 DCache 清理，兼容 UART DMA 和后续恢复 USB DMA。USB CDC 的 `TxState` 在整个异步传输期间占用该缓冲。不要通过关闭全局 DCache 代替上述内存安排。

## 上机验收

2026-09-30 台架对照：USB 内部 DMA 开启时，COM70 枚举成功但 5 秒原始接收 0 字节；在同一普通 VOFA 布局、相同 USB 占用规则下关闭内部 DMA 后，COM70 连续 20 秒收到 1003 个完整 132 字节 JustFloat 帧，帧尾间距全部为 132 字节，所有 float 有限，约 50 Hz。原始字节和 CSV 保存在 `build/vofa-usb-pio-20260930/`。随后切到 UART8 遥测、USB CDC `JID1`，STATUS 与 100/100 次 ECHO 成功；失能状态下 5 秒收到 2500 条关节快照，主机序号缺口 0、CRC 错帧 0、板端丢弃 1 条旧快照。当前仅确认失能通信；策略投入时的控制周期、手动摆腿映射、非零命令及 S2R1 走 USB 仍需分别验收。Keil map 中 USB 可写对象落在 `0x2404C000`～`0x2404FFFF`，遥测帧缓冲落在 AXI SRAM。
