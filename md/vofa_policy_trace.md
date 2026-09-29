# 策略 VOFA 采集

遥测口上电默认通过机器表指定的 UART 发送 32 通道 JustFloat 策略追踪，当前大机、小机均为 UART8 PE1，1152000、8N1。UART TX 接串口转接器 RX，两端 GND 共地，电脑选择转接器对应的 COM 口。每帧为 32 个小端 float32 和 `00 00 80 7F` 帧尾，共 132 字节。策略任务按 100 Hz 节拍生成采样，通信任务异步发送，不等待传输完成。`vofa_transport.requested/active` 使用相同枚举：1 为 UART，0 为可选的板载 USB CDC；切换及 DCache 布局见 [USB CDC 遥测发送](usb-cdc-telemetry.md)。

## 旧布局在线诊断（备用）

`task_comm.c::Robot_Control_Send_Vofa()` 的旧布局保留为备用，约 50 Hz。VOFA+ 仍选 JustFloat、32 通道。此布局的 ch0 是在线掩码：bit0 = IMU、bit1 = DR16、bit2～5 = 四路 DM、bit6～7 = 两路轮；全在线为 255，仅 DR16 离线为 253。ch1 的 bit0 = 电机使能、bit9 = RL 已投入；ch2 为 RL 状态位，ch31 为 `ctrl_fault`（仅 `FAULT_RC` 时等于 2）。先看 ch0 是否含 `0x02`，再看 ch31 与 ch1。旧布局不含策略帧类型、序号和 MCU 时间戳，不能交给下文的策略追踪解码脚本。

电机失能期间，旧布局临时用 ch23～28 定位 UART9 遥控接收边界；电机使能后，这六路恢复为上一拍动作：

| 通道 | 失能时的值 | 判读 |
| --- | --- | --- |
| ch23 | `huart9.RxState` | 正常 DMA 接收中为 34（`HAL_UART_STATE_BUSY_RX`） |
| ch24 | `hdma_uart9_rx.State` | 正常 DMA 进行中为 2（`HAL_DMA_STATE_BUSY`） |
| ch25 | UART9 RX DMA 剩余计数 | 收到字节时会变化；连续观察多帧 |
| ch26 | `dbus_rx.dma_pos` | UART9 IDLE 中断取到新字节时更新 |
| ch27 | `dbus_rx.isr_len` | 最近一次 IDLE 收到的字节数，不是累计值 |
| ch28 | `dr16.last_rx_tick`（ms） | 仅有效 DBUS 帧会刷新；离线超时后 `dr16` 被清零，值会回到 0 |

若 ch25 变化而 ch26 不变，优先核对 IDLE 中断；ch26 更新而 ch28 始终为 0，则继续查帧长度、串口格式和 `DR16_Validate()`。ch25/ch26 都不动时，先查 UART9 RX 的实际信号和 DMA 状态。通道读数是运行时诊断，不代表已经确认故障原因。

当前上电默认是下文的策略追踪。电机失能且上一帧发送完成时，将 `vofa_trace_requested` 写 0 可切到旧布局；写 1 返回策略追踪。两种布局应分别保存采集文件。S2R 上电仍不占遥测口，需显式将 `s2r_diagnostic_requested` 写 1 才切换。

本采集用于排查 chuanliantui 起立失败时，板端实际送入网络的观测与训练接口之间的差异。训练侧接口以 Wheel-Legged-Gym 仓库 `docs/deployment-contract.md` 的 `chuanliantui-25d-100hz-r1` 和对应训练实现为准；板端字段定义以本仓库 `imcalib/Algorithm/rl_observation.h`、采样实现以 `imcalib/task/task_policy.c` 和 `imcalib/Telemetry/vofa_trace.c` 为准。采集本身不证明两端数值或真机行为一致。

## 帧布局

所有帧的 ch0 是类型，ch1 是策略采样序号（模 `2^24`），ch2～4 是 MCU 单调微秒时间戳的低、中、高三个 16 位字，ch5 是状态位。三个时间字段和序号都能被 float32 精确表示。时间戳表示策略任务取到 IMU 快照的时刻，不是电脑收到串口字节的时刻。

| 类型 ch0 | ch6～30 | ch31 | 时间戳 |
| --- | --- | --- | --- |
| 0 | 实际输入网络的 25 维当前观测，顺序见 `rl_observation.h` | IMU 传感器时间戳 ms，模 `2^24` | 观测采样时刻 |
| 1 | 3 维机器表映射后的 IMU 角速度（rad/s，未乘观测缩放）、4 维四元数、3 维加速度、3 维欧拉角、6 维训练空间发布动作、4 路 DM 最新下发力矩、2 路轮最新下发力矩 | 推理耗时 µs；0 = 未推理，-1 = 推理失败 | 动作发布后的采样时刻 |
| 2～6 | 同一策略周期的历史第 0～4 帧，每帧 25 维，0 最旧、4 最新 | VOFA 入队丢帧累计，模 `2^24` | 同类型 0 |

类型 1 的 IMU 数据来自构建观测时的同一份 `imu_state` 快照；观测的前 3 维已乘训练缩放，接下来 3 维为四元数计算的投影重力。类型 1 的动作是本次发布的训练空间动作。下发力矩来自执行任务最近一次写入的值，**不保证由同一序号的动作生成**；结合两个 MCU 时间戳看先后关系，不要将二者直接视作同拍输入输出。

类型 2～6 在首次 RL 投入且历史有效时发送，此后每秒重发一次。其余周期用类型 0 的观测逐帧重建五帧历史。历史缺少类型 0、初始同步帧不齐或校验不一致时，解码器把该段标为未同步，直到下次收到完整五帧同步。

ch5 状态位（低 24 位）：0 已投入、1 观测有效、2 历史有效、3 动作可用、4 IMU 在线、5 电机使能、6 跌倒、7 模型就绪、8～12 `ctrl_fault` 的五位、13/14 DM/轮发送成功、15 遥控在线、16～19 四路 DM 在线、20～21 保留、22～23 两路轮在线。

## 采集与验收

连续发送基线为 `2 × 132 × 100 = 26.4 kB/s`，每秒五帧历史再加 `0.66 kB/s`，合计约 **27.1 kB/s**。默认 UART8 在 1152000、8N1 下的理论有效字节上限为 115.2 kB/s，转接设备的实际吞吐需实测。可选的板载 USB CDC 绕过外部串口转接设备，其 line coding 不决定 USB 链路速率。无论选择哪条链路，都以本机采集的序号缺口与历史同步结果验收。

### 直接接收原始字节

关闭 VOFA+ 对该 COM 口的连接，使用 `tools/vofa_trace_capture.py` 独占串口接收。工具不发送控制字节，不启用软硬件流控，并在打开端口前将 DTR/RTS 设为低。接收期间只保存字节和打印接收量，结束后再解码，避免 CSV 处理拖慢读取。固件仍发送原来的 0～6 类帧，本次没有增加板端诊断计数或额外帧。

在仓库根目录运行（将 COM5 换成实际串口，每次使用新的输出目录）：

```powershell
py -3 tools/vofa_trace_capture.py --list-ports
py -3 tools/vofa_trace_capture.py --port COM5 --seconds 30 --output captures/disable-01
py -3 tools/vofa_trace_capture.py --port COM5 --seconds 0 --output captures/enable-01
```

`--seconds 0` 表示持续接收，按 Ctrl+C 停止并解码。失能段约 30 秒；投入段从失能时开始记录，投入、退出后再保留几秒。每次启动创建独立目录，不会沿用上次缓存，也不会覆盖已有目录。依赖 `pyserial`，缺少时运行 `py -3 -m pip install pyserial`。

输出 `raw.bin` 为收到的原始字节；`vofa_trace.csv` 为解码结果；`summary.json` 为序号、辅助帧及历史同步检查；`capture_summary.json` 为端口、字节数、SHA-256、时长及停止原因。串口中断后保留已收到的字节并尝试解码，进程返回非零退出码。无有效观测也返回非零，不能把空文件的零缺口解释为通过。

原始解码器以 132 字节候选帧及合法帧头恢复边界，候选不合法时继续寻找，不因数据内部偶然出现 `00 00 80 7F` 就丢弃整个帧。已覆盖“零动作后跟传感器时间戳 65791”产生假帧尾的回归用例。JustFloat 没有 CRC，此检查不能证明载荷完全无损；保留 `raw.bin` 用于复核。起止落在半帧时，`ignored_bytes_or_rows` 可能非零，应结合中间的序号缺口判断。

### VOFA+ 导出的 CSV（兼容保留）

VOFA+ 在 UART 转接器 COM 口选择 `JustFloat`、32 通道、1152000、8N1 并保存 CSV。解码脚本要求 CSV 最后 32 列依次为通道 0～31，前面可有时间列。帧类型交织在同一串口流里，直接把不同类型的同一通道画在一条曲线上会混淆含义；分析时用下方脚本按 ch0 分帧。也可被动保存串口原始字节为 `.bin`，脚本同样支持。它不向机器人发送任何控制字节。

```bash
python tools/vofa_trace_decode.py vofa_capture.csv vofa_decoded_01
# 或
python tools/vofa_trace_decode.py vofa_capture.bin vofa_decoded_01
```

输出 `vofa_trace.csv` 含每个策略周期的完整观测、IMU、动作及重建历史，`summary.json` 含序号缺口、辅助帧缺失、历史同步失败和解析忽略量。建议先在电机失能状态录 30 秒，检查序号缺口、辅助帧缺失、历史校验错误均为 0；再做短时投入。若模式切换或上电造成序号重置，请分别保存文件并分析。电脑或串口助手显示的时间戳只是接收时刻，控制延迟分析使用 MCU 微秒时间戳。

比较起立故障时，先看 `obs_*` 的投影重力、角速度、关节位置与速度是否和视频姿态一致，再看 `hist_*` 是否已同步、`rl_ready`／`policy_ready` 与推理耗时是否表明策略真正接管。`action_*` 是策略输出，`dm_cmd_*`／`wheel_cmd_*` 是最近一次执行任务下发的力矩；结合各自 MCU 时间戳判断先后，不直接按同一行计算控制增益。若 `summary.json` 有丢样或历史失同步，先排除串口链路问题，再比较训练与实机数值。

当前上电默认使用策略追踪；旧布局的切换条件见上文“旧布局在线诊断（备用）”。
