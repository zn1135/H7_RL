# 整机诊断串口接线、采集与验收

本次接入在 `little-wheelleg` 分支完成（自 `a824da1` 起，变更 104），与现有控制代码**去耦**：`imcalib/Telemetry/` 只读现有状态，任务层只有 `commTask` 里的 `S2R_Pump()` 一个挂点。协议布局见 [sim2real_serial_protocol.md](sim2real_serial_protocol.md)；哪些字段本分支拿不到，以 META 的 `unavailable` / `derived` 两栏为准。当前上电默认通过 UART8 发送 [普通 VOFA JustFloat](vofa_policy_trace.md)；本页的 S2R1 二进制采集仅在显式切换后适用。

## 1. 接线和准备

- 默认使用 UART8：板端 PE1（TX）经原无线串口链路接电脑，选择该链路对应的 COM 口。板载 USB CDC 仍可在后续切换测试，现状见 [传输说明](usb-cdc-telemetry.md)。
- USB CDC 的 1152000、8N1 是 line coding，USB 传输速率不由此控制；UART8 按实际 1152000、8N1、无流控运行。VOFA+ 与本工具不能同时占用同一 COM 口。
- 遥测口上电默认发普通 VOFA JustFloat，固定通道可由 VOFA+ 接收并保存 CSV。若要使用本页 S2R1 二进制流程，需在失能、无会话且发送完成时把 `s2r_diagnostic_requested` 写 1，并改用 `tools/s2r_capture.py`；两种协议互斥。UART7 仍接 IMU，UART9 仍接遥控。无诊断串口启动电机/策略指令。

采集工具不改变任何控制路径；要采集网络策略，按原有流程（左上挡 + 右中位）投入推理。

## 2. 构建前

在仓库根运行（`python` 指 Python 3.8 或更新环境）：

```bash
python tools/s2r_build_info.py
python tools/s2r_build_info.py --check
```

生成的 `imcalib/Telemetry/s2r_build_info.h` 报告源集 SHA256、生成时的 Git 基准和 dirty 状态、真实 CubeAI 产物中的模型名/签名，以及机器配置源文件 SHA256。修改源文件后必须重生成，再用 Keil/eIDE 构建，避免烧录代码与 META 身份不一致。当前模型签名是 `4899195601babb22a7c0b46cc151ad84`（MD5），不可只凭模型文件名判断版本。

两套工程已登记 Telemetry 源目录。`.s2r_dma` 缓冲保持在 AXI SRAM、32 字节对齐；USB PCD/CDC 与描述符的可写对象使用 `0x2404C000` 起的 16 KiB 非缓存区。USB DMA 的实际落点必须以重新构建后的 GCC/Keil map 核对，旧 map 地址和内存占用只是改动前快照。`S2R_Source_Tick()` 的采样块与 `event()` 的记录仍为静态量（commTask 栈只有 512 words）。

若没有输出，先检查调试变量 `s2r_init_error`：bit0 为启动 RNG/boot_id 失败，bit1 为 META 缓冲溢出。不要用固定 boot_id 替代错误；那会把不同次上电混成一段。

### 2.1 速率档（`S2R_RATE_LOW`）

`imcalib/Telemetry/s2r_telemetry.h` 的 `S2R_RATE_LOW` 用来限流（**只改发送周期，采样仍在 commTask 1 kHz 跑**；改完必须重生成指纹并重编）。USB CDC 首轮仍保留低速档，待实测无缺口后再调整：

| 档 | CONTROL | POLICY | IMU | HEALTH | HISTORY | META 重发 | 待机带宽 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0（原设定） | 100 Hz | 每次推理 | ≤50 Hz | 10 Hz | 2 Hz | 5 s | ≈58 kB/s（投入 ≈85） |
| 1（低速） | 10 Hz | 10 Hz | 5 Hz | 5 Hz | 1 Hz | 30 s | ≈9.5 kB/s |
| **2（稳健，台架默认）** | 5 Hz | 5 Hz | 2 Hz | 1 Hz | 0.2 Hz | 20 s | **≈4.3 kB/s** |
| 3（极低，只验链路） | 2 Hz | 2 Hz | 1 Hz | 0.5 Hz | 0.1 Hz | 60 s | ≈1.6 kB/s |

历史 PowerDebugger 串口桥实测 **6.25 kB/s 干净，33.6 kB/s 丢一半字节**；可选的板载 USB CDC 不经过这条桥；默认 UART 路径需按实际转接设备重新验收。尚需在 USB CDC 实机采集中验证 CRC、序号缺口和 CDC 忙计数，之后才决定是否切到较高发送速率的档 1/0。

HEALTH 字段口径（判读时容易误读）：

- `control_count` = 本窗口**发出的 CONTROL 帧数**（档 1：2 / 200 ms；档 0：10 / 100 ms），不是内环拍数；内环拍数看 CONTROL 帧里的 `control_seq`/`state_seq`（每拍 +1）。
- `imu_age_max_us` = 距**最后一次收到 IMU 帧**的时间（与发送节流无关；档 1 下正常是 1~2 ms 量级）。
- `motor_age_max_us` = 距最后一次收到该电机帧的时间（本分支用"首次见到新帧"的采样时刻，1 ms 量化）。
- `uart_busy_total` / `telemetry_drop_total` / `telemetry_generated_total` 是**累计值**：字段名 `uart_busy_total` 为协议历史命名，当前也统计 USB CDC 忙或未配置时的采样拍；只有 `drop_total` 涨才说明队列真的丢过帧。

## 3. 被动接收

首次安装接收依赖（离线解码和协议检查无需 pyserial）：

```bash
python -m pip install pyserial
```

Linux，采集 20 秒：

```bash
python tools/s2r_capture.py --port /dev/ttyUSB0 --baud 1152000 --seconds 20 --output /tmp/s2r_capture_01
```

Windows，将设备改成实际 COM 口、输出改成新目录：

```powershell
python tools/s2r_capture.py --port COMx --baud 1152000 --seconds 20 --output s2r_capture_01
```

省略 `--seconds` 持续采集，Ctrl+C 正常收尾。程序不调用串口 write，不发送启停或切换命令；打开前请求 DTR/RTS 为低，仍建议不接这两根线。目录必须不存在，防止覆盖旧数据。

控制台每秒显示 boot/session、policy_started、policy_active、history_synchronized、metadata_ready、CRC 错误及序号缺口。接入中途需等待完整 META 和 HISTORY；META 分片限速，可能需要数秒，丢片时等待下一轮。开始事件即使丢失也能通过持续状态识别运行。

### 3.1 短窗录制：链路带宽不够时的 100 Hz 采集（协议 §13.1）

实时流被限流档压到 5 Hz 时，仍要按 md 的 100 Hz 抓一段执行细节，就用板端缓存：

1. **烧带录制功能的固件**（本分支默认已含），照常起采集：
   `python tools/s2r_capture.py --port COMx --baud 1152000 --seconds 180 --output s2r_rec_01`
2. **触发（默认全自动，不需要连调试器）**：
   - 直接**左上+右中投入** → 固件自动开录（`EVENT 9, reason 1`）
   - 动作几秒 → **回失能** → 再录 200 ms 尾巴后自动停并转入回放（`EVENT 10, reason 3`）
   - 缓存 192 KB ≈ **4.1 s**，超了会自动停（`EVENT 10, reason 0`）
   - **帧间强制空隙**：固件沿用按 UART 1152000 估算的发送时间，再空 `S2R_TX_GAP_US`（默认 1500 µs）才尝试下一帧。旧串口桥曾因积压后线速连推丢掉回放段约 95%；USB CDC 首轮先保留此保守节流，后续按实测调整。
   - 可选的手动覆盖（调试器）：`s2r_record_requested`（0→1 启动 / 写 0 停）、`s2r_replay_requested`（再回放一轮补丢帧）、`s2r_record_state` / `s2r_record_frames`（只读）
3. **在上位机怎么看**：
   - `event.csv` 里出现 `event_code` **9 → 10 → 11**（10 的 `reason_code`：0 缓存满 / 1 手动停止 / 2 再回放），照它切分实时段与回放段；
   - 回放段的 CONTROL 帧：`packet_seq` 连续、`t_us` 是**录制时刻**、flags 带 `REPLAYING(16384)`，`control_dt_us ≈ 10000`（100 Hz）；帧内容与实时帧完全同构，现有 CSV/JSONL 流程不变；
   - 录制窗口内实时 CONTROL 暂停（数据进了缓存），POLICY/IMU/HEALTH 照常 —— 所以同一会话里"实时控制流在录制窗口中空一段、随后被回放段补上"是**预期行为**。
4. 缓存容量/回放节流在 `imcalib/Telemetry/s2r_telemetry.h`：`S2R_RECORD_BYTES`(192 KB)、`S2R_RECORD_PERIOD_US`(100 Hz)、`S2R_DUMP_PERIOD_US`(150 ms)。

## 4. 输出和离线解码

```text
采集目录/
  raw.bin                       原始串口字节，包括垃圾/坏帧
  capture_summary.json          采集来源、时长、解析统计
  boot_<16位十六进制>/
    session_<6位编号>/
      frames.jsonl              所有有效帧、标志及解码字段
      meta_<编号>.json           收齐后的 META
      event.csv / policy.csv / control.csv
      imu.csv / health.csv / history.csv
```

NaN 导出为 JSON null / CSV 空字段，不能补成零。POLICY 若能重建精确历史，JSONL 另含 `network_history`；首次 HISTORY 到达前的 POLICY 在实时导出中仍标未同步，可用 HISTORY 的 policy_seq 回溯关联。CONTROL 的有效片段还要求 `used_policy_seq` 命中已同步的策略历史（缓存最近 100 个编号），不能用最近一拍网络输出替代实际使用的动作。

同步状态反映逐帧解码当时已有的证据，META 收齐后不会追改先前导出的行。所有有效帧仍完整保存；后续分析可使用同一会话的完整 META/HISTORY 回溯首段数据，不能把实时有效标记为 false 等同于原始数据已丢弃。

```bash
python tools/s2r_capture.py --input /tmp/s2r_capture_01/raw.bin --output /tmp/s2r_decoded_01
```

`packet_seq` 缺口是真实遥测生成序号的缺失；`control_seq` 因 100 Hz 抽样跨号正常。重复/倒序包会保留但不推进重建状态。遇到缺口，接收端保守清空历史缓存，下一帧完整 HISTORY 后恢复。

`host_rx_time_ns` 是电脑处理该批帧的墙钟时刻；离线重新解码时会改变，不是设备采样时间或准确的逐字节到达时间。原始文件不保存 USB 批次时序。控制与反馈延迟一律使用 MCU 的整数时间戳；不得直接减电脑时间。

## 5. 建议首轮记录

先启动被动接收，再按原有操作流程覆盖 **待机 → 预热 → 策略开始 → 起立/站稳 → 退出 → 再次投入**。同时记录：视频文件名、实物机器、地面材料/坡度、载荷、供电、气弹簧状态、固件构建来源及模型签名。暂不在真实运行中用调试器改物理参数。

优先看以下关系：

| 排查层 | 数据与判定范围 |
|---|---|
| 采集和时序 | HEALTH 的实际周期、推理/控制峰值、反馈年龄、丢帧、UART 忙与 CAN 提交失败 |
| 观测和网络 | POLICY.obs + 对应 HISTORY + META，用相同输入复算网络 |
| 控制器 | CONTROL 的目标、wrap 前后误差、虚拟力矩、补偿、Jacobian 与电机请求 |
| 执行器响应 | used_policy_seq、CAN 提交时间、接收时间及位置/速度/DM 力矩估计 |
| 整机动力学 | IMU 与关节响应结合外部视频/定位；轮电流、接触、线速度真值缺失时保留不确定性 |

默认 100 Hz CONTROL 不能证明 1 ms 内环动态没有 gap；也不能仅靠这些信号区分全部地面接触参数。整机仿真联动和拟合待接入实测数据后实现。

## 6. 模块和回退

```text
imuTask   → imu_state / hi229_data        ─┐
policyTask→ rl_control.observation/policy   │ 只读
            action_state / input_command    ├─► s2r_source.c   采样 + 变化检测
actuationTask → rl_control.torque_state     │   (自维护序号/时间戳)
                rl_output_dm/wheel_cmd_nm  ─┘         │
                                                      ▼
commTask → S2R_Pump() → 预分配 FIFO → 编码/CRC → AXI SRAM 缓冲 → 遥测口 DMA
```

本分支**不往现有任务和结构体里插钩子**：`s2r_source.c` 用变化检测推断事件（`policy.run_ok+run_fail` → 一次推理；`hi229_data.ts`/`last_rx_tick` → IMU 新帧；`dm/dji_motor_feedback[].last_rx_tick` → 电机新帧），字段拿不到就按协议写 NaN/0，不伪造。控制侧不等待串口、不创建 JSON、不分配堆内存。24 槽 FIFO 中 4 槽留给 EVENT/HISTORY/META；满时丢弃新帧并计数。事件重复 3 次仍可能全丢，必须依赖持续状态。DMA 忙时不写发送缓冲。

调试器将 `s2r_diagnostic_requested=0`，只有**电机失能、session=0、上一帧发送完成**时才恢复普通 VOFA；否则等待条件满足。设回 1 恢复 S2R1 并重发 META。传输端口由 `vofa_transport.requested` 选择（1 = UART，0 = USB CDC），切换条件见 [USB CDC 遥测发送](usb-cdc-telemetry.md)。诊断取数和计时仍执行，不能把它当作“完全关闭遥测开销”的 A/B。

## 7. 已做验证与待验收

主机可重复检查：

```bash
python tests/test_s2r.py                # 协议 8 项；C 替身检查需 CC=gcc
python tools/s2r_build_info.py --check
git diff --check
```

本分支（Windows / Keil AC5）实际做到的：

- **USB DMA 改动前的历史构建**：UV4 `-b`（CWD 必须在 `MDK-ARM/`，`.lnp` 用相对路径）→ `0 Error(s), 0 Warning(s)`；`Code=113132 RO=163408 RW=5420 ZI=93508`；`dma_buffer` @ `0x24001100`。这不验证当前 USB DMA 改动。
- 三个新增 `.c`、`tests/s2r_host_test.c`、`tests/s2r_wire_test.c` 单文件 armcc 编译 **0 警告**。
- `python tests/test_s2r.py`：**协议 8 项全过**（帧长/偏移、CRC 向量、拆包/粘包/噪声恢复、序号回绕与缺口、事件去重、会话与历史恢复、NaN 导出、离线 raw 保真）。`NativeChecks`（`tests/s2r_host_test.c`，替身驱动 `S2R_Pump`）**本机没有主机 gcc，未执行**，只做了 armcc 类型检查——需要主机编译器时按 `CC=gcc python tests/test_s2r.py` 跑。

完整 META 示例及每类帧的十六进制样例在 [测试夹具](../tests/fixtures/s2r_frames.json)。**全部是合成的主机替身数据，不是实机采集，也不能作为物理配置依据**；META 帧样例只展示其中一个分片，另有完整 JSON。

接板后逐项验收：

- [ ] 烧录当前固件，确认 UART8 转接器 COM 口能接收数据；若切换 USB CDC，再核对枚举、USB DMA 非缓存区及 META `transport=usb_cdc`。
- [ ] 实测 USB CDC 持续吞吐量、CRC、序号缺口和发送泵调度空隙；原先的 115200 B/s 预算只适用于 UART 回退。
- [ ] 重启 boot_id 改变；待机、预热、首次动作、退出和重入分段正确，持续状态与实际门控相符。
- [ ] 中途接入、重连、主动制造录制丢包后能通过 META/HISTORY 恢复，坏字节不进入有效分析。
- [ ] HEALTH 的控制帧周期/反馈年龄/队列峰值及 drop_total；对照基准固件确认遥测带来的开销可接受；commTask 栈余量（本模块全部用静态量）。
- [ ] 反馈电机顺序、有效位与 `unavailable`/`derived` 清单和实测一致；`motor_send_ok_mask` 恒 0 是否可接受。

台架验收完成前，不宣称已定位或排除了整机 sim2real gap。
