# 下位机 sysid 实施计划（轮 + 闭链腿）

> 大机器测试专用，`SYSID_ENABLE=1` 时生效。

> **上游文档**：`chuanliantui-wheel-joint-sysid-handoff.md`（算法侧交接单，定义要采什么）
> **本文用途**：把交接单翻译成下位机可执行、可验收的实施计划（改哪些文件、按什么顺序、怎么验证）。
> **状态**：按 §15 分步实施中。
> **进度**：机器配置表（两份表 + 运行时 `Machine_Select`）、单调 ns 时钟、测试模式入口（左拨杆中 + 右拨杆中）已完成；测试代码独立到 `imcalib/Sysid/`（`SYSID_ENABLE` 总开关）。按作者意见已**移除 DM 开机自检**（满量程改用达妙上位机人工核对）。armcc 全量 102 文件 0 fail / 0 warn（`SYSID_ENABLE=1`）；步 3「CAN 收发时间戳」起待做。
> **改动记录**：每处改动的输入 / 输出 / 调用链见 `md/sysid-change-map.md`。
> **数值纪律**：本文只记结构、约定和"为什么"；具体限值/力矩/电流以签字的 `manifest.yaml` 与代码常量为准。
> **依据**：`DM-J8009P-2EC V1.1 减速电机说明书 V1.0`（Specifications / MIT Mode / Register Map 三节）。
> **作者决策（已定）**：两台机器两套配置表；数据走 VOFA 串口（`VOFA_PORT` 选口，当前 1 = USART1）+ Vofa+ 存盘；测试类型编译期选、run 自动连续跑；遥控只做 deadman 和中止；时间戳按 DWT 单调 ns + CAN 收发时刻；FK 用 RL 侧定义；激励全部直发（轮=电流 raw，腿=力矩 N·m）。

---

## 0. 摘要

下位机只做两件事：**按剧本施加激励**、**把"施加了什么 + 机器实际怎么动"按同一时钟记下来**。拟合在算法侧 MuJoCo 做。

```
        试验 A：纯电流 raw                      试验 B：纯力矩 N·m
   ┌──────────────────────────┐         ┌──────────────────────────┐
   │ 0x200 电流命令 → C620     │         │ 4×MIT 力矩帧 → DM8009P    │
   │ 0x201/0x202 反馈 ← C620   │         │ FK(髋角) → 腿长/腿倾角     │
   └───────────┬──────────────┘         └───────────┬──────────────┘
               │ 打 mono_ns 时间戳                   │ 打 mono_ns 时间戳
               └──────────────┬──────────────────────┘
                              ▼
                 VOFA 帧（1152000，JustFloat）
                              ▼
                    Vofa+ 存盘 → Python 脚本
                              ▼
       data/sysid/chuanliantui/<date>/ 下的 CSV + manifest + sha256

   遥控：s1 中/上 = 开始并持续跑（run 自动切换）；s1 下位 = 中止 + 置零
```

新增三个模块：`mono_ns`（时钟）、`sysid_log`（采集与 VOFA 输出）、`sysid_program`（激励序列）。另加机器配置表、DM 满量程自检、sysid 仲裁分支。

**完成标准**：DM 参数自检通过；空跑时间戳单调、seq 连续、Vofa+ 列对齐；限值被钳住；拔线/拨杆下位立刻置零并标记 run 无效；静态 FK 与卷尺/量角器对上。

---

## 1. 决策与分工

| 项 | 决定 | 结果 |
| --- | --- | --- |
| 两台机器 | 两套配置表，宏切换 | 见 §5 |
| 数据落盘 | 不落 MCU，走 VOFA 通道 + Vofa+ | 见 §7、§12 |
| 测试类型 | **编译期选择**（每次改代码只跑一个测试） | 见 §7.6 |
| run 切换 | **自动切换、连续发送**（run 间自动插零段） | 见 §7.6 |
| 遥控用途 | 只有 deadman（s1）和中止，不做用例选择 | 见 §7.6 |
| DM 满量程 | 以电机实际配置为准，**启动时 CAN 读回自检** | 见 §8.2 |
| 时间戳 | 主控单调 ns；CAN RX 时刻 + CAN TX 完成时刻 | 见 §6 |
| FK | RL 侧定义（`leg_solver` 输出，机体系） | 见 §10、§14 |
| 激励 | 直发：轮=电流 raw；腿=力矩 N·m（kp=kd=0） | 见 §9、§10 |
| 阈值保护 | 只保留硬兜底，温度/速度/位置只记录不设阈值 | 见 §11 |
| 坐标系定义 | 交付训练端 | 见 §14 |

---

## 2. 现状盘点

### 2.1 已有的（可直接复用）

| 能力 | 位置 | 说明 |
| --- | --- | --- |
| CAN 路由 + 批量接收 + bus-off 恢复 | `can_bus.c:45-155` | 中断收帧、按 ID 分发 |
| 收发解码/编码 | `dji.c`、`dm.c` | DJI 反馈、DM MIT 编解码、多圈、在线检测 |
| 纯力矩下发 | `dm.c:286-311` | kp=0/kd=0，右腿在驱动边界取反 |
| 裸电流下发 | `dji.c:200-217` | `Dji_Send_Current(handle, 0x200, raw[4])` |
| FK / 雅可比 | `leg_solver.c:33-160` | 腿长、腿倾角、虚拟小腿、雅可比 |
| 1 kHz 硬实时节拍 | `task_actuation.c` + TIM6 | 信号量驱动，`CTRL_DT=0.001 s` |
| 1 kHz 通信节拍 | `task_comm.c:320-332` | 解析、状态、故障、VOFA |
| 使能机与故障门 | `task_comm.c:141-220` | 遥控使能、失能、翻倒检测 |
| 遥控解析 | `dr16.c/h`、`md/IO_CHAINS.md` §4 | `s1` 已用；`s2/wheel/键鼠` 本计划不需要 |
| 32 通道 VOFA 调试（上限 32） | `task_comm.c`、`Vofa_send.c` | JustFloat + DMA，500 Hz |
| 串口空闲接收框架 | `uart_idle.c/h` | UART9=DR16、UART7=HI229 在用 |
| UART8（TX+RX） | `usart.c:360-361`、`stm32h7xx_it.c:409` | 本计划只用作**数据出口**（RX 不启用） |

### 2.2 缺的（本计划的交付）

| 缺什么 | 影响 |
| --- | --- |
| 单调 ns 时钟 | 时间戳只有 1 ms 分辨率（`HAL_GetTick()`），估不出延迟 |
| CAN RX / TX 完成时刻时间戳 | 命令行与反馈行都没有可用时间基准 |
| DM 满量程自检 | 固件常量与电机实际刻度不一致时，力矩/角度整列作废（`dm.h:36-41` 现为 ±π / ±30 / ±10） |
| DM 故障码判据 | `err_raw` 是**状态码**（0=失能/1=使能/8~E=故障），现在解析了没人看（`dm.c:146`） |
| sysid 独占模式 | 执行链只有 LQR / 手动两路（`task_actuation.c:50-117`） |
| 激励序列执行器 | 没有 stiction/plateau/step/chirp 的编排与 run 自动切换 |
| 高码率 VOFA 流 | 正常 32ch@500Hz 的列定义不满足 sysid |
| 丢帧可见性 | `Vofa_send.c:17` 忙就丢帧且不留痕 |
| 机器配置表 | 电机型号/几何/限值散落在 `dji.h`、`dm.h`、`robot_control.c` |

---

## 3. 参数与前提

### 3.1 已定（作者给出 + 手册确认）

| 项 | 值 | 来源 | 落地位置 |
| --- | --- | --- | --- |
| DM 型号 | DM-J8009P-2EC V1.1，减速比 9:1，16 位双编码器 | 手册 Specifications | — |
| 额定扭矩 / 峰值扭矩 | 20 N·m / 40 N·m | 手册 | 只作参考 |
| 额定转速 / 最大空载转速 | 100 rpm@24V / 168 rpm@24V | 手册 | 只作参考 |
| **MIT 预设满量程** | `P_MAX = ±12.5 rad`、`V_MAX = ±45 rad/s`、`T_MAX = ±54 N·m` | 手册 MIT Mode（可在上位机改） | **以电机实际值为准，见 §8.2 自检** |
| 过温保护 | MOS 120 °C 关机；线圈可配（推荐 100 °C），触发后自动退出使能 | 手册 | 不设我们的阈值，只记录 |
| 许可力矩（签字值） | 20 N·m（= 额定扭矩） | 作者 | 固件**激励钳位**（刻度仍用实际 T_MAX） |
| 温度/速度/位置阈值 | 不设，只记录 | 作者 | 见 §11 |
| 轮端测试电流上限 | ±15 A = ±12288 raw | 上游文档 | sysid 硬钳 |

> 注意区分**刻度**与**钳位**：MIT 帧的量化刻度是电机的 `T_MAX/V_MAX/P_MAX`（必须一致），而"许可力矩 20 N·m"是我们自己加的命令钳位——两者混用会让 N·m 整列错。

### 3.2 待确认

| # | 待确认 | 影响 | 怎么拿 | 阻塞? |
| --- | --- | --- | --- | --- |
| 1 | 电机实际 `PMAX/VMAX/TMAX/CTRL_MODE` | 全部角度、速度、力矩 | **达妙上位机人工读一次**，与 `machine_config.c` 比对 | 一次性人工确认 |
| 2 | `G_total`（电机轴到轮轴总传动比） | 轮端速度/力矩换算（算法侧） | 机械图纸；或现场实测（§3.3） | 不阻塞采数 |
| 3 | 关节名与符号 | 关节 CSV 列名/正方向 | §14.6 提案，训练端确认 | 不阻塞 |
| 4 | `eta_total`（传动效率） | 只影响算法侧初始包络 | 先给保守估计（约 0.75）并标注"估计值" | 不阻塞 |
| 5 | P 点物理定义、`offset_phi0` 零位姿态 | 腿长/腿倾角与模型对齐 | 机械 + 台架静态核对 | 不阻塞 |
| 6 | 轮测试命令率 500 Hz 是否满足延迟估计精度 | 轮 CSV 行数 | 算法侧确认 | 不阻塞 |
| 7 | 是否加开旁证帧 kind=4 | VOFA 带宽 | 算法侧确认 | 不阻塞 |

### 3.3 `G_total` 是什么、怎么量（给非机械背景）

- C620 反馈的 `ecd_raw`、`speed_rpm` 是**电机转子**转了多少，不是轮子转了多少；中间隔着 M3508 内置 P19（约 19.2:1）和自制减速箱。
- **它是角量的缩放，不直接给 m/s**：

```
rad/s(轮轴) = rpm(转子) × 2π/60 ÷ G_total          ← 文档包络公式用的就是这个
m/s(地面)   = rad/s(轮轴) × R_轮                    ← 只有需要线速度时才再乘轮半径
τ(轮轴)     = τ(转子) × G_total × eta_total          ← 力矩是"乘"，不是除
```

- 上游文档要的是**轮轴 rad/s**，所以弧度就够；只有在训练端需要地面线速度时才用到轮半径。
- **不阻塞**：需要时现场量——用手把轮子慢慢转**整整 N 圈**（例如 10 圈），看 `ecd_raw` 累计变化量 Δ：

```
G_total = Δ / (8192 × N)      /* 8192 = 转子一圈的编码器码 */
```

试验 A 的 `baseline_sign` 段顺手就能做这个标定。

---

## 4. 架构与新增文件

### 4.1 模块划分

| 新增/改动 | 文件 | 职责 |
| --- | --- | --- |
| 新增 | `imcalib/user-lib/mono_ns.c/h` | DWT 单调 ns 时钟，中断安全 |
| 新增 | `imcalib/user-lib/machine_config.h` | 两套机器配置表（电机/几何/限值/FK 版本） |
| 新增 | `imcalib/user-lib/sysid_log.c/h` | 事件入环、VOFA 帧组装、发送泵、丢帧计数 |
| 新增 | `imcalib/task/sysid_program.c/h` | 激励序列表、chirp/PRBS 生成、phase 状态机、run 自动切换 |
| 改动 | `imcalib/user-lib/dm.c/h` | RX 中断取 ns；`Dm_Read_Param()`（读 PMAX/VMAX/TMAX/CTRL_MODE）；满量程常量 |
| 改动 | `imcalib/user-lib/can_bus.c/h` | TX 完成中断 + 完成时刻回调 + pending 表 |
| 改动 | `imcalib/user-lib/dji.c/h` | RX 中断取 ns；sysid 直发接口；钳位常量 |
| 改动 | `imcalib/task/task_actuation.c` | 新增 sysid 仲裁分支（独占） |
| 改动 | `imcalib/task/task_comm.c` | sysid 下停 32ch；安全判定接入；s1 语义 |
| 改动 | `imcalib/user-lib/Vofa_send.c/h` | 支持 sysid 帧 + 环形缓冲 + TxCplt 泵 |
| 改动 | `Core/Src/main.c` | TIM6 回调里扩展单调时钟（USER CODE 段） |
| 改动 | `MDK-ARM/*.uvprojx`、`.eide/eide.yml` | 新源文件入工程 |

CubeMX 保护区：`fdcan.c`、`usart.c`、`dma.c` 等生成文件只在 `USER CODE BEGIN/END` 内改；若需改 FDCAN 的 TX FIFO/Queue 模式，只能在 CubeMX 里改后重新生成。

### 4.2 数据流

```
[ISR] CAN RX ──(mono_ns)──┐
[ISR] CAN TX 完成 ─(mono_ns)─┤
[1kHz] 力矩下发+FK 快照 ──────┤──> sysid_log 环形缓冲 ──> 帧组装 ──> VOFA_UART DMA ──> Vofa+
[遥控 s1] deadman/中止 ───────┘（只做开关，不选用例；不入数据流）
[启动] DM 参数自检（0x7FF 读 PMAX/VMAX/TMAX/CTRL_MODE）
```

---

## 5. 机器配置表

### 5.1 组织方式（推荐）

集中到一个头文件，用宏分支，而不是在各文件里加注释开关：

```c
/* machine_config.h */
#define MACHINE_CHUANLIANTU   1     /* 1=本次 sysid 对象；切机器只改这里 */
#if MACHINE_CHUANLIANTU
    ...
#else
    ...
#endif
```

- 未定义的组合用 `#error` 拦在编译期，避免"切一半"。
- 运行期把"机器签名 + FK 版本"作为常数打进 VOFA（或至少写进 manifest），保证数据可追溯。
- 理由：漏切一处不会有编译错误，只会**静默产出错单位的数据**。

### 5.2 每套表必须包含

| 类别 | 字段 |
| --- | --- |
| 轮 | 型号、`FDCAN` 句柄、`motor_id`、`feedback_id`、`control_id`、`feedback_sign`、raw/A 刻度、raw 满量程、`G_total`、离线超时 |
| 腿 | DM 型号、`FDCAN` 句柄、`control_id`/`feedback_id`、`feedback_sign`、MIT 位置/速度/力矩满量程（用于下发与解码）、离线超时 |
| 几何 | `lu`、`lg`、`offset_f`、`offset_b`、`offset_phi0`（左/右各一套）、`mirror` |
| 限值 | 腿力矩激励钳位（20 N·m）、轮端电流钳位（±12288）、腿长粗兜底 |
| 溯源 | `FK_VERSION`、机器名 |

### 5.3 两台机器差异

| 项 | 机器① chuanliantui（本次对象） | 机器② 当前固件默认 |
| --- | --- | --- |
| 轮 | M3508 + C620 + 自制减速箱 | M2006（`dji.c:8`，`dji.h:19` 减速比 36） |
| 轮刻度 | 819.2 raw/A，±16384 ↔ ±20 A | 1000 raw/A，±10000 ↔ ±10 A |
| 总传动比 | 19.2(P19) × 自制箱比，**待确认** | 36 |
| 腿 | DM-J8009P-2EC（9:1） | DM J4310（`dm.c:5-34`） |
| MIT 力矩满量程 | 预设 ±54 N·m（以实际配置为准，自检） | ±10 N·m（`dm.h:40-41`） |
| MIT 速度满量程 | 预设 ±45 rad/s（同上） | ±30 rad/s（`dm.h:38-39`） |
| MIT 位置满量程 | 预设 ±12.5 rad（同上；`dm.h:36-37` 现为 ±π，与预设差约 4 倍） | ±π |
| 几何/零位 | **待确认** | `robot_control.c:43-63` |

---

## 6. 时间戳

### 6.1 `mono_ns`（新增）

| 项 | 设计 |
| --- | --- |
| 时钟源 | `DWT->CYCCNT`，CPU = 550 MHz（`main.c` PLL：24/3 × 68.75 = 550 MHz） |
| 开启顺序 | `CoreDebug->DEMCR |= TRCENA` → **`DWT->LAR = 0xC5ACCE55`**（M7 需要解锁）→ 清 CYCCNT → `DWT->CTRL |= CYCCNTENA` |
| 回绕处理 | CYCCNT 是 32 位，550 MHz 下 7.81 s 一圈；在**已有的 TIM6 1 kHz 中断**（`main.c`）里累加 64 位周期基数，不新增中断 |
| 读取 | `ns = (cyc_base + (DWT->CYCCNT - last_cyc)) * 20 / 11`（1000/550 = 20/11） |
| 中断安全 | 读侧取一次快照；ISR 内先读后写再更新基数，保证不会读到撕裂值 |
| 精度 | 分辨率 1.82 ns，扩展无累计漂移 |
| 依赖 | 自己开 TRCENA，不依赖调试器 |
| 验证 | 运行 10 s 与 `HAL_GetTick()` 对比，偏差 < 2 ms；连续两个 1 kHz 节拍差值应 ≈ 1 ms |

### 6.2 CAN RX 时间戳

- 位置：`dji.c:80-96 Dji_Read()`、`dm.c:90-106 Dm_Read()` 在中断回调**入口**取 `mono_ns()`（现在存 `HAL_GetTick()`）。
- 语义：主控"收到该帧并进入回调"的时刻；比帧到达总线晚 ISR 延迟（µs 级）。
- 存储：两个 feedback 结构体增加 `uint64_t rx_ns`，在线检测仍用 ms。

### 6.3 CAN TX 完成时间戳

- 现在 `can_bus.c:94-111` 只判 `HAL_FDCAN_GetTxFifoFreeLevel()` + 发，时间戳是 `HAL_GetTick()`。
- 改造：
  1. `Can_Bus_Start()` 增加 `HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_COMPLETE, <全部 TX 元素位掩码>)`。
  2. 新增 `HAL_FDCAN_TxBufferCompleteCallback(hfdcan, BufferIndexes)`：对完成的元素位取 `mono_ns()`。
  3. 维护**按 TX 元素索引**的 pending 表（每条总线 32 项，对应 `TxFifoQueueElmtsNbr = 32`）：入队时登记 `{seq, kind, 行内容快照}`，完成回调按索引取出并生成数据行。
- 为什么按元素索引配对：`fdcan.c:68` 是 `FDCAN_TX_FIFO_OPERATION`（**优先级排序**，不是严格入队顺序），按顺序配对会配错。
- 若要严格按入队顺序发（同总线两帧 DM 的顺序），需在 CubeMX 里把 TX FIFO/Queue Mode 改成 Queue 后重新生成。
- 失败路径：`AutoRetransmission = ENABLE`（`fdcan.c:45`），总线错误会延迟完成 → pending 表项加超时清理（如 50 ms）并计入 `tx_timeout_cnt`；该 run 判无效。
- 语义约定：`t_cmd` = 帧**发送完成**（帧尾上总线）。比 C620/DM 内部电流环真正生效早约 0.13 ms（1 Mbps 8 字节标准帧的线上时长）+ 驱动器内部未知延迟。这是**恒定偏置**，交给算法侧的 delay 拟合吸收，但两边必须同约定——写进 README 与 manifest。

### 6.4 使用纪律

- 所有数据列统一用 `mono_ns`；`HAL_GetTick()` 仅用于超时/在线检测。
- 一行数据里的时间戳与内容必须来自同一时刻（见 §10.2 的时序）。

---

## 7. VOFA 数据通路

### 7.1 带宽预算（硬约束）

VOFA 串口 @1152000 8N1 = **115200 B/s**。JustFloat 帧 = `4N + 4` 字节（帧尾 `00 00 80 7F`）。

| 实验 | 帧 | 通道 | 字节/帧 | 速率 | 占用 |
| --- | --- | --- | --- | --- | --- |
| A 轮 | 命令完成行 | 10 | 44 | ≤500 Hz | 22 KB/s |
| A 轮 | 反馈接收行 | 10 | 44 | ≤500 Hz | 22 KB/s |
| B 腿 | 力矩快照行 | 16 | 68 | 500 Hz | 34 KB/s |
| 旁证（可选） | DM 原始反馈行 | 12 | 52 | 500 Hz | 26 KB/s |

- 结论：**轮测试命令率 ≤ 500 Hz**（1 kHz 会把 A 推到 88 KB/s，超过链路可用余量）。
- 腿测试 500 Hz 安全（37%）；开旁证帧后 B 合计约 60 KB/s（65%），默认关。
- **现状（2026-09-20）**：实际实现是单一 37 列帧（152 B），控制节拍已提到 1 kHz；152 B × 1 kHz = 152 kB/s 超过 115.2 kB/s 线速，`SYSID_TX_DIV=4` 只发 250 Hz（38 kB/s，33%）。推帧仍是每周期一帧，接回发送泵前必须先做推帧抽取，见 `sysid-delivery.md` §5.3。

### 7.2 帧格式

**通道 0 固定为"行类型 kind"**，每帧通道数固定，不同实验不混写——Vofa+ 的列才对得齐。
每个数据行都自带 `test_id` 与 `run_id`，行是自描述的：某一帧丢了也不会污染其他行的归属。

A 轮：

| ch | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 命令行 (kind=1) | 1 | seq | t_hi | t_lo | test_id | run_id | `cmd_test_wheel_raw` | 另一轮 raw（恒 0） | 0 | 0 |
| 反馈行 (kind=2) | 2 | seq | t_hi | t_lo | test_id | run_id | `ecd_raw` | `speed_rpm` | `torque_current_raw` | `temperature_c` |

B 腿（kind=3）：

```
[3, phase_id, test_id, run_id, seq, t_hi, t_lo,
 tau_lf0, tau_lf00, tau_rf0, tau_rf00,
 leg_length_L, leg_pitch_L, leg_length_R, leg_pitch_R]
```

旁证（kind=4，默认关）：

```
[4, seq, t_hi, t_lo, test_id, run_id, trq_raw_f, trq_raw_b, vel_raw_f, vel_raw_b, temp_mos, err]
```

标记行（kind=5，run 边界）：

```
[5, event, test_id, run_id, seq, t_hi, t_lo, drop_cnt, clamp_cnt, tx_timeout_cnt]
/* event: 1=run_start, 2=run_end, 3=abort, 4=suite_end */
```

- `seq`：全局递增整数（不按 kind 分开），存盘后检查连续性判丢帧。
- `phase_id`：见 §9.3、§10.4 对照表，由上位机脚本映射成 `phase` 字符串。
- `run_id` 由固件递增分配，避免"上位机记错 run_id"。
- 标记行只用于诊断与对齐，数据行的归属不依赖它。

### 7.3 时间戳编码（float32 限制）

JustFloat 只有 float32，直接发 ns 会丢精度。拆两通道：

```
t_hi = floor(ns / 1048576)      /* 2^20 */
t_lo = ns % 1048576
ns   = t_hi * 1048576 + t_lo    /* 两者都 < 2^24，float32 精确可表示 */
```

精确范围：`t_hi < 2^24` → ns < 1.76e16 ns ≈ 4.9 小时，够用。

### 7.4 发送实现（必须改）

现在 `Vofa_send.c:17` 在 UART 忙时**直接丢帧**。改为：

```
sysid 帧 → 字节流写入环形缓冲（SPSC）
        → 若 UART 空闲则启动 DMA（HAL_UART_Transmit_DMA）
        → HAL_UART_TxCpltCallback 里继续泵下一段
        → 环满/启动失败：drop_cnt++（并保留可诊断信息）
```

- 工程里目前没有 `HAL_UART_TxCpltCallback`（HAL 弱定义），新增时按 `huart->Instance` 分支，避免影响其他串口。
- sysid 帧用**独立缓冲**，不复用 32ch 调试缓冲。
- `drop_cnt > 0` 的 run 一律判无效（见 §11）。

### 7.5 32 通道调试流的处理

- sysid 模式下**停发** `task_comm.c` 的 32ch 帧。
- 理由：同一 UART 上混不同长度的帧，Vofa+ 的列会错位，存出的 CSV 不可用。
- 调试需求改由 sysid 帧里的 `phase_id`、`seq`、`run_id`、`drop_cnt` 等通道承载。

### 7.6 触发与节奏（本次定稿）

| 项 | 方案 |
| --- | --- |
| 测试类型 | **编译期选择**（`#define SYSID_TEST ...` 或直接改序列表）。一次只编译/烧录一个测试，跑完再改代码换下一个 |
| run 切换 | **自动**：一套用例内 run 逐个自动开始、连续发送；run 之间自动插零段（文档要求的"零 → 激励 → 零"），`run_id` 自动 +1 |
| 遥控用途 | 只有两个：`s1` 中/上位 = 允许并开跑；`s1` 下位 = 立即中止 + 置零 + `event=3` |
| 重新开跑 | 一套跑完即停在全零（`event=4`），需要 s1 下→上再拨一次才会重开（防意外循环） |
| 不用的控件 | `s2`、拨轮、键鼠、UART8 RX 都不参与触发（少一条链路，少一类故障） |
| 硬要求 | 急停随手可及；一旦置零或中止，`run_valid=false` 写进 manifest |

理由：使能门本身就要求 DR16 在线（`task_comm.c:166`、`:207`），用 s1 开跑不需要新增任何使能路径；run 自动切换省掉人工按键，也避免"漏按一次导致 run 编号错位"。

### 7.7 上位机（Vofa+ + Python）

1. Vofa+：协议 JustFloat、COM 口、1152000、开启数据记录到 CSV。
2. Python 脚本（算法侧仓库里放，下位机不实现）职责：
   - 按 `kind` 拆分：轮 → 命令行 + 反馈行；腿 → 快照行（+ 旁证）。
   - 重建 `t_ns`；按 `seq` 检查连续性；检查时间戳单调。
   - 删除 QA 通道（如"另一轮 raw"），写文档要求列的 CSV。
   - 补 `run_id/test_id/phase`（行内已自带，脚本按 §9.3/§10.4 映射成字符串）。
   - 生成 `manifest.yaml` 骨架、算 `checksum.sha256`、输出质检报告（行数、丢帧、峰值电流、钳位次数）。

---

## 8. sysid 模式与仲裁

### 8.1 状态机

```
IDLE ──s1 中/上──> RECORDING（自动跑完当前用例的全部 run）──s1 下位/故障/序列结束──> IDLE
                        │
                        └── 每个 run：零段 → 激励段 → 零段 → run_id+1 → 下一个 run
```

### 8.2 DM 满量程核对（人工，一次性）

用达妙上位机连上电机读 `PMAX` / `VMAX` / `TMAX` / `CTRL_MODE`（寄存器 0x15 / 0x16 / 0x17 / 0x0A），
与 `machine_config.c` 里当前机器那一行比对：不一致就改配置表。

- 刻度错会让**力矩与角度整列作废**，所以这一步必须在第一次采数前做一次。
- 现场怀疑刻度不对的现象：力矩明显偏大/偏小、角度范围异常。此时停表检查，不要靠调参掩盖。
- 固件不做读参（按作者意见，避免常驻一套只用于一次性核对、但会干扰反馈解析的逻辑）。

### 8.3 进入条件（全部满足才发力矩）

- 遥控 s1 中/上（`robot_state.rc_enable`，`task_comm.c:136`）
- `ctrl_fault == FAULT_NONE`
- DM 参数自检通过；对应电机在线（`Dm_Is_Online`/`Dji_Is_Online`）
- 试验 B 还需 `leg_l.output.valid && leg_r.output.valid`

### 8.4 独占保证

- `task_actuation.c:50-117` 的仲裁增加第三分支：`CTRL_STRATEGY_SYSID`。
- sysid 激活时，LQR（`output_task_lqr`）与手动/RL 路径全部旁路，**只有 sysid 一处写 DM/DJI**。
- 退出 sysid 时回到原有分支，不影响现有行为（保留 A/B 对照开关，符合仓库改动哲学）。
- 试验 B 的记录阶段：`kp=0, kd=0`（`dm.c:304` 已是纯力矩），不接任何位置 PD、不用 `reference.csv`。

---

## 9. 试验 A：单轮（纯电流 raw）

### 9.1 施加方式

- 被测轮给 `cmd_test_wheel_raw = round(I_A × 819.2)`，**另一轮在同一 0x200 帧里写 0**，四腿 0 N·m 但保持使能。
- 不走 `Dji_Torque_To_Current()`（那是 N·m 通道），直接调 `Dji_Send_Current(FDCAN2, 0x200, raw[4])`。
- 硬钳 `|raw| ≤ 12288`（±15 A，文档规定的测试上限，**不是** DM 许可、也不是轮端 N·m）。
- **现状**：固件实际钳位 `SYSID_CURRENT_LIMIT_RAW=4096`（±5 A），run 表只剩 plateau ±0.5~4 A（见 `sysid-delivery.md` §4.2）；上面的 ±15 A 与全套用例是上游交接单的目标值。
- 命令率 500 Hz；反馈行按实际收到的帧率记录（C620 反馈率≈命令率，以实测为准）。
- 每次只测左轮或右轮：编译期选（`SYSID_TEST = WHEEL_L` / `WHEEL_R`）。

### 9.2 序列表（run 自动连续）

| test_id | 序列（每级时长） | run 划分 |
| --- | --- | --- |
| `baseline_sign` | 0 A 5 s → +0.5 A 1 s → 0 A 2 s → −0.5 A 1 s → 0 A 5 s | 1 run（顺便做 §3.3 的 `G_total` 标定） |
| `stiction` | 0,+0.25,+0.5,+0.75,+1,+1.25,+1.5,+2,+2.5,+3,+4,+5,+6,+8,+10,+12,+15 A，每级 1 s，首次持续转动后回零 | 正向 1 run + 负向 1 run |
| `plateau` | ±1,±1.5,±2,±2.5,±3,±4,±6,±8,±10,±12,±15 A，每个电流：0 A 2 s → 平台 5 s → 0 A 2 s | 每个电流 1 run |
| `step` | ±1,±2,±5,±10,±15 A，每个幅值：0 A 2 s → 阶跃 0.5 s → 0 A 3 s，重复 3 次 | 每个幅值 1 run |
| `holdout_step` | 重做一个 `step` | 1 run，**不参与拟合** |

### 9.3 phase_id（A）

| id | 含义 |
| --- | --- |
| 0 | 静止/零 |
| 1 | baseline_sign 正段 |
| 2 | baseline_sign 负段 |
| 3 | stiction 升 |
| 4 | stiction 降 |
| 5 | plateau |
| 6 | step |
| 7 | holdout_step |
| 15 | abort / 故障置零 |

### 9.4 每 run 产出

- `c620_command_raw.csv`：每成功发出一帧记录一行
  `run_id,test_id,phase,t_cmd_can_tx_ns,sequence_id,can_id,cmd_test_wheel_raw`
- `c620_feedback_raw.csv`：每收到一帧记录一行
  `run_id,test_id,phase,t_fb_can_rx_ns,can_id,motor_name,ecd_raw,speed_rpm,torque_current_raw,temperature_c`
- `can_id`：命令行记控制帧 ID（0x200），反馈行记实际收到的 ID（0x201/0x202），README 里写明。
- `motor_name`：由上位机按编译期选的轮填写（下位机不需要知道命名）。
- 另外要一并交给算法侧：`G_total`、`eta_total`、力矩常数的来源与数值（生成初始包络用）。**不抄 268/17、5.5 N·m、eta=1。**

---

## 10. 试验 B：闭链腿（纯力矩 N·m）

### 10.1 施加方式

- 四台 DM 全部 `torque/Nm`：MIT 帧 kp=0、kd=0，`t_ff` 按 `T_MAX` 量化（`dm.c:286-311`）。
  - 手册原文：`kp=0, kd=0` 时给定 `t_ff` 即直接输出力矩；`kp=0, kd≠0` 时给定 `v_des` 可匀速转动。本实验只用前者。
  - 下发帧里 `p_des`/`v_des` 字段仍在（8 字节固定），但乘 0 后无影响；填满量程整数即可。
- 轮 0；不用位置 PD；不用 `reference.csv`；进入记录阶段后不得有位置纠正。
- 采样率：控制节拍 1 kHz 产生快照，但串口只能带 250 Hz（文档要求 ≥200 Hz）；抽取方案待定，见 §7.1 现状。
- **位置/速度不能"忽略"**：腿长/腿倾角是从反馈 `POS` 字段算出来的，而 `POS`/`VEL` 的刻度就是 `P_MAX`/`V_MAX`（见 §8.2 自检）。忽略的只是"下发时的位置/速度目标"。

### 10.2 单周期时序（保证"同一时刻"）

```
t0  取 leg_l / leg_r 最新 FK 快照（腿长/腿倾角）+ 本周期要发的 4 个力矩（钳位后的量化值）
t1  发 4 帧 MIT 力矩（左腿 FDCAN1、右腿 FDCAN3）
t2  等 4 帧的 TX 完成回调全部到达，取最后一帧的时刻作为 t_cmd
t3  打一帧 kind=3：{phase_id, test_id, run_id, seq, t_cmd, 4×tau, 4×状态}
```

- 行内 `leg_length/leg_pitch` 是**发送时刻的最新电机角**经 FK 得到的值（与文档要求一致）。
- 四帧跨两条总线，完成时刻相差约 0.3 ms，README 里说明"整行时间戳 = 四帧完成中的最后一帧"。
- 力矩列为**量化后真正上总线的 N·m**（`dm.c:302` 之后回算），避免记录值与总线值差 1 LSB。

### 10.3 序列表

| test_id | 阶段 | 目的 |
| --- | --- | --- |
| `torque_baseline` | `baseline_start → zero_hold → baseline_end` | 全零时的腿长、腿倾角与力矩零漂 |
| `torque_step` | `zero_before → excite → zero_after` | 小幅阶跃的延迟、超调、阻尼 |
| `torque_chirp` / `torque_prbs` | `zero_before → excite → zero_after` | 频响/宽频响应 |
| `holdout_torque_*` | 同原用例 | 留出，不参与拟合 |

- 执行纪律：架空与 FK 静态核对完成 → 先记 4 路 0 N·m ≥5 s → 仅按 `torque_program.yaml` 激励 → 结束立即四路置零并再记 ≥5 s。
- 每个用例 3 次有效 run + 1 次 holdout；run 自动连续（§7.6）。
- 幅值：**钳位上限 20 N·m，激励从小开始**（建议 step 先 ±1 N·m，chirp 峰值先 ±2~3 N·m），逐步加大并保留 holdout。
- chirp/PRBS 生成：正弦扫频（`sinf` 逐点算）或 LFSR 伪随机；参数化到常量表；幅值上限检查不可跳过。

### 10.4 phase_id（B）

| id | 含义 |
| --- | --- |
| 0 | zero（未进入用例） |
| 1 | baseline_start |
| 2 | zero_hold |
| 3 | baseline_end |
| 4 | zero_before |
| 5 | excite |
| 6 | zero_after |
| 15 | abort / 故障置零 |

### 10.5 旁证帧（可选，默认关）

- kind=4 记录 DM 原始反馈（`trq_raw`、`vel_raw`、温度、`ERR`），用于把"命令→实际力矩"与"实际力矩→运动"分开拟合。
- 不混进 kind=3 行，避免破坏算法侧的列约定；是否启用由算法侧决定。

---

## 11. 安全与无效标记

### 11.1 分级

**A 级：硬兜底（必须做，不需要新数字）**

| 触发条件 | 动作 |
| --- | --- |
| 轮 raw 超 ±12288 / DM 力矩超激励钳位 | 钳到上限并累计 `clamp_cnt` |
| CAN 掉线、电机离线（10 ms 超时） | 立刻置零，run 判 invalid |
| **DM `ERR` 是故障码**（手册：`0`=失能、`1`=使能、`8`=过压、`9`=欠压、`A`=过流、`B`=MOS 过温、`C`=线圈过温、`D`=通信丢失、`E`=过载） | `ERR ∈ {8..E}` 或记录阶段出现 `ERR = 0` → 立刻置零，run 判 invalid |
| 急停 / `s1` 下位 / 遥控失联 / 翻倒 | 立刻置零，run 判 invalid |
| 腿长粗兜底（< 0.10 m 或 > 0.25 m） | 立刻置零，run 判 invalid |
| VOFA 丢帧（`drop_cnt > 0`）或 TX 完成超时 | 记录照留，run 判 invalid |

> `ERR` 是**状态码不是故障位**，判据必须是"落在故障码集合里"，不能写成 `err != 0`。

**B 级：阈值保护（作者决定不设）**

- 温度、速度、位置**不设阈值**：仍然记录（`T_MOS`、`T_Rotor`、速度、位置都是免费信息）。
- 依据：手册写明驱动器过温 120 °C 关机、线圈过温可配（推荐 100 °C），**触发后电机自动退出使能**并给出 `ERR = B/C`——我们的 A 级判据已经能接住，不需要自己再定温度线。

### 11.2 纪律

- **日志通道不是安全通道**：任何发送阻塞/丢帧都不得影响"置零"路径；置零走原有 `Dm_Send_Zero()`/`Dji_All_Stop()`。
- 无效 run 的 CSV 保留（算法侧要看现象），由 `manifest.yaml` 的 `run_valid` 与 README 说明。
- `manifest.yaml` 里未设阈值的项要**明确写"仅记录、不设阈值"并签字**，不能留空——留空会被算法侧当成"未填写不得执行"。

---

## 12. 交付数据规格

### 12.1 目录结构（算法侧仓库）

```
data/sysid/chuanliantui/<date>/wheel-<motor>-<run_id>/
    c620_command_raw.csv
    c620_feedback_raw.csv
    manifest.yaml
    README.md
    checksum.sha256
data/sysid/chuanliantui/<date>/joint-torque-<run_id>/
    joint_torque_snapshot.csv
    torque_program.yaml
    kinematics_manifest.yaml
    manifest.yaml
    README.md
    checksum.sha256
```

### 12.2 关节 CSV 列（严格按文档）

```
t_cmd_can_tx_ns,
tau_lf0_Nm,tau_lf00_Nm,leg_length_left_m,leg_pitch_left_rad,
tau_rf0_Nm,tau_rf00_Nm,leg_length_right_m,leg_pitch_right_rad
```

- 每行只有：CAN 发送时间戳、四路实际电机力矩、左右腿长、左右腿倾角。不塞 PD 目标、轮数据、参考轨迹。
- 电机名与符号的定义见 §14.6。

### 12.3 `manifest.yaml` 必填项

| 字段 | 内容 |
| --- | --- |
| `machine` | 机器签名（对应 §5） |
| `dm_scales` | 自检读回的 `PMAX/VMAX/TMAX/CTRL_MODE`（四台） |
| `limits.dm` | 力矩激励钳位（签字值）；速度/位置/温度写"仅记录、不设阈值" |
| `limits.wheel_current_a` | 轮端允许电流（±15 A = ±12288 raw） |
| `fk.version` | `FK_VERSION` |
| `fk.params` | `lu/lg/offset_f/offset_b/offset_phi0/mirror`（左右） |
| `timestamp` | 时钟源与语义（DWT ns；`t_cmd`=TX 完成；`t_fb`=RX 回调入口） |
| `runs[]` | 每次 run 的 `run_id/test_id`、`drop_cnt`、`clamp_cnt`、`tx_timeout_cnt`、`run_valid` |

### 12.4 `torque_program.yaml` 必填项

四路实际力矩序列、周期、签字安全上限（20 N·m）、`FK_VERSION`。数值由作者/算法侧给定，固件表与其一致。

### 12.5 `kinematics_manifest.yaml` 必填项

见 §14.6。

### 12.6 校验

- `checksum.sha256`：由 Python 脚本对每份 CSV 计算。
- 质检项：行数、`seq` 连续性、时间戳单调、峰值电流/力矩 ≤ 上限、丢帧计数为 0、`run_valid`。

---

## 13. 验收清单

| # | 测试 | 期望 |
| --- | --- | --- |
| 1 | DM 满量程核对 | 用达妙上位机读到的 `PMAX/VMAX/TMAX` 与 `CTRL_MODE` 和 `machine_config.c` 当前机器一致 |
| 2 | 空跑（电机不上电） | 时间戳单调、无 1 ms 台阶；命令行数 = 节拍数；`seq` 无缺口 |
| 3 | Vofa+ 存盘 → 脚本 | 列对齐、`kind` 拆分正确、ns 重建与 MCU 打印一致 |
| 4 | 限值测试 | 故意请求超限电流/力矩 → 被钳到上限，`clamp_cnt` 可见 |
| 5 | 节奏测试 | s1 上拨后 run 自动连续切换，每段边界都有 kind=5 行；s1 下位立刻中止 |
| 6 | 故障测试 | 拔 CAN 线 / 拔遥控 → 立刻置零，run 标 invalid；`ERR` 码判据生效 |
| 7 | 静态 FK 核对 | 手推到若干姿态，腿长/腿倾角与卷尺、量角器一致（文档要求的前置） |
| 8 | 时钟漂移 | 10 s 内与 `HAL_GetTick()` 偏差 < 2 ms |
| 9 | 端到端小样 | A：±0.5 A `baseline_sign`（顺带量 `G_total`）；B：5 s 全零 → 产出可用 CSV |

---

## 14. 坐标系与 FK 定义（交付强化训练端）

> 来源：`md/RL_OVERVIEW.md`（极性与坐标定义，已确认）、`leg_solver.c`、`md/IO_CHAINS.md`、DM 手册（单位在输出轴）。
> **物理符号/轴向未经台架标定不得修改。**

### 14.1 机体坐标系

- `x` 前、`y` 左、`z` 上。
- 右腿、右轮在**驱动解码边界**统一镜像到机体坐标（`dm.c`、`dji.c`），后续模块禁止重复取反。
- IMU（HI229）：加速度/角速度/欧拉角/四元数虚部的 X、Z 取反，Y 保持（详见 `md/RL_OVERVIEW.md`）。

### 14.2 髋关节角

| 量 | 定义 |
| --- | --- |
| 左髋反馈 | `feedback_sign = +1`；右髋 `= −1`（驱动层已取反） |
| 前髋几何角 | `hip_f = DM 反馈角 + π + offset_f` |
| 后髋几何角 | `hip_b = DM 反馈角 + offset_b` |
| 左右腿 | 同一套前后映射，`mirror = +1`，五连杆层不再镜像 |

### 14.3 腿任务坐标（试验 B 的交换量，与 MuJoCo 必须同名同义）

| 量 | 定义 | 正方向/零位 |
| --- | --- | --- |
| `leg_length` | `|OP|`，O = 两髋同轴中心，P = 两下杆交点（**是否即轮心销待机械确认**） | 恒正，伸腿变大 |
| `leg_pitch` | 机体系内 O→P 相对竖直的夹角，`π/2 − atan2(y_p, x_p) + offset_phi0` | **前摆为正**；零位由 `offset_phi0` 标定（左右不同） |

- 这是**机体系**定义；LQR 用的"世界系腿摆角 = `−leg_pitch + pitch`"只属于 LQR，sysid 与 RL 均用机体系。
- MuJoCo 侧必须输出同定义的 `leg_length`/`leg_pitch`，否则两边差常数，拟合直接失效。

### 14.4 其他关节与符号

| 量 | 约定 |
| --- | --- |
| 大腿角 `thigh_angle` | `wrap(qf)`，前髋上连杆绝对角（RL 观测/PD 用） |
| 虚拟小腿 `virtual_shank_angle` | `wrap(φ_a − qf − π/2)`，相对前髋电机；训练侧 `lf1/rf1` 对应此量，**不是实物电机** |
| 轮 | 左轮 `feedback_sign = +1`、右轮 `= −1`；前进为正 |
| DM 力矩下发 | 右腿在 `dm.c` 驱动边界取反一次，其他模块禁止再取反 |

### 14.5 单位与换算

| 量 | 单位 | 说明 |
| --- | --- | --- |
| 长度 | m | |
| 角度 | rad | DM 反馈的 `POS` 是**输出轴（减速后）**角度，FK 直接可用 |
| 角速度 | rad/s | DM 的 `VEL` 同为输出轴量；C620 的 `speed_rpm` 是**转子侧**，轮端需 ÷`G_total` |
| 力矩 | N·m | DM 的 `T` 为输出轴力矩；记录值用 URDF 关节正方向 |
| 轮电流 | A（电机侧）/ raw | `raw = round(I_A × 819.2)`；轮端 N·m 需 `G_total` 与力矩常数，不在下位机换算 |

### 14.6 关节名对应与 `kinematics_manifest.yaml`

**命名（保持固件顺序，待训练端确认符号）**：

| URDF 名 | 固件索引 | 控制 ID | 物理位置 | 说明 |
| --- | --- | --- | --- | --- |
| `lf0` | `DM_MOTOR_LEG_F_LFT` = 0 | 0x01 | 左前髋 | 驱动大腿 `qf`（`thigh_angle = wrap(qf)`） |
| `lf00` | `DM_MOTOR_LEG_B_LFT` = 1 | 0x03 | 左后髋 | |
| `rf0` | `DM_MOTOR_LEG_F_RGT` = 2 | 0x02 | 右前髋 | 反馈与下发在 `dm.c` 边界取反 |
| `rf00` | `DM_MOTOR_LEG_B_RGT` = 3 | 0x04 | 右后髋 | 同上 |

- 文档只要求"列名用 URDF 真实主动轴名"+"力矩方向/单位已验证"，所以我们要交的是**命名与符号的对应声明**，不需要提供 URDF 结构；训练端对照 URDF 轴线确认是否需要整列取反。
- `kinematics_manifest.yaml` 至少含：两电机基座坐标（本机两髋同轴，需写明同轴或偏移）、连杆长度（`lu`、`lg`）与销轴拓扑、电机零位与正方向（含 `+π` 几何零位的含义）、腿长参考点（O、P 的物理定义）、腿倾角零位与正方向、上表、`FK_VERSION`。

---

## 15. 实施步骤（每步可独立验证）

| 步 | 目标 | 改动 | 验证 | 回退 |
| --- | --- | --- | --- | --- |
| 0 | 确认 §3.2 各项 | 无代码 | 拿到 `G_total`、关节名确认 | — |
| 1 | 机器配置表（两份表 + 运行时选择） | 新增 `machine_config.c/h`；`dji.c`、`dm.c` 读 `machine->…`；`main.c` 一行 `Machine_Select` | 编译通过；切换机器后读数符合预期 | 保留原常量注释 |
| 2 | 单调时钟 | 新增 `mono_ns.c/h`；`main.c` TIM6 回调里扩展 | 与 `HAL_GetTick()` 对比 <2 ms | 单独开关宏 |
| 3 | CAN 收发时间戳 | `dji.c`/`dm.c` 存 `rx_ns`；`can_bus.c` 开 TX 完成 + pending 表 | 打印连续帧的 tx/rx 差值合理 | 保留 ms 字段 |
| 4 | VOFA sysid 通路 | `Vofa_send.c/h` 环形缓冲 + TxCplt 泵；`sysid_log.c/h` | 发固定测试帧，Vofa+ 列对齐、seq 连续 | sysid 关闭则走原路径 |
| 5 | 节奏与触发 | `sysid_program.c/h`：序列表、run 自动切换、零段；s1 语义接线 | s1 上拨自动连跑；下位立刻中止 | 不影响原手动遥操 |
| 6 | sysid 仲裁 | `task_actuation.c` 加第三分支 | 进入 sysid 后 LQR/手动不发力 | 开关回原分支 |
| 7 | 试验 B 最小闭环 | 序列表 `torque_baseline`；FK 快照 + 时序 | 5 s 全零行可用；静态 FK 核对 | — |
| 8 | 试验 A 最小闭环 | 500 Hz 电流命令 + 反馈行；±0.5 A `baseline_sign` + `G_total` 标定 | 行列数与 seq 正确 | — |
| 9 | 全部用例 | stiction/plateau/step/holdout、torque_step/chirp/PRBS | 逐用例验收 | — |
| 10 | 安全与无效标记 | 钳位、ERR 码判据、离线/越界、`run_valid`、计数通道 | 故障测试通过 | — |
| 11 | 交付脚本与文档 | Python 后处理、manifest 模板、README | 端到端产出符合文档目录 | — |

---

## 16. 风险与对策

| 风险 | 后果 | 对策 |
| --- | --- | --- |
| 固件满量程与电机实际配置不一致 | 角度/力矩整列作废 | §8.2 启动自检，不一致拒绝进入；读回值写 manifest |
| 把额定值当 MIT 刻度 | 同上 | 手册明确区分：额定 20 N·m/100 rpm，MIT 预设 ±12.5/±45/±54 |
| `ERR` 当故障位用 | 正常使能态被误判为故障 | 判据改为"落在故障码集合 {8..E} 内" |
| 机器切换漏切一处 | 静默产出错单位数据 | 单一宏分支 + `#error` + 启动签名通道 |
| VOFA 丢帧不留痕 | 时间基准错位，拟合失真 | `seq` + `drop_cnt`，丢帧 run 判无效 |
| 32ch 与 sysid 帧混发 | Vofa+ 列错位 | sysid 模式停 32ch |
| TX 完成配对按顺序 | 优先级模式下配错 | 按 TX 元素索引配对；必要时改 Queue 模式（CubeMX） |
| 腿倾角零位与模型不一致 | 四个任务坐标对不上 | §14.3 定义 + 静态 FK 核对 + manifest 记录 |
| 自动连跑时失控 | 机械损伤 | 急停随手可及；s1 下位立即中止；`±15 A`/力矩钳位；run 结束必回零 |
| 记录占 CPU | 影响 1 kHz 实时性 | 中断只入环、任务里打包；帧长固定、避免浮点格式化 |

---

## 17. 状态汇总

**已定**：机器配置表方案、VOFA 数据通路、测试类型编译期选 + run 自动连续 + s1 只做 deadman/中止、时间戳方案、FK 用 RL 侧定义、DM 满量程以电机实际值为准并启动自检、许可力矩 20 N·m 作激励钳位、温度/速度/位置不设阈值只记录、关节名保持固件顺序。

**待确认（不阻塞开工）**：电机实际 `PMAX/VMAX/TMAX`（自检读回即可）、`G_total`（可现场实测）、关节名符号（训练端确认）、`eta_total`（先给估计值）、P 点定义与 `offset_phi0` 零位（台架静态核对）、轮命令率 500 Hz 是否够、旁证帧是否启用。

### 17.1 实施进度（步骤表对照）

| 步 | 内容 | 状态 | 依据 |
| --- | --- | --- | --- |
| 0 | 参数确认 | ✅ | `G_total = 15.5`、腿长区间 0.14~0.34 已入配置表（变更 34） |
| 1 | 机器配置表 | ✅ | `machine_config.c/h`，两份表 + `Machine_Select` |
| 2 | 单调时钟 | ✅ | `mono_ns.c/h` |
| 3 | CAN 收发时间戳 | ✅ | 变更 32（RX）、35（TX 完成 + pending 表）、38（修 TX 中断假记录） |
| 4 | VOFA sysid 通路 | 🟡 待接回 | `sysid_log.c/h`：37 列独立帧 + 环形缓冲 + 250 Hz 发送泵已写好；但 `task_comm.c` 当前未调用发送泵、也未在测试模式停发 32 路，且 1 kHz 推帧需抽取（见 §7.1 现状） |
| 5 | 节奏与触发 | ✅ | 变更 39：激励表 + 状态机写在 `sysid_mode.c`（**未按 §4.1 新建 `sysid_program.c`**，改动更小）；s1 上 + s2 中进入，离开即中止 |
| 6 | sysid 仲裁 | ✅ | 变更 33 + 39：`task_actuation.c` 第三分支，进入后 LQR/手动不发力 |
| 7 | 试验 B 最小闭环 | ✅ 代码就绪，🟡 待台架 | `torque_baseline`（四路 0 Nm 5 s）+ FK 快照；静态 FK 卷尺核对未做 |
| 8 | 试验 A 最小闭环 | ✅ 代码就绪，🟡 待台架 | `baseline_sign` + raw 电流直发；轮命令极性按 `dji_sign.out` 换算（变更 40） |
| 9 | 全部用例 | ✅ 代码就绪，🟡 待台架 | 腿 33 run / 轮 46 run，`SYSID_PLAN` 选择 |
| 10 | 安全与无效标记 | ✅ 部分 | 钳位、离线、腿长越界、温度、中止标记已有；`err_raw` 故障码判据**未接**；环形缓冲满不计数（靠 `seq` 跳号检测） |
| 11 | 交付脚本与文档 | ✅ | `tools/sysid_export.py`（含 `--selftest`）+ `md/sysid/sysid-delivery.md` |

**本轮明确不做**（作者决定）：气弹簧补偿——留给后续优化；本轮交付给训练端的数据不含该补偿项，采集时气弹簧仍物理存在，属已知未建模外力。

**编译/自测证据**（主代理复核）：默认 / `-DSYSID_ENABLE=0` / `-DSYSID_ENABLE=1` 三种配置均 `105 files, 0 fail, 0 warn`；`py tools/sysid_export.py --selftest` 全项通过。
