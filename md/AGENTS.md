# AI 协作规范 — 轮腿平衡步兵 RL 部署

> 最后更新：2026-09-25
> 适用：Claude / Cursor / Copilot / Codex / Gemini / Kimi Code 等任何 AI 助手。
> 接手本仓库前**先读完这一篇**，再动手。

---

## §0 修改授权（最高优先级）

- **未完成修改计划确认，且作者没有明确说“可以开始修改”或同等明确授权前，禁止修改任何代码、配置、文档或工程文件。**
- “继续分析”“检查一下”“给出方案”“进行测试”等表述只授权只读检查与计划，不构成修改授权。
- 计划、修改范围或物理符号/极性发生变化时，必须重新说明计划并取得作者明确授权后再写入。

---

## §0.1 物理量绝对红线（最高约束 · 优先于本文件全部其余条款）

> **事故来源（作者原话："以后不允许做这种东西"）**：2026-09-18，AI 在"腿几何搬进机器配置表"那一批里，顺手按"参考固件推导"把大机器 `dm_sign` 的**两个前髋极性取反**（`{{1,1},{1,1},{-1,-1},{-1,-1}}` → `{{-1,-1},{1,1},{1,1},{-1,-1}}`），既未单独报备，也未在变更记录里单列。结果台架腿部解算错乱：**摆动腿时腿长跟着动、腿摆角却不变**。由作者台架测出、自行还原。

- **禁止以任何理由修改物理量**：极性（`dm_sign` / `dji_sign` / `rl.sign`）、零点（`dm_zero` / `leg_off_*` / `rl.zero`）、轴向正负、环绕约定、MIT 量程刻度（`dm_pos_max` / `dm_vel_max` / `dm_trq_max`）、`mirror`、`+ LEG_PI` 之类的符号项。**无论 AI 的推导、参考代码对照、手册解读、数值仿真多么自洽，都不构成改动理由。**
- 这类量**只能由作者在台架上实测确定**。AI 只允许做三件事：**指出可疑点、给出可执行的验证方法、解释现象**。
- AI 推出的结论**只能写在对话里供作者判断**，不得写进代码、配置或文档里的"当前值"。
- 即使作者要求修改，也必须：先复述"改哪个文件哪一行、从什么改成什么、依据是什么" → 作者确认 → 改完在 `md/sysid-change-map.md` **单独记一条**（谁 / 何时 / 依据）。
- 拿不准的物理量一律写成 **"待台架"**，不得写"已完成"。
- 违反本条视为**最严重错误**。

---

## §1 沟通方式（最高优先级）

- **全程中文**，技术术语保留英文。
- **先对齐目标再动手**：拿不准就复述"你是想达到 X 效果对吧"，确认后再干。
- **给结论 + 推荐，不要选项清单**：需要决策时给推荐 + 一句理由，不罗列方案让 TA 选。
- **简洁直接**：少废话、少铺垫、直接上结论。
- **不要瞎猜**：不准就多问，经可靠证据确认事实再回答。

---

## §2 接手项目的第一步

1. **先读本文**（AGENTS.md）+ `md/` 下各模块说明。
2. 没有文档的模块 → **先建**，这是后续高效协作的地基。
3. 文档与代码冲突时**以代码为准**，并顺手修文档。

---

## §3 改动哲学

- **外科手术式最小改动**：只动该动的，不做"顺手优化"。新代码风格、命名、注释密度**贴合周围既有代码**。
- **单一职责**：每次改动只解决一个问题。
- **一次只改一个变量**：作者在持续调参，改动要可归因。
- **保留可回退的对照开关**：上新方案不删旧路径，用运行时开关做 A/B 切换。
- **CUBE MX 保护区**：`USER CODE BEGIN/END` 之外的代码不可手动修改，由 CubeMX 管理。
- **变量定义都在最前面**：函数内局部变量也在函数开头定义。
- **少建散变量，用结构体聚合**：相关变量归进 `typedef` 结构体统一命名。尤其别为"方便 Vofa 调试"新增一堆全局变量。
- **注释规范化**：不需要大段的注释，更多的一些重点的运算逻辑核心，当然如果有什么特殊的点，也可以注释，代码中的注释不需要太多，简短重点10个字或者5个字以内就可以，大段注释在项目md或者代码总览md之中即可
- **if/for 规范化**：所有 `if`、`else if`、`else`、`for` 必须加花括号，即使只有一行。不依赖缩进。

### 任务与模块分层约束

- `freertos.c` 只负责任务创建、初始化调用、循环节拍、`osDelay` 和信号量等待；不放协议解析、运动学、控制器或电机报文代码。
- `imcalib/task/robot_control.c` 负责共享状态定义、总初始化、动作清零和模型切换；各 `task_*.c` 只实现对应任务的单周期逻辑。
- `commTask` 负责通信输入输出：DR16/DM/DJI 接收解析、状态刷新、在线/故障监测和 VOFA 调试发送；HI229 姿态链路归 `imuTask`。它不等同于纯故障监视任务，因此命名使用 `comm`/`communication`，不要继续使用含义过窄的 `monitor`。
- `policyTask` 负责观测构建和 RL 策略推理；`actuationTask` 负责按实时节拍读取已准备状态并完成执行链路。
- **策略仲裁只在 `task_actuation.c`**：左拨杆选模式（`strategy_from_remote()`：中 = LQR，上 = RL，下 / 离线 = 失能），右拨杆中位 = 投入出力，其他位 = 已选模式但零力矩。LQR 用 `lqr_balance.c` + `leg_balance.c`，RL 用 `rl_torque.c`。两套链路互不 include（除公共 `torque_output.h`），禁止在 LQR 模块引用 `rl_*.h`，也禁止在 RL 模块引用 LQR 状态。
- 五连杆几何和雅可比只能写在 `leg_solver.c/h`；姿态只能写在 `Attitude_Algorithm.c/h`；观测、策略、力矩映射分别归属对应 Algorithm 模块。任务文件只调用这些接口。
- DR16 字节解析只能归属 `dr16.c/h`。遥控死区、通道归一化归属 `user-lib/rc_command.c/h`（`commTask` 每拍填一次全局 `rc_command`，全机唯一解算点）；LQR / RL / 仲裁只读 `rc_command`，禁止再各自 `DR16_Snapshot()` 解摇杆。拨杆语义（挡位 → 策略）只在 `task_actuation.c`。不能让底层 DBUS 驱动直接操作电机。
- DM 的使能、失能、MIT 量化和报文发送归属 `dm.c/h`；DJI 零电流和轮电流发送归属 `dji.c/h`。任务层只调用语义清晰的接口，例如 `Dm_All_Disable()`、`Dji_All_Stop()`。
- 左拨杆下位的安全决策可以留在执行任务；具体 DM 失能和 DJI 零电流报文必须下沉到对应驱动模块。
- 不为了文件可读性随意新增 FreeRTOS 任务。只有当频率、优先级、截止时间或数据所有权确实不同，才新增任务；纯职责拆分优先使用模块和短函数。
- 任务职责不明确时，先向作者确认边界，禁止直接重排任务或跨模块搬运主体代码。

---

## §4 嵌入式代码注释规范

- **注释原则**：能理解具体目的，但不臃肿。宁短勿长，宁可不写也不写废话。
- **行内注释**：≤ 5 个汉字，例 `/* 零偏 */`、`/* 卡尔曼 */`
- **禁用长注释**：不在代码行尾写大段解释，详细说明写入 `md/` 下对应文件
- **函数前注释**：一行概括即可，例 `/* 读取 + 轴映射 + 异常检查 + 零偏 + 卡尔曼 */`
- **结构体字段**：可以加简短注释说明用途，≤ 5 个汉字，例 `float quat[4]; /* 四元数 */`；含义在 md 文档中展开

---

## §5 参数与文档纪律

- 项目处于**高频调参期**。**文档只记结构、方案、"为什么"，不追具体数值**——参数以代码为准，文中数值一律视为"某时刻快照、可能已过时"。
- **改了代码 → 同步文档**。
- **删文件 → 修引用**：把指向它的链接一并改掉，不留死链。

---

## §6 验证与诚实

- 本项目是 **Keil MDK / eIDE** 工程。Keil AC5 在 `D:\keil\keil_core\ARM\armcc_5\bin\armcc.exe`，完整编译参数取自 `build/CtrBoard-H7_ALL/compile_commands.json`。改完先编译；核验编译必须把 `-o` 改到临时目录，不能污染 eIDE 增量构建目录。启动汇编用 `armasm.exe`。
- **但链接、下载、上机做不了** → 物理符号、增益、轴向一律标 **"待台架 / 待实测"**。
- **能做的核验要做**：纯数学/几何推导、数值仿真（Python）有价值，做了就说"已数值核验"；但物理符号、增益、轴向只能上台架定。
- **调参观测靠 Vofa+**：显示值不对 → **先查打包/下标，再怀疑算法**。
- **如实报告**：失败就说失败，跳过就说跳过，做完验证了才说"完成"。

---

## §7 红线（未经明确许可不要做）

- ❌ **不臆改物理量（极性 / 零点 / 轴向正负 / 量程刻度 / 符号项）——见 §0.1，最高约束；只能台架标定。**
- ❌ 不用低通 / 降增益掩盖震荡——找根因从源头治。
- ❌ 不一次性大重构 / 大批量改参。
- ❌ 不删除你没创建的文件/代码——先说明，别径直删。
- ❌ 不做对外发送 / 不可逆操作前不确认。
- ❌ 代码检查时揪注释命名等次要细节，却漏掉控制链路输入输出 bug。**优先核对状态机入出链 + 当前激活算法的变量流向。**

---

## 项目文件结构

```
CtrBoard-H7_ALL/
├── Core/Src/
│   ├── main.c              ← 初始化 (mono_ns/DR16/HI229/CAN/DM/DJI + FDCAN 数据段套用) + FreeRTOS 启动
│   ├── freertos.c          ← 5 任务创建 (comm/imu/policy/actuation/default)
│   └── ...                 ← 其余 CubeMX 生成
├── imcalib/
│   ├── Algorithm/
│   │   ├── Attitude_Algorithm.c/h ← 姿态数据归一化 (HI229 四元数 + 欧拉角)
│   │   ├── imu_state.h            ← IMU 状态结构
│   │   ├── leg_solver.c/h         ← 五连杆闭链 + 雅可比 (镜像/门控)
│   │   ├── lqr_balance.c/h        ← LQR 状态估计 + 增益求值 + 状态反馈 + 遥控目标
│   │   ├── leg_balance.c/h        ← 腿长/横滚 + 力域映射 + 力矩下发
│   │   ├── lqr_gain_table.c/h     ← LQR 增益表 (MATLAB 生成物, 勿手改)
│   │   ├── torque_output.h        ← 公共力矩输出结构 (DM/DJI 分离)
│   │   ├── rl_observation.c/h     ← RL 观测构建 + 5帧历史
│   │   ├── rl_policy.c/h          ← CubeAI 推理封装 (单模型 networkzn1)
│   │   └── rl_torque.c/h          ← 动作→力矩执行层
│   ├── task/
│   │   ├── inc/robot_control.h    ← 共享状态与跨任务接口
│   │   ├── robot_control.c        ← 总初始化、动作清零、模型切换
│   │   ├── task_imu.c             ← HI229 与姿态更新
│   │   ├── task_policy.c          ← 观测构建与策略推理
│   │   ├── task_actuation.c       ← 策略仲裁 + 力矩计算与电机下发
│   │   └── task_comm.c            ← 通信、状态、遥控、故障与 VOFA (+ S2R_Pump 挂点)
│   ├── Telemetry/             ← 整机诊断遥测 S2R1 (gap 测试, 自包含, 只读现有状态)
│   │   ├── s2r_wire.c/h           ← 记录/队列/小端编码/CRC32
│   │   ├── s2r_source.c/h         ← 采样适配层 (变化检测, 自维护序号)
│   │   └── s2r_telemetry.c/h      ← 七类帧成帧、会话/事件、META、UART DMA 泵
│   ├── user-lib/
│       ├── uart_idle.c/h          ← UART IDLE+DMA 底层框架
│       ├── dr16.c/h               ← DR16 遥控器
│       ├── rc_command.c/h         ← 遥控指令: 死区 + 归一化 + 拨杆, 全机唯一解算点
│       ├── hi229.c/h              ← HI229 IMU
│       ├── can_bus.c/h            ← FDCAN 总线管理
│       ├── dm.c/h                 ← 达妙电机 (MIT)
│       ├── dji.c/h                ← DJI 轮电机
│       ├── machine_config.c/h     ← 两份电机配置表 + 编译期选择 (MACHINE_DEFAULT)
│       ├── mono_ns.c/h            ← 单调 ns 时钟 (DWT + TIM6 扩展)
│       ├── simple-function.c/h   ← 简单函数库 (一阶低通 Lowpass_* / 斜坡 Ramp_*)
│       ├── kalman.c/h            ← 带加速度输入的一维卡尔曼 (速度估计, 同 Leg2_v1)
│       ├── pid.c/h               ← 通用 PID (腿长/横滚/轮速环共用)
│       ├── dma_cache.h           ← D-Cache 清理/失效 (DMA 收发配套)
│       ├── ws2812.c/h            ← 板载 WS2812 (SPI6), commTask 周期闪烁
│       ├── BMI088driver.c/h / BMI088Middleware.c/h / BMI088reg.h ← 板载 BMI088 (SPI2, 当前未接入)
│       ├── arm_sin_f32.c / arm_cos_f32.c / arm_sin_table_f32.c ← CMSIS-DSP 1.6.0 查表 sin/cos 源码 (leg_solver 用)
│       └── Vofa_send.c/h          ← Vofa+ 调试发送
├── X-CUBE-AI/App/          ← X-CUBE-AI 生成的 networkzn1 (chuanliantui 起立策略, 2026-09-22; networkzn1.c/h + config/data/data_params)
└── md/
    ├── AGENTS.md            ← 本文件 (AI 协作规范)
    ├── CLAUDE.md            ← Claude Code 薄指针
    ├── RL_OVERVIEW.md       ← RL 部署总览: 代码链路 + 进度 + 待实测清单
    ├── LQR_PLAN.md          ← LQR 嵌入计划 + 实施记录 + 决策与遗留项
    ├── IO_CHAINS.md         ← IMU/DM/DJI 输入输出链路速查
    ├── DBUS.md              ← 遥控器解析说明
    ├── UART_IDLE_DMA.md     ← 串口接收框架说明
    ├── sim2real_serial_protocol.md ← S2R1 诊断遥测协议 (帧布局/字段语义)
    ├── sim2real_serial_capture.md  ← 接线 + 采集 + 验收清单
    ├── sysid-change-map.md  ← 每处改动的输入/输出/调用链
    └── sysid/               ← 大机器测试历史文档（固件测试模块已移除）
        ├── sysid-lower-machine-plan.md
        ├── sysid-delivery.md
        ├── controller-spec-for-mujoco.md
        └── chuanliantui-wheel-joint-sysid-handoff.md
```

`tools/sysid_export.py` ← 上位机导出 (VOFA 文件 → 契约 CSV + manifest + 校验和)
`tools/s2r_capture.py` ← S2R1 被动接收/离线解码（不发送任何指令）；`tools/s2r_build_info.py` ← 生成/校验 `imcalib/Telemetry/s2r_build_info.h` 指纹（**编译前必须重生成**）；`tests/` ← 协议一致性与替身测试（`CC=gcc python tests/test_s2r.py`）
`tools/matlab/` ← LQR 增益表 MATLAB 管线：`run_all.m`（选机器 + Q/R，日常只改这个）、`machine_table.m`（机械参数）、`build_gain_table.m`（网格 dlqr / 拟合 / 闭环检查 / 写 C）、`model_AB.m`（动力学模型），计划与进度见 `tools/matlab/LQR_MATLAB_PLAN.md`

---

## 已完成模块

| 模块 | 文件 | 状态 |
|------|------|------|
| 姿态解算 | Attitude_Algorithm.c/h | ✅ 完成 |
| 串口底层 | uart_idle.c/h | ✅ 完成 |
| 遥控器解析 | dr16.c/h | ✅ 完成，已实测 |
| HI229 IMU | hi229.c/h | ✅ 完成 |
| Vofa 调试发送 | Vofa_send.c/h | ✅ 完成，32 通道（上限 32）JustFloat DMA；通道表见 `task_comm.c` 的 `Robot_Control_Send_Vofa()` 上方注释 |
| FDCAN 总线 | can_bus.c/h | ✅ 完成 |
| DM 电机 | dm.c/h | ✅ 完成 |
| 单调 ns 时钟 | mono_ns.c/h | ✅ 新增，DWT + TIM6 扩展；`Mono_Ns_Get()` 当前仅 `rl_policy.c` 用于推理耗时计量（CAN 收发时间戳是早期 sysid 规划，已随 sysid 模块移除） |
| 机器配置表 | machine_config.c/h | ✅ 新增，**两份表 + 编译期选择**（`machine_config.h` 的 `MACHINE_DEFAULT`，`Machine_Id()` 只读查询；无运行时切换接口） |
| DJI 轮电机 | dji.c/h | ✅ 完成，减速比已修正 |
| 五连杆 | leg_solver.c/h | ✅ 完成，thigh_angle 根因修复已验证；三角函数走 CMSIS-DSP 查表（`LEG_TRIG_LIBM=1` 退回 libm） |
| RL 观测 | rl_observation.c/h | ✅ 缩放/默认角已按训练侧填；🟡 大机器 `.rl` 候选 A 已填且 `configured=1`（`machine_config.c` 大机器表，数值以代码为准、待台架），小机器 `.rl` 未配置（`configured=0`） |
| CubeAI 推理 | rl_policy.c/h | ✅ 单模型 networkzn1，已接进 policyTask（`ctrl_task_body()` 按 `machine->rl.configured` 门控，无 `infer_enable` 字段；投入后另有 `RL_WARMUP_STEPS` 预热）；🟡 待台架 |
| 力矩执行层 | rl_torque.c/h | ✅ 完成，DM/DJI 分离输出 + 轮子 PID |
| 任务框架 | task/robot_control.c + task_*.c | ✅ 完成，已上机验证 |
| 遥控指令 | user-lib/rc_command.c/h | ✅ 四轴归一化 + 拨杆，commTask 填、其余只读；ch1 死区 10 |
| LQR 增益表 | lqr_gain_table.c/h | ✅ 由 `tools/matlab/run_all.m` 管线生成物（勿手改），当前板上表为小机器 sjtu5 模型输出（表号/日期/Q/R 见 `lqr_gain_table.c` 头注释）；🟡 大机器表未接入（`machine_config.c` 大机器 `lqr_configured=0`）；历史：早期为参考上车表移植，已被 MATLAB 管线输出覆盖 |
| LQR 状态估计与控制律 | lqr_balance.c/h | ✅ 编译通过，含 `lqr_debug` 运行时通道/限幅 A/B；🟡 **待台架** |
| 腿部力控与下发 | leg_balance.c/h | ✅ 编译通过；🟡 **待台架** |
| 策略仲裁 | task_actuation.c | ✅ 编译通过（左拨杆 中=LQR / 上=RL / 下=失能；右拨杆中位=投入，其他=零力矩）；🟡 待台架 |
| 简单函数库 | user-lib/simple-function.c/h | ✅ 一阶低通 + 斜坡函数，编译通过 |
| 速度卡尔曼 | user-lib/kalman.c/h | ✅ 照抄 Leg2_v1，编译通过；🟡 加速度符号待台架 |
| 整机诊断遥测 | Telemetry/s2r_wire + s2r_source + s2r_telemetry | ✅ 移植完成（去耦采样 + 单挂点 `S2R_Pump()`，Keil 全量编译链接 0 error；协议测试 8/8）；🟡 **待台架采集**（接线/验收见 `md/sim2real_serial_capture.md`） |

---

## 关键约束

- **时钟**：HSE 24MHz → PLL → SYSCLK 550MHz，APB1 137.5MHz，定时器时钟 275MHz
- **控制频率**：actuationTask 由 TIM6 信号量驱动，频率随机器表编译期切换（`machine_config.h` 的 `MACHINE_TIM6_PERIOD`/`MACHINE_CTRL_DT`，`tim.c` USER CODE 2 里套用）：当前 `MACHINE_DEFAULT` = 大机器 → Period=1999 → **500 Hz**（`CTRL_DT=0.002f`，2026-09-26 起，变更 100），小机器 Period=999 → **1 kHz**（`CTRL_DT=0.001f`）。两者 Prescaler 均 274。LQR、RL 共用该节拍；policyTask 由同一节拍按 `MACHINE_POLICY_DIV` 分频唤醒（500/5、1000/10 = 100 Hz，变更 103）
- **LQR 腿长工作区间**：机器表区间与 K 表拟合域 0.13~0.23 m 的交集，只夹拨轮目标；投入不查实测腿长（同 Leg2，变更 92），趴地投入靠腿长 PID 撑起
- **LQR 辅助 PID**：当前保留左右腿长 PID 和横滚 PID；防劈叉 PID 已移除。参数以 `leg_balance.h` 为准，投入时清 PID 历史。腿长区间按本机自标，不照抄 Leg2（投入已不查实测腿长，无"投入下限"门槛）
- **FDCAN**：HSE 24 MHz，仲裁段 1 Mbps = NominalPrescaler=3 / Seg1=5 / Seg2=2（`fdcan.c` 三路一致；**发**只发经典帧、只走仲裁段；**收**不挑帧格式，经典帧与 FD 帧都收——达妙跑 FD 1M/4M，回帧是 FD，见变更 101）。数据段参数写死在 `fdcan.c`（FDCAN1 = FD_BRS + 1/4/1，FDCAN2 = classic + 3/5/2，FDCAN3 = classic + 1/4/1），`machine_config.h` 的 `MACHINE_FDCAN13_DATA_*` 目前**不生效**（`main.c:74-94` 的套用函数整段注释掉了）——**换机器不改任何 CAN 时序**
- **BMI088**：SPI 通信，驱动输出已是 rad/s 和 g，不要重复转换（单位换算以 `BMI088driver.h` 为准，待作者确认）。**当前未接入**：`main.c:160` `IMU_Init()` 已注释（"暂不使用"），驱动文件保留在 `imcalib/user-lib/`
- **HI229 姿态**：直接使用模块输出的四元数 + 欧拉角，Attitude_Algorithm 只做归一化和单位转换；取轴与符号来自机器表 `machine->imu`（`task_imu.c` 应用），驱动 `hi229.c/h` 只出原始值
- **标定**：500ms (200ms 暖机 + 300ms 采样)（历史记录：当前代码中已找不到对应标定/暖机流程，`imcalib`/`Core` 无相关实现；疑属已移除的 sysid/标定模块。作者 2026-09-25 确认：按现状保留为历史说明）
- **串口接收**：IDLE+DMA Circular，不使用 Resync，任务层校验
- **VOFA 调试**：32 通道 JustFloat、500Hz（`commTask` 1 kHz 周期内 `send_div` 每 2 拍发一次，`task_comm.c::Robot_Control_Send_Vofa()`）；当前是 **RL 布局**（ch3~6 下发力矩 / ch7、8 轮指令 / ch9~12 实测 DM 力矩 / ch13~16 观测四腿角 / ch17~22 关节速度 / ch23~28 上次动作 / ch29、30 轮电流 raw / ch31 `ctrl_fault`），LQR 布局整段留在该函数注释里可整段换回。**与 S2R1 遥测同口互斥**：`S2R_DIAGNOSTIC_DEFAULT=0`（或调试器把 `s2r_diagnostic_requested` 写 0）且失能、无会话时才会发 VOFA
- **诊断遥测 (gap 测试)**：`imcalib/Telemetry/` 自包含模块（`s2r_wire` 编码/队列 + `s2r_source` 只读采样 + `s2r_telemetry` 成帧/DMA 泵），**只读**现有状态、不改任何现有结构体；任务层唯一挂点是 `task_comm.c::comm_task_body()` 末尾的 `S2R_Pump()`。与 32 路 VOFA 同口互斥（`S2R_DIAGNOSTIC_DEFAULT`、`s2r_diagnostic_requested`，变更 104）。**`S2R_RATE_LOW` 限流档**：0 = 原设定（待机 ≈58 kB/s）、1 = 低速 ≈9.5 kB/s、2 = 稳健 ≈4.3 kB/s（**台架默认**）、3 = 极低 ≈1.6 kB/s（台架 USB 桥带不动 1152000 时用，换真 USB‑TTL 后改回 0，见 `md/sim2real_serial_capture.md` §2.1）。**短窗录制**：**默认全自动**（`S2R_RECORD_AUTO=1`）—— 投入建会话即按 md 的 100 Hz 把 CONTROL 完整快照存进 192 KB 片内缓存（≈4.1 s，满即停），失能后再录 200 ms 尾巴即停并自动慢速回放导出，**全程不需要调试器**；调试器写 `s2r_record_requested`/`s2r_replay_requested` 仍可手动覆盖（`s2r_record_state`/`s2r_record_frames` 只读），对应协议 §10 第一条建议 + §13.1。未插桩的字段按协议写 NaN/0，清单在 META 的 `unavailable`/`derived`。**改源码后必须先 `python tools/s2r_build_info.py` 再编译**（否则烧录代码与 META 身份不一致）
- **DJI 力矩常数**：`per_raw` 按型号满电流堵转力矩 / 满 raw × (`machine->dji_gear_ratio` / 标准减速比) 缩放，见 `dji.c` 的 `Dji_Torque_To_Current()`；**Kt 绝对值仍待台架实测**
- **机器切换**：改 `imcalib/user-lib/machine_config.h` 的 `MACHINE_DEFAULT`（两份表在 `machine_config.c`，含刻度、满量程、限幅、**极性**，以及 **IMU 取轴与符号 `.imu`**、**RL 关节映射 `.rl`**）；DM 的 PMAX/VMAX/TMAX 以电机实际配置为准，用达妙上位机读一次与配置表比对
- **CMSIS-DSP**：CubeMX 的 X-CUBE-ALGOBUILD 只生成头文件 `Middlewares/ST/ARM/DSP/Inc/arm_math.h`（1.7.0），**不挂库、不加源**。本工程用源码方式：`imcalib/user-lib/arm_sin_f32.c` / `arm_cos_f32.c`（照抄 `Drivers/CMSIS/DSP/Source` 1.6.0）+ `arm_sin_table_f32.c`（只截 513 点 `sinTable_f32`），头文件走相对路径 `#include "../../Drivers/CMSIS/DSP/Include/arm_math.h"`；两套工程都不需要改包含目录，eIDE 靠 `srcDirs` 自动扫到，Keil 已登记进 `imcalib/user-lib` 组。**不要把 `arm_common_tables.c` 整个当源文件编**（armcc 不拆数据段，700 KB 表整段进 flash），**也不要挂 `Drivers/CMSIS/DSP/Lib/ARM` 下的 .lib**：目录里 19 个库只有 `arm_cortexM7lfdp_math.lib` 对应本机，多挂时 armlink 不报错、静默取第一个（软浮点）；eIDE 开着时手改 `eide.yml` 几秒内被覆盖。再要用别的 DSP 函数，照同样办法把对应源文件抄进 user-lib
- **单位/坐标系/轴向**是嵌入式控制的头号 bug 源——改任何涉及姿态、力矩、符号、量纲的代码前，先确认约定。

---

## 参考实车代码

`XYEGA_RM2026_WheelLeg_Infatry_RLdeploy-main/source_code`，所有公式以 `chassis.cpp` / `math_core.cpp` 为准。
