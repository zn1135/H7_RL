# sysid 改动链路速查

> 2026-09-24：本文是历史变更记录。固件 Sysid 测试模块及相关入口已清理；下文提到的测试符号和编译配置不代表当前代码。

## 2026-09-25 · imuTask 轮询改 1kHz

- 输入：作者 2026-09-25 决定：imuTask 轮询 500Hz→1kHz，与控制节拍对齐（作者改 `Core/Src/freertos.c:207` `osDelay(2)`→`osDelay(1)`）。
- 输出：`imu_state` 更新延迟由 ≤2ms 收到 ≤1ms；HI229 去重（`ts`）保证同帧不重复处理；UART 接收缓冲取走频率翻倍。
- 调用链：`imuTask → HI229_Process → Attitude_Update → imu_state`（LQR/RL/comm 只读）。
- 核对：armcc 编译 `freertos.c` 0 错 0 警（-o 指向临时目录）；未链接、未上机。
- 备注：HI229 模块实际输出率与 128B 接收缓冲余量仍待台架复核。

## 2026-09-25 · 使能判定收敛为唯一函数（等价重构）

- 输入：作者 2026-09-25 批准：一个函数确定 `rc_enable`、后续复用；纠正方向——先判 `rc_enable` 再入后续。
- 输出：`imcalib/task/task_actuation.c` 新增 `strategy_rc_enable()`；`strategy_from_remote()` 改为消费 `robot_state.rc_enable` + s1；`imcalib/task/task_comm.c:74-76` 改调该函数；`task_comm.c:119-120` FAULT_ACTION 首判据改为 `ctrl_strategy == CTRL_STRATEGY_RL`（`robot_control.h:22`）；`robot_control.h` 新增声明。
- 调用链：comm 使能机 / actuation 策略仲裁 / 故障判定。
- 核对：armcc 编译 task_actuation / task_comm / robot_control，以并行任务结果为准，0 错 0 警；未链接、未上机。
- 备注：行为与改前严格等价；建议台架逐档回归：左下/中/上 × 配置组合验证使能与策略一致。

## 2026-09-25 · CAN 发送临界区 + FAULT_ACTION 不可用判据

- 输入：作者 2026-09-25 授权：第 2 点 CAN 发送竞态修复；第 3 点 FAULT_ACTION 增判据。
- 输出①：`imcalib/user-lib/can_bus.c` 的 `Can_Bus_Transmit`（`can_bus.c:136`）把空闲检查→HAL 调用→`last_tx_tick` 更新用 `taskENTER_CRITICAL`/`taskEXIT_CRITICAL` 包住，消除与中断/高优先级任务发送的交错窗口（窗口 <2 µs）。
- 输出②：`imcalib/task/task_comm.c::Robot_Fault_Update`（`task_comm.c:81`）新增判据：`output_task_rl_engaged()` 且 `action_state.rl_ready==0` 持续 ≥100 ms → `FAULT_ACTION`（函数内 static 计时，未投入/恢复即清零；warmup 期 `rl_ready=1` 不误报）。
- 调用链：actuation/comm 发送链；comm 故障判定链（`Robot_Fault_Update` → `ctrl_fault` → 使能门禁）。
- 核对：armcc 编译 can_bus / task_comm / dm / dji / task_actuation 五个文件，以并行任务结果为准，0 错 0 警；未链接、未上机。
- 备注：第 1 点 IMU ts 冻结洞按作者决定仅作排查提示、未改代码。

## 2026-09-25 · RC 离线阈值 50ms + 推理失败 last_action 修正

- 输入：作者决定：第 5 点 RC 离线 100→50 ms 实施；第 7 点失败 last_action 修正加入；第 4 点保持现状、第 6 点不做。
- 输出①：`imcalib/user-lib/dr16.h:8` `DR16_OFFLINE_MS` 100u→50u，`DR16_Online()`（`dr16.c:84`）与 `FAULT_RC` 判定随之提前。
- 输出②：`imcalib/task/task_policy.c` 推理失败分支补 `RL_Observation_Set_Last_Action(..., action_t)`（`RL_Policy_Run` 已清零 `action_t`，`rl_policy.c:147`），obs 19-24（`RL_OBS_LAST_ACTION`，`rl_observation.h:44`）与实际执行的零力矩一致。
- 调用链：comm 遥控链 / policy 观测链。
- 核对：armcc 编译 dr16 / rc_command / task_comm / task_actuation / task_policy 五个文件，以并行任务结果为准，0 错 0 警；未链接、未上机。
- 备注：`FAULT_ACTION`（`robot_control.h:93`）与 DM 使能/失能看门狗 100 ms 未变；接收链丢帧（第 6 点）按决定不做。

## 2026-09-25 · RL 轮目标速度加 ±20 rad/s 限幅

- 输入：作者授权：轮目标速度按 ±20 rad/s 限幅；其余待确认项按现状为准。
- 输出：`imcalib/Algorithm/rl_torque.c` 新增 `RL_TQ_WHEEL_VEL_MAX 20.0f`（`rl_torque.c:10`）；`vel_ref[VJ_L_WHEEL]` / `vel_ref[VJ_R_WHEEL]` 两处 `clampf(..., ±RL_TQ_WHEEL_VEL_MAX)`（`rl_torque.c:227-230`）。
- 调用链：`output_task_body → solve_rl → RL_Torque_Compute`。
- 核对：armcc 单文件编译 0 错 0 警（`-o` 临时目录，未污染 `build/`）；未链接、未上机。
- 备注：`|a_wheel|>2` 时目标饱和（act×10 折 ±20 rad/s）；`last_action` 仍记录未饱和动作，待训练侧确认。
- 另：作者同时确认其余待确认项（辅助 PID、卡尔曼 Q、标定历史）按现状为准。

## 2026-09-25 · RL 力矩失败语义修复 + ADC1/OCTOSPI2 死链移除

- 输入：作者 2026-09-25 授权两件事：①RL 力矩计算失败要「标注错误并发零力矩」；②删除未使用的 ADC1 与 OCTOSPI2（外置闪存）死链，BMI088 与 WS2812 保留。
- 输出①：`imcalib/task/task_actuation.c` 的 `solve_rl()`（148-156 行）接住 `RL_Torque_Compute` 返回值，失败（0u）→ `torque->valid = 0u` 并 return，由 `output_dispatch()`（`task_actuation.c:43`）走零力矩路径（`Dm_Send_Zero` + `Dji_All_Stop`）；成功才 `valid = 1u`。
- 输出②：删除 `Core/Src/adc.c`、`Core/Inc/adc.h`、`Core/Src/octospi.c`、`Core/Inc/octospi.h`；引用清理 `Core/Src/main.c`（去 `#include "adc.h"/"octospi.h"` 与 `MX_ADC1_Init()/MX_OCTOSPI2_Init()` 调用）、`Core/Src/dma.c`（去 DMA1_Stream0 NVIC 配置块）、`Core/Src/stm32h7xx_it.c`（去 `extern hdma_adc1` 与 `DMA1_Stream0_IRQHandler`）、`Core/Inc/stm32h7xx_it.h`（去对应声明）；`MDK-ARM/CtrBoard-H7_ALL.uvprojx`、`MDK-ARM/CtrBoard-H7_ALL.uvoptx` 移除对应文件条目。保留：BMI088、WS2812、HAL 库条目（`stm32h7xx_hal_adc.c` / `stm32h7xx_hal_ospi.c` 仍在）。
- 调用链：`output_task_body → solve_rl → RL_Torque_Compute`；删除项原本零调用、无调用链。
- 核对结论：改动文件（task_actuation / main / dma / stm32h7xx_it）Keil AC5 armcc 编译 0 错 0 警（`-o` 指向临时目录，未污染 `build/`）；uvprojx/uvoptx 经 XML 解析合法；`Core/Src`、`imcalib` 全量 grep 无 `hadc1/hdma_adc1/hospi2/MX_ADC1_Init/MX_OCTOSPI2_Init` 残留。**未链接、未下载、未上机**。
- 备注：`.eide/eide.yml` 的 `Core/Src/adc.c`/`octospi.c` 条目**两次手工移除均被 VS Code/eIDE 进程回写**（16:37、16:46 各一次），当前 eide.yml 仍列这两条；处置：需在 eIDE 界面里移除这两个文件条目，或关闭 VS Code 后再改（同变更 84 情形）；CubeMX `.ioc` 未动，重新 generate 会把 adc/octospi 生成回来（彻底移除应在 CubeMX 里删）；`build/` 下 `compile_commands.json` 仍列旧文件，属构建产物、下次构建自然刷新。本批未触及任何极性/零点/轴向/量程等物理量数值。
- 另：作者同时确认 RL/LQR 若干「待作者确认」参数标注为有意设计（详见各 md 的 2026-09-25 标注）。

## 2026-09-25 · 文档同步（md 对齐当前代码，未改代码）

- 输入：作者授权"把项目 md 更新到与当前代码一致"；当前工作区代码（含未提交改动）为唯一事实来源。
- 输出：本次同步 6 份 md 的修正主题——`md/AGENTS.md`（协作规范、文件树、关键约束）、`md/RL_OVERVIEW.md`（RL 链路与待实测清单）、`md/LQR_PLAN.md`（LQR 计划、参数与遗留项）、`md/IO_CHAINS.md`（IMU/DM/DJI/遥控 I/O 链路）、`md/DBUS.md`（DR16 解析）、`md/UART_IDLE_DMA.md`（UART IDLE+DMA 接收框架）；描述修到与当前代码一致，历史段落保留并标注。
- 调用链：不涉及代码——纯文档同步，固件调用链与配置未动。
- 核对结论：未改任何 `.c/.h`；物理量（极性/零点/轴向/量程/符号项）数值一律未改、不新增，与代码不一致处只加"以代码为准、待台架复核"标注；可调参数以代码为准。本账本历史条目未改。

## 2026-09-24 · 测试代码清理

- 输入：当前仲裁只支持左中 LQR、左上 RL、左下失能；Sysid、手动腿测、仅偏航测试均无当前入口。
- 输出：删除 `imcalib/Sysid/`、CAN 发送时间戳测试结构、手动腿测 PID 与目标、仅偏航测试函数、Leg2 腿长对照函数、力映射自检字段和速度反号对照字段；同步工程源目录与 VOFA 注释。
- 调用链：`rc_command` → `task_actuation.c` → LQR 或 RL 正常求解 → `output_dispatch()`；`leg_solver.c` 仍计算力映射有效性，LQR 和 RL 的控制参数、机器表物理量未改。
- 核对：Keil AC5 编译现存 96 个 C 文件及启动汇编，各 0 错；未链接、未上机，物理表现仍待台架确认。

> 用途：把每次改动的 **输入 / 输出 / 调用链 / 核对结论** 记下来，方便回退与查错。
> 维护规则：一次改动 = 一节；先写链路，再写"已核对"与"待台架"。
> 配套：设计见 `md/sysid/sysid-lower-machine-plan.md`；I/O 总览见 `md/IO_CHAINS.md`。
> 最后更新：2026-09-25

---

## 变更 1 · `machine_config.h`（新增：机器配置表）

**输入**：宏 `MACHINE_CHUANLIANTUI`（0/1，可用 `-DMACHINE_CHUANLIANTUI=1` 覆盖）

**输出**（导出的宏）：

| 宏 | 机器① chuanliantui | 机器② 本机 |
| --- | --- | --- |
| `MACHINE_NAME` | `"chuanliantui"` | `"local-m2006-j4310"` |
| `DJI_WHEEL_MODEL` | 1 (DJI_M3508) | 0 (DJI_M2006) |
| `DJI_CURRENT_MAX_RAW` | 16384 | 10000 |
| `DJI_NM_PER_RAW` | 0.30×20/16384 | 0.18×10/10000 |
| `DJI_GEAR_RATIO` | 19.2 × 自制箱比（占位 1.0） | 36 |
| `DM_MIT_POS_MAX` / `VEL_MAX` / `TRQ_MAX` | 12.5 / 45 / 54 | 3.14159 / 30 / 10 |
| `SYSID_DM_TRQ_CLAMP` | 20 | 10 |

**调用链**：纯宏，无函数调用。引用方只有 `dji.c`、`dm.c`（全仓库 grep 确认）。

**核对结论**
- 旧宏 `DJI_NM_PER_RAW_M2006/M3508`、`DJI_CURRENT_MAX_M2006/M3508`、`DJI_GEAR_RATIO` 在代码里已无其它引用（文档引用已同步）。
- 机器② 的导出值与改前逐个一致 → 现机行为不变。
- 本文件不 include 任何模块头，避免循环依赖。
- 两个分支都编译通过：`-DMACHINE_CHUANLIANTUI=1` 下 `dji.c`、`dm.c` 均 rc=0。

**待办**：`CJL_WHEEL_BOX_RATIO` 仍是 1.0f 占位；腿几何（`lu/lg/offset_*`）还没进表，仍在 `robot_control.c:43-63`。

---

## 变更 2 · `dji.c` / `dji.h`（轮子刻度改从配置表取）

**输入**
- `Dji_Torque_To_Current(index, torque_nm)`：轮端约定 N·m
- `Dji_Send_Current(hfdcan, can_id, raw[4])`：裸 raw（供 sysid 直接发电流用）
- 反馈入口：FDCAN2 的 0x201 / 0x202 帧

**输出**
- `int16 raw` → 0x200 八字节帧（4 个 int16 大端）
- `Dji_Encoder_To_Rad()` / `Dji_Rpm_To_Rad_S()`：轮轴 rad / rad·s⁻¹
- `dji_motor_feedback[]`：`angle_raw / vel_raw / current_raw / angle_total / *_rad`

**调用链（下发）**
`task_actuation.c:output_task_body()` →（LQR 分支 `leg_balance.c` / 手动- RL 分支 `rl_torque.c`）→ `Dji_Send_Wheel_Torque(l, r)` → `Dji_Torque_To_Current()` → `Dji_Send_Current()` → `Can_Bus_Transmit()` → FDCAN2。

**调用链（接收）**
FDCAN2 RX 中断 → `HAL_FDCAN_RxFifo0Callback()`（`can_bus.c:114`）→ 路由表 → `Dji_Read()` → `raw_pending` →（commTask 1 kHz）`Dji_Parse()` → `dji_motor_feedback[]` → `Motor_State_Update()`（`task_comm.c:89`）→ `motor_state.dji` → RL 观测 / LQR 速度。

**核对结论**
- 机器② 的 6 个常量与改前逐项一致（型号 M2006、±10000、0.00018、36，RAD/RPM 换算公式未动）。
- `index` 参数现在只做边界检查；两轮同型号由配置表保证（混型不会再静默用错系数）。
- 新增编译期保护 `dji_wheel_model_check`：型号宏与 `dji_motor_type_t` 枚举错位就直接编译报错。
- `dji_motor_config[]` 里只改了 `.type` 的来源，`motor_id / feedback_id / control_id / feedback_sign` 未动。

**待台架**：`DJI_NM_PER_RAW` 沿用旧值，仍未实测。

---

## 变更 3 · `dm.c` / `dm.h`（MIT 量程 + 读参 + 启动自检）

**输入 / 输出**

| 单元 | 输入 | 输出 |
| --- | --- | --- |
| `Dm_Param_Read(index, rid, *value)` | 电机下标、寄存器号 | 0/1 成功标志 + 32 位寄存器值 |
| `Dm_Scale_Check()` | 无（自己发读帧） | 写全局 `dm_scale_check`（四台的 PMAX/VMAX/TMAX/CTRL_MODE + valid/match 标志） |
| `Dm_Read()` 拦截分支 | CAN 回帧 | 读参回帧 → `param_*`；正常反馈 → `raw_data`（与改前一致） |

**调用链（自检）**
`main.c` USER CODE 2 → `Dm_Scale_Check()` → 四台各 4 次 `Dm_Param_Read()` → `Can_Bus_Transmit(0x7FF, [CANID_L, CANID_H, 0x33, RID])` →（DM 电机）→ 回帧 → FDCAN1/3 RX 中断 → 路由（feedback_id）→ `Dm_Read()` 拦截（仅 `dm_param_wait=1` 时）→ `param_pending` + `param_raw_data` → `Dm_Param_Read()` 轮询取出并校验 → `dm_scale_check`。

**调用链（消费）**
`task_comm.c:Robot_Control_Send_Vofa()`（1 kHz / 5）→ 读 `dm_scale_check.all_valid / all_match` → VOFA ch3（bit2 / bit3）。

**核对结论**
- 拦截只在 `dm_param_wait = 1`（每次读帧发出→回帧/超时之间）生效；正常运行该标志为 0，`Dm_Parse()` 路径与改前完全一致。
- 回帧三重校验：`D[2]==0x33`、`D[3]==RID`、`D[0..1]==控制 ID`，避免错配。
- 读不到（无电机 / 总线异常）只是 `valid=0`，不阻塞启动、不影响控制。
- `Dm_Parse()`、`Dm_Mit_Control()`、`Dm_Send_Torque()`、`Dm_Send_Zero()` 逻辑未改，只换常量来源。
- `dm_motor_feedback_t` 增加 `param_pending` + `param_raw_data[8]`（每台 +9 字节）。

**已知取舍**
- 读参窗口内若恰好来一帧 `D[2]==0x33` 的正常反馈，会被当成回帧吃掉（仅启动期、电机未使能、1 帧）。
- 启动最多多花 4×4×10 ms = 160 ms（全读不到时）。

**待台架**：四台读回的 PMAX/VMAX/TMAX 是否等于固件常量（决定 ch3 的 bit3），以及 CTRL_MODE 是否为 1。

---

## 变更 4 · `main.c`（启动顺序 + TIM6 回调）

**启动顺序（USER CODE 2）**
`Mono_Ns_Init()` → `DR16_Init()` → `HI229_Init()` → `Can_Bus_Init()` → `Dji_Init()` → `Dm_Init()` → `Dm_Scale_Check()` → GPIO。

**TIM6 500 Hz 回调（`HAL_TIM_PeriodElapsedCallback`）**
`Mono_Ns_Tick()` → `osSemaphoreRelease(ctrl_tick_sem_handle)`。

**调用链**
`osKernelStart` 前：`Mono_Ns_Init`（开 DWT）→ 各外设 init → `Dm_Scale_Check`（需要 CAN 已启动，故必须排在 `Can_Bus_Init` 之后）。
调度器启动后：`actuationTask` → `output_task_init()` → `HAL_TIM_Base_Start_IT(&htim6)` → TIM6 ISR → `Mono_Ns_Tick()` + 释放信号量 → `output_task_body()`。

**核对结论**
- `Mono_Ns_Init()` 排在 `Can_Bus_Init()` 之前，保证第一帧 CAN 就有 ns 可用。
- `Dm_Scale_Check()` 排在 `Can_Bus_Init()` 之后，否则读帧发不出去。
- 自检用 `HAL_GetTick()` 做超时，HAL tick（TIM23）在 `HAL_Init()` 后即工作，不依赖调度器。

**待台架**：上电时序与自检耗时的实测。

---

## 变更 5 · `task_comm.c`（VOFA ch3 增加自检位）

**输入**：`dm_scale_check.all_valid`、`dm_scale_check.all_match`
**输出**：`dbg[3] = motor_enabled + base_action_locked×2 + all_valid×4 + all_match×8`
**调用链**：commTask（1 kHz）→ `Robot_Control_Send_Vofa()`（每 5 周期发一次）→ `Vofa_Send()` → UART8 DMA。

**核对结论**：低 2 位语义不变（`ch3 & 0x03` 的老用法仍有效）；新增位只在 sysid/自检语境下有含义。文档 `md/VOFA_SEND.md` 已同步。

---

## 变更 6 · `mono_ns.c/h`（新增：单调 ns 时钟）

**输入**：`DWT->CYCCNT`（CPU 周期，1.82 ns/LSB，550 MHz）
**输出**：`uint64_t Mono_Ns_Get()`（上电起算的单调 ns）

**调用链**
- `Mono_Ns_Init()`：`main.c` USER CODE 2 第一行 → 开 `TRCENA` → `DWT->LAR=0xC5ACCE55`（M7 解锁）→ 清 CYCCNT → 开 `CYCCNTENA`
- `Mono_Ns_Tick()`：TIM6 500 Hz ISR → 把 32 位 CYCCNT 的增量累加到 64 位基数
- `Mono_Ns_Get()`：目前无人调用；步 3 起由 CAN RX 中断、CAN TX 完成回调、sysid 帧组装调用

**核对结论**
- Tick 与 Get 都在 PRIMASK 临界区内完成"基数 + 增量"的一次性快照 → 不会出现撕裂值，也不会重复计数。
- CYCCNT 32 位回绕周期 = 2³² / 550 MHz ≈ 7.81 s；Tick 由 500 Hz 调用，远小于回绕周期。
- `cyc × 1000` 在 64 位下约 9 小时内不会溢出（1.8e19 / 1000 / 550e6 ≈ 9 h）。
- `DWT->LAR` 与 `CoreDebug_DEMCR_TRCENA_Msk` 在本工程 CMSIS（`core_cm7.h:1145`、`:1693`）中存在，编译通过。
- 工程登记：Keil 组 `imcalib/user-lib` 已加入 `mono_ns.c`；eIDE 走 `srcDirs` 自动扫描，无需改 yml。

**待台架**：跑 10 s 与 `HAL_GetTick()` 对比（偏差应 < 2 ms）；连续两个节拍差值应 ≈ 2 ms。

---

## 变更 7 · 切换机器到 chuanliantui（只改一个宏）

**输入**：`MACHINE_CHUANLIANTUI` 由 0 改为 1（`machine_config.h`）
**输出（现在生效的常量）**：轮 = M3508 / ±16384 raw / 0.000366 N·m·raw⁻¹ / 总传动比 19.2；腿 = PMAX 12.5、VMAX 45、TMAX 54；激励钳位 20 N·m
**调用链**：没有新增调用，只是在编译期替换 `dji.c`、`dm.c` 里的常量；功能链路与变更 2/3 相同。

**核对结论**
- 默认（=①）与 `-DMACHINE_CHUANLIANTUI=0`（=②）两边全量编译都通过：各 99 文件 0 fail / 0 warn。
- 反馈帧格式两边一样（M2006/C610 与 M3508/C620 都是 8192 线转子侧 + 同样 8 字节布局），所以 `Dji_Parse()` 不用改。

**这次切换带来的可见差异**

| 量 | 旧配置（本机） | 新配置（chuanliantui） |
| --- | --- | --- |
| 轮速/轮角换算 | `0.10472/36` | `0.10472/19.2`（若自制箱比 ≠1，读数按比例偏） |
| 同一 N·m 命令对应的电流 | ÷0.00018 | ÷0.000366（同值下电流更小） |
| DM 力矩量化满量程 | ±10 N·m | ±54 N·m（同 N·m 下 raw 更小） |
| DM 位置满量程 | ±π | ±12.5（腿长/腿倾角读数整体变大约 4 倍） |

**仍是占位 / 未确认的**
1. `CJL_WHEEL_BOX_RATIO = 1.0f`（自制减速箱比），只影响轮端速度/角度的读数与 RL 观测。
2. 腿几何仍是旧机数值（`robot_control.c:43-63`），影响腿长/腿倾角与力矩在两条髋上的分配。
3. DM 的总线与极性沿用旧接线假设（FDCAN1 左腿 / FDCAN3 右腿、ID 1~4、左 +1 右 −1）。

**首次上电要看的四件事**
1. VOFA ch3 是否含 **+4**（四台都读到参数且 MIT 模式）与 **+8**（三项满量程与固件一致）；不含 +8 就先在上位机核对 PMAX/VMAX/TMAX。
2. 腿的伸/收方向是否与动作一致（方向反了就是 `dm.c` 表里的 `feedback_sign` 问题）。
3. 轮子转向与速度读数是否合理。
4. 是否报 `FAULT_MOTOR`（10 ms 无反馈即判离线）。

---

## 变更 8 · 测试模式入口（左中 + 右中）

**输入**：遥控 `s1`、`s2`（左/右拨杆）。判定条件 `remote.s1 == DR16_SW_MID && remote.s2 == DR16_SW_MID`
**输出**：`ctrl_strategy = CTRL_STRATEGY_SYSID`（枚举值 2）→ VOFA ch30 显示 2；测试模式内下发零力矩
**调用链**
- 输入：`commTask`（1 kHz）→ `Remote_Control_Update()` → `DR16_Process()` 解析 `dr16.s2`（`dr16.c:34`）
- 判定：`actuationTask`（500 Hz）→ `output_task_body()` → `DR16_Snapshot()` → 三分支策略判定
- 输出：`output_task_sysid()` → `Dm_Send_Zero()` + `Dji_All_Stop()` → CAN
- 观测：`commTask` → `dbg[30] = (float)ctrl_strategy`

**核对结论**
- 全量编译 99 文件 0 fail / 0 warn。
- `s2` 非中位时行为与改前逐字一致（LQR 仍走中位分支、手动/RL 走上位分支）。
- 安全链未被绕过：`motor_enabled` 仍由「s1 非下位 + 无故障 + 未翻倒」决定；s1 下位立刻失能。
- 进测试模式时强制 `lqr_running = 0`，保证从测试模式切回 LQR 时会重新按当前姿态锁存。

**现在进去只置零（空壳）**：激励序列与数据采集在步 3~9 接入；当前作用是把入口/旁路/观测这条链打通，便于先上机验证拨杆读数。

**待台架**：右拨杆 `s2` 是否真能读到（`md/IO_CHAINS.md` 曾标"未接线"，指的是"未分配功能"）；进测试模式后 ch30 应显示 2。

---

## 变更 9 · 电机配置拆成两份表 + 测试代码独立成 `imcalib/Sysid/`

**目的**：两台机器随时切换；测试代码与主干分开，关掉开关即回到原样。

**输入 / 输出**

| 单元 | 输入 | 输出 |
| --- | --- | --- |
| `machine_table[]` / `machine` | `Machine_Select(id)` | 当前机器的轮刻度/减速比、DM 三个满量程、激励钳位 |
| `Sysid_Mode_Run()` | 无 | 测试模式下的电机输出（当前 = 零力矩） |
| `Sysid_Dm_Check()` | 无 | `sysid_dm_check`（四台满量程 + MIT 模式 + valid/match） |
| `SYSID_ENABLE` | 编译期宏 | `0` = 测试代码不被调用；`1` = 打开 |

**调用链**
- `main.c` USER CODE 2 → `Machine_Select(MACHINE_ID_CHUANLIANTUI)` → `machine` 指针
- `dji.c`（`Dji_Encoder_To_Rad` / `Dji_Rpm_To_Rad_S` / `Dji_Torque_To_Current`）、`dm.c`（`Dm_Parse` / `Dm_Send_Torque` / `Dm_Send_Zero`）→ 读 `machine->…`
- `task_actuation.c` →（`#if SYSID_ENABLE`）→ `Sysid_Mode_Run()`
- `main.c` →（`#if SYSID_ENABLE`）→ `Sysid_Dm_Check()`；`task_comm.c` →（`#if`）ch3 显示位

**核对结论**
- armcc 全量：`SYSID_ENABLE=1` 与 `=0` 都是 **103 文件 0 fail / 0 warn**。
- 两份表都编进固件，切换只改 `main.c` 一行 `Machine_Select(...)`（将来可接运行时输入）。
- 关闭开关时：策略仲裁回到原来的两分支；VOFA ch3 与改动前逐位一致；不跑自检。
- 登记：Keil `machine_config.c` 进 `imcalib/user-lib` 组 + 新组 `imcalib/Sysid`；IncludePath 加 `../imcalib/Sysid`；eIDE 的 `srcDirs` / `includeList` 同步。

**待台架**：上电后 ch3 的 +4/+8；`Machine_Select` 换表后读数是否符合预期。

---

## 变更 10 · 按作者意见回退：去掉 DM 开机自检；dji 恢复"型号参数在驱动、机器表只定型号 + 减速比"

**回退了变更 3 与变更 5**（变更 3 里"MIT 量程改从配置表取"的部分保留）。

**改动**
- 删除 `imcalib/Sysid/sysid_dm_check.c/h`：连带去掉 `Dm_Param_Read()`、`Dm_Read()` 的读参拦截、`dm_param_wait`、反馈结构体的 `param_*` 字段、`dm.h` 的 RID/超时/容差宏
- `main.c` 去掉自检调用；`task_comm.c` 的 VOFA ch3 恢复原样（不再有 +4/+8）
- `dji.h` 恢复 `DJI_CURRENT_MAX_M2006/M3508`、`DJI_NM_PER_RAW_M2006/M3508`（**电机固有参数**）
- `machine_config.c` 的轮字段改为 `dji_type`(0=M2006, 1=M3508) + `dji_gear_ratio`
- `dji.c` 的 `Dji_Torque_To_Current()` 按 `machine->dji_type` 选型号常数（恢复"按型号取参数"的结构）

**理由**
- DM 满量程用达妙上位机人工核对一次即可，不值得常驻一套读参逻辑
- M2006 与 M3508 协议/驱动完全相同，只差参数（力矩常数、满量程、减速比），驱动结构不必大改

**核对**：见附录 A 的全量编译（`SYSID_ENABLE=1/0`）。

**遗留**：DM 满量程核对方式改为"上位机人工核对"（计划 §3.2 / §8.2 已同步）。

---

## 变更 11 · 换机器收敛成一行；去掉 DM 的 `type` 标签；力矩限幅改满限幅

**① 换机器只改一行**
- `machine_config.h` 新增 `#define MACHINE_DEFAULT MACHINE_ID_LOCAL`（**换机器改这一行**）
- `machine_config.c`：表指针初始化改为 `&machine_table[MACHINE_DEFAULT]`
- `main.c`：`Machine_Select(MACHINE_DEFAULT);`（不再是硬编码的机器号）
- 说明：`machine_config.c` 里那行 `const machine_cfg_t *machine = ...` 只在"没人调用 `Machine_Select` 时"生效；实际生效点是这个宏。以后要接运行时切换，调用 `Machine_Select(id)` 即可。

**② 去掉 `dm_motor_config_t.type` 与 `dm_motor_type_t`**
- 原来 `.type = DM_MOTOR_J4310` 只被 `Dm_Init()` 用来做一次边界检查，不参与协议/刻度；换电机后它会变成过期标签
- 现在：字段与枚举删除，`Dm_Init()` 的检查里去掉了 type 一项；DM 的实际刻度来自 `machine->dm_pos_max / dm_vel_max / dm_trq_max`

**③ 力矩限幅改"满限幅"（按机器取）**
- `machine_config.h` 的 `sysid_trq_clamp` 更名为 `dm_trq_clamp`，并新增 `dji_trq_clamp`（满限幅 = 型号常数 × 满量程电流）
- 值：大机器 腿 20 N·m / 轮 6.0 N·m；小机器 腿 10 N·m / 轮 1.8 N·m
- `rl_torque.c` 删掉 `RL_TQ_LEG_LIMIT/WHEEL_LIMIT/JUMP_WHEEL_LIMIT` 三个硬编码 5/5/4 N·m，改为 `machine->dm_trq_clamp` / `machine->dji_trq_clamp`（Jump 模式不再单独收窄）
- 末端仍有硬件级钳位：`dm.c` 按 MIT 满量程、`dji.c` 按 raw 满量程

**核对**：全量编译（`SYSID_ENABLE=1/0`）见附录 A。

---

## 变更 12 · DM/DJI 配置表统一（共用 `motor_cfg_t`）+ 输出极性集中

**改动**
- `can_bus.h` 新增共用结构 `motor_cfg_t`：`handle / feedback_id / control_id / feedback_sign / output_sign`
- `dm.h` → `typedef motor_cfg_t dm_motor_config_t;`；`dji.h` → `typedef motor_cfg_t dji_motor_config_t;`（两个原结构体删除）
- **dji 表删掉 `motor_id`**（原先只被 `Dji_Init()` 用来做一次边界检查）
- 两张表都补 `output_sign`（取值与 `feedback_sign` 相同：左 +1 / 右 -1）
- 极性只剩两个统一入口：
  - **反馈**：`Dm_Parse()` / `Dji_Parse()` 用 `feedback_sign`
  - **输出**：`Dm_Send_Torque()`（`output_sign` 为负则取反）、`Dji_Send_Wheel_Torque()`（`torque × output_sign`）
- 调用方的零散负号删除：`rl_torque.c` 右轮 `-tau_v[VJ_R_WHEEL]`、`leg_balance.c` 右轮 `-u[LQR_U_WR]`
  （净行为不变：原来在调用方取反，现在在驱动边界按 `output_sign` 取反一次）

**⚠️ 本轮修掉一个我上一轮引入的 bug**
- 编辑 `dji.c` 时误删了右轮的 `.feedback_id = 0x202u` → 右轮反馈永远匹配不上 → 判离线 → `FAULT_MOTOR` → 整车失能。已恢复。

**核对**：全量编译 102 文件 0 fail / 0 warn；配置结构里已无 `motor_id`（反馈结构体里的 `motor_id` 是 DM 回帧字段，保留）。

---

## 变更 13 · 极性搬进机器配置表（`dm_sign` / `dji_sign`）

**动机**：极性随**安装**变（装机/换电机/换机器都可能翻），放在驱动表里等于"换机器要改两处"；放机器表里就跟机器走。

**改动**
- `machine_config.h` 新增 `motor_sign_t { int8_t fb; int8_t out; }`，机器结构体加：
  - `motor_sign_t dm_sign[MACHINE_LEG_NUM]`（前左/后左/前右/后右）
  - `motor_sign_t dji_sign[MACHINE_WHEEL_NUM]`（左/右）
  - 常量 `MACHINE_LEG_NUM = 4` / `MACHINE_WHEEL_NUM = 2`
- `machine_config.c` 两台机器都填上（当前都是左 +1、右 -1）
- `motor_cfg_t`（`can_bus.h`）**去掉两个 sign 字段** → 只剩 `handle / feedback_id / control_id`
- 驱动表（`dm.c` / `dji.c`）删掉所有 `.feedback_sign` / `.output_sign` 行
- 使用点收敛成 4 处，全部读 `machine->…`：
  - 反馈：`Dm_Parse()` → `machine->dm_sign[i].fb`；`Dji_Parse()` → `machine->dji_sign[i].fb`
  - 输出：`Dm_Send_Torque()` → `machine->dm_sign[i].out`；`Dji_Send_Wheel_Torque()` → `machine->dji_sign[…].out`
- 两个驱动加编译期检查：`MACHINE_LEG_NUM == DM_MOTOR_NUM`、`MACHINE_WHEEL_NUM == DJI_MOTOR_NUM`

**顺带修掉一个 warning**：搬完后 `Dm_Parse()` 里的 `config` 变量没人用了（→ 删除声明）。

**核对**：全量编译 102 文件 0 fail / 0 warn。

**换机器要改的（现在全部集中）**：`machine_config.h` 的 `MACHINE_DEFAULT` + 对应机器在 `machine_config.c` 里的参数与极性。

---

## 变更 14 · 接上"总输出开关" + VOFA 串口改成可切换宏

**① 总输出开关（原来只是死代码）**
- 现状：`torque_output_enabled` 在 `robot_control.c` 里被声明并赋值，但**全工程没人读**（文档写了、实现没有）
- 改法（第一版曾放在 `output_task_body()` 开头早退，会冻住所有目标/误差/LQR 通道 → 已按作者意见改掉）：
  - `task_actuation.c` 新增 `output_send(const torque_output_t *)`，作为**唯一的下发点**
  - 开关为 0 时 → 只发 `Dm_Send_Zero()` + `Dji_All_Stop()`；为 1 时 → 正常 `Dm_Send_Torque()` + `Dji_Send_Wheel_Torque()`
  - LQR 链路与手动/RL 链路都改成调用 `output_send(&torque)`
  - **控制链路本身照常运行**：策略仲裁、LQR 状态估计+控制、RL 力矩计算、目标的误差量都还在算 → VOFA 的 `dbg[5]/[9]/[13]/[17]`（目标）、`[7]/[11]/[15]/[19]`（虚拟力矩）、`[30]`（策略）、`[31..47]`（LQR）都会正常刷新
- 当前值：`robot_control.c:39` = **0**（按作者要求关闭所有输出，供关节回馈测试）
- 注意两点：
  - 零力矩帧**必须继续发**，因为 DM 电机只在收到帧时才回状态帧；停了就收不到回馈
  - 开关为 0 时 `dbg[20..23]` 显示的是"算出来的力矩"而不是"发出去的力矩"（实际发出去的是 0）；`output_debug_dm_sent` 标志为 0 才代表真的没发力矩
- 恢复：把该行改回 `1u`

**② VOFA 串口选择宏**
- `Vofa_send.h` 由裸 `#define VOFA_UART &huart8` 改为 `VOFA_PORT` 数字选择（8=UART8 默认 / 1=USART1），`VOFA_UART` 由其推导
- 支持构建系统覆盖：`-DVOFA_PORT=1`（Keil 的 Define / eIDE 的预定义宏），不必改文件
- 填其它数字 **编译期报错**（`#error`），避免选到被占用/无 TX DMA 的口
- 已确认各口：UART8（921600 + DMA1_Stream7 NORMAL ✓）、USART1（921600 + DMA1_Stream6 NORMAL ✓、无模块占用）、UART7（HI229 占用 + TX DMA CIRCULAR ✗）、UART9（DR16 占用 + 无 TX DMA ✗）

**核对**：`VOFA_PORT=8`、`VOFA_PORT=1` 各 103 文件 0 fail / 0 warn；`VOFA_PORT=7` 如期报 `#error`。

---

## 变更 15 · CAN 总线分配也进机器配置表；IMU 新包核对结论

**① 总线分配进配置表**
- 接线差异：大机器 = 四个 DM 全在 **FDCAN1**、两个 3508 在 **FDCAN3**；小机器 = DM 左腿 FDCAN1 / 右腿 FDCAN3、轮 **FDCAN2**
- `machine_config.h`：机器结构体加 `uint8_t dm_bus[MACHINE_LEG_NUM]`（1/2/3 = FDCANx）与 `uint8_t dji_bus`
- `machine_config.c`：chuanliantui `{1,1,1,1}` + `3`；LOCAL `{1,1,3,3}` + `2`
- `can_bus.h/.c`：新增 `Can_Bus_Handle(bus)`（复用原有句柄表）
- `motor_cfg_t` 去掉 `handle` 字段 → 只剩 `feedback_id / control_id`（总线是机器级）
- `dm.c` / `dji.c`：注册与发送统一用 `Can_Bus_Handle(machine->dm_bus[i])` / `Can_Bus_Handle(machine->dji_bus)`

**核对**
- 全量编译 103 文件 0 fail / 0 warn；驱动里 `->handle` 已无残留
- 逐台脚本比对：LOCAL 腿 `[1,1,3,3]` / 轮 `2` 与 HEAD **完全一致** → 小机器行为不变 ✓
- CHUANLIANTUI：腿全 `1`、轮 `3` ✓

**② IMU 大机器新包：不需要改代码**
- 旧包 `tag(1) id(1) rev[2] prs(4) ts(4) acc gyr mag eul quat` = 76 字节
- 新包 `tag(1) status(2) tmp(1) prs(4) ts(4) acc gyr mag eul quat` = **同样 76 字节**
- 两者只有 **offset 1-3** 的含义不同（`id+rev` ↔ `status+tmp`），而固件只读 `ts/acc/gyr/eul/quat`（offset 4 之后）→ **偏移完全一致**
- 结论：只要帧头（0x5A 0xA5）、长度字段（76）、波特率（921600）不变，解析无需改动
- 若之后要显示新包的 `status` / `tmp`，再加两个字段即可（暂缓）

---

## 变更 16 · DM 反馈 ID 兼容两种 Master ID

**背景**：换大机器后 ch0=193（只有 IMU + 两轮在线），四台腿全离线；总线上没有匹配到任何反馈帧。
原因：DM 回帧 ID = 电机里的 **Master ID + CAN ID**，表里假设的是 `0x10 + ID`（→ 0x11..0x14）；另一批电机的 Master ID 常为 **0**（→ 回帧就是 0x01..0x04）。

**改动**：`dm.c` 的 `Dm_Init()` 对每台电机**注册两个反馈 ID**：`feedback_id`（0x11..0x14）与 `control_id`（0x01..0x04），两者都路由到同一 ctx。

**核对**：编译 103 文件 0 fail / 0 warn；路由数仍在 `CAN_BUS_ROUTE_MAX = 8` 内（大机器 FDCAN1 恰好 8、FDCAN3 2；小机器 4/4/2）。

**若仍离线**：说明电机 CAN ID 不是 0x01..0x04（我们发帧没人应答）或未上电/终端电阻缺失 → 用达妙上位机读每台的 CAN ID / Master ID。

---

## 变更 17 · DR16 接收放宽 + 加三个诊断通道

**怀疑**：`DR16_Process()` 原来是 `if (len == DR16_FRAME_LEN)`（严格 18 字节）才解析；若 IDLE 分包导致 19 字节或丢 1 字节，会**整帧丢弃** → 现象正是"接收机通信正常但主控认不到"。

**改动**
- `dr16.c`：解析条件放宽为 `len >= DR16_FRAME_LEN`（尾部带下一帧字节也能解析）；帧长不足仍丢弃
- `dr16.c/.h`：新增三个诊断量 `dr16_idle_cnt`（IDLE 事件数）、`dr16_last_len`（最近一帧字节数）、`dr16_ok_cnt`（解析成功帧数）
- `task_comm.c`：把 **ch45/46/47**（原 LQR 左腿三个量）临时改为这三个诊断量；`md/VOFA_SEND.md` 已标注，调试完恢复

**判定表**

| ch45 (idle) | ch46 (len) | ch47 (ok) | 结论 |
| --- | --- | --- | --- |
| 不涨 | 0 | 0 | UART9 一个字节都没收到 → 数据线（**PD14**）/接收机模式（必须 DBUS，SBUS 收不到） |
| 涨 | 恒 18 | 涨 | 一切正常（ch0 的 bit1 应亮） |
| 涨 | 乱跳 | 不涨 | 分包/丢字节 → 本轮放宽后应恢复 |

**核对**：编译 103 文件 0 fail / 0 warn。

---

## 变更 18 · VOFA ch45~47 改为 FDCAN1 接收诊断

**改动**
- `can_bus.h/.c`：`can_bus_t` 增加 `last_rx_id`（RX 中断里记录）；新增 `Can_Bus_Rx_Count(bus)` / `Can_Bus_Last_Rx_Id(bus)`
- `task_comm.c`：**替换**（不新增）ch45/46/47：
  - ch45 = **FDCAN1 收帧计数**（累计）
  - ch46 = **FDCAN1 最近一帧 CAN ID**
  - ch47 = DR16 解析成功帧数（保留遥控诊断）
- `md/VOFA_SEND.md` 同步标注

**怎么读**
| 现象 | 结论 |
| --- | --- |
| ch45 不涨、ch46 = 0 | FDCAN1 上一帧都收不到（经典配置下收到 FD 帧会被判格式错误，**这是现在 1 Mbps 配置的预期现象**） |
| 改成 FD(1M/4M) 后 ch45 开始涨、ch46 在 17/18/19/20（0x11~0x14）跳 | **腿回馈通了** → ch0 四台腿应同时亮 |
| ch45 涨但 ch46 是别的值 | 电机回帧 ID 不是 0x11~0x14 → 把该值告诉我，路由按它改 |

**核对**：编译 103 文件 0 fail / 0 warn。

---

## 变更 19 · ch42~47 改为六台电机反馈观测

**改动**（`task_comm.c`，只替换、不新增通道）
| 通道 | 内容 |
| --- | --- |
| ch42 | DM 左前髋位置 `dm.pos_rad[F_LFT]` |
| ch43 | DM 左后髋位置 |
| ch44 | DM 右前髋位置 |
| ch45 | DM 右后髋位置 |
| ch46 | 左轮位置 `dji.angle_total_rad[LFT]`（多圈累计） |
| ch47 | 右轮位置 `dji.angle_total_rad[RGT]`（多圈累计） |

**怎么读**：掰腿 → ch42~45 变化；转轮 → ch46/47 变化。**值在动 = 该电机通信正常**；一直不动 = 没通。
六路都是位置（速度快看不出来）：DM 是逻辑坐标（右侧已取反）；轮是多圈累计角（转起来持续变化，不绕回）。

**核对**：编译 103 文件 0 fail / 0 warn；`md/VOFA_SEND.md` 同步标注。

---

## 变更 20 · 腿几何（杆长 + 零位偏置）搬进机器配置表

**改动**
- `machine_config.h`：`machine_cfg_t` 增加
  | 字段 | 含义 |
  | --- | --- |
  | `leg_lu` / `leg_lg` | 大腿杆长 / 小腿杆长（m） |
  | `leg_off_f[2]` | 前髋零位偏置（左/右，rad） |
  | `leg_off_b[2]` | 后髋零位偏置（左/右，rad） |
  | `leg_off_phi0[2]` | 虚拟腿摆角零位偏置（左/右，rad）（原文误写"虚拟小腿"，2026-09-21 更正：它只加在 `virtual_leg_angle` 上） |
- `machine_config.c`：两份机器表各填一组；大机器 `lu=0.21 / lg=0.25`，零点**照抄参考固件表** `{0.476998, -1.974491} / {1.974491, -0.476998}`（前左/前右 / 后左/后右）。
- `robot_control.c`：五连杆几何与偏置从 `machine->leg_*` 读取（新增 `#include "machine_config.h"`），不再硬编码。

**输入 / 输出 / 调用链**
- 输入：`Machine_Select()` 选中的机器表
- 输出：`leg_l/leg_r.config`（`lu/lg/offset_f/offset_b/offset_phi0`）
- 调用链：`main.c:134` → `Machine_Select` → `freertos.c:117` → `Robot_Control_Init()` → `leg_*.config.*` → `task_comm.c` `Leg_State_Update()` 组 `input.hip_*` → `Leg_Solve()` → `output.*` → `rl_torque.c` / `lqr_balance.c` / `leg_balance.c` / `task_policy.c`

**怎么读**：`leg.output.valid` 为 0 说明几何/偏置不成立（腿长落到 0.136~0.46 m 之外或开方为负）。

**核对**：编译 103 文件 0 fail / 0 warn；换机器只改 `MACHINE_DEFAULT` 一处。

**待台架**：零点照抄参考固件（`实际值 = raw × 极性 + 零点`），**未实测**。注意 `task_comm.c:110` 前髋还额外 `+ LEG_PI`（原有代码，未动），所以前髋**实际零点 = 图值 + π**（左前 = 3.6186 rad）。腿长/倾角离谱 → 按卷尺 + 角度计重标。`leg_off_phi0` 两台都还是占位值。

⚠️ 本节描述的 `leg_off_f`/`leg_off_b` 字段已被变更 22 移到 `dm_zero`，以变更 22 为准。

---

## 变更 21 · VOFA ch31~47 改为腿部解算 + 电机观测（临时占用 LQR 通道）

**改动**（`task_comm.c`，只替换、不新增通道）
| 通道 | 内容 |
| --- | --- |
| ch31 / ch39 | 左 / 右 腿长 `virtual_leg_length`（m） |
| ch32 / ch40 | 左 / 右 腿摆倾角 `virtual_leg_angle`（rad） |
| ch33 / ch41 | 左 / 右 大腿角 `thigh_angle`（rad） |
| ch34 / ch42 | 左 / 右 虚拟小腿角 `virtual_shank_angle`（rad） |
| ch35~38 | 左前髋位置 / 左后髋位置 / 左前髋速度 / 左后髋速度 |
| ch43~46 | 右前髋位置 / 右后髋位置 / 右前髋速度 / 右后髋速度 |
| ch47 | 解算有效掩码（1=左, 2=右, 3=两侧） |

**输入 / 输出 / 调用链**
- 输入：`leg_solver` 输出的 `leg_l/leg_r.output`，`motor_state.dm.pos_rad/vel_rad_s`
- 输出：`dbg[48]` → `Vofa_Send(dbg, 48u)` → 当前 `VOFA_PORT` 串口
- 调用链：`comm_task_body()` → `Motor_State_Update()` → `Leg_Debug_Send()` → `Vofa_Send()`

**怎么读**
| 现象 | 结论 |
| --- | --- |
| ch47 ≠ 3 | 有一侧解算无效，后面数值都不用信 |
| ch31/ch39 竖直时 ≠ 卷尺量的轴心-足端距离 | 杆长或偏置不对 |
| 左右摆成对称姿态时 ch31 与 ch39 反向 | 右腿前后电机表项顺序或 `dm_sign` 反了 |
| ch35~38/ch43~46 值不动 | 该侧电机没通 |

**核对**：编译 103 文件 0 fail / 0 warn；`md/VOFA_SEND.md` 已同步（含原始 LQR 布局的恢复表）。

⚠️ 本节描述的 48 通道布局已被变更 24 取代，以变更 24 为准。

---

## 变更 22 · 电机零点搬到 dm.c 解码层（原始值与零点值分开）

**动机**：原来电机零点（`offset_f`/`offset_b`）在任务层 `Leg_State_Update()` 里叠加，导致 VOFA 上看到的 `pos_rad` 是"裸解码角"而不是"实际被消费的关节角"，调试时容易误判。搬到 `dm.c` 解码层后，零点后值 `pos_zero_rad` 是**唯一被消费的电机位置**（进腿部解算、进 PID、进 RL 观测），原始解码角仍然保留在 `dm_motor_feedback[].pos_rad` / `motor_state.dm.pos_rad[]`（调试器 Watch 可见），两个值互不覆盖。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/user-lib/machine_config.h` | `machine_cfg_t` 删除 `leg_off_f[2]` / `leg_off_b[2]`，新增 `float dm_zero[MACHINE_LEG_NUM]`（4 台腿电机零点，顺序前左/后左/前右/后右） |
| `imcalib/user-lib/machine_config.c` | 两份机器表填 `.dm_zero`；大机器见 `MACHINE_ID_CHUANLIANTUI` 表项，小机器见 `MACHINE_ID_LOCAL` 表项（数值与原 `leg_off_f/leg_off_b` 逐项一致） |
| `imcalib/user-lib/dm.h` | `dm_motor_feedback_t` 新增 `float pos_zero_rad`（`pos_rad` 注释改为"解码角"，`pos_zero_rad` 注释"加零点"） |
| `imcalib/user-lib/dm.c` | `Dm_Parse()` 解码后新增 `feedback->pos_zero_rad = feedback->pos_rad + machine->dm_zero[i]` |
| `imcalib/task/inc/robot_control.h` | `dm_motor_state_t` 新增 `float pos_zero_rad[DM_MOTOR_NUM]` |
| `imcalib/task/task_comm.c` | `Motor_State_Update()` 同时拷贝 `pos_rad` 和 `pos_zero_rad`；`Leg_State_Update()` 改用 `dm.pos_zero_rad[]` 组 `input.hip_f/hip_b`（前髋 `+ LEG_PI` 保留，任务层不再加任何 off 项）；VOFA ch35/36/43/44 改为 `dm.pos_zero_rad[]` |
| `imcalib/Algorithm/leg_solver.h` | `leg_config_t` 删除 `offset_f` / `offset_b`（`offset_phi0` 保留，仍被求解器使用） |
| `imcalib/task/robot_control.c` | `Robot_Control_Init()` 不再写 `leg_*.config.offset_f/offset_b` |

**输入 / 输出 / 调用链**

```
machine_config.c 的 .dm_zero
    │
    │  Dm_Parse() @ imcalib/user-lib/dm.c
    ▼
dm_motor_feedback[].pos_zero_rad  (= pos_rad + dm_zero[i])
    │
    │  Motor_State_Update() @ task_comm.c
    ▼
motor_state.dm.pos_zero_rad[]
    │
    │  Leg_State_Update() @ task_comm.c
    ▼
leg_l/leg_r.input.hip_f = pos_zero_rad[F] + LEG_PI   (前髋额外 +π)
leg_l/leg_r.input.hip_b = pos_zero_rad[B]
    │
    │  Leg_Solve()
    ▼
消费方:
  rl_torque.c    — 位置环 PID 当前值
  task_policy.c  — RL 观测 obs.joint_pos
  lqr_balance.c  — LQR 状态
  leg_balance.c  — 腿部力控
```

**怎么读**
- VOFA ch35/36/43/44 是零点后值，就是进解算和 PID 实际消费的那个值。
- 原始解码角没有上 VOFA，需要时用调试器看 `motor_state.dm.pos_rad[]`。

**数值等价性**：这次是纯搬家，**任何数值都没有变化**。旧代码 `hip_f = pos_rad + π + offset_f`、`hip_b = pos_rad + offset_b`；新代码 `pos_zero_rad = pos_rad + dm_zero[i]`，然后 `hip_f = pos_zero_rad + π`、`hip_b = pos_zero_rad`，两边相加结果完全一致。

**核对**：编译 103 文件 0 fail / 0 warn；`grep` 确认全工程 `offset_f` / `offset_b` / `leg_off_f` / `leg_off_b` 已无残留。

**待台架**：大机器 4 个零点是从参考固件表照抄的，未经实测；前髋因为代码里原有 `+π`，其等效零点是"`.dm_zero` 值 + π"。

---

## 变更 23 · 大机器 MIT 位置满量程 12.5 → ±π（作者读上位机确认）

**改动**（`machine_config.c` 大机器段，只改一行）
- `.dm_pos_max`：`12.5f` → `3.14159f`
- `.dm_vel_max = 45.0f` / `.dm_trq_max = 54.0f` **保持不变**（作者在上位机核对：速度 ±45、力矩 ±54、反馈与输出同一套刻度）

**输入 / 输出 / 调用链**
- 输入：`machine->dm_pos_max`（唯一读点 `dm.c:162`）
- 输出：`dm_motor_feedback[].pos_rad` → `pos_zero_rad` → `motor_state.dm.pos_zero_rad[]` → `leg.input.hip_f/hip_b` → `Leg_Solve()` → 腿长/腿角/雅可比 → LQR / RL 观测 / 力矩映射
- **没有位置下发路径**：全工程无 `Dm_Float_To_Uint` 编码位置（只有力矩编码，`dm.c:272` / `dm.c:304`），所以此改动只影响反馈解码

**为什么改**：手册特征表为"磁编（单圈 输出轴一圈绝对位置）"，备注①"上电后，电机位置输出限定在 [-π,π]rad 之间"。原值 12.5 取自手册中"Pos 预设 ±12.5"一句，但同一段紧接着标注"（下图仅作示例，与实际数据无关）"，且 12.5 会让**所有反馈角度放大 12.5/π ≈ 3.98 倍**。

**怎么读**：改后 ch35/36/43/44（零点后电机角）应收敛到 ±π 量级；ch31/ch39（腿长）应落回 0.136~0.46 m 的合理区间。

**核对**：编译 103 文件 0 fail / 0 warn。

**待台架**：`dm_vel_max = 45` / `dm_trq_max = 54` 为作者读上位机所得，尚未与固件其余量纲联调验证；力矩刻度错会让"下发值"和"回读值"反向偏差同一倍数（实际 = 下发 × 电机TMAX/固件TMAX），直接污染系统辨识。

---

## 变更 24 · VOFA 通道从 48 砍到 27（上限 32）

**动机**：原 48 通道里有大量已过期的 RL 手动遥操量、旧 LQR 通道和轮子诊断量，实际只用到约一半。精简到 27 路可降低带宽（帧长从 196 → 112 字节）并留出裕量（`VOFA_MAX_CH` = 32）。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/user-lib/Vofa_send.h` | `VOFA_MAX_CH` 从 48 改为 **32**（注释：上限 32；当前实际发 27 路） |
| `imcalib/task/task_comm.c` | `Robot_Control_Send_Vofa()` **整体重写**：`dbg[48]` → `dbg[VOFA_MAX_CH]`，发送 `Vofa_Send(dbg, 27u)`；删掉旧的 RL 通道（大腿/小腿的当前/目标/误差/虚拟力矩）、旧轮子量、旧 LQR 通道；新增电机原始解码角 ch3~6 与零点后角 ch7~10 成对显示 |

**新通道表**：27 路，详见 `md/VOFA_SEND.md`「当前 VOFA 通道布局」。

**输入 / 输出 / 调用链**
`Robot_Control_Send_Vofa()`（`task_comm.c`）→ 组装 `dbg[27]`（在线掩码 / 解算有效 / 策略 / 电机原始角 / 零点后角 / 速度 / 左右腿解算 / 下发力矩）→ `Vofa_Send(dbg, 27u)`（`Vofa_send.c`）→ `VOFA_UART` DMA 发送

**怎么读**
- ch3/7 成对：同一台电机的"原始解码角 vs 零点后角"，差一个 `dm_zero` 常数。
- ch1 = 3 表示两腿解算都有效；不是 3 则有一侧几何/偏置不成立。
- 200Hz 发送（每 5 个 1kHz 周期一次），帧长 27×4+4 = 112 字节。

**核对**：编译 103 文件 0 fail / 0 warn。

⚠️ 该 27 路布局已被变更 31 取代，以变更 31 为准。

---

## 变更 25 · 腿长工作区间进机器配置表

**改动**
| 文件:行 | 内容 |
| --- | --- |
| `machine_config.h:47-48` | 新增字段 `leg_len_min` / `leg_len_max` |
| `machine_config.c:28-29` | 大机器 `0.04 / 0.46` — **AI 填的理论极限占位值，待作者给实测工作区间** |
| `machine_config.c:51-52` | 小机器 `0.10 / 0.20`（作者给值） |
| `lqr_balance.h:35` | 删除 `LQR_LEG_LEN_MIN/MAX`（原 0.13 / 0.21，小机器遗留） |
| `lqr_balance.c:3 / 64-67 / 114` | 加 `#include "machine_config.h"`；使能判定与目标限幅改读 `machine->leg_len_min/max` |

**输入 / 输出 / 调用链**：`machine_config.c` 的表 → `machine->leg_len_*` → `LQR_Enable_Latch()`（腿长是否在有效域内）与 `LQR_Target_Update()`（腿长目标限幅）。

**核对**：编译 103 文件 0 fail / 0 warn；`grep LQR_LEG_LEN` 全工程无残留。

**待台架**：大机器的 0.04 / 0.46 是 `|lg−lu| ~ lu+lg` 的理论极限，不是实测工作区间。

---

## 变更 26 · ⚠️ 事故：AI 擅改大机器前髋极性 → 作者台架测出并还原

**事故**
- AI 在变更 20 同一批里，依据"参考固件极性表反推"，把大机器 `dm_sign` 从 `{{1,1},{1,1},{-1,-1},{-1,-1}}` 改成 `{{-1,-1},{1,1},{1,1},{-1,-1}}`（两个前髋取反），**未单独报备、未单独记录**。
- 台架现象：**摆动腿时腿长跟着动、腿摆角却不变**。原因：同一条腿两台电机在模型里一正一反 → 两杆"对转" → 杆夹角差 `Δ=qf−qb` 在变（腿长乱动），而两杆平均方向不变（腿摆角不动）。
- 2026-09-18 作者用"摆动/伸缩"测试判定"两个前髋反了"，**自行改回** `{{1,1},{1,1},{-1,-1},{-1,-1}}`；改回后 VOFA 通道与腿部解算正常。

**处置**
- 已在 `md/AGENTS.md` 新增 **§0.1 物理量绝对红线（最高约束）**：极性 / 零点 / 轴向 / 量程 / 符号项一律不得由 AI 改动，只能台架标定；AI 只允许"指可疑点 + 给验证方法 + 解释现象"。
- 今后此类改动即便作者要求，也必须先复述"改哪一行、从什么改成什么、依据是什么"，并在本文件**单独记一条**。

**教训（写给后续 AI）**：相位 / 极性只有台架说了算。参考固件、手册、仿真只能用来**提假设**，不能用来**改值**。

---

## 变更 27 · 打开总输出，手动遥操联调（极性台架自检）

**改动（只有一处）**
| 文件:行 | 内容 |
| --- | --- |
| `imcalib/task/robot_control.c:40` | `torque_output_enabled = 0u` → `1u`（总输出打开） |

**手动控制链路（本来已接好，无需另接线）**
`task_policy.c` `Manual_Lock_On_Enable()`（使能边沿锁存当前姿态为基准，因此使能不跳）→ `Remote_Command_Apply()` 按摇杆生成 `action_state.a` → `task_actuation.c:137` `RL_Torque_Compute()` → `output_send()` → `Dm_Send_Torque()` / `Dji_Send_Wheel_Torque()`。

**触发条件（缺一不可）**：遥控在线 → 左拨杆**上位** → 无故障且未翻倒（`motor_enabled`）→ 两腿解算有效（ch1 = 3）→ `base_action_locked`。

**摇杆映射**
| 遥控 | 动的虚拟关节 | 幅度 |
| --- | --- | --- |
| 左摇杆竖直 `ch3` | 大腿角（左右同时） | ±2 rad 目标偏移 |
| 左拨轮 `wheel` | 虚拟小腿角（左右同时） | ±2 rad |
| 右摇杆竖直 `ch1` | 轮子速度（两个轮同时） | ±80 rad/s |

**极性自检（代码层硬约束）**：每台电机的 `dm_sign[i].out` 必须等于 `dm_sign[i].fb`，否则该关节位置环变成正反馈。当前 `machine_config.c:18` 为 `{{1,1},{1,1},{-1,-1},{-1,-1}}`，四条都相等 ✓。

**核对**：编译 103 文件 0 fail / 0 warn。

**注意**：中位 = LQR（未实测）；大机器 `leg_len_min/max` 仍是 AI 填的占位值 0.04/0.46，会让 LQR 使能判定轻易通过 → 联调期间左拨杆必须停在上位。

---

## 变更 28 · 首次上电联调：限幅降到安全值

**改动（作者要求"先做安全测试，后续再放开限幅"，只改两行）**
| 文件:行 | 原值 | 现值 | 恢复值 |
| --- | --- | --- | --- |
| `machine_config.c:12` `dji_trq_clamp` | 6.0 | **1.5 Nm** | 6.0 |
| `machine_config.c:16` `dm_trq_clamp` | 20.0 | **3.0 Nm** | 20.0 |

- 只动"我们自己的限幅"，**没动 MIT 刻度**（`dm_pos_max / dm_vel_max / dm_trq_max` 按 §0.1 属物理量，未触碰）；`dm_trq_max` 行加了"勿改"注释。
- 大机器腿部额定 20 Nm，3.0 Nm ≈ 15%：够让腿在架子上慢速动作，推不动整机、也伤不到结构。

**未改动但决定"目标给多大"的三个数（属控制参数，未获授权不动）**
| 位置 | 值 | 效果 |
| --- | --- | --- |
| `task_policy.c:8` `MANUAL_ACTION_SCALE` | 4.0 | 满杆动作 ±4 → 关节目标偏移 ±2 rad（`RL_TQ_POS_SCALE 0.5`） |
| `rl_torque.c:8` `RL_TQ_WHEEL_VEL_SCALE` | 20.0 | 满杆轮速目标 **±80 rad/s** |
| `lqr_balance.h:44-45` LQR 输出限幅 | 1.5 / 2.0 Nm | 本来就是保守值，无需改 |

**核对**：编译 103 文件 0 fail / 0 warn。

---

## 变更 29 · 🐞 修 CAN 误判：未使用的总线不再计入在线检查（作者报"无法使能"）

**现象**：大机器上电机永远无法使能（遥控在线、拨杆到位也不出力）。

**根因（可证）**
- `can_bus.c` 的 `Can_Bus_Init()` 无条件启动 3 条 FDCAN；`Can_Bus_Online(true)` 对**每一条**总线都要求 100 ms 内有帧，否则判 DEAD。
- 大机器 `dm_bus={1,1,1,1}` + `dji_bus=3` → **FDCAN2 上没有任何登记设备**，必然在 100 ms 后判 DEAD → `task_comm.c:169` 永久置 `FAULT_CAN`(0x08) → `task_comm.c:207` 的 `enable_request` 恒为 0 → **使能永不成立**。
- 小机器 `dm_bus={1,1,3,3}` + `dji_bus=2`，三条总线都有设备，所以这个坑一直没暴露。

**改动**（`imcalib/user-lib/can_bus.c`，`Can_Bus_Online()` 循环开头加 4 行守卫）
```c
/* 本机没在这条总线登记设备 → 不要求流量 */
if (bus->route_cnt == 0u)
{
    bus->alive_prev = bus->alive_cnt;
    bus->dead_since = now;
    continue;
}
```

**输入 / 输出 / 调用链**：`machine_config.c` 的 `dm_bus[]` / `dji_bus` → `Dm_Init()` / `Dji_Init()` 在对应总线 `Can_Bus_Register()` → `bus->route_cnt` → `Can_Bus_Online()` 只检查有设备的总线 → `task_comm.c:169` `FAULT_CAN` → `Robot_Enable_Update()` 的使能门禁。

**核对**：编译 103 文件 0 fail / 0 warn。

**怎么验**：Watch `ctrl_fault` 应为 **0**；ch0 掩码应为 **255**。

---

## 变更 30 · ⚠️ 轮子抖震排查 + 限幅放开到满值

**1. 限幅放开（作者要求"限幅给大"）**
| 文件:行 | 变更 28 的安全值 | 现值 |
| --- | --- | --- |
| `machine_config.c:12` `dji_trq_clamp` | 1.5 | **6.0**（M3508 输出轴满力矩） |
| `machine_config.c:16` `dm_trq_clamp` | 3.0 | **20.0**（作者许可力矩） |

**2. 抖震排查（作者报"右轮速度环抖震很厉害，现在大腿也有"）**

代码层面**核对通过**的点（排除嫌疑）：
- 控制节拍：TIM6 = 275 MHz/(550×1000) = **500 Hz**，`pid_calc` 的 `dt` 固定 0.002 ✓ 一致。
- PID 本体：纯 P（腿 3.5 / 轮 8.0），环绕施加在**误差**上 ✓ 正确；`abs_limit` 的 NaN 保护 ✓。
- 环路符号自洽：`dm_sign` / `dji_sign` 每条都 `out == fb` ✓。
- 轮子力矩换算：`DJI_NM_PER_RAW_M3508 = 0.30×20/16384` 就是**输出轴 6 Nm 满量程**，不是漏乘减速比 ✓。
- DM 力矩场与量程 ✓。

**首要怀疑：右轮反馈极性与实机相反 → 正反馈 → 指令力矩在 ±限幅间来回打**，抖动经结构传到整条右腿（解释了"先前小腿、现在大腿"）。
- 判据（不用开输出）：手把两轮**同向**（机器人前进方向）转，Watch `motor_state.dji.vel_rad_s[0]/[1]` 应**同号**；一正一负即该路反馈反了。
- 判据（低速小目标）：Watch `rl_control.torque_state.last_torque.dji[1]`；在 ±6 之间来回打 = 正反馈；小幅波动且转速跟得上目标 = 增益/负载问题。

**次要怀疑**：`rl_torque.c:84-85` 轮速环 `wheel_pid[..][0] = 8.0` 对**空载**轮子偏高（纯 P 一阶环极点 = 1 − K·dt/J：空载 J≈0.01 → 1.6 越过稳定边界；落地 J≈0.09 → 0.18 稳）→ 同样表现为"架起来抖、落地不抖"。**未改**（属控制参数，待极性结论）。

**核对**：编译 103 文件 0 fail / 0 warn。**极性表一个字未动。**

**3. 追加：作者反馈"闭环正常、就是抖动很厉害，感觉是软件问题"后的复核**

代码层面**全部验证正确**（排除嫌疑）：
- TIM6 = 275 MHz/(550×1000) = **500 Hz**，`pid_calc` 的 dt = 0.002 ✓ 一致
- tick = 1000 Hz；commTask `osDelay(1)`≈1 kHz、imuTask 2 ms、policyTask 10 ms ✓
- 中断优先级 TIM6/DMA/FDCAN/UART 全为 5，`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5` → ISR 内调 RTOS API **合法** ✓
- 任务优先级 actuation(Realtime) > imu/policy(High) > comm(AboveNormal) → **控制任务最高** ✓
- FDCAN1 = 1 M/4 M FD_BRS（DM 电机）、FDCAN3 = CLASSIC 1 M（C620）✓ 帧格式正确
- 环路符号 fb == out ✓（与"闭环方向正常"一致）

**改动**：`imcalib/Algorithm/rl_torque.c:84-85`，STABLE 档轮速环 P：**8.0 → 2.0**。

依据（可算，不是"降增益掩盖"）：轮速环是纯 P 的一阶采样环，闭环极点 = 1 − K·dt/J。
- 本机空载轮子 J ≈ 0.01 kg·m²（M3508 转子惯量折到输出轴 ≈ 9e-3 + 轮盘 ≈ 7e-4）
- dt = 2 ms → 稳定上限 K < 2J/dt ≈ 10；环里还有 1~2 个周期的反馈/执行延迟，工程上打 3~5 折 → **K ≈ 2~3**
- 原来 8.0 超出 3~6 倍 → 表现就是"跟得上目标，但叠一层高频抖" ✓

**腿的位置环未动**：ω_n = √(3.5/0.05) ≈ 8.4 rad/s（≈1.3 Hz），离 500 Hz 采样极限很远，自身不会高频抖 → 腿上的抖判断为**被轮子振动经结构带上来**。验证：轮子断电或夹住时，腿是否还抖。

**未改但记为隐患**：`ws2812.c:16-22` 的 `WS2812_Ctrl()` 在 **commTask** 里做阻塞式 SPI（含 `while (State != HAL_SPI_STATE_READY);` 无超时死等 + 100 次阻塞发送），每 ~10 ms 触发一次，会推迟 commTask 里的 DM 反馈解码 → 让控制环吃到更旧的反馈。属可优化项，等作者定。

---

## 变更 31 · VOFA 换成虚拟关节 PID 视图（27 → 32 路）

**改动**（`imcalib/task/task_comm.c`，`Robot_Control_Send_Vofa()` **整体重写**）

| 变化 | 说明 |
| --- | --- |
| 发送路数 | `Vofa_Send(dbg, 27u)` → `Vofa_Send(dbg, 32u)`（`VOFA_MAX_CH` 仍为 32，32 路全部使用） |
| 帧长 | 27×4+4 = 112 字节 → 32×4+4 = **132 字节** |
| 新增 ch3~20 | 6 个虚拟关节 PID（含两个轮）：当前值 `get` / 目标值 `set` / 输出 `pos_out`，各 6 路（`vj_all[6]={0,1,2,3,4,5}` = 左大腿/左小腿/左轮/右大腿/右小腿/右轮） |
| 新增 ch25~26 | 轮子下发力矩：左轮 / 右轮 |
| 新增 ch27/28 | 腿长：左 / 右 `virtual_leg_length` |
| 新增 ch29~31 | 小腿雅可比：左 `vshank_jac[0]/[1]` + 右 `vshank_jac[0]` |
| 删除 | 误差块 `err`（err = 目标 − 当前，图上可目视）；电机原始解码角；电机速度；腿摆倾角；腿电机零点后角；左右腿的大腿角/虚拟小腿角 |

**新通道表**：32 路，详见 `md/VOFA_SEND.md`「当前 VOFA 通道布局」。

**输入 / 输出 / 调用链**

```
rl_control.torque_state.controller[]  (6 个虚拟关节 PID，含轮)
rl_control.torque_state.last_torque   (电机下发力矩，DM + DJI)
leg_l|r.output.virtual_leg_length     (腿长)
leg_l|r.output.vshank_jac[]           (小腿雅可比)
    │
    │  Robot_Control_Send_Vofa() @ task_comm.c（每 5 个 1kHz 周期）
    ▼
dbg[32]
    │
    │  Vofa_Send(dbg, 32u) @ Vofa_send.c
    ▼
VOFA_UART DMA 发送（132 字节 + 4 字节帧尾）
```

**为什么这么改**：作者要查"小腿不动"的原因，需要虚拟关节的当前/目标/PID输出 以及小腿雅可比，才能完整追踪"虚拟力矩 → 雅可比 → 电机指令力矩"这条链路。追加后扩展到 6 个虚拟关节（含两个轮），并补上轮子下发力矩，使全部执行链路可见。

**核对**：编译 103 文件 0 fail / 0 warn。

---

## 变更 32 · 系统辨识前置：CAN 接收打 ns 时间戳（计划缺口 A）

**背景**：按 `md/sysid/sysid-lower-machine-plan.md` 执行"6 关节建模"，缺口 A 是全部数据采集的前置（`rx_ns` 决定每一行的时刻精度）。

**改动**（4 个文件，无控制行为变化、不涉及任何物理量）
| 文件:行 | 内容 |
| --- | --- |
| `imcalib/user-lib/dm.h:54` | `dm_motor_feedback_t` 新增 `uint64_t rx_ns;` |
| `imcalib/user-lib/dji.h:47` | `dji_motor_feedback_t` 新增 `uint64_t rx_ns;` |
| `imcalib/user-lib/dm.c:94` | `Dm_Read()` ISR 入口记录 `Mono_Ns_Get()`；新增 `#include "mono_ns.h"` |
| `imcalib/user-lib/dji.c:94` | `Dji_Read()` ISR 入口同上 |

**输入 / 输出 / 调用链**：CAN RX 中断 → `HAL_FDCAN_RxFifo0Callback` → `Dm_Read()` / `Dji_Read()` → 打 `rx_ns`（DWT + TIM6 扩展的单调 ns）→ `raw_pending` → `Dm_Parse()` / `Dji_Parse()` 解码 → 后续 sysid 数据行用 `rx_ns` 作时间戳。

**为什么安全**：`Mono_Ns_Get()` 只读 DWT_CYCCNT + 一个由 TIM6 维护的纪元；TIM6 与 FDCAN 中断同为优先级 5（不能互相抢占），读不会撕裂；ISR 内耗时几十 ns，DM 反馈 2000 帧/s 下开销可忽略。

**核对**：编译 103 文件 0 fail / 0 warn。

**待台架验证**：连续帧的 `rx_ns` 差值应单调递增且 DM 约 2 ms（500 Hz）量级；与 `last_rx_tick` 的毫秒部分应一致。

---

## 变更 33 · 测试模式的遥控进入条件改为"左上位 + 右中位"（作者定）

**改动**（`imcalib/task/task_actuation.c:76-83`，只改条件与注释）
- 进入 `CTRL_STRATEGY_SYSID` 的条件：`s1 == 中位 && s2 == 中位` → **`s1 == 上位 && s2 == 中位`**
- 其余不变：`s1 中位` → LQR；否则 → 手动遥操/RL。因此**只要左拨杆不在上位、或右拨杆不在中位，下一个控制周期（2 ms）就立刻离开测试模式**。

**依据**：作者 2026-09-18 明示"改成 s1 上位跟 s2 中位再开启，然后只要不是这个就立马退出"。

**注意**：`SYSID_ENABLE` 仍为 0，测试分支当前不参与编译；等 sysid 主体（计划缺口 C/D）落地后再打开。

**核对**：编译 103 文件 0 fail / 0 warn。

---

## 变更 34 · 大机器两项机械参数按作者实测填入（腿长区间 + 轮总减速比）

**依据**：作者 2026-09-18 确认"0.14 到 0.34 是实际区间""总减速比是 15.5"。

| 位置 | 原值 | 现值 | 来源 |
| --- | --- | --- | --- |
| `machine_config.c:10` `dji_gear_ratio` | `19.2f * CJL_WHEEL_BOX_RATIO`（=19.2，占位） | **`15.5f`** | 作者实测（转子→轮子总比） |
| `machine_config.c:27-28` `leg_len_min/max` | 0.04 / 0.46（AI 填的理论极限） | **0.14 / 0.34** | 作者实测工作区间 |
| `machine_config.c:3-4` | `CJL_WHEEL_BOX_RATIO` 占位宏（1.0） | **删除** | 总比已含箱比，占位宏失效 |

**影响面**
- `dji_gear_ratio`：是 `Dji_Rpm_To_Rad_S()` / `Dji_Encoder_To_Rad()` 的除数 → 轮子速度/角度读数按 19.2/15.5 = 1.24 倍变化（现在是真实值）。
- `leg_len_min/max`：`LQR_Enable_Latch()` 的使能判定与 `LQR_Target_Update()` 的腿长目标限幅（`imcalib/Algorithm/lqr_balance.c`）。

**核对**：编译 103 文件 0 fail / 0 warn；`grep CJL_WHEEL_BOX_RATIO` 全工程无残留。

**⚠️ 连带未处理项（等作者确认）**：`dji.h` 的 `DJI_NM_PER_RAW_M3508 = 0.30f * 20.0f / 16384.0f` 中那个 **0.30 已隐含 19.2 的减速比**（0.015625 Nm/A × 19.2 = 0.30；佐证：×20 A = 6.0 Nm = M3508 输出轴堵转力矩；M2006 同构：0.005 × 36 × 10 A = 1.8 Nm）。总比改成 15.5 后**轮子力矩换算会偏大 1.24 倍**。建议改成按 `machine->dji_gear_ratio` 显式换算（对 36 / 19.2 标准机型数值完全等价）。

---

## 变更 35 · 系统辨识前置：CAN 发送完成打 ns 时间戳 + TX pending 表（计划缺口 B）

**改动**（只动 `imcalib/user-lib/can_bus.c` / `can_bus.h`：+113 / +30 行）
| 位置 | 内容 |
| --- | --- |
| `can_bus.h` | 新增类型 `can_tx_pending_t` / `can_tx_done_t`；`can_bus_t` 新增 `tx_complete_cnt`、`tx_pending[32]`、`tx_ring_w/r`、`tx_drop_cnt`；新增 4 个 API 声明 |
| `can_bus.c` `Can_Bus_Start()` | 追加 `HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_COMPLETE, 0xFFFFFFFF)` |
| `can_bus.c` `Can_Bus_Transmit()` | 改为调用新函数 `Can_Bus_Transmit_Tagged(..., 0, 0)`，**签名与语义完全不变**（`last_tx_tick` 照旧更新） |
| `can_bus.c` 新增 `Can_Bus_Transmit_Tagged()` | 入队后在 pending 表登记 `{kind, seq}`，元素索引取 `HAL_FDCAN_GetLatestTxFifoQRequestBuffer()` 的位掩码再转索引 |
| `can_bus.c` 新增 `HAL_FDCAN_TxBufferCompleteCallback()` | 对 `BufferIndexes` 每一位取 `Mono_Ns_Get()`，按元素索引从 pending 表取出标签，写入 SPSC 完成环形缓冲（每条总线 64 项，满则丢最旧并 `tx_drop_cnt++`） |
| `can_bus.c` 新增 `Can_Bus_Tx_Pop()` / `Can_Bus_Tx_Complete_Count()` / `Can_Bus_Tx_Drop_Count()` | 任务侧取完成事件与计数 |

**关键设计**：FDCAN 配的是 `FDCAN_TX_FIFO_OPERATION`（优先级排序），完成顺序 ≠ 入队顺序 → **必须按 TX 元素索引配对**，故用 pending 表 + 位掩码取索引。

**独立核对（AI 亲自验证，非子代理自述）**
1. `HAL_FDCAN_GetLatestTxFifoQRequestBuffer()` 返回的是**位掩码**：HAL 文档同一段明确说该返回值可交给 `HAL_FDCAN_AbortTxRequest(BufferIndexes)` 使用，而后者取掩码 → `31 - __CLZ(mask)` 取索引正确 ✓
2. `HAL_FDCAN_TxBufferCompleteCallback(hfdcan, BufferIndexes)` 签名与 HAL 弱定义一致（不一致会编译报错，实测 0 warn）✓
3. TX 完成中断的两条 NVIC 线（`FDCANx_IT0/IT1`）在三路 FDCAN 上都已使能且 ISR 都调 `HAL_FDCAN_IRQHandler`（`Core/Src/stm32h7xx_it.c:281/309/295/323/439/453`）→ 中断会真正进来 ✓

**已知小瑕疵（不阻塞）**：环形缓冲**溢出**时由 ISR 推进 `tx_ring_r`，与任务侧 `Can_Bus_Tx_Pop()` 同时写该索引；正常不溢出时是严格 SPSC，无影响。`can_tx_done_t.valid` 字段当前冗余未被读取。

**核对**：编译 103 文件 0 fail / 0 warn（AI 亲自跑）。

**待台架验证**：`Can_Bus_Tx_Complete_Count(1)` 应 ≈ 发送帧数；`Can_Bus_Tx_Pop()` 出的 `tx_ns` 差值应合理、单调；`Can_Bus_Tx_Drop_Count()` 应为 0。

---

## 变更 36 · 轮电机力矩刻度按总减速比缩放 + 限幅降到物理上限（作者批准）

**依据**
- 作者 2026-09-18 批准："用配置表配置是可以的""第二个也可以"。
- 参考工程交叉验证：`XYEGA_RM2026_.../readme.md:118` 明写"轮电机3508…在我们 **16.33** 减速比的减速箱条件下，它的最大输出力矩也就是 **5Nm，超过了会失控**"。用我们的常数按比例推：`6.0 × 16.33 / 19.2 = 5.1 Nm` ✓ 与参考一致 → 证明 **原来的 0.30 里确实含 19.2**，必须随实机总比缩放。
- 参考工程驱动层同样按减速比算：轮子配置带 `.reduction_radio`，力矩上限 = `CURRENT_BIT_2_A_M3508 × getDJITorqueConstant()`（按电机实例，含减速比）。

**改动**
| 文件:行 | 内容 |
| --- | --- |
| `imcalib/user-lib/dji.h:17-27` | 删 `DJI_NM_PER_RAW_M2006/M3508`；改为 `DJI_NM_FULL_M2006/M3508`（满电流输出轴堵转力矩，标准比下）+ `DJI_RATIO_STD_M2006/M3508`（36 / 19.2） |
| `imcalib/user-lib/dji.c:46-60` | `Dji_Torque_To_Current()` 的 `per_raw` 改为 `满力矩 / 满raw × (实机总比 / 标准比)` |
| `imcalib/user-lib/machine_config.c:12` | 大机器 `dji_trq_clamp` 6.0 → **4.8**（15.5 箱比下的输出轴物理上限） |

**数值自检（AI 亲自算，非子代理自述）**
| 机型 / 总比 | `per_raw` | 满量程 |
| --- | --- | --- |
| M2006 @36（小机器） | 1.800e-4 | **1.80 Nm**（与原值完全一致 → 小机器行为不变） |
| M3508 @19.2（标准比） | 3.662e-4 | **6.00 Nm**（与原值完全一致） |
| M3508 @15.5（大机器实机） | 2.956e-4 | **4.84 Nm** ← 修正后的真值 |

**核对**：编译 103 文件 0 fail / 0 warn；`grep DJI_NM_PER_RAW` 全工程无残留。

**遗留**：轮端力矩常数仍未台架实测（`md/RL_OVERVIEW.md` 记的"悬臂挂砝码法"）；本次只把"减速比"这一因子放对。

---

## 变更 37 · VOFA 通道换成 sysid 数据自检视图（作者台架核对用）

**背景**：作者要求"先把这些需要我观测的数据替换 vofa 通道，让我观测一下对不对，做台架验证"。独立 sysid 帧（计划缺口 C）仍在后面做，本次只是把将来要进帧的数据先摆到 Vofa+ 上核对。

**改动**
| 位置 | 内容 |
| --- | --- |
| `imcalib/user-lib/Vofa_send.h:9-12` | 新增布局开关 `VOFA_LAYOUT`（`#ifndef` 保护，可 `-DVOFA_LAYOUT=n` 覆盖）：1 = 虚拟关节 PID 视图，**2 = sysid 数据自检（当前默认）** |
| `imcalib/task/task_comm.c` | `Robot_Control_Send_Vofa()` 拆成"共用状态块 + `#if` 两套布局"；新增 layout 2 的自检填充 |
| `imcalib/task/task_comm.c` 头部 | 新增 `#include "machine_config.h"`（layout 2 用 `machine->dji_sign / dji_bus / dm_bus`） |

**layout 2 通道表（32 路，帧长 132 字节，200Hz）**
| ch | 内容 | 单位 / 来源 |
| --- | --- | --- |
| 0~2 | 在线掩码 / 解算有效 / 策略号 | 与之前一致 |
| 3~6 | 腿**指令力矩** 前左/后左/前右/后右 | Nm，`last_torque.dm[]` |
| 7~10 | 腿**零点后角** 同上 | rad，`motor_state.dm.pos_zero_rad[]` |
| 11~12 | 腿长 左/右 | m |
| 13~14 | 腿摆倾角 左/右 | rad |
| 15~16 | 轮**指令电流 raw** 左/右 | `Dji_Torque_To_Current(i, tau × out_sign)` |
| 17~18 | 轮**转速** 左/右 | rad/s |
| 19~20 | 轮**编码器 raw** 左/右 | `dji_motor_feedback[].angle_raw` |
| 21~22 | 轮**实际电流 raw** 左/右 | `dji_motor_feedback[].current_raw` |
| 23 | 轮温度（左） | `temp_raw` |
| 24 | 轮总线 **TX 完成间隔** | µs（相邻两次取到的完成时刻之差） |
| 25 | 轮 **RX 到达间隔** | µs |
| 26 | 腿总线 **TX 完成间隔** | µs |
| 27 | 轮总线本周期 **TX 完成条数** | 每周期应为 1 |
| 28 | 腿总线本周期 **TX 完成条数** | 每周期应为 4 |
| 29 | 轮总线 **TX 丢帧累计** | 必须为 0 |
| 30~31 | 轮 RX 时刻原始拆分 hi/lo | `rx_ns >> 20` / `rx_ns & 0xFFFFF` |

**注意（写给后续）**：该视图用 `Can_Bus_Tx_Pop()` 取完成事件，**会消费掉事件**。等真正的 sysid 数据帧落地时，本视图必须让位（不能与数据帧抢事件），或改成只读计数。

**核对**：两种布局都编译通过 —— `py cc_check.py` 与 `py cc_check.py -DVOFA_LAYOUT=1` 均 **103 文件 0 fail / 0 warn**。

**待作者台架核对**：腿 4 路力矩/角度的方向；轮指令 raw 与实测转速的正负关系；`rx_ns`/`tx_ns` 间隔是否 ≈ 2000 µs（500Hz）且单调；`tx_drop` 是否为 0。

---

## 变更 38 · 🐞 修 TX 完成中断登记：只认本机登记过的元素（作者台架测出）

**现象（作者台架读数）**：ch27 / ch28（每窗口 TX 完成条数）**稳定在 64**；ch29（TX 丢帧）持续增长。

**根因（可证）**
- 缺口 B 的 `HAL_FDCAN_TxBufferCompleteCallback()` 对 `BufferIndexes` 掩码里**每一个置位的元素**都写一条完成记录，**没有检查该元素是否本机登记过**。
- 每窗口稳定 64 = **环形缓冲容量（`CAN_TX_RING_CAP` = 64）**，即缓冲每窗口都被填满。按"**32 位 × 每窗口约 2 次中断 = 64**"推断：硬件把**整个 TXBCF 掩码（32 位全置位）**都报给了回调，其中绝大多数元素没有任何登记信息 → 被当成"完成"写进环 → 环立即满 → 持续丢帧。

**改动**（`imcalib/user-lib/can_bus.c/h`）
| 位置 | 内容 |
| --- | --- |
| `can_bus.h:31-36` | `can_tx_pending_t` 增加 `uint8_t valid;` |
| `can_bus.c:174-178` | `Can_Bus_Transmit_Tagged()` 登记时置 `valid = 1` |
| `can_bus.c:239-270` | ISR 内**先判 `valid`，未登记的位直接跳过**（不计数、不写环）；处理完清 `valid` |

**预期（修后）**：ch27 ≈ **2~3** /窗口（轮 500Hz × 200Hz 采样）、ch28 ≈ **10** /窗口（腿 4 台 × 500Hz）、ch29 **保持 0**。

**核对**：两种布局（`VOFA_LAYOUT` 1/2）均编译 103 文件 0 fail / 0 warn。

---

## 变更 39 · 测试模式补全：激励序列 + 状态机 + 事件标记 + 安全

**目的**：把 sysid 从"空壳零力矩"补成完整的数据采集链路：激励序列、自动跑批、事件标记、安全/无效判定。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_config.h` | `SYSID_ENABLE` 默认改为 **1**；新增 `SYSID_PLAN` 宏（1=仅腿 2=仅轮 3=全部，默认 1） |
| `imcalib/Sysid/sysid_mode.c` | **整体重写**：新增 run 描述结构 + 激励序列表 + 单周期状态机 |
| `MDK-ARM/CtrBoard-H7_ALL.uvprojx` | `imcalib/Sysid` 组新增 `sysid_log.c`（之前缺失，SYSID_ENABLE=1 会链接失败） |

**新增宏（sysid_mode.c 文件顶部，易改）**

| 宏 | 值 | 含义 |
| --- | --- | --- |
| `SYSID_TRQ_LIMIT_NM` | 3.0 | 腿力矩限幅 Nm，待台架 |
| `SYSID_CURRENT_LIMIT_RAW` | 12288 | 轮电流限幅 raw（±15A） |
| `SYSID_RAW_PER_A` | 819.2 | C620 raw/A |
| `SYSID_TEMP_LIMIT_C` | 80 | 温度限 °C，待台架 |
| `SYSID_PLAN` | 1 | 1=仅腿 2=仅轮 3=全部 |

**激励序列表（sysid_runs[]，共 79 run）**

腿（SYSID_PLAN=1/3，33 run）：

| 用例 | run 数 | 结构 |
| --- | --- | --- |
| `torque_baseline` | 1 | 全零 5s |
| `torque_step` | 24 | 4路 × ±1/±2/±3 Nm，每 run：zero 2s → step 0.5s → zero 3s × 3 |
| `torque_chirp` | 4 | 4路各 1.5 Nm，0.2→10 Hz 扫频 10s，前后 zero 2s/3s |
| `holdout_torque_chirp` | 4 | 同 chirp 但 2.0 Nm |

轮（SYSID_PLAN=2/3，46 run，默认不启用）：

| 用例 | run 数 | 结构 |
| --- | --- | --- |
| `baseline_sign` | 2 | 左右各 1 run：0A 5s → +0.5A 1s → 0A 2s → -0.5A 1s → 0A 5s |
| `stiction` | 2 | 正/负各 1 run（左轮）：17 级阶梯，每级 1s |
| `plateau` | 22 | ±1~±15A 各 run（左轮）：zero 2s → 平台 5s → zero 2s |
| `step` | 10 | ±1~±15A 各 run（左轮）：zero 2s → 阶跃 0.5s → zero 3s × 3 |
| `holdout_step` | 10 | 重做 step |

**状态机**

```
每个 tick:
  重入检测 (>10ms 未调用 → 重新初始化)
  安全检查 → 失败则 abort + 停止
  循环回绕 (run_idx >= 总数 → 回到 0)
  run 开始 → 推 kind=5 标记 (phase_or_event=-1)
  计算目标值 → 限幅 → 发送命令 → 快照 → 推数据行
  run 结束 → 发零 → 推 kind=5 标记 (-2) → 下一个 run
```

**事件标记**

| phase_or_event | 列 5 | 列 6 | 列 7 | 列 8 |
| --- | --- | --- | --- | --- |
| -1 (run 开始) | run_index | test_id | 总段数 | 0 |
| -2 (run 结束) | 0 | 0 | 0 | 0 |
| -3 (中止) | 0 | 0 | 0 | 0 |

**安全链**

| 条件 | 动作 |
| --- | --- |
| `torque_output_enabled==0` | abort + 停止 |
| DM 掉线 (`Dm_Is_Online` 失败) | abort + 停止 |
| 腿解算无效 (`!leg_*.output.valid`) | abort + 停止 |
| 腿长超出 `leg_len_min/max` | abort + 停止 |
| DM 温度 > 80°C | abort + 停止 |
| 拨杆离开组合 | actuation 任务不调用 → 隐式中止（自动回零） |

停止后必须拨杆离开再回来才重新启动。

**轮电流接口**

直接调用 `Dji_Send_Current()` 发 raw 电流（`can_bus.h` 已有），不经 `Dji_Send_Wheel_Torque()`（不应用 `dji_sign.out`）。amplitude(A) × 819.2 = raw，限幅 ±12288。`tau_cmd[]` 列记录 raw 值（kind=3 行）。

**时间戳**

保持现有 pop 方式：先发命令，再从 CAN TX 完成环取最晚时刻。每周期同时 pop 腿总线和轮总线（修复原有只 pop 腿总线导致轮总线环溢出的隐含问题）。leg/wheel 行分别取对应总线的 TX 时刻。

**输入 / 输出 / 调用链**

```
actuationTask (500Hz) → output_task_body()
  → 拨杆判定: s1=UP + s2=MID → ctrl_strategy=CTRL_STRATEGY_SYSID
  → Sysid_Mode_Run()
    → sysid_safe() → 安全门禁
    → sysid_target() → 计算激励值
    → Dm_Send_Torque() / Dji_Send_Zero() (腿行)
    → Dm_Send_Zero() / Dji_Send_Current() (轮行)
    → sysid_fill_fb() → 快照
    → Sysid_Log_Push() → 环形缓冲

commTask (1kHz) → comm_task_body()
  → Sysid_Log_Send_Pump() → 500Hz DMA 发送
```

**核对**：4 种配置均 105 文件 0 fail / 0 warn：
- 默认（`SYSID_ENABLE=1, SYSID_PLAN=1`）
- `-DSYSID_ENABLE=0`
- `-DSYSID_PLAN=2`
- `-DSYSID_PLAN=3`

**已知取舍**

- 环形缓冲满时 `Sysid_Log_Push()` 返回 false，现有代码只对局部变量 `snap.whl_drop_cnt++`（不生效），此 bug 未修（不影响 CAN tx_drop 计数）
- 轮测试默认只测左轮（stiction/plateau/step），如需右轮可在表里追加
- marker 行的 `t_cmd_ns` 取自最近一次 TX 完成（可能滞后 1 个周期 ≈2ms）

**待台架**

- `SYSID_TRQ_LIMIT_NM = 3.0` 是否需要放开（大机器额定 20 Nm）
- `SYSID_TEMP_LIMIT_C = 80` 是否合适
- DM 温度 `temp_mos` / `temp_rotor` 80°C 阈值
- 轮命令与物理正方向的对应关系（变更 40 已改为按 `dji_sign.out` 换算，训练端不必再自行标定）

**⚠️ 列数未变**：仍为 31 列，帧长 128B，`sysid-delivery.md` 和 `sysid_export.py` 不需要同步。

---

## 变更 40 · 复核修正：轮命令极性同域 + 帧内轮命令列约定 + 导出填充

**背景**：变更 39 完成后主代理复核，发现两处不一致（不影响腿测试，只影响轮测试）：

1. 轮命令走 `Dji_Send_Current()` 直发 raw，绕过了 `dji_sign.out`；而帧内轮反馈在 `dji.c` 解析时已按 `dji_sign.fb` 取反 → **右轮（out = −1）命令与反馈不同域**，配对会出现"负增益"假象。
2. 帧内其实已经带了轮命令值（kind=3 行的列 5/6 写的是逻辑 raw），但列约定没写进 `sysid_log.h` / 交付文档，导出脚本把 `cmd_test_wheel_raw` 留空。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_mode.c` | ① 轮分支上线前按 `machine->dji_sign[target].out` 换算，帧内仍记逻辑 raw；② `sysid_safe()` **删掉腿长区间门与温度门**（只留 `torque_output_enabled`、DM 离线、解算无效三个硬故障门），对齐作者既定决策"位置/速度/温度只记录、不设阈值"；`SYSID_TEMP_LIMIT_C` 保留但标注"当前未启用" |
| `imcalib/Sysid/sysid_log.h` | 注释补"行类型补充约定"：kind=3 列 5/6 = 轮命令 raw；kind=5 列 5~8 = run_index/test_id/总段数/0；kind=5 的列 3/4 仅作参考 |
| `tools/sysid_export.py` | `_write_wheel_csv_cmd()` 增 `side` 参数，按侧填 `cmd_test_wheel_raw`（左取列 5、右取列 6），两处调用同步 |
| `md/sysid/sysid-delivery.md` | 新增 §1.3.1「轮电流行（kind=3）列 5/6 约定」；§1.3 补标记行 `t_cmd_ns` 说明；按 §0.1 删去未经台架确认的极性断言（腿摆角正方向、"X 轴 = 前进方向"、轮 +0.5A 转向预期），改为"作者台架标定项"；§5.2/5.3 改成与现状一致（自动跑批、单轮时长、`SYSID_ENABLE` 默认 1、`torque_output_enabled` 前提）；§5.5 修正导出命令（`py` + `--out`）并补产物清单与列名说明 |
| `.gitignore` | 新增 `/data`（sysid 采集与导出产物不入库） |

**输入 / 输出 / 调用链**

- 轮：输入 `sysid_runs[].target`（0=左 1=右）、`sysid_to_raw(幅值A)` = 逻辑 raw、`machine->dji_sign[target].out`；输出 `Dji_Send_Current()` 的线上 raw = 逻辑 raw × 输出极性，帧列 5/6 记**逻辑** raw（与列 23~28 反馈同域）。
  调用链：`actuationTask → Sysid_Mode_Run() → sysid_clamp_f/sysid_to_raw → Dji_Send_Current → Can_Bus_Transmit → dji_bus`
  导出链：`VOFA 文件 → tools/sysid_export.py → wheel-<side>-<run>/c620_command_raw.csv`
- 腿链路未动：`sysid_clamp_f → Dm_Send_Torque → dm.c 内按 dm_sign.out 换算`，帧列 5~8 记逻辑 Nm，与腿反馈同域。

**核对**

- 编译：默认 / `-DSYSID_ENABLE=0` / `-DSYSID_ENABLE=1` 均 105 文件 0 fail / 0 warn（主代理复核）
- 导出自测：`py tools/sysid_export.py --selftest` 全项通过
- 未动任何物理量：`dji_sign` / `dm_sign` / `dm_zero` / `leg_off_phi0` 等值一律未改，只是新增"按表使用极性"的调用

**待台架**

- 轮命令与轮实际转动的物理方向仍需作者目视确认（表内 `dji_sign` 是作者标定值）
- 若作者要求帧内改记线上 raw（未乘极性），只需改列 5/6 赋值那一行，并同步交付文档与导出脚本

---

## 变更 41 · 诊断加固：心跳行 + 状态码 + 丢帧可见（作者台架排障）

**背景**：作者上机后看到 ch0=1、ch2（段号）恒为 1、ch5~8 全 0。旧代码在两种完全不同的故障下表现一模一样：①状态机没跑/被停机；②VOFA+ 没换成 JustFloat 连接或列错位。旧代码还有两处静默失败：

1. **停机后完全不推帧** → "流断了"和"没数据"无法区分
2. **环形缓冲满时丢帧不计任何数**（变更 39 已记录该 bug）→ 丢帧看不见

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_mode.c` | ①`sysid_safe()` → `sysid_fault()`，返回状态码（`SYSID_ST_*`）；②新增心跳行推送 `sysid_push_heartbeat()`，每 250 ms 一行、**任何状态都推**；③新增 `reinit_cnt`（重入重置累计，`Init` 里累加不清零）；④心跳行复用列 9~12 放诊断位 |
| `imcalib/Sysid/sysid_log.c` | ①新增 `volatile uint32_t sysid_log_drop_cnt / sysid_log_busy_cnt`；②`Sysid_Log_Push()` 失败时累加 drop；③发送泵**先判串口空闲再取数据**（原来先 `ring_r++` 再判断，UART 忙时那一帧被静默丢掉），忙则累加 busy 并保留数据 |
| `imcalib/Sysid/sysid_log.h` | 加 `SYSID_EVENT_HEARTBEAT 0`；注释补心跳行列定义；`extern` 两个诊断计数 |
| `md/sysid/sysid-delivery.md` | 新增 §1.3.2 心跳行与状态码表 |

**输入 / 输出 / 调用链**

- 输入：`stopped` / `stop_code`（`sysid_fault()` 的返回值）、`run_idx` / `run_active` / `tick_in_run`、`sysid_log_drop_cnt` / `sysid_log_busy_cnt` / `reinit_cnt`、`Mono_Ns_Get()`
- 输出：`kind=5, phase_or_event=0` 的心跳行进环形缓冲 → 发送泵 → VOFA；列 5~12 = run_idx / test_id / phase / 状态码 / 重入 / 丢帧 / 串口忙 / 心跳计数
- 调用链：`actuationTask → Sysid_Mode_Run() → hb_tick 计数 → 125 拍(250ms) → sysid_push_heartbeat() → Sysid_Log_Push() → commTask → Sysid_Log_Send_Pump() → HAL_UART_Transmit_DMA`
- 停机路径：`sysid_fault()≠0 → 推 ABORT 标记(-3) + stop_code=fault + stopped=1`，之后每周期只发零力矩，但**心跳照推**（列 8 = 停机原因）
- 导出侧：`tools/sysid_export.py` 的 `RunSplitter` 只认 -1/-2/-3 为 run 边界，心跳行（0）不影响切分，也不写入任何 CSV（未改动工具）

**核对**

- 编译：默认 / `-DSYSID_ENABLE=0` / `-DSYSID_ENABLE=1` 三种配置 0 fail / 0 warn
- 导出自测：`py tools/sysid_export.py --selftest` 全项通过
- 未动任何物理量；未改帧长（仍 31 列 / 128 B）

**待台架**

- 心跳周期 250 ms 是否合适（可改 `SYSID_HB_TICKS`）
- 作者需重新 Build + Download 才能看到心跳行

---

## 变更 42 · 激励幅值提高（作者要求：3 Nm 顶不动气弹簧）

**背景**：台架实测列 5 偶尔出现 +1 Nm 脉冲（说明状态机、激励、发送都正常），但四个髋角（列 9~12）几乎不动 —— 1~3 Nm 相对气弹簧 + 自重太小，信噪比不够，拟合不出摩擦/延迟。作者要求把激励给大。

**改动**（全部在 `imcalib/Sysid/sysid_mode.c`）

| 位置 | 原值 | 新值 |
| --- | --- | --- |
| `SYSID_TRQ_LIMIT_NM`（文件顶部宏） | 3.0f | **8.0f** |
| `sysid_runs[]` 的 `torque_step` 幅值（24 个 run） | ±1 / ±2 / ±3 Nm | **±2 / ±4 / ±6 Nm** |
| `torque_chirp` 幅值（4 个 run） | 1.5 Nm | **4.0 Nm** |
| `holdout_torque_chirp` 幅值（4 个 run） | 2.0 Nm | **5.0 Nm** |
| `md/sysid/sysid-delivery.md` | §4.1 用例表幅值、§5.3 限幅值同步 | — |

**输入 / 输出 / 调用链**

- 输入：`sysid_run_t.amplitude`（新幅值）→ `sysid_target()` → `sysid_clamp_f(target, ±SYSID_TRQ_LIMIT_NM=8)` → `Dm_Send_Torque()`
- 输出：帧列 5~8 = 已限幅的实际下发力矩（现最大 ±6 Nm，仍低于 8 Nm 上限）
- 轮用例不受影响（幅值是安培，上限 `SYSID_CURRENT_LIMIT_RAW = 12288` 未变）

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 安全性：最大 6 Nm 单路、每段仅 0.5 s（阶跃）或 4~5 Nm 正弦（扫频），相对 `dm_trq_clamp = 20 Nm` 仍保守；腿周边需清空
- 未动任何物理量；未改帧长

**待台架**

- 6 Nm 是否会让腿撞限位过猛（若撞击剧烈，把最大幅值降到 4 Nm 或缩短阶跃段）
- 8 Nm 上限是否需要按结构承受能力再调（作者定）

---

## 变更 43 · 新增腿用例预压（工作点）+ 限幅放到满限幅

**背景**：台架发现腿悬空时被气弹簧顶在**最长限位**：往"伸"的方向推力矩全被限位吃掉，只有"收"的方向能动；腿长只覆盖工作区间（0.14~0.34 m）最上端；1~3 Nm 也看不出版应。作者要求加"初始目标力矩"作为工作点，从这个值起测。

**改动**（`imcalib/Sysid/sysid_mode.c`）

| 位置 | 内容 |
| --- | --- |
| 文件顶部 | 新增 `SYSID_PRELOAD_L_N` / `SYSID_PRELOAD_R_N`（沿腿力 N，负=收腿，0=不加）。初值 -80（≈11Nm/髋）；作者台架试出"太大"，改为 **-22（≈3Nm/髋）** |
| 文件顶部 | `SYSID_TRQ_LIMIT_NM`：3.0 → 8.0 → **20.0**（= 电机满限幅，给预压+激励留头寸） |
| 腿分支 | 非 baseline 的腿 run：`tau_cmd[F/B_LFT] += 预压_L × leg_jac[0][0/1]`，右侧同理，然后整组限幅 |
| `md/sysid/sysid-delivery.md` | 新增 §4.3 预压说明（换算、调法、削平检查） |

**输入 / 输出 / 调用链**

- 输入：宏常量 `SYSID_PRELOAD_*_N`、解算输出 `leg_l/leg_r.output.leg_jac[0][0..1]`（腿长对前/后髋角的偏导，`leg_solver.c` 的 `leg_jac[0][*]`）
- 计算：`τ_f = 预压 × ∂l0/∂q_f`，`τ_b = 预压 × ∂l0/∂q_b` —— 等价于"沿腿加一根常力弹簧"，负值收腿
- 输出：与激励相加后按 ±20 Nm 限幅 → `Dm_Send_Torque()`；帧列 5~8 记录**预压+激励的净力矩**
- 调用链：`actuationTask → Sysid_Mode_Run() → sysid_target() → 预压叠加 → sysid_clamp_f → Dm_Send_Torque → Can_Bus_Transmit`
- 轮分支不变（轮测试要求四路腿零命令，故不加预压）；`torque_baseline` 不加预压（保纯零基线）

**数值换算（几何数值核验）**

用 `leg_solver.c` 的几何（`lu=0.21, lg=0.25`）在腿长 0.14~0.34 m 范围内扫 2669 个姿态，得到 `|∂l0/∂q| ≈ 0.138 m/rad`（最大 0.1435），因此：

```
单髋力矩(Nm) ≈ 沿腿力(N) × 0.138      →   43 N ≈ 6 Nm,  80 N ≈ 11 Nm
```

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 未动任何物理量（没有改 dm_sign/dm_zero/leg_off_phi0）；未改帧长
- `md/sysid-change-map.md` 里预先记录：预压力矩会与被激励电机叠加，**预压 + 激励 > 20 Nm 会削平**

**待台架**

- `SYSID_PRELOAD_*_N` 的实际取值（作者按列 17/19 的腿长迭代，目标 0.22~0.25 m）
- 预压符号方向（负是否真的是收腿）
- 20 Nm 限幅是否合适（预压是持续力矩，激励是短脉冲）

---

## 变更 44 · 预压调到 6Nm + 预压分量/削平标志上 VOFA

**背景**：作者台架反馈 `-22 N`（≈3 Nm/髋）"力太小"，要求调大；并要求把预压相关数据放进 VOFA 便于检测。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_mode.c` | ①`SYSID_PRELOAD_L_N/R_N`：-22 → -43 → 作者再调为 **-34**（≈4.7 Nm/髋）；②新增静态 `sysid_pre[4]` 保存本周期预压分量，与激励分开计算后再相加限幅；③帧填充时把预压分量写入 `snap.preload[]`，并统计被限幅的电机数 `snap.clamp_cnt` |
| `imcalib/Sysid/sysid_log.h` | `sysid_snap_t` 加 `preload[4]` / `clamp_cnt`；注释补"kind=1 行复用列 23~27" |
| `imcalib/Sysid/sysid_log.c` | `assemble_frame()`：kind=1 行把 23~27 写成预压分量与削平计数（kind=3/5 行 23~28 含义不变） |
| `md/sysid/sysid-delivery.md` | 新增 §1.3.3 腿行列 23~27 复用说明 |

**输入 / 输出 / 调用链**

- 输入：`SYSID_PRELOAD_*_N`、`leg_l/leg_r.output.leg_jac[0][*]`
- 计算：`sysid_pre[i] = 预压 × leg_jac[0][k]` → `tau_cmd[i] = clamp(tau_cmd[i] + sysid_pre[i], ±20)`
- 输出（VOFA 31 列，腿行）：列 5~8 = 净力矩，**列 23~26 = 四个髋的预压分量 Nm，列 27 = 削平电机数**，列 17~20 = 腿长/腿摆角
- 调用链：`Sysid_Mode_Run()` → 腿分支（预压+激励）→ 快照（`snap.preload/clamp_cnt`）→ `Sysid_Log_Push` → `commTask` → `Sysid_Log_Send_Pump` → `assemble_frame()` 按 kind 分支写列 23~27 → DMA → VOFA
- 导出侧：`tools/sysid_export.py` 的腿 CSV 只读列 3/4/5~8/17~20，**不受影响**（无需改工具）

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 未动任何物理量；帧长仍 31 列 / 128 B
- 削平判据：`|净力矩| >= 20 Nm` 即计入 `clamp_cnt`

**待台架**

- -43 N（≈6 Nm/髋）是否合适（作者按列 17/19 腿长继续迭代）
- 列 27 是否出现 >0（出现即说明预压+激励超限，需减幅或减预压）

---

## 变更 45 · 预压改为遥控滚轮实时可调 + 预压力值上 VOFA

**背景**：作者台架反馈"只有摆角、没有伸缩，摆角还很小"（-34 N ≈ 4.7 Nm/髋 仍压不动腿，且改一次值就要重烧一次）。为了不靠"改数-重烧"试凑，把预压做成**运行时可用遥控滚轮连续调节**，并把当前的沿腿力数值直接送上 VOFA。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_mode.c` | ①新增 `#include "dr16.h"`；②新增宏 `SYSID_PRELOAD_TUNE`（1=滚轮实时调，0=用固定宏）与 `SYSID_PRELOAD_TUNE_N`（滚轮到底 = 150 N）；③`Sysid_Mode_Run()` 顶部每个周期读一次滚轮并算出 `pre_n`（`-[0..150] N`，双向）；④腿分支用 `pre_n` 统一驱动左右腿；⑤快照填 `preload_n` |
| `imcalib/Sysid/sysid_log.h` | `sysid_snap_t` 加 `float preload_n;`；注释补列 28 |
| `imcalib/Sysid/sysid_log.c` | `assemble_frame()`：kind=1 行的列 28 写 `preload_n` |
| `md/sysid/sysid-delivery.md` | §1.3.3 加列 28；§4.3 加"实时调节模式"用法 |

**输入 / 输出 / 调用链**

- 输入：`DR16_Snapshot().wheel`（遥控滚轮原始值，±660）→ `DR16_Deadline(raw, 20)` 去死区 → 限幅 → 归一化 `[-1, 1]`
- 计算：`pre_n = -150 N × 归一化值`（正 = 伸腿，负 = 收腿） → `sysid_pre[F/B] = pre_n × leg_jac[0][*]`
- 输出（VOFA 腿行）：列 28 = 当前预压沿腿力 N；列 23~26 = 四个髋的预压分量 Nm；列 27 = 削平计数
- 调用链：`actuationTask → Sysid_Mode_Run() → DR16_Snapshot → pre_n → 腿分支预压 → 快照 → 发送泵 → VOFA`
- 反向固化：读列 28 的值 → 填回 `SYSID_PRELOAD_L_N/R_N` → `SYSID_PRELOAD_TUNE` 改 0 → 重新编译

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 未动任何物理量；帧长仍 31 列；滚轮只在测试模式的腿用例里被读取，不影响其他模式（`task_policy.c` 的 height_cmd 只在非测试模式用）
- 轮用例、baseline 用例不加预压（`preload_n` 记 0）

**待台架**

- 滚轮转到哪个方向是"收腿"（双向可试，不会做反）
- 定下来的预压力值需要作者确认后固化
- 若滚轮到底（150 N ≈ 21 Nm/髋）仍压不动腿，说明不是力不够，而是**腿正顶在机械限位上/解算姿态接近奇异**，要换思路（机械调整或改激励方式）

---

## 变更 46 · 预压固化 6 Nm/髋（作者拍板，关闭滚轮实时模式）

**背景**：变更 45 的滚轮实时调节已用于台架定值；作者决定"按 6 Nm 来"，并指出换算误差（雅可比随姿态变化）。

**改动**（`imcalib/Sysid/sysid_mode.c` 文件顶部）

| 宏 | 原值 | 新值 |
| --- | --- | --- |
| `SYSID_PRELOAD_L_N` / `SYSID_PRELOAD_R_N` | -34 | -43（6 Nm）→ -51（7 Nm）→ -58（8 Nm）→ **-54（7.5 Nm，作者最终定值）** |
| `SYSID_PRELOAD_TUNE` | 1（滚轮实时调） | **0（用固定宏）** |
| 换算注释 | 补一句"雅可比随姿态变，实际力矩以帧列 23~26 为准" | — |

`SYSID_PRELOAD_TUNE_N`（滚轮到底 = 150 N）保留，随时可把 `SYSID_PRELOAD_TUNE` 改回 1 再用滚轮试。

**输入 / 输出 / 调用链**

- 输入：`SYSID_PRELOAD_L_N/R_N = -43 N`
- 计算：`sysid_pre[F/B] = -43 × leg_jac[0][*]`（左右腿同值）→ 与激励相加 → ±20 Nm 限幅 → `Dm_Send_Torque()`
- 输出：帧列 23~26 = 四个髋的**实际**预压力矩 Nm（随姿态的雅可比浮动）→ 作者可直接读数核对是否 ≈ ±6 Nm
- 滚轮在 TUNE=0 时不再被读取（`Dial` 分支编译掉）

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 未动任何物理量；帧长仍 31 列

**待台架**

- 列 23~26 的实际值是否 ≈ −6 Nm（若非，说明该姿态下雅可比与平均 0.138 差别较大，需按实测值反推沿腿力）
- 6 Nm 能否把腿压到 0.22~0.25 m；若不能，优先用机械方式把腿放到中段

---

## 变更 47 · 两条腿同步激励（作者要求"两支腿一起测试"）

**背景**：原表每次只激励一条腿的一个电机（target 0~3 依次），对侧腿只吃预压。作者要求两条腿一起测。改动是**对称激励**：激励某一路时，对侧腿的对应电机给同一个逻辑力矩值（`0↔2`、`1↔3`）。

**为什么是对的**：左右电机在驱动边界已按 `dm_sign` 处理，两条腿在解算里用的是同一套镜像后的坐标，所以**同一个逻辑力矩值 = 镜像同向的物理动作** ✓ 两腿受力对称，机体只受竖直合力、不产生偏转。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_mode.c` | 新增宏 `SYSID_EXCITE_BOTH`（默认 **1**）；腿分支激励赋值后加一行 `tau_cmd[target ^ 2] = tau_cmd[target]`；文件顶部加注释 |
| `md/sysid/sysid-delivery.md` | §4.1 `torque_step` 力矩说明标注"两条腿的对应电机同步" |

**输入 / 输出 / 调用链**

- 输入：`run->target`（0=前左 1=后左 2=前右 3=后右）、`sysid_target()` 的激励值、`SYSID_EXCITE_BOTH`
- 输出：`tau_cmd[target] = 激励`，`tau_cmd[target ^ 2] = 同值` → 再叠加预压 → ±20 Nm 限幅 → `Dm_Send_Torque()`
- 帧记录不变：列 5~8 = 四路净力矩 → 训练端能看到"两路同值"的实际下发值 ✓
- 关掉方式：`SYSID_EXCITE_BOTH = 0` 即回到单腿激励

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 未动任何物理量；帧长仍 31 列；轮用例不受影响（只改腿分支）

**待台架**

- 对称激励下机体是否稳定（两腿同向发力 → 竖直合力；若悬吊有弹性会上下晃）
- 与单腿激励的数据质量对比（单腿激励通道分离更干净；双腿同时可以一次拿两倍数据）

---

## 变更 48 · 帧扩到 33 列：补输入输出时间戳 + 腿解算导数（作者要求"所有测试数据都要有"）

**背景**：作者要求 VOFA 上有全部测试相关数据（输入输出时间戳、各电机输出力矩、解算得到的腿部数据）。原 31 列缺 **DM 反馈到达时刻** 和 **腿长/腿摆角速度**（交接文档要求回放比对"值 + 导数"）。同时把 4 列预压分量挤出帧（净力矩列 5~8 已含预压，`preload_n` 列 30 保留总量，够用）。

**新布局（33 列 / 136 B / 66 kB/s，占 921600 的 71.6%）**

| 列 | 内容 | 变化 |
| --- | --- | --- |
| 0~22 | kind / seq / phase / t_cmd / 四路力矩 / 四路髋角 / 四路角速度 / 腿长摆角 L,R / t_rx_whl | 不变 |
| **23/24** | **DM 反馈最近接收时刻 hi/lo** | 新增 |
| **25/26** | **左腿长速度 / 左腿摆角速度** | 新增 |
| **27/28** | **右腿长速度 / 右腿摆角速度** | 新增 |
| **29** | 削平计数 clamp_cnt | 原位（原 27） |
| **30** | 预压沿腿力 preload_n | 原位（原 28） |
| 31/32 | 轮丢帧 / 腿丢帧累计 | 原 29/30 |

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_log.h` | `SYSID_LOG_FRAME_N` 31→33；注释块重写；结构体去掉 `preload[4]`，新增 `t_dm_rx_ns` / `d_leg[2]` / `d_pitch[2]` |
| `imcalib/Sysid/sysid_log.c` | `assemble_frame()` 尾部重排（kind=1 用 23~30，kind=3/5 保留轮数据） |
| `imcalib/Sysid/sysid_mode.c` | `sysid_fill_fb()` 增加：四个 DM 反馈 `rx_ns` 取最新；四个解算导数；快照填值改为不再写 `preload[i]` |
| `tools/sysid_export.py` | `FRAME_FLOATS` 31→33（二进制解析按帧尾定位，长度必须同步） |
| `md/sysid/sysid-delivery.md` | §1.1/§1.2/§1.3.3 同步到 33 列 |

**输入 / 输出 / 调用链**

- 输入：`dm_motor_feedback[i].rx_ns`、`leg_l/r.output.d_virtual_leg_length`、`d_virtual_leg_angle`
- 输出：VOFA 帧 23~30 列；导出 CSV 的列名与顺序**不变**（腿 CSV 只取 3/4/5~8/17~20），无需改训练端契约
- 调用链：`Leg_State_Update()（commTask）→ leg_solver 算出 d_*` → `Sysid_Mode_Run() → sysid_fill_fb()` → 快照 → 发送泵 → `assemble_frame()` → DMA

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 导出自测：`py tools/sysid_export.py --selftest` 全项通过
- 带宽：136 B × 500 Hz = 68 kB/s（921600 的 **73.8%**，含帧尾）——比原来 128 B 高 6 个百分点，仍是安全范围

**待台架**

- 33 列下 VOFA+ 的通道数要改成 33（否则显示会错位）
- 旧窗口的通道名建议按新表重命名

---

## 变更 49 · 预压回到 8 Nm + 新增《VOFA 数据对照表》

**背景**：作者要求把预压给回 8 Nm；并指出 VOFA 列太多显得乱，需要一份给训练端看的完整对照文档。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_mode.c` | `SYSID_PRELOAD_L_N/R_N`：-54（7.5 Nm）→ **-58（8 Nm，作者最终值）** |
| `md/sysid/vofa-channel-map.md` | **新增**：33 列逐列对照（列号/名称/单位/来源），按行类型的差异、标记/心跳行、时间戳还原、CSV 字段来源对照、快速自查表、"看起来不对"的常见解释 |
| `md/sysid/sysid-delivery.md` | §1.2 的长表改为"快速索引 + 指向新文档"，避免两处维护 |
| `md/AGENTS.md` | 文件树补 `vofa-channel-map.md` |

**输入 / 输出 / 调用链**

- 预压：同变更 43~46，值改回 -58 N（≈8 Nm/髋），帧列 5~8 记净力矩、列 30 记预压沿腿力
- 文档：`vofa-channel-map.md` 是列定义的**单一出处**，`sysid-delivery.md` §1.2 与 `imcalib/Sysid/sysid_log.h` 注释块必须与它保持一致（改列必须同步这三处 + `tools/sysid_export.py` 的 `FRAME_FLOATS`）

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 新文档内容与 `sysid_log.h` 注释块逐列核对一致

**待台架**

- 8 Nm 预压能否把腿压到 0.22~0.30 m（作者观察列 17/19）

---

## 变更 50 · 发送余量 + 串口卡死自恢复（作者反馈"偶尔 VOFA 卡住没数据"）

**背景**：作者反馈 VOFA 偶尔卡住没数据。查发送链路：帧 136 B × 500 Hz = 68 kB/s，占 921600 的 **73.8%**，而单帧在线上要 1.476 ms、发送泵每 2 ms 才被调一次 → 只剩 0.52 ms 余量，受 FreeRTOS 1 ms tick 抖动影响会出现跳周期 → 环形缓冲堆积丢帧；而"完全卡住"最可能是 **HAL 发送状态机卡死**（丢一次 TC 中断，`gState` 永远 BUSY，泵会永远提前返回）。**不是波特率问题**（波特率不匹配会乱码/列错位，不会"偶尔停一下"）。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_log.h` | 新增 `SYSID_TX_DIV`（默认 **4** = 250 Hz）与 `SYSID_TX_STALL_MS`（默认 50 ms）；extern `sysid_log_stall_cnt` |
| `imcalib/Sysid/sysid_log.c` | ①分频改用 `SYSID_TX_DIV`；②发送前加**卡死看门狗**：连续忙 ≥50 ms 就 `HAL_UART_AbortTransmit()` 强制复位状态机并计数；③新增 `sysid_log_stall_cnt` |
| `imcalib/Sysid/sysid_mode.c` | 心跳行新增列 13 = 串口卡死自恢复次数 |
| `md/sysid/vofa-channel-map.md` | 心跳行列 13 含义；§1.1 发送频率说明 |
| `md/sysid/sysid-delivery.md` | §1.1 发送频率 500 → 250 Hz |

**输入 / 输出 / 调用链**

- 输入：`send_div`、`VOFA_UART->gState`、`HAL_GetTick()`
- 输出：常态 250 Hz 发送（占链路 37%）；卡死超时则 abort 并恢复发送；计数进心跳行列 11（忙跳过）/列 13（卡死恢复）
- 调用链：`commTask → Sysid_Log_Send_Pump() → gState 检查/看门狗 → HAL_UART_AbortTransmit(超时) → ring 取帧 → assemble_frame → Clean_Tx → HAL_UART_Transmit_DMA`
- 250 Hz 仍高于交接契约的 ≥200 Hz 下限

**核对**

- 编译：`-DSYSID_ENABLE=1` 0 fail / 0 warn
- 未动任何物理量；帧长仍 33 列

**待台架**

- 250 Hz 下 VOFA 是否稳定（稳定后可把 `SYSID_TX_DIV` 改回 2 试 500 Hz）
- 心跳列 13 是否出现非 0（出现即说明确实发生过 HAL 状态卡死）
- VOFA+ 侧的"记录到文件"功能本身也可能拖慢接收，建议卡顿时先关记录试

---

## 变更 51 · 测试方式改为「位置扫描」（作者定：real2sim 位置跟踪，而非直接力矩）

**背景**：作者/训练端确认单关节开环力矩辨识意义不大（参数不准），轮腿的 gap 主要来自并联耦合。改为 **real2sim**：真机驱动关节跟踪预设轨迹并录包，MuJoCo 用同一套控制律跟踪同一条轨迹，比对角度曲线后调仿真参数。因此下位机要提供的是**位置跟踪 + 记录（指令/实测/力矩）**，而不是直接下发力矩。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/Sysid/sysid_mode.c` | ①新增测试方式开关 `SYSID_MODE`（`SYSID_MODE_POSE`=0 位置扫描，默认；`SYSID_MODE_TORQUE`=1 原力矩激励）；②位置扫描参数 `SYSID_POSE_RAMP_S`(0.5s)/`SYSID_POSE_HOLD_S`(2s)/`SYSID_POSE_KP`(25)/`SYSID_POSE_KD`(300)；③新增模板 `TPL_POSE` 与姿态表 `sysid_pose_runs[]`（大腿角 {-0.15,0,+0.15} × 虚拟小腿角 {2.50,2.80,3.10} 共 9 个姿态）；④腿分支新增位置扫描实现：线性斜坡到目标 → 组装 RL 动作 → 调 **`RL_Torque_Compute()`**（复用 RL 同一条力矩链路）→ 下发；⑤`Sysid_Mode_Init()` 里初始化虚拟关节 PD（只保留腿的位置环，轮增益归零）；⑥力矩模式与预压代码全部用 `#if SYSID_MODE` 保留 |
| `imcalib/Sysid/sysid_log.h` | 帧 33 → **35 列**（144 B）；新增列 33/34 = `thigh_tgt` / `shank_tgt`；结构体加 `pose_tgt[2]` |
| `imcalib/Sysid/sysid_log.c` | `assemble_frame()` 写列 33/34 |
| `tools/sysid_export.py` | `FRAME_FLOATS` 33 → 35 |
| `md/sysid/controller-spec-for-mujoco.md` | **新增**（子代理写）：控制律复刻说明书——PD 精确离散形式（D 不除 dt）、增益/周期/限幅/环绕、虚拟关节→电机映射、MIT 帧 kp=kd=0 证据、轨迹格式、气弹簧与摩擦两个坑 |

**位置扫描的控制链**

- 目标：`(大腿角, 虚拟小腿角)`，线性斜坡 0.5 s 从**上一姿态目标**滑到本姿态目标，再保持 2 s
- 控制器：`τ_i = kp·wrap180(q_des−q) + kd·(e[k]−e[k−1])`，kp=25、kd=300（等效阻尼 0.6 Nm·s/rad），500 Hz
- 映射：`τ_前髋 = τ_大腿 + τ_小腿×vshank_jac[1]`、`τ_后髋 = τ_小腿×vshank_jac[0]`，电机侧 kp=kd=0（纯力矩执行），限幅 20 Nm
- 记录：帧列 33/34 = 当前目标角（指令），列 9~12 = 实测髋角，列 5~8 = 实际下发力矩

**核对**

- 编译三种配置全部 0 fail / 0 warn：位置扫描（默认）、`-DSYSID_MODE=1`（力矩）、`-DSYSID_ENABLE=0`
- 导出工具自测通过
- 未动任何物理量；力矩模式与预压机制完整保留（改 `SYSID_MODE` 即可回退）

**待台架**

- `SYSID_POSE_KP/KD` 是否合适（kp=25 偏软，静差可能 0.2~0.4 rad；跟踪不好就加 kp，kd≈kp×12）
- 9 个姿态是否都可到达（撞限位的姿态数据判无效）
- 气弹簧造成的系统偏置需要训练端在仿真里等效加入，否则角度曲线不可能重合

---

## 变更 52 · 加「手动 PD 测试模式」+ 姿态表只跑 2 遍 + 降增益

**背景**：作者要求：先把 kp/kd 降下来；先用**手动**方式测 PD 闭环效果；测试次数改为 **2 次**，不要一直循环。

**改动**（`imcalib/Sysid/sysid_mode.c`）

| 项 | 原 | 新 |
| --- | --- | --- |
| 测试方式 | 位置扫描 / 力矩 | 新增第三种 **`SYSID_MODE_MANUAL`（默认）**：目标角由遥控给，用来手测闭环 |
| `SYSID_POSE_KP` / `KD` | 25 / 300 | **10 / 120**（等效阻尼 0.24 Nm·s/rad） |
| 姿态表遍数 | 一直循环 | **`SYSID_LOOP_CNT = 2`**：整表跑 2 遍后自动停机（状态码 **5 = 表跑完**），改 0 可恢复一直循环 |
| 手动模式摇杆 | — | **左摇杆上下（ch3）→ 大腿角 ±0.6 rad；滚轮（wheel）→ 虚拟小腿角 ±0.6 rad**，零点 = 进入测试模式时的实测姿态 |
| 手动模式表 | — | 单个 3600 s 的长 run（不进姿态表、不循环、不会被遍数停机） |

**输入 / 输出 / 调用链**

- 手动：`DR16_Snapshot()` → `sysid_stick()`（去死区+限幅→[-1,1]）→ `thigh_t = 进入姿态 + 摇杆×范围`、`shank_t = 进入姿态 + 滚轮×范围` → 组装 RL 动作 → `RL_Torque_Compute()` → 4 路腿力矩 → CAN
- 记录不变：列 33/34 = 当前目标角（手动模式也记，便于回看），列 9~12 实测角，列 5~8 实际力矩
- 停机：`loop_cnt` 达到 `SYSID_LOOP_CNT` → 只发零力矩 + 心跳（状态码 5）

**核对**

- 四种编译配置全部 0 fail / 0 warn：手动 PD（默认）、位置扫描（`-DSYSID_MODE=0`）、力矩（`-DSYSID_MODE=1`）、关闭测试（`-DSYSID_ENABLE=0`）

**待台架**

- 手动模式下 kp=10 是否偏软（静差大 → 加 kp，kd≈kp×12）
- 手动模式的角范围 ±0.6 rad 是否合适（撞限位就改 `SYSID_MAN_*_RANGE`）

---

## 变更 53 · 去掉 D 项 + 删除新增的「手动 PD 模式」（作者：手动测试用原来的左上模式即可）

**背景**：作者指出 ①D 项去掉；②"手动模式不就是原来的 RL 测试（左拨杆上位）吗，为什么要新增"。核查确认：**左上（不动右拨杆）= 正常手动模式**，其链路是 `task_policy.c` 的 `Manual_Lock_On_Enable()`（使能瞬间锁存当前姿态为 `base_action`）+ 摇杆偏移 → `RL_Torque_Compute()`（同一套虚拟关节 PD + 雅可比映射 + 限幅），**没有策略在环**（`task_policy.c` 的 `ctrl_task_body()` 不调用策略推理）。也就是说它已经是"零点=使能姿态、摇杆给目标偏移"的手动 PD 测试，变更 52 新增的那套是重复实现。

**改动**（`imcalib/Sysid/sysid_mode.c`，全部为删除/降值）

| 项 | 原 | 新 |
| --- | --- | --- |
| `SYSID_POSE_KD` | 120 | **0**（D 项去掉） |
| `SYSID_MODE_MANUAL`（测试方式第 3 种） | 有，且为默认 | **删除**，默认回到 `SYSID_MODE_POSE` |
| `SYSID_MAN_THIGH_RANGE` / `SYSID_MAN_SHANK_RANGE` / `SYSID_MAN_DEADBAND` | 有 | **删除** |
| `sysid_stick()` 辅助函数 | 有 | **删除** |
| 手动长 run 表 `sysid_manual_runs[]` | 有 | **删除** |
| 腿分支里的手动目标计算 | 有 | **删除**，只保留位置扫描的斜坡逻辑 |

**现在的两套测试方式**

- `SYSID_MODE_POSE = 0`（默认）：位置扫描，自动跑姿态表 `SYSID_LOOP_CNT = 2` 遍后停机（状态码 5）
- `SYSID_MODE_TORQUE = 1`：力矩激励（原方案）
- 手动 PD 手测：**不进测试模式**，左拨杆上位即可（增益用 RL 模型参数表里的值，不在本模块里）

**核对**

- 编译：位置扫描（默认）/ 力矩 / 关闭测试，三种配置全部 `0 fail / 0 warn`
- `grep` 确认 `MANUAL` / `sysid_stick` / `SYSID_MAN_*` 无残留

**待台架**

- 位置扫描用 `SYSID_POSE_KP = 10`、`KD = 0`：D 去掉后若出现摆动/振荡，需要降 kp；采集前请把最终值固定并告知训练端（手动模式用的是另一套增益，不要混）
- 手动模式（左上）的增益是 RL 模型参数表的 `p_gains`（大腿 3.5 / 小腿 15.5，D=0），比位置扫描软

---

## 变更 54 · 去掉 VOFA_LAYOUT，只留一套 VOFA 输出（作者：不要多套布局）

**背景**：作者要求去掉 `VOFA_LAYOUT` 那套机制与多套 VOFA 输出，通道直接改成测试相关数据，后续只改参数即可。另外大腿角区间改为 45°~145°。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/user-lib/Vofa_send.h` | **删除 `VOFA_LAYOUT` 宏**（保留 `VOFA_MAX_CH`、`VOFA_PORT`） |
| `imcalib/task/task_comm.c` | `Robot_Control_Send_Vofa()` 删除 `#if VOFA_LAYOUT == 1`（虚拟关节 PID 视图）整个分支，只保留测试相关通道那一套（力矩 / 髋角 / 腿长摆角 / 轮 / 时间戳）；注释改为"通道内容直接改这个函数" |
| `imcalib/Sysid/sysid_mode.c` | 姿态表大腿角改为 **45° / 95° / 145°**（0.7854 / 1.6581 / 2.5307 rad）；`SYSID_POSE_RAMP_S` 0.5 → **1.5 s**（跨度大使斜坡给足时间） |

**现在的 VOFA 输出只有两种（各自一条路径，不再有布局开关）**

- **正常模式**：32 路 FireWater 调试帧（测试相关通道），改通道就改 `task_comm.c` 的 `Robot_Control_Send_Vofa()`
- **测试模式**：35 列 JustFloat sysid 帧（列定义见 `md/sysid/vofa-channel-map.md`），改列就改 `Sysid/sysid_log.h` + `sysid_log.c` + `tools/sysid_export.py` 的 `FRAME_FLOATS`

**核对**

- 编译：`-DSYSID_ENABLE=1` 与 `-DSYSID_ENABLE=0` 均 105 文件 0 fail / 0 warn
- `grep VOFA_LAYOUT` 在 `imcalib/` 下已无残留

**待台架**

- 大腿角 45°~145° 的**坐标约定核对**：固件里 `thigh_angle = wrap(前髋电机角 + π)`，即 VOFA 列 9 读数 + π；先用列 33（目标）与列 9 对照确认，不一致就按实测改表

---

## 变更 55 · 斜坡改成标准线性插值 + 小腿角区间改 2.3~3.0

**改动**（`imcalib/Sysid/sysid_mode.c`）

| 项 | 原 | 新 |
| --- | --- | --- |
| 姿态表虚拟小腿角 | 2.50 / 2.80 / 3.10 | **2.30 / 2.65 / 3.00**（大腿角仍 45°/95°/145°） |
| 斜坡实现 | 每周期用 `sysid_pose_prev` 做插值（prev 每周期被改写成中间值 → 形状不是直线，前慢后快） | 新增 `sysid_pose_from[2]`：**每段开头锁存起点**（= 上一段目标；首段 = 进入时实测角），然后 `目标 = 起点 + (本段目标 − 起点) × min(1, t/1.5s)` —— **标准线性斜坡**，到 1.5 s 精确等于目标 |
| 段尾的"记住本段目标" | 有 | 删除（起点改在段首锁存，`sysid_pose_prev` 只在段首写入本段目标） |

**验证设计依据**：帧列 33/34 记录的是**每周期实际的斜坡值**，训练端按记录值回放，所以斜坡形状不影响比对；但改成标准直线后，目标序列可以由姿态表 + 起点完整复现。

**核对**：`-DSYSID_ENABLE=1`（位置扫描）与 `-DSYSID_MODE=1`（力矩）均 105 文件 0 fail / 0 warn。

---

## 变更 56 · 斜坡函数化：lowpass → simple-function（新增 Ramp_*）

**背景**：作者要求斜坡不要在测试模块里自己做，而是做成**可复用函数**，放在原 `lowpass` 文件里，并把文件改名为 `simple-function`，用正确的斜坡函数形式。

**改动**

| 文件 | 变化 |
| --- | --- |
| `imcalib/user-lib/lowpass.c/h` → **`simple-function.c/h`** | 文件改名（低通 API `Lowpass_*` 不变） |
| `simple-function.h/.c` | 新增斜坡类型与函数：`ramp_t {out, rate}`、`Ramp_Init(r, rate)`、`Ramp_Update(r, target, dt)`、`Ramp_Reset(r, value)`；**形式 = 斜率限制**：每周期最多 `rate×dt`，到目标直接等于目标（不超调、目标中途变化也正确） |
| `imcalib/Algorithm/lqr_balance.h` | `#include "lowpass.h"` → `"simple-function.h"` |
| `MDK-ARM/CtrBoard-H7_ALL.uvprojx` | 工程文件项 `lowpass.c` → `simple-function.c` |
| `imcalib/Sysid/sysid_mode.c` | 删除自写的插值斜坡（`sysid_pose_from`/`sysid_pose_prev`）；改为 `thigh_t = Ramp_Update(&sysid_ramp_th, run->amplitude, 0.002f)`、`shank_t` 同理；首次进入时用 `Ramp_Reset()` 对齐实测角；`SYSID_POSE_RAMP_S`（时间）→ `SYSID_POSE_RAMP_RATE = 1.2f`（rad/s） |
| `md/AGENTS.md` | 文件树同步为新文件名 |

**输入 / 输出 / 调用链**

- 输入：目标角 `run->amplitude` / `run->amp2`、`dt = 0.002 s`、`rate = 1.2 rad/s`
- 输出：限斜率后的目标角 → 组装 RL 动作 → `RL_Torque_Compute()` → 4 路腿力矩；帧列 33/34 记录该斜坡值
- 调用链：`Sysid_Mode_Run() → Ramp_Update()（simple-function.c）→ act_buf → RL_Torque_Compute()`

**核对**：三种配置（位置扫描默认 / 力矩 `-DSYSID_MODE=1` / 关闭 `-DSYSID_ENABLE=0`）均 105 文件 0 fail / 0 warn；`grep lowpass.h`、`grep sysid_pose_from` 无残留。

---

## 变更 57 · 重构 `Robot_Control_Send_Vofa()`（纯整理，行为不变）

**动机**：函数内联组装 32 通道、逻辑分组不清晰，作者反馈"太乱了"。

**改动**（`imcalib/task/task_comm.c`，只重构，不改任何通道含义）

| 变化 | 说明 |
| --- | --- |
| 拆出 `Vofa_Fill_Status()` | ch0~2：在线掩码 / 解算有效 / 策略号 |
| 拆出 `Vofa_Fill_Leg()` | ch3~14：腿指令力矩 / 零点后角 / 腿长 / 腿摆角 |
| 拆出 `Vofa_Fill_Wheel()` | ch15~23：轮指令电流 / 转速 / 编码器 / 实际电流 / 温度 |
| 拆出 `Vofa_Fill_BusDiag()` | ch24~31：总线 TX 间隔/条数/丢帧 + RX 间隔/时刻拆分 |
| 主函数 | 分频 + SYSID 早退 + 依次调 4 个填充函数 + `Vofa_Send(dbg, 32u)` |
| 通道表注释 | 主函数上方新增 32 路逐列说明（列号 / 含义 / 单位 / 来源） |
| `static` 局部变量 | `rx_prev` / `tx_prev_whl` / `tx_prev_leg` 从主函数搬到 `Vofa_Fill_BusDiag()` 内部，作用域更小 |

**输入 / 输出 / 调用链**：与变更 54 完全一致，纯重构，无行为变化。
- 输入源：`motor_state` / `rl_control` / `leg_l/r` / `dji_motor_feedback` / `imu_state` / `DR16_Online()` / `Can_Bus_Tx_Pop()` / `Can_Bus_Tx_Drop_Count()` / `machine->dji_sign/dji_bus/dm_bus`
- 输出：`Vofa_Send(dbg, 32u)` → 当前 `VOFA_PORT` 串口

**核对**：`-DSYSID_ENABLE=1`（默认）与 `-DSYSID_ENABLE=0` 均 105 文件 0 fail / 0 warn。

---

## 变更 58 · 切回小机器（`MACHINE_DEFAULT` → `MACHINE_ID_LOCAL`，作者：测小机器 LQR）

| 文件 | 改动 |
| --- | --- |
| `imcalib/user-lib/machine_config.h` | `MACHINE_DEFAULT`：`MACHINE_ID_CHUANLIANTUI` → `MACHINE_ID_LOCAL` |
| `imcalib/user-lib/machine_config.c` | 未改；`machine` 初值 `&machine_table[MACHINE_DEFAULT]` 自动指向小机器表 |

**为什么只改这一行**：机器相关的量（型号/减速比/限幅/极性/零点/总线/腿几何/腿长区间）全部在两份表里，切换机器只切指针。

**切换后实际生效参数（大机器 → 小机器）**

| 项 | 大机器 `chuanliantui` | 小机器 `local-m2006-j4310` |
| --- | --- | --- |
| 轮型号/减速比/限幅 | M3508 / 15.5 / ±4.8 N·m | **M2006** / 36.0 / ±1.8 N·m |
| 腿电机 (DM) | J8009P：±π / 45 rad/s / 54 N·m | **J4310**：±π / 30 rad/s / 10 N·m |
| 腿力矩限幅 | 20 N·m | 10 N·m |
| 腿总线 | FDCAN1 ×4 | **FDCAN1 左腿 + FDCAN3 右腿** |
| 轮总线 | FDCAN3 | **FDCAN2** |
| DM 零点 | {0.476998, 1.974491, …} | {-0.03, -0.04, -0.038, -0.023} |
| 腿几何 lu / lg | 0.21 / 0.25 | 0.13087 / 0.15240 |
| 腿长区间 | 0.14 ~ 0.34 | 0.10 ~ 0.20 |

**输入 / 输出 / 调用链**
- 定义：`machine_config.h` 的 `MACHINE_DEFAULT` → `machine_config.c` 的 `machine` 指针 → 全工程只读 `machine->`
- 消费者：`dm.c`（极性/零点/量程/限幅/总线）、`dji.c`（型号→`per_raw`、减速比、总线）、`leg_solver.c`（lu/lg/腿长区间/phi0）、`lqr_balance.c`（`leg_len_min/max` 参与 LQR 投入判定与腿长目标夹取）、`rl_torque.c`（限幅）、`task_comm.c`（VOFA 换算）
- 运行时切换口：`Machine_Select(id)`（`main.c` 上电调用 `Machine_Select(MACHINE_DEFAULT)`）

**核对**：armcc 全量 105 文件 0 fail / 0 warn（默认配置）。

**已确认不需要改的**：**FDCAN 波特率不用动**。`can_bus.c:91-92` 的发送模板是 `BitRateSwitch=FDCAN_BRS_OFF` + `FDFormat=FDCAN_CLASSIC_CAN`，发出的全是**经典 CAN 帧**，只走仲裁段（nominal）时序；三路 FDCAN 的 nominal 都是 `24 MHz / (3×8) = 1 Mbps`（HSE 24 MHz 直供 FDCAN）。FDCAN1 上那个 4 Mbps 的 DataPrescaler 只影响 FD 数据段，对经典帧**不起作用**。（待台架：若小机器 J4310 曾被达妙上位机改成非 1 Mbps，则以电机实际波特率为准。）

**待台架 / 未决**
- LQR K 表拟合域 0.13~0.23 m，小机器腿长区间 0.10~0.20 m：站姿腿长低于 0.13 m 时增益为外推值。
- `lqr_balance.c:9` 的 `LQR_WHEEL_R = 0.04f` 硬编码，未进配置表；小机器轮径若不等于 0.04 m，速度估计会成比例偏。
- 测试模式（左上 + 右中）的姿态表仍是按大机器几何标的，小机器上不要进。

---

## 变更 59 · 关测试开关 + FDCAN1 数据段改回原值 + VOFA 换成 LQR 观测帧（作者：开始测小机器 LQR）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Sysid/sysid_config.h` | `SYSID_ENABLE`：1 → **0**（测试代码整块不参与编译，策略仲裁回到 LQR / 手动两路） |
| `Core/Src/fdcan.c` | FDCAN1 `DataPrescaler` 1→3、`DataTimeSeg1` 4→5、`DataTimeSeg2` 1→2 |
| `CtrBoard-H7_ALL.ioc` | 同上三行（与 CubeMX 保持同源，重新生成不会变回 4 Mbps） |
| `imcalib/task/task_comm.c` | `Robot_Control_Send_Vofa()` 的 32 通道内容整块换成 LQR 观测；函数上方补通道表注释；删掉 `const pid_t *pid;` 与 `sysid_wheel_cmd_raw` 的 extern 引用 |

**为什么改 FDCAN1**：作者要求把大机器那次的改动还原。注：`can_bus.c:91-92` 发送模板是 `BRS_OFF + CLASSIC_CAN`，仲裁段 1 Mbps 不变，这 3 行只影响 FD 数据段（当前固件用不到）；改回去是为了与 CubeMX 配置、与两机器一致的原始状态对齐。

**VOFA 新帧（32 通道 / JustFloat / 200 Hz，`vofa_div < 5u` 分频不变）**

| 通道 | 含义 | 来源 |
| --- | --- | --- |
| ch0 | 在线掩码：IMU / 遥控 / 髋 4 / 轮 2 | `imu_state.online`、`DR16_Online()`、`motor_state.dm.online[]`、`motor_state.dji.online[]` |
| ch1 | 状态位：使能 / 跌倒 / 左腿有效 / 右腿有效 | `robot_state.*`、`leg_l/r.output.valid` |
| ch2 | 策略号 0 手动 / 1 LQR | `ctrl_strategy` |
| ch3~6 | 四髋位置（零点后 rad） | `motor_state.dm.pos_zero_rad[]` |
| ch7~10 | 左腿：大腿角 / 虚拟小腿角 / 虚拟腿摆角 / 腿长 | `leg_l.output.*` |
| ch11~14 | 右腿：同上 | `leg_r.output.*` |
| ch15~16 | 腿长目标（左/右 m） | `lqr_state.leg_len_tgt[]` |
| ch17~19 | 俯仰角 (rad) / 俯仰角速度 (rad/s) / 前进速度 (m/s) | `lqr_state.x[THB / DTHB / DS]` |
| ch20~23 | LQR 输出 (N·m)：左轮 / 右轮 / 左髋 / 右髋 | `lqr_state.u[WL / WR / BL / BR]` |
| ch24~25 | 轮转速 (rad/s) | `motor_state.dji.vel_rad_s[]` |
| ch26~27 | 轮实测电流 (A) | `dji_motor_feedback[].current_raw / 819.2` |
| ch28~31 | 髋力矩反馈 (N·m)：前左/后左/前右/后右 | `dm_motor_feedback[].trq_nm` |

**输入 / 输出 / 调用链**
- 输入：上面表里各来源（都在 `commTask` 之前由 `Dm_Parse/Dji_Parse/Motor_State_Update/Leg_State_Update` 刷好；LQR 的 `lqr_state` 由 `actuationTask` 的 LQR 分支刷新）
- 输出：`Vofa_Send(dbg, 32u)` → `VOFA_PORT`（当前 1 = USART1 @1152000）→ JustFloat
- 调用链：`comm_task_body()` → `Robot_Control_Send_Vofa()`；末尾两段 `Can_Bus_Tx_Pop` 清环保持不变
- `SYSID_ENABLE=0` 后：`task_actuation.c` 的 sysid 分支、`task_comm.c` 的 10 通道轮帧分支整块编译掉；`Sysid/*.c` 编成空单元

**核对**：默认（`SYSID_ENABLE=0`）与 `-DSYSID_ENABLE=1` 均 **105 文件 0 fail / 0 warn**。

**待台架**：小机器上先只看 ch7~14（手搬腿 → 大腿角/小腿角/摆角/腿长是否跟手、左右是否一致），确认后再进 LQR。

---

## 变更 60 · 小机器 LQR 批次 1：清测试残留、隔离时间戳、修复解算竞态与使能链

| 文件 | 改动 |
| --- | --- |
| `imcalib/task/task_comm.c` | VOFA 恢复 2 分频 500 Hz；删除正常链路 CAN 完成环清空与腿速度自检；两腿 `Leg_Solve()` 用调度器锁保护；接入 DM 使能看门狗、故障码和 ch1 bit4~7 使能位 |
| `imcalib/task/inc/robot_control.h` | 删除无人消费的 `leg_debug_history_t` |
| `imcalib/user-lib/dr16.c/h` | 删除过时的三项接收诊断计数 |
| `imcalib/user-lib/can_bus.c/h` | TX 完成登记、完成中断、环形缓冲与统计 API 全部限制在 `SYSID_ENABLE=1`；正常固件直接入发送 FIFO |
| `imcalib/user-lib/dm.c/h`、`dji.c` | RX 纳秒时间戳仅在 sysid 构建启用；新增 DM 使能/故障判定和每台独立 100 ms 使能看门狗 |
| `imcalib/Algorithm/leg_balance.c/h` | 新增 `Leg_Balance_Reset()`，只清四个 PID 的历史与输出，不改参数 |
| `imcalib/task/task_actuation.c` | LQR 投入锁存成功时复位辅助 PID |

**输入 / 输出 / 调用链**
- 正常通信：`comm_task_body()` → `Leg_State_Update()` → `vTaskSuspendAll()` → 两次 `Leg_Solve()` → `xTaskResumeAll()`；高优先级 `actuationTask` 不再读到求解中途的 `valid=0`。
- DM 状态：反馈字节 0 高 4 位 → `Dm_Parse().err_raw` → `Dm_Is_Enabled()` / `Dm_Has_Fault()`；故障码 `0x8~0xE` 进入 `FAULT_MOTOR`。
- DM 看门狗：`Robot_Enable_Update()` 在 `motor_enabled=1` 时调用 `Dm_Enable_Watchdog()`；仅对在线且 `err_raw=0` 的电机按各自计时每 100 ms 重发 `DM_CMD_ENABLE`。
- LQR 投入：`LQR_Enable_Latch()` 返回 1 → `Leg_Balance_Reset()` → 首个控制周期从清零的 PID 历史开始。
- VOFA：`commTask 1 kHz` → 2 分频 → `Vofa_Send(dbg, 32)` 500 Hz；ch1 bit4~7 依次表示左前、左后、右前、右后 DM 的 `err_raw==1`。
- sysid 时间戳：`SYSID_ENABLE=1` 时保留 `Can_Bus_Transmit_Tagged()` → TX 完成回调 → 环形缓冲 → `Sysid_Mode_Run()` 出队；关闭时不登记、不启用 TX complete 中断，DM/DJI RX 中断也不读 `Mono_Ns_Get()`。

**核对**
- 未改 `dm_sign` / `dji_sign`、零点、MIT 量程、镜像、腿几何和 `MACHINE_DEFAULT`。
- VOFA 32 路下标未移动，只扩展 ch1 高 4 位；帧长仍为 132 B，500 Hz 约 66 kB/s。
- Keil AC5 按 `build/CtrBoard-H7_ALL/compile_commands.json` 全量编译：默认配置与 `-DSYSID_ENABLE=1` 均为 **105 文件，0 fail / 0 warn**。

**待台架**
- 失能任一 J4310 后确认仍有反馈且 ch1 对应 bit 清零，100 ms 看门狗能重新使能。
- 人为触发 DM `0x8~0xE` 故障码，确认 `FAULT_MOTOR` 置位并停止输出。
- 反复进入 LQR，确认首拍辅助 PID 不再因旧历史产生冲击；竞态修复只完成代码与编译核对，实时行为待台架。

---

## 变更 61 · 小机器 LQR 批次 2：控制频率 500 Hz → 1 kHz

| 文件 | 改动 |
| --- | --- |
| `imcalib/task/inc/robot_control.h` | 新增统一控制周期 `CTRL_DT=0.001f` |
| `imcalib/task/task_actuation.c`、`imcalib/Algorithm/rl_torque.c`、`imcalib/Sysid/sysid_mode.c` | 删除分散的 `0.002f` / `OUTPUT_DT`，统一引用 `CTRL_DT` |
| `CtrBoard-H7_ALL.ioc`、`Core/Src/tim.c` | TIM6 Prescaler 549 → 274，Period 保持 999，对应 275 MHz / 275 / 1000 = 1 kHz |
| `Core/Src/main.c`、`imcalib/user-lib/mono_ns.c` | TIM6 与单调时钟注释同步为 1 kHz |
| `imcalib/Algorithm/leg_balance.h` | 按作者确认，腿长/防劈叉/横滚三个辅助 PID 的 KD 分别由 25000/250/50 改为 0，先使用纯 P，待台架单独标定 D |
| `imcalib/Sysid/sysid_mode.c`、`sysid_log.c/h` | sysid 单周期同步为 1 kHz；心跳改 250 tick 保持 250 ms；`SYSID_TX_DIV=4` 保持 1 kHz commTask → 250 Hz 发送 |

**输入 / 输出 / 调用链**
- 时钟：TIM6 275 MHz → `(Prescaler+1)=275` → `(Period+1)=1000` → 1 kHz 中断 → `ctrl_tick_sem_handle` → `actuationTask`。
- 时间步：`CTRL_DT` → LQR 目标/状态/腿部力控、RL 虚拟关节 PID、sysid 激励时序，所有控制计算使用同一个 1 ms 周期。
- 辅助 PID：腿长、防劈叉、横滚误差 → `pid_calc()`；本批 `kd=0`，D 输出恒为 0，KP 与前馈不变。
- sysid：`Sysid_Mode_Run()` 1 kHz 采样；心跳 `1000×250 tick=250 ms`；发送泵仍由 `commTask 1 kHz / SYSID_TX_DIV 4 = 250 Hz`。

**总线负载估算与核对**
- 小机器腿总线每 1 ms 约 2 发 2 收，经典 CAN 1 Mbps 估算负载约 **52%**。
- 小机器轮总线每 1 ms 约 1 发 2 收，经典 CAN 1 Mbps 估算负载约 **39%**。
- `SYSID_ENABLE=1` 时台架观察 `Can_Bus_Tx_Drop_Count()`；正常固件可临时观察 `HAL_FDCAN_GetTxFifoFreeLevel()`，确认发送 FIFO 不持续归零。
- `SYSID_TX_DIV` 针对通信发送泵，不跟随控制频率翻倍；本批核对后保留 4。

**核对**
- TIM6 `.ioc` 与生成代码数值一致；控制相关硬编码 `0.002f` 已从指定链路清除。
- 未改机器选择、腿长、轮径、极性、零点、量程、镜像和 IMU 轴。
- Keil AC5 全量编译：默认配置与 `-DSYSID_ENABLE=1` 均为 **105 文件，0 fail / 0 warn**。

**待台架**
- 示波器或任务计数确认 TIM6/actuationTask 实际为 1 kHz，并检查是否出现信号量积压。
- 观察三路 FDCAN 发送 FIFO、sysid drop 计数和电机在线状态，确认 1 kHz 下无掉帧。
- 三个辅助 PID 当前无 D 阻尼；落地前按既定分通道顺序低限幅验证，D 项后续只能依据台架数据单独恢复。

---

## 变更 62 · 小机器 LQR 批次 3：调试开关、限幅、K 表腿长域与轮径入表

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/lqr_balance.h/c` | 新增全局 `lqr_debug`；默认符号保持现状；轮/髋/腿长 PID 三个输出开关；轮/髋运行时限幅；K 表域 0.13~0.23 m |
| `imcalib/Algorithm/leg_balance.c` | 关闭轮通道时轮输出为 0；关闭髋通道时 `Tp=0`；关闭腿长 PID 时足端力只留固定前馈；PID 始终继续计算 |
| `imcalib/user-lib/machine_config.h/c` | `machine_cfg_t` 新增独立轮半径 `wheel_r`；大机器 0.04 m（占位待实测），小机器 0.03 m（Leg2_v1 建模值） |

**`lqr_debug` 默认值与行为**

| 字段 | 默认值 | 行为 |
| --- | ---: | --- |
| `vel_leg_comp_sign` | `-1.0f` | `wheel_vel + sign×d_virtual_leg_angle - omg_pitch`；默认与改前完全相同，`+1` 仅供台架 A/B |
| `wheel_enable` | `1` | 关闭只把最终 DJI 输出置 0 |
| `hip_enable` | `1` | 关闭只把左右 `Tp` 置 0 |
| `len_pid_enable` | `1` | 关闭后两腿 `F` 只保留 `LEG_BALANCE_F_FEEDFORWARD`，腿长与横滚 PID 输出都不下发 |
| `trq_max_wheel` | `machine->dji_trq_clamp` | 小机器默认 1.8 N·m，替代原 1.5 N·m 宏 |
| `trq_max_hip` | `5.0f` | 替代原 2.0 N·m 宏，低于 J4310 10 N·m 上限 |

**输入 / 输出 / 调用链**
- 调试器 Watch → `lqr_debug` → `LQR_State_Update()` 的腿摆速度补偿、`LQR_Control_Update()` 一级限幅、`Leg_Balance_Compute()` 通道门与二级限幅；未增加 VOFA 通道。
- 腿长有效域：`max(machine->leg_len_min, 0.13)` 到 `min(machine->leg_len_max, 0.23)` → `LQR_Enable_Latch()` 投入判定与 `LQR_Target_Update()` 目标夹取。小机器实际为 **0.13~0.20 m**。
- 轮速度：DJI 输出轴角速度 × `machine->wheel_r` → 轮心线速度；轮径是机器表独立字段，没有叠加进腿长。

**物理量变更确认（§0.1 单列）**
- 谁 / 何时：作者于 **2026-09-20** 明确确认小机器轮径从开源 `Leg2_v1` 查取，并确认速度补偿运行时 A/B 开关。
- 依据：`Leg2_v1/轮腿上交建模MATLAB/WBR_modeling.mlx` 的小机器参数组写明 `R_w_ac=0.03 m`；大机器活动参数组为 `0.04 m`。
- 实际写入：`machine_config.c` 大机器 `wheel_r=0.04f`（占位、待实测），小机器 `wheel_r=0.03f`；原 `lqr_balance.c` 固定宏 `0.04f` 删除。
- 符号：`vel_leg_comp_sign=-1.0f` 对应改前 `wheel_vel - d_virtual_leg_angle - omg_pitch`，默认行为不变；`+1.0f` 不作为当前物理结论，只允许台架比较。

**核对**
- `MACHINE_DEFAULT=MACHINE_ID_LOCAL` 不变；杆长、机器腿长表、极性、零点、MIT 量程、镜像、`+LEG_PI` 与 IMU 五个轴宏均未改。
- 开关关闭时 PID 仍更新，仅门控最终物理输出，重新开启不会因暂停计算产生额外历史跳变。
- Keil AC5 全量编译：默认配置与 `-DSYSID_ENABLE=1` 均为 **105 文件，0 fail / 0 warn**。

**待台架**
- 架空看 ch19，将 `vel_leg_comp_sign` 在 `-1/+1` 间切换，选择速度估计波动更小的一侧；确定后写死并删除字段。
- 小机器 K 表当前按 **0.04 m** 轮径生成，而机器表采用 **0.03 m**，轮通道尺度偏差约 25%；待用 `Leg2_v1` 的 `WBR_modeling.mlx` 以小机器参数重跑 K 表。
- 大机器 `wheel_r=0.04 m` 仍是占位值，必须实测后才能标定完成。
- `trq_max_hip=5.0 N·m`、各通道开关与 0.13~0.20 m 投入域均待按小机器台架顺序验证；IMU 轴本批未动。

---

## 变更 63 · 小机器 LQR 批次 4：主文档同步与 sysid 文档归档

| 文件 | 改动 |
| --- | --- |
| `md/AGENTS.md` | 文件树新增 `md/sysid/`；关键约束同步 1 kHz、机器表∩K 表域、VOFA 500 Hz；模块表改为 `simple-function` 并补 `lqr_debug` |
| `md/RL_OVERVIEW.md` | 控制频率改 1 kHz、100 Hz 推理改每 10 周期；删除旧 VOFA 通道复制表并指向 `VOFA_SEND.md`；时钟修为 550 MHz；D-Cache 状态改为已开启且 VOFA DMA 前 Clean |
| `md/LQR_PLAN.md` | 常量表同步运行时限幅、腿长域和机器轮径；频率清单标完成；新增 `lqr_debug` 用法；替换为小机器台架顺序；遗留项与双配置编译结果同步 |
| `md/IO_CHAINS.md` | DM `err_raw` 使能看门狗/故障链；机器配置表来源；VOFA 改为单一文档链接；LQR 链同步 1 kHz、轮径、腿长交集与调试门 |
| `md/VOFA_SEND.md`、`imcalib/user-lib/Vofa_send.h` | 按当前代码重写 32 路表，删除乱码；默认 `VOFA_PORT=1`、1152000、500 Hz 与 D-Cache Clean 同步；sysid 只保留目录指针 |
| `md/DBUS.md`、`md/UART_IDLE_DMA.md` | 通读保留；修正 DR16 接受 `len>=18`、UART9=DR16、UART7=HI229 的现行拓扑 |
| `md/sysid/` | 六份大机器测试文档移入该目录，文件头统一标注“大机器测试专用，`SYSID_ENABLE=1` 时生效”，交叉路径同步 |
| `md/hip-test-vofa.md` | 删除；旧 32 通道测试布局已被变更 59 替代，需要时从提交 `87c6628` 恢复 |

**输入 / 输出 / 调用链**
- 代码与配置作为输入 → `AGENTS / RL_OVERVIEW / LQR_PLAN / IO_CHAINS / VOFA_SEND` 分别提供规则、架构、控制器、I/O 与观测通道的单一入口。
- 正常 VOFA：`task_comm.c` 当前 32 路 → `VOFA_SEND.md`；不再在多个总览文档复制易过时的通道表。
- 大机器 sysid：`SYSID_ENABLE=1` → `imcalib/Sysid/` → `md/sysid/`；正常小机器 LQR 文档与大机器测试材料分目录。
- 文档移动后，仓库内旧 `md/<sysid文件>` 路径统一改为 `md/sysid/<文件>`；Markdown 相对链接检查无死链。

**核对**
- `CLAUDE.md` 与记忆文件未改；`md/sysid-change-map.md` 保留在 `md/` 根目录且继续作为全工程账本。
- `MACHINE_DEFAULT=MACHINE_ID_LOCAL`、物理极性、零点、量程、镜像与 IMU 轴均未因文档整理改动。
- `DBUS.md`、`UART_IDLE_DMA.md` 已对照当前 `dr16.c`、`hi229.c` 与中断入口核对。
- `hip-test-vofa.md` 已删除，可由 Git 恢复；其余六份文档为移动并保留内容。
- Keil AC5 全量编译：默认配置与 `-DSYSID_ENABLE=1` 均为 **105 文件，0 fail / 0 warn**。

**待台架**
- `LQR_PLAN.md` 的小机器顺序仍须逐项实测；IMU 轴、腿摆速度补偿符号、输出限幅和大机器轮径均未在文档中冒充完成。
- 大机器 sysid 文档虽已归档并标开关范围，但 1 kHz 改频后的采样/发送丢帧行为需在再次启用前重新核对。

---

## 变更 64 · 删除 VOFA 通道文档 + 各 md 对齐代码现状（作者：更新各 md，把 vofa 通道 md 都删了）

| 文件 | 改动 |
| --- | --- |
| `md/VOFA_SEND.md`、`md/sysid/vofa-channel-map.md`、`md/sysid/wheel-test-vofa.md` | **删除**。正常 32 路通道表只保留 `task_comm.c` 里 `Robot_Control_Send_Vofa()` 上方的注释；sysid 列定义以 `sysid_log.c::assemble_frame()` 为准；旧文可从提交 `24efef2` 恢复 |
| `md/AGENTS.md`、`md/CLAUDE.md`、`md/RL_OVERVIEW.md`、`md/IO_CHAINS.md` | 去掉指向三份通道 md 的链接与文件树条目；VOFA 描述统一为 JustFloat、500 Hz、"布局以代码为准"；模块表标注 sysid 发送泵未接 |
| `md/sysid/sysid-delivery.md` | 帧 33 列 → **37 列 / 152 B**（补列 33~36）；心跳状态码补 5；轮用例改成代码实际的 10 个 plateau run（±0.5~4 A，钳位 ±5 A）；`SYSID_ENABLE` 默认 0、`SYSID_PLAN` 默认 2、补 `SYSID_MODE`；测试限幅 10 Nm；VOFA+ 通道数 37；写明发送泵未接与 1 kHz 推帧问题；去掉 FireWater 说法；文件头去重 |
| `md/sysid/controller-spec-for-mujoco.md` | 日期改回 2026-09-20；PD 参数 25/300 → **10/0**；姿态表改为 45°/90°/135° × 2.40/2.60/2.80；斜坡改为 `Ramp_Update()` 斜率限幅（0.4/0.2 rad/s，t_pre 4.2 s + 保持 3 s）；右腿动作值按各自 `dof_pos`；限幅层级补 `SYSID_TRQ_LIMIT_NM`；帧列 35 → 37；行号同步 |
| `md/sysid/sysid-lower-machine-plan.md` | 数据流/时钟/风险中的 500 Hz → 1 kHz；§7.1 补当前带宽现状；§10.1 采样率说明；§17.1 步 4 改为"待接回"；文件头去重 |
| `md/sysid-change-map.md` | 本条；附录 B 时钟行 500 Hz → 1 kHz |

**核对时发现、本次未改（属代码，需作者授权后另开变更）**
- `imcalib/task/task_comm.c`：`SYSID_ENABLE=1` 时没有任何地方调用 `Sysid_Log_Send_Pump()`（提交 `48b087e` 有该调用，`87c6628` 改成 10 通道轮帧直发时去掉，之后小机器 LQR 批次里 10 通道分支也删了），也没有"测试模式停发 32 路"的早退。现状：快照只进环形缓冲，串口上仍是 32 路 LQR 帧。
- `Sysid_Mode_Run()` 每个 1 ms 周期推一帧，发送泵最多 250 Hz 取一帧，接回后 128 帧环形缓冲约 0.17 s 即溢出；152 B × 1 kHz = 152 kB/s 也超过 1152000 波特的 115 kB/s 线速。需要先定"每 N 拍推一帧"的抽取方案。
- 过期注释：`sysid_log.h` 顶部列表仍写"33 列 / @921600 / 每 2 个周期发一次"；`tools/sysid_export.py` docstring 仍写"31 列"；`sysid_mode.c:722` 注释写 1 rad/s（实际 0.4/0.2）；`SYSID_POSE_RAMP_RATE` 宏无人引用。
- `sysid_mode.c` 的 `#ifndef SYSID_PLAN` 兜底值 1 与 `sysid_config.h` 的 2 不一致（后者先包含，实际生效 2）。

**输入 / 输出 / 调用链**
- 正常 VOFA：`comm_task_body()` → `Robot_Control_Send_Vofa()`（2 分频）→ `Vofa_Send(dbg, 32)` → `VOFA_UART`（USART1 @1152000）JustFloat；通道含义见该函数上方注释。
- sysid 帧：`Sysid_Mode_Run()` → `Sysid_Log_Push()` → 环形缓冲 → （**缺调用**）`Sysid_Log_Send_Pump()` → `assemble_frame()` 37 列 → DMA。
- 文档入口：`AGENTS.md` 关键约束 → `task_comm.c` 注释 / `sysid_log.c`；训练端 → `sysid-delivery.md` §1.2 快速索引 + `controller-spec-for-mujoco.md` §6/§7。

**核对**
- `md/` 内 Markdown 链接无死链；`VOFA_SEND.md` / `vofa-channel-map` / `wheel-test-vofa` 仅在本账本历史条目中以文件名出现（保留作记录）。
- 未改任何代码、配置、极性、零点、量程、镜像与 `MACHINE_DEFAULT`；本次只动 md，未编译。

**待台架 / 待决定**
- 大机器再次启用 sysid 前：作者定抽取方案 → 接回发送泵 + 测试模式停发 32 路 → 台架看心跳行列 10/11（drop/busy）恒 0、`seq` 连续、VOFA+ 37 列对齐。

---

## 变更 65 · 小机器 LQR 参数对齐 Leg2_v1（作者：把 D 加回去、参数按 Leg2 来；腿长区间与投入下限按本机自标）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/leg_balance.h` | 腿长/防劈叉/横滚 KD：0/0/0 → **50000/500/100**（Leg2_v1 `Code/Task/balance.h` 原值） |
| `imcalib/Algorithm/lqr_balance.c` | `lqr_debug.trq_max_hip` 默认：5.0 → `machine->dm_trq_clamp`（小机器 10 N·m） |
| `imcalib/Algorithm/lqr_balance.h` | `LQR_RC_YAW_MAX`：3.0 → **5.0 rad/s**（Leg2_v1 `app_rc.c`）；`LQR_K_LEN_MIN` 0.13 **不动** |
| `imcalib/user-lib/machine_config.c` | 小机器 `wheel_r`：0.03 → **0.04**（物理量，见下方单列）；`leg_len_min/max` 0.10/0.20 **不动** |
| `md/LQR_PLAN.md` | §一 加决策 9；§1.1/§2.4/§五/§八 数值同步；§六 ① 拆成 ①a~①d 四项符号/零点测法；新增 §十 与 Leg2_v1 参数对照表 |
| `md/AGENTS.md`、`md/RL_OVERVIEW.md` | 辅助 PID 描述同步 |

**为什么**
- KD：两边 PID 的 D 项都是 kd×(本拍误差−上拍误差)、不除 dt，周期同为 1 kHz，Leg2 原值就是同一物理阻尼。腿长环是 1000 N/m 弹簧顶机身，无阻尼会以约 3 Hz 弹跳；50000 折合约 50 N·s/m。之前减半是 500 Hz 时的换算，改 1 kHz 时作者先归零，现在不再需要换算。
- 髋限幅：Leg2 正常模式虚拟髋力矩不限幅，电机侧只受 MIT ±10 N·m；本机原 5 N·m 比 Leg2 严一倍，大扰动时会先饱和。
- 偏航量程：只影响满杆转向速度，按"参数一样"取 5.0。
- 轮径：见下。

**物理量变更确认（§0.1 单列）**
- 谁 / 何时：作者于 **2026-09-20** 回复"其他的按你说的改"，确认小机器轮半径 0.03 → 0.04。
- 依据：Leg2_v1 `Code/User/app_config.h` 的 `WHEEL_R 0.04f`；`轮腿上交建模MATLAB/WBR_modeling.mlx` 生效参数组 `R_w_ac = 0.04`（机身 5.25 kg、轮 0.13463 kg，与 Leg2 `BODY_MASS`/`WHEEL_MASS` 一致）；现用 K 表就是按这组生成。
- 纠错：变更 62 引用的"mlx 小机器参数组 R_w_ac=0.03"是**被百分号注释掉**的另一台车参数组（机身 1.103 kg、半轮距 0.075），属 AI 误读，本次纠正。
- 实际写入：`machine_config.c` 小机器行 `wheel_r = 0.04f`；大机器 0.04 占位不变，仍待实测。
- 复核：卡尺量轮子直径应接近 80 mm；即便有出入，先用 0.04 复现 Leg2 行为，因为 K 表与速度估计必须用同一个值。

**作者保留本机自标（未改）**
- 腿长区间 `leg_len_min/max` = 0.10/0.20，LQR 投入下限 `LQR_K_LEN_MIN` = 0.13（Leg2 为 0.12~0.29、默认站姿 0.12）。提醒：使能时腿长须 ≥0.13，否则 `LQR_Enable_Latch()` 返回 0 只发零力矩。

**输入 / 输出 / 调用链**
- KD：`Leg_Balance_Init()` → `PID_struct_init(kd)` → `pid_calc()` 的 `dout = d×(err[NOW]−err[LAST])` → F / Tp；`Leg_Balance_Reset()` 投入时清历史，首拍 D 输出为 0，无冲击。
- 髋限幅：`LQR_Init()` → `lqr_debug.trq_max_hip` → `LQR_Control_Update()` 虚拟髋力矩夹取 + `Leg_Balance_Compute()` 四台 DM 力矩夹取；仍可在调试器 Watch 里临时压低。
- 偏航：`LQR_Target_Update()` → `target[LQR_X_DPHI] = axis × 5.0`。
- 轮径：`LQR_State_Update()` → `whl × machine->wheel_r` → `x[LQR_X_DS]`；与 K 表生成参数一致后，LQR_PLAN "轮径与 K 表待重跑"一项关闭。

**核对**
- 未改极性、零点、MIT 量程、镜像、`+LEG_PI`、IMU 轴宏与 `MACHINE_DEFAULT`。
- K 表核对：`lqr_gain_table.c` 与 `Leg2_v1/Code/Matlab/LQR_K_WBR.c` 除 include 行外逐字节一致。
- Keil AC5 全量编译：默认配置与 `-DSYSID_ENABLE=1` 均为 **105 文件，0 fail / 0 warn**。未链接、未上机。

**待台架（作者：等会测试并修改）**
- ①a 腿摆角零位 `leg_off_phi0`（小机器表现为大机器换算值 −0.13/−0.07，Leg2 为 0）：腿竖直看 ch9/ch13 应 ≈0。
- ①b IMU 轴与极性：本机 `euler_rad[0]` 装的是模块 Roll 通道，`gyro_rad_s[1]` 是模块 Y 轴角速度，角与角速度不在同一根轴；Leg2 两者同取第 1 路且抬头为正。抬头看 ch17/18，只改 `lqr_balance.c` 顶部五个 `LQR_IMU_*` 宏并核对翻倒检测；右倾看 `lqr_state.roll` 应为正；架空手转车体，两轮应出反向阻转力矩。
- ①c 轮速腿摆补偿 `vel_leg_comp_sign`：推导上复现 Leg2 应为 +1，架空推腿看 ch19 取波动小者。
- ①d 转向通道符号：右摇杆推右应右转，Leg2 对该通道取负号。
- D 项恢复后首次落地观察腿长是否抖动（Leg2 手册：腿部振荡就降 KD）。

---

## 变更 66 · LQR 前置"手动腿测"：左拨杆中位 = 摇杆直接给腿长 + 虚拟腿摆角（作者：先测手动、先不输出看极性）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/leg_solver.c` | 删掉第 176 行行尾误敲的 `FFFFFFFF`（上次提交带进来的，armcc 报 2 个错，整个工程编不过） |
| `imcalib/Algorithm/lqr_balance.h/c` | 新增 `LQR_RC_ANG_MAX`（摆角满杆 ±0.5 rad）、`lqr_state_t.leg_ang_tgt[2]`；`LQR_Enable_Latch()` / `LQR_Target_Update()` 加 `manual` 参数；新增静态 `LQR_Len_Range()`：手动腿测取机器区间 0.10~0.20，LQR 才与 K 表域 0.13~0.23 求交；手动时左摇杆 Y (ch3) 直接给摆角目标，锁存时摆角目标清零 |
| `imcalib/Algorithm/leg_balance.h/c` | 新增摆角 PD 宏 `LEG_BALANCE_ANG_KP/KD`（10 / 0，待台架）、`leg_ang[2]` PID、`tau[4]` 髋力矩命令观测；把"力域映射 + 限幅 + 轮限幅"抽成静态 `Leg_Balance_Output()`，`Leg_Balance_Compute()` 改为调它；新增 `Leg_Balance_Manual()`：F = 腿长 PID + 8 N 前馈（无横滚），Tp = 摆角 PD（**不取反**），轮 = 0；`Leg_Balance_Reset()` 一并清 6 个 PID |
| `imcalib/task/inc/robot_control.h` | 枚举加 `CTRL_STRATEGY_LQR_MANUAL`（= 3） |
| `imcalib/task/task_actuation.c` | 仲裁：左拨杆中位 + 右拨杆中位 = LQR，左中 + 右非中 = 手动腿测；两者共用使能/锁存/复位逻辑，模式切换时 `lqr_running=0` 重新锁存；手动腿测不查 IMU；新增 `output_task_lqr_manual()` |
| `imcalib/task/robot_control.c` | `torque_output_enabled` 初值 1 → **0**（作者：先不输出，看极性对了再开） |
| `imcalib/task/task_comm.c` | ch2 注释补 2 测试 / 3 手动腿测；**ch20~23 在 ch2=3 时改为四髋力矩命令**（前左/后左/前右/后右，`leg_balance.tau[]`），其余模式仍是 LQR 输出；通道下标未动 |
| `md/LQR_PLAN.md` | 决策 2 改写、加决策 10；§2.1/§2.4 补条目；新增 §2.7 手动腿测链路；§六 加 ⓪a~⓪c 台架步骤；§八 加"手动腿测链路"行 |
| `md/IO_CHAINS.md`、`md/RL_OVERVIEW.md`、`md/AGENTS.md` | 拨杆语义、总输出初值、策略仲裁描述同步 |

**为什么**
- 作者要在 LQR 之前，用与 RL 手动遥操同样的方式（摇杆直接给目标 → PID → 雅可比 → 电机）单独验证"腿长 / 虚拟腿摆角 → 力域映射 → 四髋力矩"的极性与雅可比；不碰 K 表、不碰 IMU，出了问题只可能是这一段。
- Tp 不取反：`force_map = leg_jacᵀ`，Tp 就是与解算摆角 `virtual_leg_angle` 共轭的广义力，PD 直接作用即可；LQR 那处取反是因为模型 θ_ll 与本工程摆角反号（§3.1），与本链路无关。
- 手动腿测的腿长区间不与 K 表域求交：K 表在这条链里不用，作者标定的机器区间 0.10~0.20 才是物理边界；否则腿收到 0.13 以下就进不去手动模式。
- `torque_output_enabled=0`：作者要求先不输出，VOFA 看命令方向对了再开。

**输入 / 输出 / 调用链**
- 仲裁：`DR16_Snapshot()` → `s1==MID` → `s2==MID ? LQR : LQR_MANUAL` → `ctrl_strategy`（VOFA ch2）。
- 投入：`motor_enabled && legs valid`（手动不查 `imu_state.online`）→ `LQR_Enable_Latch(manual)`：腿长在 `LQR_Len_Range()` 内才返回 1 → `leg_len_tgt` 锁当前、`leg_ang_tgt=0` → `Leg_Balance_Reset()`。
- 每拍：`LQR_Target_Update(manual=1)`：拨轮 × 0.3 m/s 积分 → `leg_len_tgt`（夹到机器区间）；ch3 × 0.5 rad → `leg_ang_tgt` → `Leg_Balance_Manual()`：`pid_calc(leg_len, virtual_leg_length, tgt)` → F；`pid_calc(leg_ang, virtual_leg_angle, tgt)` → Tp → `Leg_Balance_Output()`：`Leg_Force_Map_Forward(F, Tp)` → 前/后髋 → `clampf(±trq_max_hip)` → `torque.dm[]`、`lb->tau[]`；轮 0 → `output_send()`（总输出关时只发零力矩，`lb->tau` 仍是计算值）。
- `lqr_debug.hip_enable=0` 把 Tp 清零、`len_pid_enable=0` 只留 8 N 前馈，两条链共用。
- 观测：ch9/13 摆角、ch10/14 腿长、ch15/16 腿长目标、ch20~23 四髋命令、ch28~31 髋力矩反馈；`lqr_state.leg_ang_tgt`、`leg_balance.F/Tp/tau` 调试器 Watch。（通道号已由变更 67 重排，以 67 为准）
- 顺带修正：原 `Leg_Balance_Compute()` 对 `Leg_Force_Map_Forward()` 的返回值不检查，`force_valid=0` 时会把未初始化的 `tau` 发出去；现在返回 0 走零力矩，`lb->tau` 同步清零。

**核对**
- 未改极性、零点、MIT 量程、镜像、`+LEG_PI`、IMU 轴宏、`MACHINE_DEFAULT`、腿长区间、LQR 里的 Tp 取反；VOFA 32 路下标未动，帧长 132 B 不变。
- 雅可比有限差分：0.10~0.20 m 内 1148 个姿态，`leg_jac` 与数值导数最大差 1e−10（长度行 6e−11、角度行 5e−10）。
- 手动闭环方向：3 组 `offset_phi0`（−0.13 / −0.07 / 0）× 全区间姿态 × 4 种目标偏移共 10416 例，沿 `Jᵀ[F;Tp]` 方向的虚位移全部使腿长/摆角朝目标移动，0 例反向。
- Keil AC5 按 `compile_commands.json` 全量编译：默认配置与 `-DSYSID_ENABLE=1` 均 **103 文件，0 fail / 0 warn**（`-D` 只加给 `.c`，汇编启动文件不吃该选项）。未链接、未上机。

**待台架（按 LQR_PLAN §六 ⓪）**
- ⓪a 总输出关着：左摇杆前推 → ch20~23 同号（对称站姿前后髋各约 −0.5×Tp）；拨轮上推 → ch15/16 升、前后髋命令异号；手掰腿 → 命令朝拉回方向。
- ⓪b 开输出（`torque_output_enabled=1`，`trq_max_hip` 先压 3 N·m）：前推腿前摆、两腿同向；回中腿竖直（顺便定 ①a 零位）；拨轮腿长跟目标（8 N 前馈约 8 mm 静差属正常）。
- 摆角 PD 10/0 是起步值，振荡就先降 KP；极性/零点异常只报现象不改数。

---

## 变更 67 · VOFA 换成"腿测 + 力域"观测帧（作者：把需要的数据放进去，开始测试）

| 文件 | 改动 |
| --- | --- |
| `imcalib/task/task_comm.c` | `Robot_Control_Send_Vofa()` ch3~31 重排（下表）；去掉变更 66 里 ch20~23 按模式切换的分支，全程一套固定布局；ch2 注释不变 |
| `imcalib/Algorithm/leg_balance.h/c` | `leg_balance_t` 的 `tau[4]`（变更 66）换成 `torque_output_t cmd`（六路力矩命令观测，含轮），`Leg_Balance_Output()` 成功时整份拷贝、失败时清零；`Leg_Balance_Reset()` 一并清 F / Tp / cmd |
| `imcalib/task/task_actuation.c` | 手动腿测每拍也调 `LQR_State_Update()`，**只为 ch27~29 观测**，不参与控制（IMU 掉线时该函数直接返回 0，不影响腿测）；LQR / 手动腿测未投入时每拍 `Leg_Balance_Reset()`，避免 VOFA 显示上一次的旧命令 |
| `md/LQR_PLAN.md` | 决策 10、§2.5、§2.7、§六 ⓪~② 通道号全部改到新布局 |
| `md/IO_CHAINS.md`、`md/RL_OVERVIEW.md` | 去掉"大腿角 / 虚拟小腿角上 VOFA"与"不用 IMU"的说法 |

**新帧（32 通道 / JustFloat / 500 Hz，帧长 132 B 不变）**

| 通道 | 含义 | 来源 |
| --- | --- | --- |
| ch0 | 在线掩码：IMU / 遥控 / 髋 4 / 轮 2 | 同前 |
| ch1 | 状态位：使能 / 跌倒 / 左腿有效 / 右腿有效 / 四髋使能 | 同前 |
| ch2 | 策略号 0 手动 / 1 LQR / 2 测试 / 3 手动腿测 | `ctrl_strategy` |
| ch3~6 | 四髋位置（零点后 rad）前左 / 后左 / 前右 / 后右 | `motor_state.dm.pos_zero_rad[]` |
| ch7~10 | 左摆角 / 左腿长 / 右摆角 / 右腿长 | `leg_l/r.output.virtual_leg_angle / virtual_leg_length` |
| ch11~14 | 左摆角目标 / 右摆角目标 / 左腿长目标 / 右腿长目标 | `lqr_state.leg_ang_tgt[] / leg_len_tgt[]` |
| ch15~18 | 足端力 F 左 / 右 (N)，虚拟髋扭矩 Tp 左 / 右 (N·m) | `leg_balance.F[] / Tp[]` |
| ch19~22 | 髋力矩命令 (N·m) 前左 / 后左 / 前右 / 后右（总输出关时仍是计算值） | `leg_balance.cmd.dm[]` |
| ch23~26 | 髋力矩反馈 (N·m) 前左 / 后左 / 前右 / 后右 | `dm_motor_feedback[].trq_nm` |
| ch27~29 | 俯仰角 / 俯仰角速度 / 前进速度（LQR 状态，手动腿测仅观测） | `lqr_state.x[THB / DTHB / DS]` |
| ch30~31 | 轮力矩命令 (N·m) 左 / 右（手动腿测恒为 0） | `leg_balance.cmd.dji[]` |

去掉的：大腿角 / 虚拟小腿角（RL 坐标，本阶段不用）、LQR 原始输出 `u[]`（髋侧由 Tp 与四髋命令替代，轮侧由轮力矩命令替代）、轮转速、轮实测电流。

**为什么**
- 作者要开始手动腿测，"先不输出看极性"需要在 VOFA 上同时看到：目标（摆角 / 腿长）→ 力向量（F / Tp）→ 四髋命令 → 四髋反馈，整条链每一级都可见，哪一级符号不对一眼能定位。
- 不再按模式切换通道含义：作者的原则是"显示值不对先查下标"，一个通道两种含义是新的坑。
- 手动腿测顺带跑状态估计：ch27~29 在安全的手动模式下就能做 §六 ①b（IMU 轴 / 极性）和 ①c（轮速腿摆补偿）的观察，不必先进 LQR。

**输入 / 输出 / 调用链**
- `Leg_Balance_Output()` → `torque->dm/dji` → `lb->cmd = *torque`（映射失败返回 0 前已清零）→ `Robot_Control_Send_Vofa()` ch19~22 / ch30~31。
- `output_task_body()` LQR / 手动腿测分支未投入 → `Leg_Balance_Reset()` → F / Tp / cmd 清零 → ch15~22、ch30~31 为 0；`leg_len_tgt / leg_ang_tgt` 保留上次值（ch11~14 不清）。
- `output_task_lqr_manual()` → `LQR_State_Update()`（IMU 在线才写 `x[]`）→ ch27~29。

**核对**
- 未改极性、零点、MIT 量程、镜像、`+LEG_PI`、IMU 轴宏、`MACHINE_DEFAULT`、控制律；`Vofa_Send(dbg, 32u)` 与 `VOFA_MAX_CH` 不变。
- Keil AC5 全量编译：默认配置与 `-DSYSID_ENABLE=1` 均 **103 文件，0 fail / 0 warn**。未链接、未上机。

**待台架**：按 `LQR_PLAN.md` §六 ⓪a → ⓪b。

---

## 变更 68 · 🐞 手动腿测投不进去（作者台架：目标角一直是零）

**现象**：左中位进手动腿测，ch2 = 3，但 ch11/12 摆角目标恒 0，推摇杆没反应。

**根因（可证）**：变更 66 让手动腿测沿用了 `LQR_Enable_Latch()` 的"实测腿长必须在区间内"门槛（手动取机器区间 0.10~0.20）。总输出关着、电机零力矩时，小机器腿架空垂到机械限位 ≈0.20 m（作者实测；几何极限 lu+lg≈0.28 到不了），读数正好压在区间上沿，浮点略超 0.20 就被拒；趴地则缩到 0.10 以下。两种姿态都投不进 → 锁存返回 0 → `output_task_lqr_manual()` 一次都没跑 → `leg_ang_tgt` / `leg_len_tgt` / F / Tp / 四髋命令全为 0。这个门槛是给 LQR 防"使能瞬间弹起"的，手动腿测目标锁当前值本来就不会跳，不需要。

**顺带核对（作者问"腿长是否还没加轮径"）**：不加，也不该加。本工程 `virtual_leg_length = |OP|` 是髋心到轮轴距离；Leg2_v1 `Leg_Position` 同一定义（其常数 0.0398891754 = 2·l1·l2，髋距项系数 0.0F），两者在工作区间逐点差 3e−9；Leg2 把这个值直接喂 `LQR_K_WBR(leg_len_lft, leg_len_rgt)`，K 表按它拟合。轮径只在 `LQR_State_Update()` 的 `ω·R_w` 里用。

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/lqr_balance.c` | `LQR_Enable_Latch()`：只有 `manual=0` 才查区间，手动腿测任意腿长都投入；`LQR_Target_Update()`：手动时拨轮的夹取区间扩到 `[min(0.10, 当前目标), max(0.20, 当前目标)]`——目标在区间外只能往区间里拨，进区间后不再出去 |
| `imcalib/task/task_actuation.c`、`inc/robot_control.h` | 新增 `output_task_lqr_engaged()` 返回 `lqr_running` |
| `imcalib/task/task_comm.c` | ch1 加 **bit8 = LQR / 手动腿测已投入**（`state_bits` 改 `uint16_t`），一眼能看出链路有没有跑 |
| `md/LQR_PLAN.md` | §2.7 投入条件、§六 ⓪a/⓪b 同步；⓪b 补"架空时腿垂在机械限位 ≈0.20，开输出后要先拨轮往下拨"；§2.7 补腿长定义不含轮径 |

**输入 / 输出 / 调用链**
- 手动腿测：`motor_enabled && legs valid` → `LQR_Enable_Latch(manual=1)` 直接锁 `leg_len_tgt = 实测`、`leg_ang_tgt = 0` → 返回 1 → 每拍 `LQR_Target_Update(manual=1)`：`lo = min(len_min, tgt)`、`hi = max(len_max, tgt)`，积分后夹到 `[lo, hi]`。
- 开输出时的安全性：目标 = 实测，腿长 PID 误差为 0，只有 8 N 前馈；腿垂在机械限位 ≈0.20 时前馈顶着限位不动，拨轮下拨才按 0.3 m/s 收腿，无阶跃。
- LQR 路径（`manual=0`）逻辑不变：仍要求 0.13~0.20 才投入，仍夹在 0.13~0.20。
- VOFA：`output_task_lqr_engaged()` → ch1 bit8（0x100）。

**核对**
- 未改极性、零点、MIT 量程、镜像、`+LEG_PI`、IMU 轴宏、控制律、VOFA 下标（只扩 ch1 高位）。
- Keil AC5 全量编译：默认配置与 `-DSYSID_ENABLE=1` 均 **103 文件，0 fail / 0 warn**。

**待台架**：重新按 §六 ⓪a：ch1 应为 509（bit8 已投入 + 四髋使能 + 两腿有效 + 使能），推左摇杆 ch11/12 应跟着走。

---

## 变更 69 · 手动腿测首次出力：摆角 PD 调小 + 总输出打开（作者：把 PD 参数调小，然后把输出打开，先进行测试）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/leg_balance.h` | `LEG_BALANCE_ANG_KP` 10 → **5**，`LEG_BALANCE_ANG_KD` 仍 0 |
| `imcalib/task/robot_control.c` | `torque_output_enabled` 初值 0 → **1** |
| `md/LQR_PLAN.md`、`md/RL_OVERVIEW.md` | 参数表、§2.7、§六 ⓪a/⓪b、总开关说明同步 |

**为什么**：作者决定跳过"不出力看命令"直接上台架；摆角环先减半，满杆 0.5 rad 时 Tp = 2.5 N·m、单髋约 1.25 N·m。腿长 PID（1000/50000）和髋限幅（10 N·m）未动，仍是变更 65 定的 Leg2 值。

**输入 / 输出 / 调用链**：`LEG_BALANCE_ANG_KP` → `Leg_Balance_Init()` → `pid_calc(leg_ang)` → `Tp`；`torque_output_enabled=1` → `output_send()` 真正下发 `Dm_Send_Torque()` / `Dji_Send_Wheel_Torque()`。

**核对**：未改极性、零点、MIT 量程、镜像、IMU 轴宏、腿长 PID、限幅。Keil AC5 全量编译默认与 `-DSYSID_ENABLE=1` 均 **103 文件，0 fail / 0 warn**。

**待台架**：按 §六 ⓪b。首次出力前建议调试器 `lqr_debug.trq_max_hip=3`，防腿长环方向反时以满限幅顶限位。

---

## 变更 70 · 摆角 PD 恢复 10，准备下地直接跑 LQR（作者：恢复 PD 跟一些参数，下地试一下，直接用 LQR）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/leg_balance.h` | `LEG_BALANCE_ANG_KP` 5 → **10**（回到变更 66 值） |
| `md/LQR_PLAN.md` | 参数表、§2.7 同步；§六 ① 前补 ①0（从手动腿测切进 LQR 的手法）、①1（位移目标 −0.12 会让车后退 12 cm） |

其余参数已是 Leg2 值，未动：腿长 1000/50000、防劈叉 30/500、横滚 500/100、前馈 8 N、髋限幅 10、轮限幅 1.8、轮径 0.04、K 表；`torque_output_enabled` 保持 1。

**核对**：未改极性、零点、量程、IMU 轴宏。Keil AC5 全量编译默认与 `-DSYSID_ENABLE=1` 均 **103 文件，0 fail / 0 warn**。

**待台架（LQR 首次下地，§六 ①~⑥ 的符号项全部未定）**：IMU 俯仰轴与极性（①b）、腿摆角零位（①a）、轮速腿摆补偿符号（①c）、转向符号（①d）。最先看的是"前倾时轮子往前追"——反了就是 ①b，立即左下位。

---

## 变更 71 · 摆角 PD 照抄 Leg2 自救腿角环 + 轮补偿观测量（作者：虚拟腿摆角也按 Leg2 参数配置试一下）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/leg_balance.h` | `LEG_BALANCE_ANG_KP/KD` 10/0 → **20/30**；新增 `LEG_BALANCE_ANG_TP_MAX 4.0f`（三者同 Leg2 `Code/App/app_self_rescue.h` 的 `SELF_RESCUE_KP/KD/TP_MAX`） |
| `imcalib/Algorithm/leg_balance.c` | `Leg_Balance_Manual()` 的 Tp 先夹到 ±4 N·m 再进力域映射 |
| `imcalib/Algorithm/lqr_balance.h/c` | `lqr_state_t` 加 `whl[2]`（补偿后的轮对地角速度，只供观测） |
| `md/LQR_PLAN.md` | 参数表、§2.7；§六 ①c 改成"着地扶机身摆腿看 x[1] 归零"的测法；§八 补偿符号一行同步 |

**为什么可以直接照抄**
- Leg2 自救环 `Pid_Update(&self_rescue, 目标, leg.ang)` 作用在"腿相对机身的摆角"上，与本工程 `virtual_leg_angle` 同一个量；D 项两边都是 kd×(本拍误差−上拍误差)、同为 1 kHz，KD 直接照抄。
- 符号：Leg2 腿角前摆为负、其 Tp 进 `Leg_Tougue` 也与本工程反号，两次反号抵消，正增益在本工程坐标下仍是正增益（已在 §3.1 数值核验的等价关系上推得）。
- Tp 上限 4 N·m 是 Leg2 与 20/30 配套的保护（"扫腿时机体反作用太大"），照抄；单髋约 2 N·m，在 10 N·m 限幅之内。

**轮速腿摆补偿现状（作者问）**
- 公式：`whl = 轮速(已除减速比) + sign × d_virtual_leg_angle − 俯仰角速度`，再 `×R_w` 进速度估计；`sign = lqr_debug.vel_leg_comp_sign`，默认 −1。
- 本工程约定（前摆为正、轮前滚为正）下推导：足端前摆时电机壳体反向转，编码器多读一份摆角速度，应减掉 → −1。Leg2 原式 `vel − d_ang − omg_pitch` 因其腿角前摆为负、逐字翻译成本工程约定是 +1；差异只能来自两边编码器/IMU 正向约定，台架 A/B 定。
- 测法见 §六 ①c：着地、扶住机身、手动腿测摆腿，`lqr_state.x[1]` 接近 0 的那个符号是对的。

**核对**：未改极性、零点、量程、IMU 轴宏、LQR 控制律；Keil AC5 全量编译默认与 `-DSYSID_ENABLE=1` 均 **103 文件，0 fail / 0 warn**。

**待台架**：20/30 首次出力若腿抖，先把 KD 减半；补偿符号按 ①c 定后写死并删掉 `vel_leg_comp_sign`。

---

## 变更 72 · VOFA 换成 IMU 极性测试帧（作者：开始对 IMU 进行测试，清空 VOFA，填入 IMU 相关数据）

| 文件 | 改动 |
| --- | --- |
| `imcalib/task/task_comm.c` | `Robot_Control_Send_Vofa()` ch3~31 整块换成 IMU 链路（下表）；新增 `#include "hi229.h"` 读原始快照；四元数投影重力复用 `RL_Observation_Project_Gravity()`；作者已自行注掉 2 分频（1 kHz 直发），本批把 `vofa_div` 声明一并注掉消警告 |
| `md/LQR_PLAN.md` | §2.5/§2.7/§六 里的通道号全部改成变量名（帧内容按阶段变，通道号不再写进文档），①b 补 IMU 帧测法 |
| `md/RL_OVERVIEW.md`、`md/IO_CHAINS.md` | VOFA 说明去掉"500 Hz"、注明当前为 IMU 测试帧 |

**IMU 测试帧（32 通道 / JustFloat）**

| 通道 | 含义 | 来源 |
| --- | --- | --- |
| ch0~2 | 在线掩码 / 状态位（bit1 跌倒，bit8 已投入）/ 策略号 | 同前 |
| ch3~5 | 模块原始欧拉角 Roll / Pitch / Yaw（deg，未乘符号） | `HI229_Snapshot().eul[]` |
| ch6~8 | 模块原始角速度 X / Y / Z（deg/s，未乘符号） | `.gyr[]` |
| ch9~11 | 模块原始加速度 X / Y / Z（G） | `.acc[]` |
| ch12~14 | `imu_state.euler_rad[0/1/2]`（rad，已乘符号）；[0] 给 LQR 当俯仰、[1] 横滚、[2] 偏航 | `task_imu.c` |
| ch15~17 | `imu_state.gyro_rad_s[0/1/2]`（rad/s，已乘符号）；[1] 给 LQR 当俯仰角速度、[2] 偏航角速度 | 同上 |
| ch18~20 | 四元数投影重力 X / Y / Z（机体系，RL 阶段已验证） | `RL_Observation_Project_Gravity(imu_state.quat)` |
| ch21~24 | LQR 吃到的：俯仰 `x[8]` / 俯仰角速度 `x[9]`（低通）/ 偏航角速度 `x[3]`（低通）/ 横滚 `lqr_state.roll` | `LQR_State_Update()` |
| ch25~28 | 世界系腿摆角 `x[4]` 左 / `x[6]` 右 / 角速度 `x[5]` / `x[7]` | 同上 |
| ch29 | 速度估计 `x[1]` | 同上 |
| ch30~31 | 解算摆角（机体系）左 / 右 | `leg_l/r.output.virtual_leg_angle` |

ch21~29 只在 LQR / 手动腿测投入后更新，其余随时有效。

**为什么这么排**：一次动作能沿"模块原始 → 乘符号 → LQR 状态"三级同时看，哪一级把轴或符号弄错一眼可辨；原始加速度和四元数重力是两个不依赖欧拉角约定的独立参照（静止时加速度哪一轴 ≈ ±1 G 就是竖直轴，抬头时哪一轴变化就是前后轴）。

**1 kHz 直发的后果**：32 路帧 132 B × 1 kHz = 132 kB/s，超过 1152000 bps 串口的约 115 kB/s；`Vofa_Send()` 在上一帧 DMA 未完时直接丢帧，实际约 870 帧/s，看图无影响。

**核对**：未改极性、零点、IMU 符号宏、控制律；Keil AC5 全量编译默认与 `-DSYSID_ENABLE=1` 均 **103 文件，0 fail / 0 warn**。

**待台架**：§六 ①b，把"抬头 / 右倾 / 左转"三个动作下 ch3~17 各路的正负报回来再定 `LQR_IMU_*` 宏。

---

## 变更 73 · IMU 极性进机器配置表 + 欧拉角槽位改正（作者：做一个 IMU 极性配置表，跟机器配置表放一起）

| 文件 | 改动 |
| --- | --- |
| `imcalib/user-lib/machine_config.h` | 新增 `imu_cfg_t`（`eul_src[3]` / `eul_sign[3]` / `gyr_sign[3]` / `acc_sign[3]` / `quat_sign[3]`），`machine_cfg_t` 加 `.imu` |
| `imcalib/user-lib/machine_config.c` | 两份表各填一组（下表）；大机器照抄原宏值、注明待实测 |
| `imcalib/user-lib/hi229.h` | 删除 12 个 `HI229_*_SIGN_*` 全局宏（已搬入表），驱动只出原始值 |
| `imcalib/Algorithm/imu_state.h` | `imu_state_t` 加 `acc_g[3]`（G，乘符号后） |
| `imcalib/task/task_imu.c` | 改读 `machine->imu`：欧拉角按 `eul_src` 取路再乘 `eul_sign`；角速度/加速度/四元数乘各自符号；新填 `acc_g` |
| `imcalib/task/task_comm.c` | VOFA 换成 IMU 测试帧第二版：前 9 路为乘过极性后的欧拉角（俯仰/横滚/偏航）、角速度 X/Y/Z、加速度 X/Y/Z；原始欧拉角/角速度挪到 ch26~31，原始加速度不再发 |
| `md/IO_CHAINS.md`、`md/LQR_PLAN.md` | IMU 链路与表、①b、§八、§十 同步 |

**物理量变更确认（§0.1 单列）**
- 谁 / 何时：作者于 **2026-09-21** 台架看原始值后指出："小机器 Roll/Yaw 的欧拉角、角速度、加速度极性都反，Pitch 没问题，轴没问题；即 Roll/Yaw 角与角速度取反，acc X/Z 取反，说的是原始值。"
- 符号值：与原 `hi229.h` 宏**逐项相同**（eul −1/+1/−1 按 Roll/Pitch/Yaw，gyr −1/+1/−1，acc −1/+1/−1，quat −1/+1/−1），本次只是搬家，乘后的数值不变。
- **槽位变更（真正改了行为的一处）**：原 `task_imu.c` 把 `sample.eul[0]`（模块 Roll）写进 `euler_deg[0]`，而 `ATTITUDE_PITCH = 0`，即 LQR 俯仰角、翻倒检测、`lqr_state.roll` 全部取错路（俯仰拿的是横滚，横滚拿的是俯仰）；角速度那边 `LQR_IMU_GYRO_PITCH = 1`（Y 轴）本来就对。按作者"轴没问题"，表里写 `eul_src = {1, 0, 2}`：俯仰←Pitch 路、横滚←Roll 路、偏航←Yaw 路。改后 `euler_rad[ATTITUDE_PITCH]` = +模块 Pitch，`euler_rad[ATTITUDE_ROLL]` = −模块 Roll。依据：作者台架陈述 + 角速度已用 Y 轴当俯仰、角与角速度须同轴。
- 待作者复核（作者原话"做完这个之后我再检查一次乘过极性后的数据"）：见 §六 ①b。

**小机器 `.imu` 表**

| 字段 | 值 | 读法 |
| --- | --- | --- |
| `eul_src` | {1, 0, 2} | 俯仰←模块 Pitch(1)，横滚←Roll(0)，偏航←Yaw(2) |
| `eul_sign` | {+1, −1, −1} | 俯仰不反，横滚反，偏航反 |
| `gyr_sign` | {−1, +1, −1} | X（横滚轴）反，Y（俯仰轴）不反，Z（偏航轴）反 |
| `acc_sign` | {−1, +1, −1} | X 反，Y 不反，Z 反 |
| `quat_sign` | {−1, +1, −1} | 与上面一致（模块绕 Y 装反 180°） |

**输入 / 输出 / 调用链**：`HI229_Snapshot()` → `imu_task_body()`：`euler_deg[i] = eul_sign[i] × eul[eul_src[i]]`、`gyro_rad_s[i] = gyr_sign[i] × gyr[i] × π/180`、`acc_g[i] = acc_sign[i] × acc[i]`、`quat[1..3] = quat_sign × quat[1..3]` → `Attitude_Update()` → `imu_state` → LQR（`euler_rad[ATTITUDE_*]`、`gyro_rad_s[1]/[2]`）、翻倒检测（`euler_rad[ATTITUDE_PITCH]`）、RL（`gyro_rad_s`、`quat`，本次数值不变）。

**核对**
- 未改 `dm_sign` / `dji_sign` / 零点 / 量程 / 腿几何 / `LQR_IMU_*` 宏 / `MACHINE_DEFAULT`；RL 观测用的角速度与四元数数值不变。
- `grep HI229_.*_SIGN_` 全工程无残留。
- Keil AC5 全量编译默认与 `-DSYSID_ENABLE=1` 均 **103 文件，0 fail / 0 warn**。

**待台架**：§六 ①b 三个动作复核乘过极性后的量；大机器 `.imu` 表待实测。

---

## 变更 74 · 状态估计每拍必算，与挡位解耦（作者：解算/估计一直算，控制律和出力才看挡位）

**现象**：IMU 测试帧 ch15~23（`lqr_state.x[]`、`roll`）恒 0——`LQR_State_Update()` 原来只在 LQR / 手动腿测投入后被调用，测 IMU 时没使能、没投入，这些量从未算过。

**作者定的分层**：解算与估计（五连杆、IMU、LQR 状态）每拍都算，同 RL 观测；控制律（LQR 求和 / RL 推理）只在对应挡位算；出力只在使能、解算有效、投入等条件齐全时才发。

| 文件 | 改动 |
| --- | --- |
| `imcalib/task/task_actuation.c` | `output_task_body()` 开头每拍无条件调 `LQR_State_Update()`（`wheel_vel` 在此取一次，RL 分支重复赋值删掉）；`output_task_lqr()` / `output_task_lqr_manual()` 不再各自调状态估计，LQR 分支改看 `lqr_state.valid` 再算控制律 |
| `imcalib/Algorithm/lqr_balance.h/c` | `lqr_state_t` 加 `valid`（状态估计有效），`LQR_State_Update()` 入口清零、成功置 1；`LQR_Enable_Latch()` 只清位移积分 `pos` 与 `x[0]`，**不再复位三个低通**（常跑已是热态，复位反而制造一次从 0 恢复的瞬态） |
| `imcalib/task/task_comm.c`、`md/LQR_PLAN.md` | 注释与 §2.2 链路图同步，写入分层原则 |

**输入 / 输出 / 调用链**：每拍 `output_task_body()` → `LQR_State_Update()`（IMU 在线 + 两腿有效才写 `x[]`、`valid=1`，否则 `valid=0`、数值保持）→ 挡位仲裁 → 投入后 `LQR_Target_Update()` → `valid ? LQR_Control_Update() : 零力矩` → `Leg_Balance_Compute()` → `output_send()`。手动腿测不消费 `x[]`。位移积分在未投入期间也在累加，投入时 `pos`、`x[0]` 清零，首拍 LQR 看到的位移误差就是 −0.12 目标本身，无旧值冲击。

**核对**：未改极性、零点、控制律、增益；Keil AC5 全量编译默认与 `-DSYSID_ENABLE=1` 均 **103 文件，0 fail / 0 warn**。

---

## 变更 75 · 🐞 失能分支不可达：使能后拨杆下位 / 遥控离线 / 翻倒都不失能（作者台架测出）

**现象**：作者报"左拨杆下位没有失能，遥控离线也没有失能，翻倒也没有失能"。

**根因（可证，`git show fae783f`）**：变更 60 接入 DM 使能看门狗时，`Robot_Enable_Update()` 写成

```c
if (enable_request && !motor_enabled) { 使能 }
if (motor_enabled) { Dm_Enable_Watchdog(); }
else if (!enable_request && motor_enabled) { 失能 }   /* else 分支只在 motor_enabled==0 时进入, 条件永假 */
```

失能分支永远不可达：`motor_enabled` 一旦置 1 再也回不到 0，`Dm_All_Disable()` 再也不会发。后果分两级：
- 拨杆下位 / 遥控离线：执行任务走零力矩分支，腿是软的，但 DM 仍处使能态，`motor_enabled` 仍为 1；看门狗还在给任何掉使能的电机重发使能。
- **翻倒 / 电机离线 / CAN 故障：`motor_enabled` 不清零，LQR 与手动腿测分支的投入条件仍成立，控制器继续出力**——翻倒后车还在蹬。变更 60 之前的版本没有这个问题。

| 文件 | 改动 |
| --- | --- |
| `imcalib/task/task_comm.c` | `Robot_Enable_Update()` 改成"使能沿 / 失能沿"互斥的 `if / else if`，看门狗放在后面按 `motor_enabled` 二选一 |
| `imcalib/user-lib/dm.c/h` | 新增 `Dm_Disable_Watchdog()`：总失能期间，在线且 `err_raw==1` 的电机每 100 ms 重发失能（失能帧丢了也兜得住），与使能看门狗对称 |
| `md/RL_OVERVIEW.md`、`md/IO_CHAINS.md` | 使能状态机描述同步 |

**输入 / 输出 / 调用链**：`rc_enable`（s1≠下 且遥控在线）、`ctrl_fault`（IMU / 遥控 / 电机 / CAN / 动作）、`fallen` → `enable_request` → 沿检测 → `Dm_All_Enable()` / `Dji_All_Stop()`+`Dm_All_Disable()` → 之后每拍按 `motor_enabled` 跑对应看门狗 → `motor_enabled` 被 `output_task_body()` 的投入条件消费，为 0 时 LQR / 手动腿测 / RL 全部只发零力矩。

**核对**
- 未改极性、零点、控制律。Keil AC5 全量编译默认与 `-DSYSID_ENABLE=1` 均 **103 文件，0 fail / 0 warn**。

**待台架（A2，作者要求"失能保底必须确保"）**
1. 拨杆下位：ch1 bit0 清零、bit4~7 四髋使能位清零（≤100 ms），腿软。
2. 遥控关机：同上，`ctrl_fault` 出现 bit1（=2）。
3. 翻倒：车前倾超 80°，ch1 bit1 亮、bit0 与 bit4~7 清零；扶回 60° 内 bit1 灭、bit0 与 bit4~7 恢复，LQR 若在中位会重新锁存（腿长须在 0.13~0.20）。
4. 拔一台 DM 的 CAN：`ctrl_fault` bit2（=4），四髋失能。

---

## 变更 76 · 拔一路 CAN 后"没全部失能"的解释 + 使能位显示修正（作者台架）

**现象**：拔掉一路 CAN（一条腿的两台 DM），另一路的电机失能了，被拔那路的两台仍显示使能。

**分析**
- 软件侧链路是通的：被拔那路 10 ms 内 `Dm_Is_Online()` 掉 → `FAULT_MOTOR`；100 ms 后该总线 `Can_Bus_Online()` 判 DEAD → `FAULT_CAN`；`enable_request=0` → 失能沿发 `Dm_All_Disable()` + `Dji_All_Stop()`，另一路的电机与轮子随之失能，失能看门狗再每 100 ms 补发。
- 被拔那路的两台**主控物理上够不到**：失能帧发到断了的总线上，控制器只会一直重发（`AutoRetransmission=ENABLE`），电机永远收不到。它们保持断线前的最后状态——断线前正在出力就保持那个力矩。**这一层只能靠电机自己的超时保护**：达妙电机有"CAN 通信超时"参数（上位机参数表里的 TIMEOUT，单位 ms，0 = 不检测），设了以后超过该时间没收到指令，电机自报 0xD 通信丢失并自动失能（Leg2 驱动指南 §7.1 同样描述）。本工程发 MIT 帧是 1 kHz，超时设 20~50 ms 即可，四台都要设，用达妙上位机改、断电重上电生效。
- VOFA 显示上的一个坑：`Dm_Is_Enabled()` 原来只看 `err_raw==1`，电机离线后 `err_raw` 是断线前的旧值，ch1 bit4~7 会一直亮着"使能"，让人以为它没失能。现在离线一律显示未使能（状态未知 ≠ 使能）。

| 文件 | 改动 |
| --- | --- |
| `imcalib/user-lib/dm.c` | `Dm_Is_Enabled()` 加 `Dm_Is_Online()` 条件 |

**输入 / 输出 / 调用链**：`Dm_Is_Enabled()` 只被 `task_comm.c` 的 VOFA ch1 bit4~7 消费，不进控制与使能判定（那两处用的是 `online[]` 与 `Dm_Has_Fault()`）。

**核对**：Keil AC5 全量编译默认与 `-DSYSID_ENABLE=1` 均 **103 文件，0 fail / 0 warn**。

**待作者做（电机侧，AI 做不了）**：达妙上位机逐台把 TIMEOUT 设为 20~50 ms（确认参数名与单位以上位机为准），重上电后复测：拔线 → 被拔那两台应在超时后自己失能（LED 状态 / 重新接回后反馈 `err_raw=0xD`），另一路在 100 ms 内失能。接回后先左下位再上位，主控会先发失能再发使能，0xD 需要清错的话在使能前发 0xFB。

---

## 变更 77 · VOFA 换成"零位 + 速度估计 + 出力"帧，速度估计加符号取反对照（作者：接着下一步）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/lqr_balance.h/c` | `lqr_state_t` 加 `ds_alt` 与 `lpf_vel_alt`；`LQR_State_Update()` 末尾把轮速腿摆补偿符号取反再算一遍速度估计写入 `ds_alt`（只供台架 A/B，不进控制；符号定后连 `vel_leg_comp_sign` 一起删） |
| `imcalib/task/task_comm.c` | VOFA 第三版（下表）；去掉 `hi229.h` include（IMU 帧已完成使命） |
| `md/LQR_PLAN.md` | §六 ①c 改成"两条曲线看哪条平"的测法，并补推车前置检查 |

**帧（32 通道）**

| 通道 | 含义 |
| --- | --- |
| ch0~2 | 在线掩码 / 状态位 / 策略号 |
| ch3~6 | 四髋位置（零点后 rad） |
| ch7~8 | 解算摆角 左 / 右（腿竖直时读零位） |
| ch9~10 | 腿长 左 / 右 |
| ch11~12 | 摆角速度 左 / 右 |
| ch13~15 | 摆角目标 / 腿长目标 左 / 右 |
| ch16~19 | 四髋力矩命令 |
| ch20~23 | F 左 / 右、Tp 左 / 右 |
| ch24~25 | 轮转速 左 / 右（轮轴 rad/s） |
| ch26~27 | 速度估计 `x[1]` / 对照 `ds_alt` |
| ch28~29 | 俯仰角 / 俯仰角速度 |
| ch30~31 | 轮力矩命令 左 / 右 |

**为什么两条一起发**：作者不用调试器切 `vel_leg_comp_sign`，两条候选同屏一次动作就能定，避免来回烧录。

**核对**：未改极性、零点、控制律；`ds_alt` 无消费者。Keil AC5 全量编译默认与 `-DSYSID_ENABLE=1` 均 **103 文件，0 fail / 0 warn**。

**待台架**：§六 ①a（零位读数）、①c（推车方向与量级、摆腿看哪条平）。

---

## 变更 78 · 腿摆角零位由作者台架标定（作者自行改表）

**物理量变更确认（§0.1 单列）**
- 谁 / 何时：作者于 **2026-09-21** 按 §六 ①a（腿吊铅垂线竖直读 `virtual_leg_angle`）自行改 `machine_config.c` 小机器表 `leg_off_phi0`：{−0.13, −0.07} → **{−0.12, −0.17}**。
- 依据：作者台架读数；此前值是大机器换算值抄来的占位。
- 影响：`virtual_leg_angle` 零点、LQR 世界系腿摆角 `x[4]/x[6]` 与站立目标 −0.05 的参考、手动腿测摆角目标 0 的位置。
- AI 未改任何代码；本条只作记录。

---

## 变更 79 · LQR 分级放开改成宏 + VOFA 露出轮输出原值 + 俯仰方向约定更正（作者：直接做 E）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/lqr_balance.c` | 顶部新增 `LQR_DBG_WHEEL_ENABLE`（0）/ `LQR_DBG_HIP_ENABLE`（0）/ `LQR_DBG_LEN_PID_ENABLE`（1）/ `LQR_DBG_TRQ_MAX_WHEEL`（0.5）四个宏，`LQR_Init()` 用它们初始化 `lqr_debug`（轮限幅再与机器表取小）。作者不用调试器，改宏烧录即可分级 |
| `imcalib/task/task_comm.c` | VOFA ch24/25 由轮转速改为 `lqr_state.u[0/1]`（LQR 轮输出原值，未门控未限幅），轮不出力也能看方向 |
| `md/LQR_PLAN.md` | §2.5、§六 ①b/②/③/④/⑥、§八、§十 同步；**更正俯仰方向约定** |

**俯仰方向约定更正（重要）**
- 之前文档与我给作者的 IMU 测法都写"抬头为正"，作者据此定了 `eul_sign[0]=+1`、`gyr_sign[1]=+1`（抬头读正）。复核模型后确认这是**错的**：
  - 本工程 `x[θ_l] = −θ + pitch`：机身低头 β 而腿在世界系竖直时，腿在机体系里足端前偏 β（θ=+β），要 `x[θ_l]=0` 必须 `pitch=+β` → 低头为正。
  - K 表轮行 `θ_b` 增益 −1.11、`θ_l` 增益 −1.40，u = K·(0−x) → 机身/腿"上端前倾"时轮子前驱去接，也要求 θ_b 正 = 低头。
  - Leg2 对原始俯仰取 −1 能站，其注释"Pitch抬头+"与自身公式矛盾，之前抄了注释。
- 按 §0.1 不由 AI 改表：先做 E0（低头看 ch24/25 应为正），作者看到反号后再决定把 `machine_config.c` 小机器 `.imu.eul_sign` 第 0 项与 `.gyr_sign` 第 1 项改成 −1。翻倒检测取绝对值不受影响；四元数只供 RL 重力投影，不动。

**核对**：Keil AC5 全量编译默认与 `-DSYSID_ENABLE=1` 均 **103 文件，0 fail / 0 warn**。

**待台架**：§六 ③ E0 三个动作（手转车体、右摇杆推右、机头下压）看 ch24/25 正负。

---

## 变更 80 · 撤掉分级宏、髋轮全开；作者改定俯仰符号（作者：不需要这么多宏，直接髋轮全开，pitch 极性我改好了）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/lqr_balance.c` | 删除变更 79 加的四个 `LQR_DBG_*` 宏，`LQR_Init()` 回到全开：轮 1 / 髋 1 / 腿长环 1，轮限幅取机器表 1.8、髋 10 |
| `md/LQR_PLAN.md` | §2.5、§六 ③④ 改成"架空看方向 → 落地"两步 |

**物理量变更确认（§0.1 单列）**
- 谁 / 何时：作者于 **2026-09-21** 自行改 `machine_config.c` 小机器表 `.imu`，把俯仰方向改成模型要求的"低头为正"（见变更 79 的推导）。具体值以文件为准。
- AI 未改表；本条只作记录。

**核对**：Keil AC5 全量编译默认与 `-DSYSID_ENABLE=1` 均 **103 文件，0 fail / 0 warn**。

**待台架**：§六 ③（架空看方向）→ ④（落地）。

---

## 变更 81 · 速度估计改成 Leg2 的卡尔曼 + 加速度前馈；腿长目标改共用默认值；VOFA 第四版（作者：先将速度估计改成 Leg2 那种形式）

| 文件 | 改动 |
| --- | --- |
| `imcalib/user-lib/kalman.c/h`（新增） | 照抄 Leg2_v1 `Code/Tool/kalman.c`：`v += a·dt; p += Q; p ≤ P_MAX; k = p/(p+R); v += k(z−v); p = (1−k)p`。差别：`dt` 每拍传入、P 上限做成字段。已登记进 `MDK-ARM/CtrBoard-H7_ALL.uvprojx` user-lib 组 |
| `imcalib/Algorithm/lqr_balance.h` | `lqr_debug_t` 加 `vel_src`（0 低通 / 1 卡尔曼）、`acc_fwd_sign`；`lqr_state_t` 加 `ds_raw` / `ds_lpf` / `ds_kf` / `a_fwd` / `kf_vel` |
| `imcalib/Algorithm/lqr_balance.c` | 新增 `LQR_Accel_Forward()`：四元数把 `acc_g` 转到世界系，取水平分量投影到机体 x 轴的水平方向 ×9.81；`LQR_State_Update()` 两条速度并行算，`x[1] = vel_src ? ds_kf : ds_lpf`；`LQR_Init()` 初始化卡尔曼（P0 0.1 / Q 0.005 / R 0.01 / P_MAX 0.5，同 Leg2 `Body.h`），`vel_src=1`、`acc_fwd_sign=+1` |
| `imcalib/Algorithm/lqr_balance.c` | 作者同批：`LQR_Enable_Latch()` 两腿腿长目标改锁到 `LQR_LEG_LEN_INIT`（共用一个值，不再各锁各的实测——之前投入时两腿目标不一致，拨轮同步升降永远拉不平）；位移目标 / 腿摆角目标暂改 0 供测试，Leg2 原值 −0.12 / −0.05 待零点定后恢复 |
| `imcalib/task/task_comm.c` | VOFA 第四版：ch3~6 改原始解码角 `dm.pos_rad[]`（标零点用）；ch12 = `ds_kf`、ch13 = `a_fwd`（原右摆角速度 / 摆角目标）；ch30/31 = 横滚角 / 横滚补偿力（原轮力矩命令）。通道注释同步 |
| `imcalib/Algorithm/leg_balance.h` | 作者同批：腿长 KP 1000 → 1500（调参，非 Leg2 值） |
| `md/LQR_PLAN.md` / `md/AGENTS.md` | §2.8 卡尔曼说明与验收；文件树、模块表登记 kalman |

**数据流**

```
imu_state.quat / acc_g ──► LQR_Accel_Forward() ──► a_fwd (×acc_fwd_sign)
                                                        │
wheel_vel + 腿运动学 ──► ds_raw ──┬─► Lowpass α=0.3 ──► ds_lpf ──┐
                                 └─► Kalman(a_fwd, ds_raw) ──► ds_kf ──┼─► vel_src ──► x[1] ──► 位移积分 / K 表
```

**一处如实说明**：Leg2 这组参数下卡尔曼稳态增益 k = 0.5，本质是"α=0.5 的低通 + 每拍 a·dt 的前馈"，加速度项贡献很小；相对旧路径的主要变化是去掉 α=0.3 的滞后。它解决不了两轮不对称（绕圈）那类问题。

**未动物理量**：`dm_sign` / `dji_sign` / `dm_zero` / `leg_off_phi0` / `.imu` 全未改。`acc_fwd_sign` 是新加的运行时 A/B 字段，默认 +1，由作者台架定后写死。

**核对**：Keil AC5 `lqr_balance.c` / `task_comm.c` / `leg_balance.c` / `task_actuation.c` 默认与 `-DSYSID_ENABLE=1` 各 0 err 0 warn；`kalman.c` 用 `simple-function.c` 同参数编译 0 err 0 warn。未链接、未上机。

**待台架**：LQR_PLAN §2.8 三步（静止零漂 → 推车看加速度符号 → 投入对比）。

---

## 变更 82 · 开偏航角环 + 转向通道取负 + VOFA 露出偏航/轮速（作者：先把 yaw 的角度环开起来试一下；yaw 遥控输入是错误的，也要改过来）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/lqr_balance.h` | `lqr_debug_t` 加 `yaw_hold`（默认 1）；`lqr_state_t` 加 `yaw_tgt` |
| `imcalib/Algorithm/lqr_balance.c` | 新增 `LQR_Wrap_Pi()`；`LQR_Enable_Latch()` 投入时 `yaw_tgt = x[2]`；`LQR_Target_Update()` 转向摇杆非零时 `yaw_tgt` 跟随当前角、`target[2] = yaw_tgt`、**`target[3] = -axis_yaw × 5.0`**；`LQR_Control_Update()` 偏航角项由"一律跳过"改成 `yaw_hold ? K·wrap(target−x) : 跳过` |
| `imcalib/task/task_comm.c` | VOFA：ch3/ch4 = 偏航角 / 偏航角目标（原左髋原始角）；ch5/ch6 = 轮速反馈 左/右（原右髋原始角）；ch27 = 偏航角速度 x[3]（原 `ds_alt`，字段保留供调试器） |
| `md/LQR_PLAN.md` | §2.3 / §2.5 / §2.6 / §八 / §十 同步 |

**为什么**：偏航角速度环只有阻尼（腿长 0.15 时每 rad/s 仅 ±0.25 N·m 差动），两轮摩擦或受载有一点不对称就会稳定慢转，阻尼环不会把朝向拉回来。K 表偏航角一列本来就有增益，开它相当于加回正。Leg2 同表但注释掉了这一环。

**符号变更单列（§0.1）**：转向通道 `axis_yaw` 取负，依据是作者台架"右推左转"（2026-09-21）+ Leg2 `app_rc.c` 同样取负。只改遥控目标的符号，不改任何反馈极性；偏航角/角速度反馈的极性未动。

**核对**：AC5 `lqr_balance.c` / `task_comm.c` / `leg_balance.c` / `task_actuation.c` 默认与 `-DSYSID_ENABLE=1` 各 0 err 0 warn（输出到临时目录）。未链接、未上机。

**待台架**：投入站立看 ch3 与 ch4 之差是否收敛、绕圈是否消失；右摇杆推右车应右转。要退回 Leg2 原样：调试器 `yaw_hold = 0`。

---

## 变更 83 · 偏航角与偏航角速度两列默认不参与（作者：先关掉偏航角和角速度，看关闭后的区别）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/lqr_balance.h` | `lqr_debug_t` 加 `yaw_rate_hold` |
| `imcalib/Algorithm/lqr_balance.c` | `LQR_Init()`：`yaw_hold` 默认 1 → **0**，`yaw_rate_hold` 默认 **0**；`LQR_Control_Update()`：`j == LQR_X_DPHI && !yaw_rate_hold` 时跳过该列 |
| `md/LQR_PLAN.md` | 新增决策 10；§2.3 φ/dφ 行、§2.5 开关说明、§八、§十 同步；顺带改正过时值（站立目标 0/0、腿长 KP 1500、机器区间 0.09~0.21 → LQR 实际 0.13~0.21） |
| `md/IO_CHAINS.md` | 遥控映射行注明两偏航列默认不参与 |

**为什么**
- 作者要对比"完全不控偏航"与现状的区别，判断绕圈是否来自偏航环。不注释代码而加开关，是为了台架上三种状态可直接切换：都 0 = 偏航完全不控（转向摇杆无效）；`yaw_rate_hold=1` = Leg2 原样只有角速度阻尼；再 `yaw_hold=1` = 变更 82 的角度环。
- 同一批曾按作者要求把 LQR 区间改为只取机器表、投入腿长目标改 0.13，作者当天撤回，已还原为"机器表 ∩ K 表域"与 0.18，本条只保留偏航开关。

**输入 / 输出 / 调用链**
- `lqr_debug.yaw_hold` / `yaw_rate_hold` → `LQR_Control_Update()` 求和循环 → `u[]`；两列关闭时 `target[2]/[3]` 与 `x[2]/[3]` 仍在算，只是不进力矩。

**核对**
- 未改极性、零点、MIT 量程、镜像、IMU 表、`MACHINE_DEFAULT`、机器表区间、K 表域宏。工作区里 `machine_config.c` 零点归零与 `task_comm.c` ch3~6 改四髋零点后角是作者自己在标零点，本条未动。
- AC5 编译：`lqr_balance.c` / `leg_balance.c` / `task_actuation.c` / `task_comm.c` 默认与 `-DSYSID_ENABLE=1` 各 **0 err 0 warn**（`-o` 到临时目录）。未链接、未上机。

**待台架**
- 两偏航列都关时看绕圈是否仍在；在就不是偏航环的锅；不在就依次置 `yaw_rate_hold=1`、`yaw_hold=1` 看哪一步把绕圈带回来，对比 ch27 与偏航角。

---

## 变更 84 · Leg2 `Leg_Position` 并排跑上 VOFA + 三角函数改 CMSIS-DSP + 两套工程挂 DSP 库（作者：并排跑一起做，看是否是解算问题；把目前的也换成 DSP 库）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/leg_solver.c` | 顶部加 `LEG_TRIG_LIBM` 开关（默认 0）：`cosf/sinf/sqrtf` 全部换成 `LEG_COSF/LEG_SINF/Leg_Sqrtf` 宏，0 = `arm_cos_f32/arm_sin_f32/arm_sqrt_f32`（查表，同 Leg2），1 = math.h；文末新增 `Leg_Position_Leg2(phi1, phi4)`，逐行照抄 Leg2 `Code/Matlab/Leg_Position.c:87-107`，杆长常数写死，只返回腿长 |
| `imcalib/Algorithm/leg_solver.c`（补） | 作者 eIDE 编译报 `cannot open source input file "arm_math.h"`（eIDE 包含目录没有 `Middlewares/ST/ARM/DSP/Inc`）→ include 改为相对路径，最终指向 `"../../Drivers/CMSIS/DSP/Include/arm_math.h"`（1.6.0，与三个源文件同一版本），同 `task_actuation.c` 的 `../Sysid/` 写法，两套工程都不依赖包含目录 |
| `imcalib/Algorithm/leg_solver.h` | 声明 `Leg_Position_Leg2()` |
| `imcalib/task/task_comm.c` | VOFA 第五版：ch12/13 由 `ds_kf` / `a_fwd` 改为 Leg2 解算的左/右腿长（左 `(pos_zero[前左] + π, pos_zero[后左])`，右 `(−pos_zero[后右] + π, −pos_zero[前右])`，负号撤销驱动层右侧取反、再按 Leg2 前后互换）；ch3~6 注释改成作者已改的四髋零点后角；头注释同步 |
| `imcalib/user-lib/arm_sin_f32.c` / `arm_cos_f32.c`（新增） | 照抄 `Drivers/CMSIS/DSP/Source/FastMathFunctions` 1.6.0，只把两行 include 改成相对路径 `../../Drivers/CMSIS/DSP/Include/...`（diff 确认其余逐字相同） |
| `imcalib/user-lib/arm_sin_table_f32.c`（新增） | 从 `arm_common_tables.c` 第 56926~57021 行截出 `sinTable_f32[513]`，逐行一致；不编整个 `arm_common_tables.c`（armcc 不拆数据段，700 KB 表会整段进 flash） |
| `MDK-ARM/CtrBoard-H7_ALL.uvprojx` | 三个新源登记进 `imcalib/user-lib` 组（`kalman.c` 之后）；本条先前加的 `Lib` 组 .lib 条目已撤掉 |
| `.eide/eide.yml` | **AI 未能改动**：手改三次（加库、加包含目录、删多余库）都在几秒到几分钟内被正在运行的 eIDE 扩展用内存里的模型覆盖回去。改成源码方式后 eIDE 不需要任何配置改动（`srcDirs` 自动扫 `imcalib/user-lib`，include 是相对路径）。作者在面板里加的 `Drivers/CMSIS/DSP/Lib/ARM` 下 19 个 .lib 现在无害（对象先于库解析，`arm_sin/cos_f32` 由本工程 .o 提供，试链接确认 0 个库成员被拉入），但建议在面板里删掉，别留着 |
| `md/LQR_PLAN.md` / `md/AGENTS.md` / `md/IO_CHAINS.md` | §2.9 新增对照说明；§2.8 验收改为调试器看 `ds_kf`/`a_fwd`；§八/§十 各加一行；AGENTS 模块表与关键约束加 CMSIS-DSP 接法；IO_CHAINS §6 一句 |

**为什么**
- 作者把零点、摆角偏置照抄 Leg2 后腿长仍是 0.10~0.20，怀疑解算。数值上两边几何已逐点一致（3e−9 m），但要在台架上把"解算"这个嫌疑彻底关掉，最直接的是 Leg2 原函数并排跑、两条线叠着看；不替换控制链是因为值相同时替换没有信息量，而 Leg2 那套 `Leg_Tougue` 的 Tp 符号与右腿输出序和本机不同，整套换进控制链会出力错向。
- 三角函数改 DSP 是作者要求"目前的也换成 DSP 库"；顺带消掉 libm 与查表之间 ≤ 4e−6 m 的差异，两条线可以精确重合。
- 先走了预编译库路线（`arm_cortexM7lfdp_math.lib`），但 eIDE 那侧 AI 改不动配置、作者又把整目录 19 个库都挂了上去（armlink 静默取第一个软浮点库），作者说"这些你做就好"→ 改成源码方式：三个源文件进 `user-lib`，两套工程零配置。
- CubeMX 勾了 ALGOBUILD 的 DSP Library，但它只生成 1.7.0 头文件，Keil/eIDE 两套构建之前都没链任何 DSP 对象（compile_commands 里无 DSP 源，.lnp 里无 .lib）。挂 `Drivers/CMSIS/DSP/Lib/ARM` 下 1.6.0 的预编译库而不是照 Leg2 编源文件：库按对象/数据分段（`.rodata.sinTable_f32` 独立），只拉 2 KB；armcc 编 `arm_common_tables.c` 会把 700 KB 表整段拉进 flash。

**输入 / 输出 / 调用链**
- `motor_state.dm.pos_zero_rad[]` → `Leg_Position_Leg2()`（task_comm，每拍）→ `dbg[12]/[13]` → VOFA。不进 `leg_l/leg_r`、不进任何控制。
- `Leg_Solve()` 内部 `LEG_COSF/LEG_SINF/Leg_Sqrtf` → `arm_cos_f32/arm_sin_f32/arm_sqrt_f32`（`arm_cortexM7lfdp_math.lib`；sqrt 是头文件内联 VSQRT，负数入参出 0，与原 `disc` 钳位逻辑不冲突）。

**核对**
- 未改极性、零点、MIT 量程、镜像、`+LEG_PI`、IMU 表、机器表、控制律；`Leg_Solve` 的几何/雅可比/力映射公式一行未动，只换三角函数实现。作者工作区里 `dm_zero`/`leg_off_phi0` 归零与 ch3~6 改四髋角是作者自己在标零点，本条只同步注释。
- AC5 编译：`leg_solver.c` / `task_comm.c` 默认、`-DSYSID_ENABLE=1`、`-DLEG_TRIG_LIBM=1` 三配置各 **0 err 0 warn**（`-o` 到临时目录）；`fromelf -s` 确认 DSP 版 `leg_solver.o` 只引用 `arm_sin_f32/arm_cos_f32/__hardfp_atan2f`，不再引用 `sinf/cosf/sqrtf`。
- **整机试链接**：用 eIDE 现成 `.obj` + 新两个 .o + .lib，改 `.lnp` 输出到临时目录，AC5 armlink `--strict` **0 err 0 warn**；ROM 743076 → 745856 B（+2.7 KB = 正弦表 2052 B + 两函数 284 B + `Leg_Position_Leg2`）。作者的 `.axf/.map/.obj` 未动。未下载、未上机。
- include 改相对路径后，用 eIDE 原样包含目录（不含 `DSP/Inc`）重编 `leg_solver.c` 三配置 + `task_comm.c` 各 **0 err 0 warn**；从 `MDK-ARM/` 目录按 Keil 的 `../` 源路径编也 0/0。
- **按 eIDE 现状挂全部 19 个 DSP 库试链接：armlink 不报错**，但 `arm_sin_f32/arm_cos_f32` 取自列表第一个 `arm_ARMv8MBLl_math.lib`（ARMv8-M 基础版、无 FPU 软浮点），不是给 M7 双精度 FPU 编的那份；只挂 `arm_cortexM7lfdp_math.lib` 时取自正确的库，0 warn，ROM 745856 B。
- **源码方式（最终）**：三个新源 + `leg_solver.c` + `task_comm.c` 默认 / `-DSYSID_ENABLE=1` / `-DLEG_TRIG_LIBM=1` 三配置各 0 err 0 warn；从 `MDK-ARM/` 按 Keil 路径编也 0/0；`sinTable_f32` 独立 `.constdata` 2052 B。整机试链接：不挂库 0 err 0 warn；按作者现状挂 19 个库也 0 err 0 warn 且 0 个库成员被拉入（`arm_sin/cos_f32` 来自本工程 .o）。作者 `.axf/.map/.obj` 未动。

**待台架**
- 腿从趴地摆到垂到限位，ch9 与 ch12、ch10 与 ch13 应全程重合；重合 = 解算排除，剩下只有电机内部零点与尺子；不重合把两条线截图。
- Keil 侧第一次编译看链接是否报 `arm_sin_f32` 未定义（若 Keil 没读到 Lib 组新条目，重新打开工程）。
- eIDE 侧：直接重新编译即可，不需要改任何配置；`Lib` 文件夹里作者加的 19 个 DSP 库建议在面板里全部删掉（现在无害，但没用）。eIDE 打开状态下不要手改 `eide.yml`。

---

## 变更 85 · MATLAB 增益表管线阶段 0：复现板上现表（作者：开始执行 matlab 接入计划，尝试复现目前这个 K 表）

| 文件 | 改动 |
| --- | --- |
| `tools/matlab/`（新增，26 个文件） | 入口 `run_all.m`；`config/` 机器表（local / chuanliantui，与 `machine_config.c` 同思路可切换）+ Q/R 表；`model/sjtu5/` Leg2 5 方程模型（符号推导 → `matlabFunction` 缓存）；`design/` 网格扫描 + c2d + dlqr + poly22 拟合；`emit/` 直接写 C；`check/` 五项核对；`ref/` Leg2 原件副本；目录职责见 `LQR_MATLAB_PLAN.md` §四 |
| `tools/matlab/output/lqr_gain_table.c`、`report_local-sjtu5-20260922-0945.txt` | 复现产物与全程报告 |
| `.gitignore` | 加 `/tools/matlab/cache`、`/tools/matlab/*.log` |
| `tools/matlab/LQR_MATLAB_PLAN.md` | 状态、目录树、阶段 0 结果、进度表、§十二 运行方式；纠正"升 poly33 要改板上求值器"的说法（多项式在生成的 C 里，板上不用改） |
| `md/AGENTS.md` | 文件树加 `tools/matlab/` 一行 |

**结果（表号 local-sjtu5-20260922-0945）**
- A/B：本管线符号推导 vs Leg2 `AB_WBR_gen.m`，9 个腿长组合相对差 **0**。
- K：拟合 K vs `ref/LQR_K_WBR.m`（板上现表的 MATLAB 原型），121 网格点最大相对差 **3.5e-11**，7 个非网格/非对称点 **1.3e-11**。
- C：生成的 C vs 板上 `imcalib/Algorithm/lqr_gain_table.c`，21×21 点 × 40 元素最大相对差 **1.6e-8**（float 舍入）；两份文件的系数逐项相同（如 `K_sym[0]` 的 1.35945499 / 3.81089163 / 2.06506157 / 7.03364897 / 0.394434452 / 0.895083785）。
- 闭环：拟合 K 全网格离散谱半径最大 0.998929；`check_machine_config`（wheel_r / 杆长 / 区间 / 限幅 / CTRL_DT / K 表域）全一致；`check_c_compile` AC5 0 err 0 warn（`-o` 到临时目录）。
- 结论：**板上现表 = Leg2 `WBR_modeling.mlx` 那组 Q/R（`[16000 1200 1000 870 2500 365 2500 365 10500 2000]` / `[5480 5480 650 650]`）+ sjtu5 模型 + 小机器参数 + 0.13:0.01:0.23 网格 + poly22**，本管线已能 1:1 出这张表，导出格式、下标、符号全对。

**顺带看到（信息，未处理）**
- 现表 poly22 对 dlqr 真值的拟合残差最大 0.026（相对 1.5%，K(4,1) 位移→右髋），是现表自带的误差；要压就 `fit_K(S, 3)`，板上不用改。
- 作者 2026-09-22 已把 `machine_config.c` 小机器区间改成 0.13~0.23（同批把 `dm_zero` / `leg_off_phi0` 归零在标零点），MATLAB 机器表已镜像 0.13/0.23；零点归零那两项与本管线无关、未动。

**输入 / 输出 / 调用链**
- `run_all` → `machine_load` / `lqr_weights` → `scan_grid`（`model_AB` → `AB_sjtu5_gen` → `c2d` → `dlqr`）→ `fit_K` → `emit_gain_table` → 五个 `check_*` → `output/report_*.txt`（diary）。
- 复现模式判定：机器 local + 模型 sjtu5 + Q/R tag `mlx-2026-07-27` + 取行 nearest；此时 `check_vs_ref` 与 `check_c_vs_board` 是判据，否则只作信息（调参后 K 当然和现表不同）。

**核对**
- 板上代码零改动：`imcalib/`、`Core/` 未碰；`build/CtrBoard-H7_ALL/.obj/` 未被写入。
- 未改极性、零点、量程、镜像、IMU 表、`MACHINE_DEFAULT`。

**下一步（阶段 1）**
- `tune/whatif.m` + `tune/sim_real.m`（搬 PSO v2 真实层仿真），改 Q/R → 出表 → 核对 → 只换 `lqr_gain_table.c` 一个文件上板 → 本账本记"表号 + Q/R + 现象"。

---

## 变更 86 · MATLAB 管线精简：28 → 19 个文件（作者：有些重复冗余，三点都做）

| 项 | 改动 |
| --- | --- |
| 合并 | `config/machine_default.m` + `model_default.m` → `config/pipeline_config.m`（多带 `fit_order`）；`config/machine_pending.m` 并入 `machine_load.m`（`m.pending`）；`model/sjtu5/leg_row_sjtu5.m` 并入 `sjtu5_param_vec.m` 局部函数 |
| 删除 | `check/check_c_vs_board.m` + `check/compare_c_tables.py`（与 `check_vs_ref` 同证"和现表一样"，还依赖 Python；阶段 0 证据留在变更 85 与 `output/report_local-sjtu5-20260922-0945.txt`）；`ref/AB_WBR_gen.m`（与 `cache/AB_sjtu5_gen.m` 逐字相同，已证差 0）；`model/newton15/README.md` |
| `check/check_vs_ref.m` | 去掉 A/B 对照段，只留 K 对照；只在复现模式跑，平时跳过，结果行显示 `-` |
| `run_all.m` | 读 `pipeline_config`；报告固定 `output/report_latest.txt` 每次覆盖，拷上板时手动另存 `report_<表号>.txt`；结果行改四项 |
| `emit/emit_gain_table.m` | 待实测清单改读 `m.pending` |
| `ref/README.md`、`LQR_MATLAB_PLAN.md` §四/§5.4/§5.5/§十/§十二、`md/AGENTS.md` | 同步 |

**核对**：重跑 `run_all`（复现模式）四项全过，K 对照仍为网格 3.5e-11 / 非网格 1.3e-11，AC5 0 err 0 warn；板上代码未动。

---

## 变更 87 · MATLAB 管线再精简：19 → 2 个代码文件，只留闭环检查（作者：选机器 / Q/R / 网格拟合 / 写 C 统一进 run_all；板上对比、复现对照、AC5 编译都删；按名字取表没必要）

| 项 | 改动 |
| --- | --- |
| `tools/matlab/run_all.m` | 一个文件按节排：§1 选择（机器 / 模型 / 拟合阶次）、§2 Q/R + tag、§3 两张机器表 `machine_table()`、`scan_grid` / `fit_K` / `eval_K_poly` / `check_closed_loop` / `emit_gain_table` 全为局部函数；表号改为 `机器-模型-qr_tag-时间`；闭环未通过时 C 照写但控制台与文件头标"不要上板" |
| `tools/matlab/model_AB.m` | 模型接口 + `sjtu5_param_vec` / 取行 + `build_sjtu5` 符号推导，全为局部函数；缓存路径改为同级 `cache/` |
| 删除 | `config/`（`pipeline_config`、`machine_load`、`machine_local`、`machine_chuanliantui`、`lqr_weights`）、`model/`、`design/`、`emit/`、`check/`（含板上机器表比对、AC5 编译、Leg2 原表对照）、`ref/`（`LQR_K_WBR.m`、README） |
| `tools/matlab/LQR_MATLAB_PLAN.md` | §四目录、§5.1/5.2/5.4/5.5、§六阶段 1/2 文件名、§七、§八、§十、§十二 同步 |

**为什么**：作者嫌文件多；阶段 0 已证复现，Leg2 原表对照与板上字段比对"对比过一次就够"；AC5 单文件编译整项目编译时同样会查。闭环谱半径保留，是唯一能在烧板前判定"这组 Q/R 站不站得住"的检查。

**代价（已写进计划 §四 / §八）**：板上 `machine_config.c` 改了轮径 / 杆长 / 区间 / 限幅，`run_all.m` §3 要手动跟着改，没有脚本再替你查。

**核对**：删缓存后重跑 `run_all`（复现组 Q/R）：符号推导 13.3 s、扫描 121/121、闭环谱半径 0.998929、拟合残差 0.0256；生成 C 的系数集与板上 `lqr_gain_table.c` 逐项相同（仅打印位数不同，如 `0.717670977F` vs `0.717671F`，同一 float）。板上代码未动。

---

## 变更 88 · MATLAB 管线按"多久改一次"拆成四个文件（作者：机械参数不常改单独放；网格拟合等验证过的中间计算也单独放）

| 文件 | 改动 |
| --- | --- |
| `tools/matlab/run_all.m` | 只剩 §1 选择、§2 Q/R + tag、一行调 `build_gain_table`（35 行）|
| `tools/matlab/machine_table.m`（新） | 两张机器表从 `run_all` 搬出，内容不变 |
| `tools/matlab/build_gain_table.m`（新） | `scan_grid` / `fit_K` / `eval_K_poly` / `check_closed_loop` / `print_K_nominal` / `emit_gain_table` / `fmt_float` 从 `run_all` 搬出，加入口函数负责目录、表号、diary、打印、缓存、结论 |
| `tools/matlab/model_AB.m` | 未动 |
| `tools/matlab/LQR_MATLAB_PLAN.md` | §四目录、§5.1/5.4、§八、§十、§十二 同步 |

**为什么**：作者定"常改的（Q/R）、不常改的（机械参数）、验证过不该改的（中间计算）分开"，日常只看 `run_all.m`。报告仍走 diary：命令行窗口照常显示，`report_latest.txt` 只是副本。

**核对**：重跑 `run_all`（复现组 Q/R）：扫描 121/121、闭环谱半径 0.998929、拟合残差 0.0256；生成 C 的 40 行 `K_sym[]` 与变更 87 那次逐字节相同。板上代码未动。

---

## 变更 89 · 管线自动部署 + 去掉 Q/R 标签（作者：拷贝这一步直接在 MATLAB 端做；Q/R 值直接写进表头就行）

| 文件 | 改动 |
| --- | --- |
| `tools/matlab/run_all.m` | §1 加 `CFG.deploy = true`；§2 删 `qr_tag` |
| `tools/matlab/build_gain_table.m` | 去掉 `qr_tag` 参数与文件头 "Q/R 组" 行（Q、R 两行本来就在）；表号改为 机器-模型-时间；闭环通过且 `CFG.deploy` 时 `copyfile` 覆盖 `imcalib/Algorithm/lqr_gain_table.c`，未通过不动板上文件 |
| `imcalib/Algorithm/lqr_gain_table.c` | 首次由管线自动覆盖（表号 local-sjtu5-20260922-1258，Q/R 为现表那组）。与被覆盖的 Coder 版逐系数按 float 比对 120 个值全同，只是打印位数与文件头不同；**板上数值零变化** |
| `LQR_MATLAB_PLAN.md` §四 / §5.5 / §十二 | 同步 |

**核对**：重跑 `run_all` 闭环通过，自动覆盖成功；`.h` 未动。整项目编译由作者在 eIDE 做。

---

## 变更 90 · 遥控指令统一解算到 `rc_command`，策略仲裁拆成短函数（作者：输入指令统一在外面做，内部直接读；前进统一 ch1，死区 10）

| 文件 | 改动 |
| --- | --- |
| `imcalib/user-lib/rc_command.c/h` | **新增**。`rc_command_t`：`vel`(ch1 右摇杆Y) / `yaw`(ch0 右摇杆X) / `len`(拨轮) / `ang`(ch3 左摇杆Y) 四轴去死区归一化 [-1,1] + `s1` / `s2` / `online`；`Rc_Command_Update(cmd, dr16快照)`，离线全零。死区常量 `RC_DEADBAND_VEL=10`、其余 20。Keil 已登记进 `imcalib/user-lib` 组，eIDE 靠 `srcDirs` 自动扫到 |
| `imcalib/task/robot_control.c/h` | 全局 `rc_command_t rc_command`；`robot_control.h` include `rc_command.h` |
| `imcalib/task/task_comm.c` | `Remote_Control_Update()` 在 `DR16_Snapshot()` 后调 `Rc_Command_Update(&rc_command, &remote)`，全机唯一的摇杆解算点 |
| `imcalib/Algorithm/lqr_balance.c/h` | `LQR_Target_Update()` 第二参数 `const dr16_t *` → `const rc_command_t *`；删 `LQR_RC_Axis()` 与 `LQR_RC_DEADBAND`；`axis_vel/yaw/len` 直接取 `cmd->vel/yaw/len`；头文件不再 include `dr16.h`。换算常数、偏航锁存、腿长积分与夹取、转向取负一律不动 |
| `imcalib/task/task_policy.c` | 删 `RC_Axis()` / `RC_Axis_Wheel()` / `DR16_Snapshot()`；`Remote_Command_Apply()` 只读 `rc_command`：`vx_cmd ← vel`、`yaw_cmd ← yaw`、`height_cmd ← len`；手动偏移 `thigh ← ang`、`shank ← len`、`wheel ← vel` |
| `imcalib/task/task_actuation.c` | 结构整理，行为不变：新增 `output_zero()`（原五处重复的零力矩四行）、`strategy_from_remote(cmd, prev)`（拨杆 → 策略，含"左中+右非中 = 保持上一策略"）、`lqr_engage_update(manual)`（原 136~156 行投入锁存）、`output_task_rl()`（原 RL 分支）；`output_task_body()` 改为 状态估计 → 定策略 → `switch` 三选一。`output_task_lqr/_manual()` 不再收 `dr16_t`，直接传 `&rc_command`；文件不再 include `dr16.h` |

**行为变化（作者拍板）**：
- RL 观测的前进指令 `vx_cmd` 由 ch3（左摇杆 Y）改为 ch1（右摇杆 Y），与 LQR、RL 手动轮偏移统一。推理尚未接进任务，当前无实际影响。
- ch1 死区 20（LQR）/ 100（RL 手动轮偏移）统一为 **10**；ch0 / 拨轮 / ch3 仍为 20。

**输入 / 输出 / 调用链**：`commTask` 1 ms：`DR16_Process()` → `DR16_Snapshot()` → `Rc_Command_Update()` → `rc_command`（全局）。消费端：`actuationTask` 1 kHz `output_task_body()` → `strategy_from_remote(&rc_command)` 定挡位 → LQR / 手动腿测 `LQR_Target_Update(&rc_command)`；`policyTask` 10 ms `Remote_Command_Apply()` 读 `rc_command` 写 `input_command` 与手动偏移。`Sysid_Mode_Run()` 的预压拨轮仍自己读 `DR16_Snapshot()`，未动。

**核对**：armcc 编 `rc_command.c` / `task_comm.c` / `task_actuation.c` / `task_policy.c` / `robot_control.c` / `lqr_balance.c` / `leg_balance.c` / `sysid_mode.c`，默认与 `-DSYSID_ENABLE=1` 各一遍，0 fail / 0 warn，`-o` 指向临时目录。`grep` 无 `LQR_RC_DEADBAND` / `LQR_RC_Axis` / `RC_Axis` 残留。整项目链接与台架由作者做；仲裁语义与摇杆映射 **待台架** 复核。

---

## 变更 91 · 仲裁改为"左拨杆选模式、右拨杆中位投入"，嵌套 switch，测试模式退出仲裁（作者：左下全停、左中 LQR、左上 RL；右中代表开始用对应模式解算输出；测试代码 #if 0）

| 文件 | 改动 |
| --- | --- |
| `imcalib/task/inc/robot_control.h` | 枚举末尾加 `CTRL_STRATEGY_DISABLE`（=4），原 0~3 编号不变 |
| `imcalib/task/task_actuation.c` | `strategy_from_remote(cmd)` 只看 s1：中 = LQR、上 = RL(`CTRL_STRATEGY_MANUAL`)、下 / 离线 = DISABLE，去掉"保持上一策略"；`output_task_body()` 外层 `switch (strategy)`，LQR / RL 两路内层 `switch (rc_command.s2)`：右中 → 投入锁存 + 出力，其他位 → 零力矩；新增 `output_task_lqr_idle()`（清投入标志 + `Leg_Balance_Reset` + 零力矩），DISABLE 也走它。sysid 的 include / `CTRL_STRATEGY_SYSID` 分支改成 `#if 0`，仲裁不再有测试入口 |
| `imcalib/task/task_comm.c` | VOFA ch2 注释加 `4 失能`，`0 手动` 改标 `0 RL` |

**拨杆语义**：

| 左 s1 | 右 s2 | 结果 |
| --- | --- | --- |
| 下 / 离线 | 任意 | DISABLE：零力矩（电机失能由 comm 的 `rc_enable` 决定） |
| 中 | 中 | LQR 投入：`lqr_engage_update` 锁存 → `output_task_lqr()` |
| 中 | 上 / 下 | LQR 已选未投入：零力矩，状态估计照算 |
| 上 | 中 | RL 投入：`output_task_rl()` |
| 上 | 上 / 下 | RL 已选未投入：零力矩 |

**输入 / 输出 / 调用链**：`rc_command.s1` → `strategy_from_remote()` → `ctrl_strategy`（VOFA ch2）；`rc_command.s2` 只在 `output_task_body()` 内层 switch 判投入。手动腿测 `CTRL_STRATEGY_LQR_MANUAL` 仍无遥控入口。

**核对**：armcc 编 `task_actuation.c` / `task_comm.c` / `robot_control.c` / `task_policy.c` / `sysid_mode.c`，默认与 `-DSYSID_ENABLE=1` 各一遍，0 fail / 0 warn。拨杆语义 **待台架**。

---

## 变更 92 · LQR 投入不再查实测腿长（作者：先把腿长界限放开，后面再做倒地自起状态机；后续参考 Leg2）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/lqr_balance.c` | `LQR_Enable_Latch()` 删掉 "实测腿长必须在 K 表域 0.13~0.23 内才投入" 的检查，同 Leg2（Leg2 只夹目标、不查实测）；签名不变，三个参数 `(void)` |
| `imcalib/task/task_actuation.c` | 使能沿注释同步 |

**现象**：左中 + 右中不站起。VOFA ch0=255（全在线）、ch1=253（使能 / 两腿有效 / 四髋使能，bit8 未投入）、ch2=1（LQR 已选）→ 卡在 `LQR_Enable_Latch()`：趴地腿长 < 0.13 每拍被拒。原流程靠手动腿测先撑腿到 0.15，现仲裁无手动腿测入口。

**未动**：`LQR_Len_Range()` / `LQR_K_LEN_MIN/MAX` 仍用于 `LQR_Target_Update()` 拨轮目标夹取；`LQR_LEG_LEN_INIT = 0.18` 投入目标。

**风险**：投入瞬间腿长目标 0.18 与趴地实测差 ~0.05 m，腿长 PID（KP 1000）首拍出几十牛伸长力，K 表在 0.13 以下为多项式外推。Leg2 同样做法。**待台架**。

**核对**：armcc `lqr_balance.c` / `task_actuation.c` 默认与 `-DSYSID_ENABLE=1` 各一遍。

---

## 变更 93 · 输出任务改三层：解算只算、分发唯一（作者：得到解算结果后统一分发给电机，不要每个解算各自一个分发函数）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/torque_output.h` | `torque_output_t` 加 `valid` 字段（0 = 本拍无有效命令）；加 `Torque_Output_Clear()` |
| `imcalib/task/task_actuation.c` | 重写为三层：① `LQR_State_Update()` 每拍估计（不变）；② `solve_lqr / solve_lqr_manual / solve_rl` 只写 `torque`、置 `valid`，不碰驱动；③ `output_dispatch()` 唯一下发点：`valid=0` 或 `torque_output_enabled=0` → `Dm_Send_Zero + Dji_All_Stop`，否则 `Dm_Send_Torque + Dji_Send_Wheel_Torque`。删掉 `output_send / output_zero / output_task_lqr / output_task_lqr_manual / output_task_rl / output_task_lqr_idle` 六个函数，`switch` 里 7 处发送点归一为函数末尾一行 |
| `md/LQR_PLAN.md` §2.2 / §2.7 | 同步 |

**为什么**：原来每个策略分支各自 `output_send()` / `output_zero()` / `return`，改输出行为（总开关、限幅、斜坡、极性）要改多处。现在改输出只动 `output_dispatch()`，改控制律只动 `solve_*()`。

**行为零变化**：每种情况下发的数值与之前逐位相同——
- LQR 投入且解算成功 → 原样下发（同 `output_send`）
- LQR 未投入 / 解算失败 / IMU 无效 → 零力矩（同 `output_task_lqr_idle` / `output_zero`），`lqr_idle()` 仍清 `leg_balance` 观测值
- RL 前提不齐 → 零力矩；齐 → 原样下发
- 失能 → 零力矩
- 总输出关 → 零力矩，计算照常
- sysid 分支仍 `#if 0`，接入时改走 dispatch（`sysid_mode.c` 内部 10 处 `Dm_Send_*` 本次未动）

**核对**
- AC5 编译 `task_actuation.c / leg_balance.c / rl_torque.c / task_policy.c / task_comm.c / sysid_mode.c / robot_control.c` 默认与 `-DSYSID_ENABLE=1` 各 0 err 0 warn（`-o` 到临时目录）。
- `grep` 旧函数名：代码与文档无残留。
- `leg_balance.cmd` / `rl_torque.last_torque` 是整结构体拷贝，会多带一个 `valid` 字节，VOFA ch16~19 只读 `dm[]`，不受影响。
- 未改极性、零点、量程、限幅、门控逻辑。未链接、未上机。

**台架**：投入 LQR 看 ch16~19 / ch24~25 与改前同一状态下数值一致；拨杆切换、失能、总输出关三种情况电机应即时零力矩。

---

## 变更 94 · 位移列开关 + 速度目标斜坡（作者：松摇杆后车直接回到最初位置；偏航列关掉仍绕圈，怀疑位移环在推）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/lqr_balance.h` | `lqr_debug_t` 加 `pos_hold`（默认 1）、`vel_ramp`（默认 5 m/s²）；`lqr_state_t` 加 `vel_tgt` |
| `imcalib/Algorithm/lqr_balance.c` | `LQR_Target_Update()`：速度目标不再阶跃，按 `vel_ramp·dt` 每拍逼近摇杆值，小于一步时收口到精确 0；`target[DS] = vel_tgt`。`LQR_Enable_Latch()` 投入时 `vel_tgt=0`。`LQR_Control_Update()` 求和循环 `j == LQR_X_S && !pos_hold` 跳过 |
| `md/LQR_PLAN.md` §2.3 / §2.5 | 同步 |

**为什么**
- 原逻辑"有速度指令不积分、松杆立刻开始积"：松杆瞬间车还有速度，减速滑行的整段路程被记进 `pos`，控制器再把车拉回去，看起来像回到起点。斜坡后减速段目标非 0、不积分，目标到 0 时车已基本停稳。Leg2 同样有 5 m/s² 斜坡。
- `pos_hold` 与 `yaw_hold` 同一思路：运行时关掉位移列，看绕圈是否消失，用来把绕圈归因到位移环还是别处。

**核对**：AC5 `lqr_balance.c / leg_balance.c / task_actuation.c / task_comm.c` 默认与 `-DSYSID_ENABLE=1` 各 0 err 0 warn。未改极性、零点、量程。未链接、未上机。

**台架**
1. `pos_hold=0` 投入站立：绕圈消失 → 位移环在推（两轮摩擦不对称把同向推力变成差动）；仍绕 → 不是位移环。
2. `pos_hold=1`：推杆走 1 m 松开，看 ch5（`pos`）松杆后涨到多少、车停在哪；应停在松杆点附近几厘米内。回到起点 → 报现象。
3. 要退回旧行为：`vel_ramp=0`。

---

## 变更 95 · networkzn1 接入：单模型封装 + 观测参数 + 推理路径 + 关节映射表（作者：可以，你先改）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/rl_policy.c/h` | 四模型封装收成 networkzn1 单模型：`rl_model_t` 只剩 `RL_MODEL_STANDUP`；静态检查 2 输入 (25/125)、2 输出 (6/3)；`RL_Policy_Run()` 读输出 0 动作、输出 1 latent 存 `policy->latent`，DWT 计耗时 `run_us`，计数 `run_ok/run_fail`。头文件新增推理路径参数宏：`RL_CMD_VX_MAX/YAW_MAX/HEIGHT_MIN/MAX`（全 0，待训练侧）、`RL_WARMUP_STEPS`（50）、`RL_ACTION_CLIP`（0 = 不裁）、`RL_LATENT_SIZE` |
| `imcalib/Algorithm/rl_observation.c/h` | 训练侧观测参数进宏（gyro 0.25、cmd [2.0, 0.25, 5.0]、关节速度 0.05、默认角 [−0.06, 0.10, 0.06, −0.10]），`RL_Observation_Param_Init()` 填入并置 `configured=1`；`RL_Observation_Build()` 末尾整体裁剪 ±100 |
| `imcalib/user-lib/machine_config.c/h` | 新增 `rl_map_t`（`sign[6]` / `zero[4]` / `configured`）与机器表字段 `.rl`；两台机器都填 0、`configured=0` |
| `imcalib/Algorithm/rl_torque.c/h` | 参数只剩 STANDUP 一组：`dof_pos = zero + sign × 训练默认角`（按机器表算），Kp/Kd/轮增益全 0 待训练侧；轮速尺度 20 → 10；腿 D 项改为 `− Kd × q̇`（pid 的 Δe 微分置 0）；删 `spin_mode/jump_mode` 与 `act[]` 拷贝 |
| `imcalib/task/inc/robot_control.h` | `action_state_t` 加 `rl_ready`；`rl_control_state_t` 加 `infer_enable` / `infer_phase`；声明 `output_task_rl_engaged()` |
| `imcalib/task/robot_control.c` | 只初始化 STANDUP 参数；`infer_enable=0`、`infer_phase=0`；`Action_State_Clear()` 清 `rl_ready` |
| `imcalib/task/task_policy.c` | 两条路径：`infer_enable=0` 旧手动遥操不变（obs 改用映射后关节，无人消费）；`infer_enable=1` 走 `RL_Infer_Body()`：投入 (`output_task_rl_engaged()`) → `RL_Command_From_Rc()` → `RL_Control_Update_Observation()`（`RL_Joint_Map()` 固件角 → 训练关节，未配置返回 0）→ 预热 N 步零动作 → `RL_Policy_Run()` → 裁剪 → `Set_Last_Action` → 乘 sign → `RL_Action_Publish(rl_ready=1)`；未投入 / 观测无效 / 推理失败发零动作 `rl_ready=0` |
| `imcalib/task/task_actuation.c` | 新增 `rl_engaged` + `output_task_rl_engaged()`（左上 + 右中 + 使能）；`solve_rl()` 前提改为 `base_action_locked || rl_ready` |
| `imcalib/task/task_comm.c` | ch1 加 bit9 RL 已投入；RL 模式覆盖 ch3~6、ch15~31（投影重力 / 状态位 / 动作 / 观测关节 / 力矩 / 耗时），头注释同步 |
| `imcalib/Sysid/sysid_mode.c` | `RL_MODEL_STABLE` → `RL_MODEL_STANDUP`（只为 `-DSYSID_ENABLE=1` 仍可编；测试模式仍 `#if 0`） |
| `MDK-ARM/CtrBoard-H7_ALL.uvprojx` | 去掉残留的 upstairs 四个条目（CubeMX 重生成后只有 networkzn1 是对的） |
| `X-CUBE-AI/App/` | 旧 jump/pin/stable/upstairs 32 个文件由**作者自行删除**（目录时间 22:04，本条未动） |
| `md/RL_OVERVIEW.md` / `AGENTS.md` / `DBUS.md` / `IO_CHAINS.md` | §3.2~3.6、§四、§五、§七同步，新增 §八（训练侧待提供清单 + 开关 + VOFA）；AGENTS 文件树 / 模块表 / §0.1 加 `rl.sign/rl.zero`；DBUS / IO_CHAINS 加推理路径指令行 |

**为什么**
- 训练同学的模型是 2 输出（actions + latent），旧封装按 1 输出写；四个旧网络已不在 Keil 工程，`rl_policy.c` 再引用它们 Keil 链接必失败。
- 训练是串联代理关节，实机是五连杆：符号和零点不能由 AI 定，所以做成机器表 `.rl`，默认未配置整链门控关，与 `.imu` 同思路。
- D 项改用关节速度，是为了和仿真 PD（`Kp(q_des − q) − Kd q̇`）一致，也避免每 10 ms 目标跳变的微分冲击；Kd=0 时与之前完全等价。
- 预热 N 步零动作复现训练侧"首次轮接地前零动作、PD 与历史照跑"。
- `infer_enable` 默认 0：作者当前在小机器上测 LQR，旧手动遥操路径和 LQR 行为都不受影响。

**输入 / 输出 / 调用链**
- `actuationTask` 1 kHz：`rc_command.s1/s2` + `motor_enabled` → `rl_engaged` → `solve_rl()`（要 `base_action_locked || rl_ready`）→ `RL_Torque_Compute()` → `output_dispatch()`。
- `policyTask` 10 ms：`output_task_rl_engaged()` → 指令 → 观测（`machine->rl` 映射）→ 预热 / 推理 → `action_state`（固件动作、`rl_ready`）。
- VOFA 由 `commTask` 读 `rl_control.*`、`action_state.rl_ready`、`torque_state.last_torque`（未投入时力矩通道乘 0）。

**未动物理量**：`dm_sign` / `dji_sign` / `dm_zero` / `leg_off_phi0` / `.imu` / `MACHINE_DEFAULT` 全未改；新加的 `.rl.sign/.zero` 全 0 且 `configured=0`，由作者台架定。转向指令取负是遥控语义，同变更 82。

**核对**
- AC5：18 个文件（改动的 + 含 `robot_control.h` / `machine_config.h` 的）默认与 `-DSYSID_ENABLE=1` 各 **0 err 0 warn**（`-o` 到临时目录）。
- 整机试链接：eIDE 现成 `.obj` + 本次新 .o + networkzn1 三个 .o，去掉四个旧网络 .o，输出到临时目录，armlink **0 err 0 warn**；ROM 745 172 → 272 036 B，RW 72 960 → 57 552 B（四个旧网络与四块 activation 去掉）。作者 `.axf/.map/.obj` 未动。
- `grep` 无 `RL_MODEL_STABLE/JUMP/PIN/UPSTAIRS`、`spin_mode`、`ai_stable_*` 等残留。
- 未下载、未上机。

**待训练侧 / 待台架**：见 `RL_OVERVIEW.md` §八。顺序：① 训练侧给 lf0/lf1 定义、PD、指令范围 → 作者定 `.rl` → ② 总输出关 + `infer_enable=1` 看 VOFA 动作 → ③ 填增益看力矩方向 → ④ 架空 → ⑤ 下地。上大机器前 `MACHINE_DEFAULT` 切 `MACHINE_ID_CHUANLIANTUI`。

---

## 变更 96 · LQR 分层验证旋钮：俯仰零偏 + 轮速俯仰补偿符号 + 位移积分启动阈值 + 左轮分项 + VOFA 第六版（作者：太多补偿融合在一起，一项一项验证；关位移环匀速往后走；松杆回到上电位置）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/lqr_balance.h` | `lqr_debug_t` 加 `pitch_comp_sign`（默认 −1 = 现行公式）、`pitch_off`（默认 0）、`pos_arm_vel`（默认 0 = 旧逻辑）；`lqr_state_t` 加 `u_col[10]`（左轮十列分项）、`pos_armed` |
| `imcalib/Algorithm/lqr_balance.c` | `LQR_Init()` 填三个默认值；`LQR_State_Update()`：`pitch = euler − pitch_off`，轮对地角速度两处（含 `ds_alt` 对照）的 `− omg_pitch` 改为 `+ pitch_comp_sign × omg_pitch`；位移积分改为"有指令清零撤防 → 目标回零且 |x[1]| < pos_arm_vel 才启动"；`LQR_Enable_Latch()` 投入时 `pos_armed=1`。`LQR_Control_Update()` 求和循环改为 `term` 逐项写，`i == 左轮` 时存 `u_col[j]`，`continue` 改成 `term = 0`，数值与原来逐位相同 |
| `imcalib/task/task_comm.c` | VOFA 第六版：ch11/12 = `whl[0/1]`（原大腿角）、ch13 = `vel_tgt`、ch14 = `pos_armed`（原虚拟小腿角）、ch27 = `x[4]` 左腿摆角世界系（原与 ch4 重复的偏航角速度）；ch15~19 轮速和 / 轮速 / 轮电流为作者先前所放，保留；头注释整块重写 |
| `md/LQR_PLAN.md` | §2.3 位移行、§2.5 三个新旋钮、§六新增 ①e 与"⑤ 分层验证"表、§八新增三项 |

**为什么**
- 作者台架已排除反馈侧极性（偏航角 / 角速度 / 轮号 / 轮速 / 俯仰 / 两腿摆角全对，RL 链路两轮硬件正常），关位移环后车**匀速**后退。匀速 = 净力矩 0 = 车身正处在真实平衡姿态，此时 ch28 读到的就是俯仰零偏；`pitch_off` 让作者在调试器里把这个读数填回去，不用等车站稳。放在测量侧（同 Leg2 `IMU_PITCH_OFFSET`），IMU 零偏和重心偏移两种来源都能消。
- `pitch_comp_sign`：现行 `whl = ω_电机 − ω_pitch` 与推导相反——轮子卡住、车身前倾时编码器读到的已经是 −ω_pitch，再减一次得到"轮子在倒转"，实际它没动；后果是俯仰阻尼比 K 表设计值少约三分之一。Leg2 同样写法（`Body.c:89`），且 §0.1 禁止 AI 改符号，故默认值不动、只给 A/B 开关，台架按 §六 ①e 定。
- `pos_arm_vel`：斜坡 5 m/s² 让速度目标 0.24 s 内归零，车还在滑行，滑行距离全记进 `pos` 再被拉回，短距离推杆就是"回到起点"。阈值到了才启动积分，Leg2 注释掉的 `vel_kalman ∈ ±0.5` 门控是同一思路。
- `u_col[]`：分层验证时要知道左轮力矩是谁在推（位移 / 速度 / 俯仰 / 腿摆角），Watch 里直接看十列。

**输入 / 输出 / 调用链**
- `lqr_debug.pitch_off` → `LQR_State_Update()` 的 `pitch` → `x[4]/x[6]/x[8]`、速度运动学 cos/sin 项；翻倒检测仍用 `imu_state` 原值不受影响。
- `lqr_debug.pitch_comp_sign` → `whl[0/1]` → `ds_raw` → `ds_lpf/ds_kf` → `x[1]` → 位移积分；`ds_alt` 对照同步。
- `lqr_debug.pos_arm_vel` + `target[1]` + `x[1]` → `pos_armed` → `pos` → `x[0]`。
- `u_col[j]` 只写不读，供 Watch。

**未动物理量**：`dm_sign` / `dji_sign` / `dm_zero` / `leg_off_phi0` / `.imu` / K 表 / 三个默认值全部等于现行为，刷新固件后手感不变。

**核对**：AC5 `lqr_balance.c` / `leg_balance.c` / `task_actuation.c` / `task_comm.c` 默认与 `-DSYSID_ENABLE=1` 各 **0 err 0 warn**（`-o` 到临时目录，已删）。未链接、未上机。

**待台架**：`LQR_PLAN.md` §六 ⑤ 分层表（基线 → 零偏 → 补偿符号 → 位移环 → 偏航角速度 → 偏航角 → 横滚 → 启动阈值），一步一个量。

---

## 变更 96 · actuationTask 整体改回 500 Hz（作者：按训练文档，直接把整个函数弄成 500 Hz，用 RL 时不用 LQR）

| 文件 | 改动 |
| --- | --- |
| `Core/Src/tim.c`、`CtrBoard-H7_ALL.ioc` | TIM6 Period 999 → **1999**，Prescaler 274 不变：275 MHz / 275 / 2000 = 500 Hz（与变更 61 同一处，反向） |
| `imcalib/task/inc/robot_control.h` | `CTRL_DT` 0.001f → **0.002f**，LQR / RL / sysid 的时间步统一跟随 |
| `Core/Src/main.c`、`imcalib/user-lib/mono_ns.c` | TIM6 回调与单调时钟注释同步为 500 Hz（`Mono_Ns_Tick()` 只要求 ≥ 每 7.8 s 一次，500 Hz 无影响） |
| `imcalib/task/task_actuation.c` | **无功能改动**。本条一度做过"RL 分支两拍算一次"的分频版，作者否决后撤回；撤回时误用 `git checkout` 把变更 95 在本文件的四处（`rl_engaged`、`output_task_rl_engaged()`、`solve_rl()` 前提 `rl_ready`、MANUAL 分支写 `rl_engaged`）一并还原，已重新补回，`grep` 核对四处都在 |
| `md/AGENTS.md`、`md/RL_OVERVIEW.md`、`md/LQR_PLAN.md` §五 | 频率相关行同步；LQR_PLAN §五 顶部加注：本节以下是 1 kHz 时的记录 |

**为什么**：训练侧文档写"策略 100 Hz、PD 内环 500 Hz"。我先提议只让 RL 分支分频、TIM6 与 LQR 保持 1 kHz（怕动到正在调的 LQR），作者明确否决："用 RL 时不会用 LQR，直接整个改成 500 Hz"，按作者决定执行。

**对 LQR 的影响（作者已知，未处理）**：`pid_calc` 的 D 是每拍误差差分、不除 dt，腿长 / 防劈叉 / 横滚三组 KD（50000 / 500 / 100）是 1 kHz 照抄 Leg2 的值，500 Hz 下等效阻尼翻倍；`LQR_State_Update` 位移积分、`vel_ramp`、拨轮腿长积分、卡尔曼都吃 `CTRL_DT`，自动跟随。再上 LQR 时由作者定：TIM6 回 1 kHz，或三组 KD 折半。

**其他跟随项**：RL 的 PD 用速度 D，与频率无关；sysid（关着）心跳 `SYSID_HB_TICKS=250` 在 500 Hz 下变成 500 ms，未动；commTask 仍 1 kHz，VOFA 不变；CAN 负载减半。

**核对**：AC5 编译 `tim.c / main.c / mono_ns.c / robot_control.c / task_actuation.c / task_policy.c / task_comm.c / rl_*.c / lqr_balance.c / leg_balance.c / kalman.c / simple-function.c / sysid_*.c / dm.c / dji.c / machine_config.c` 共 20 个文件，默认与 `-DSYSID_ENABLE=1` 各 **0 err 0 warn**（`-o` 到临时目录）；整机试链接到临时目录 armlink **0 err 0 warn**，ROM 272 196 B。作者 `.axf/.map/.obj` 未动。未下载、未上机。

**待台架**：示波器或计数确认 actuationTask 500 Hz；RL 台架顺序见 `RL_OVERVIEW.md` §八。

---

## 变更 97 · 按训练仓库填 RL 参数 + 关节映射候选（作者：训练仓库在 D:\迅雷下载\leg\...，从里面找到定义，开始完善 RL 输入输出链路）

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/rl_torque.c` | `RL_Torque_Param_Init()`：腿 Kp 0 → **10**、Kd 0 → **1.0**，轮速度增益 0 → **0.1**（训练 `control.stiffness/damping`）；新增 `RL_TQ_LEG_TRQ_MAX 40` / `RL_TQ_WHEEL_TRQ_MAX 3.9`，`RL_Torque_Compute()` 在雅可比映射前把虚拟关节力矩裁到该上限（Isaac 的 `torque_limits` 就裁在虚拟关节），机器表限幅仍在映射后；**`RL_TQ_WHEEL_VEL_SCALE` 20 → 10**：变更 95 账本写了这一条，但那次 Edit 实际没落盘（提交 3a943aa 里仍是 20），本条才真正改到 |
| `imcalib/Algorithm/rl_policy.h` | `RL_CMD_VX_MAX` 0、`RL_CMD_YAW_MAX` 0（起立训练域 [0,0]）、`RL_CMD_HEIGHT_MIN/MAX` 0 → **0.20**（拨轮无效）；`RL_WARMUP_STEPS` 50 → **10**；`RL_ACTION_CLIP` 0 → **100** |
| `md/RL_OVERVIEW.md` | §3.4 PD 段、§3.5 指令/预热/推理行、§四 待实测表、§五 ①、§8.1 整段重写为"训练侧定义 + 关节映射候选 A/B + 台架判定 + 左右归属" |
| `md/sysid-change-map.md` | 本条 |

**从训练仓库核对到的定义（文件与数值）**
- `wheel_legged_gym/envs/chuanliantui/chuanliantui_config.py`：DOF `[lf0, lf1, lfwheel, rf0, rf1, rfwheel]`；l1 0.21 / l2 0.25；`fk_offset_hip 0.664720554456` / `fk_offset_knee 1.626002936793`；默认角 ∓0.06 / ±0.10；`decimation 5`、`sim.dt 0.002`（PD 500 Hz、策略 100 Hz）；`pos_action_scale 0.5`、`vel_action_scale 10`；`stiffness f0/f1 10`、`damping f0/f1 1.0、wheel 0.1`。
- `envs/base/legged_robot.py::_compute_torques`：`τ = Kp(act×0.5 + 默认 − q) + Kd(act×10·[轮] − q̇)`，再裁 `torque_limits`（URDF effort 40/40/3.9）；`step()` 先 `clip_actions=100` 再存 `self.actions`（obs 里的 last_action 是 clip 后）；`dof_vel_use_pos_diff=True`，训练关节速度是 500 Hz 位置差分。
- `envs/base/legged_robot_config.py::normalization`：`ang_vel 0.25`、`dof_pos 1.0`、`dof_vel 0.05`、`clip_observations 100`；指令缩放 `[2.0, 0.25, 5.0]`。
- `chuanliantui_standup_config.py`：`commands.ranges lin_vel_x [0,0]、ang_vel_yaw [0,0]、height [0.20, 0.20]`；课程 `pre_unlock_target_height 0.30 / post 0.20`；`standup.initial_dof_pos [11.0, 0, 0, −11.0, 0, 0]`（0.15 m 地面后摆）、`wheel_contact_force_threshold 1 N`；`check_termination()`：首次任一轮接地后下一策略步才推理。`sim2sim/mj_sim2sim_ct.py` 把 11 rad `wrap_to_pi` 成 −1.566；`COMMANDS.md` 起立回放 `--cmd_vx 0 --cmd_height 0.20`。
- `sim2sim/chuanliantui.xml`（真实闭链）：第二台髋电机 `lf00` 与 `lf0` 同轴，经 0.1134 曲柄 → 0.135 连杆 → 三角块（销到大腿 0.1134 处）→ 0.0966 推杆 → 小腿摇臂（膝后 0.067）。`ClosedChainAdapter.map_virtual_torques()`：`τ_lf0电机 = τ_lf0 + (∂lf1/∂lf0)·τ_lf1`，`τ_lf00电机 = (∂lf1/∂lf00)·τ_lf1`，与固件 `vshank_jac` 映射同形。

**机构等价性核验（用 URDF 几何、Python 数值）**
- 三角块曲柄 0.1134 = 0.54 × 0.21，连杆 0.135 = 0.54 × 0.25，是对称五连杆的缩比联动。逐姿态解闭链（两次圆交）得轮心，与固件 `Leg_Solve_Geometry()` 同公式的五连杆解比较：零姿态及 9 个手选姿态一致到 1e−4，工作区间连续扫描 4415 个姿态（lf1 ∈ [−0.12, 0.77]）最大差 **2.2e−11 m**。结论：固件解算器对大机器成立，`virtual_shank_angle` = 训练 lf1 差常数和符号；本条未动解算器。

**关节映射推导（§0.1：只写在文档与账本作候选，未写 `.rl`）**
- 固件平面 x 前、y 下，训练 FK 平面 x 后、z 下：`θ_fk = π − ψ_fw`。由 `θ1 = lf0 + 0.6647`、`θ2 = lf1 + 1.6260`、固件 `vs = φa − qf − π/2`：
  候选 A（固件 +x = URDF +x）：`lf0 = −(thigh − 2.476872)`，`lf1 = −(vs − 3.086386)`，右腿符号取反；轮 lfwheel = −左轮速（左轮轴 (0,−1,0)，正转 = 向后滚），rfwheel = +右轮速 → `sign {−1,−1,−1,+1,+1,+1}`，`zero {2.476872, 3.086386, 2.476872, 3.086386}`。零姿态代入 lf0 = 3.6e−15、lf1 = −4.4e−16；默认站姿固件应读 thigh ≈ 2.54、vs ≈ 2.99 rad。
  候选 B（固件 +x = URDF −x）：左腿 `sign +1`、`zero {0.664721, 0.055207}`，右腿反号，轮符号反过来，且 gyro/重力 x、y 要取反（`.rl` 需加帧符号字段）；站姿 thigh ≈ 0.60。
- 判定方法：默认站姿看 VOFA ch11 / ch13。左右归属另问训练侧：URDF `lf0` 在 y = −0.179，忠实右手系下 "lf" 是实机右腿；若 CAD 是镜像导出则 lf = 左腿但 IMU y 相关量取反。

**行为变化**：`infer_enable` 仍默认 0，手动遥操路径不变；打开推理后腿 PD 不再是零力矩（Kp 10 / Kd 1），但 `.rl` 未配置整链仍门控关。`RL_TQ_WHEEL_VEL_SCALE` 20 → 10 也影响手动遥操路径的轮目标（ch1 满杆 4 × 10 = 40 rad/s，仍被 `RL_TQ_WHEEL_VEL_MAX` 20 夹住）。

**核对**：AC5 20 个文件默认与 `-DSYSID_ENABLE=1` 各 **0 err 0 warn**（`-o` 到临时目录，含作者本批改的 `lqr_balance.c` / `task_actuation.c` / `task_comm.c` / `dji.c`）；整机试链接到临时目录 armlink **0 err 0 warn**，ROM 272 280 B。作者 `.axf/.map/.obj` 未动。未下载、未上机。

**未动物理量**：`.rl.sign/.zero`、`dm_sign`、`dji_sign`、`dm_zero`、`leg_off_phi0`、`.imu`、`MACHINE_DEFAULT` 全未改。

**待训练侧 / 待台架**：① 训练侧确认 model_6000 的起立课程是否已解锁（高度 0.20 还是 0.30）、URDF 左右与 x 向、11 rad 初态在 sim2sim 里是否起立成功；② 作者站姿判定候选 A/B → 填 `.rl`；③ 之后按 `RL_OVERVIEW.md` §五 ②~⑤。

---

## 变更 99 · RL 观测极性验证准备：总输出关 + 推理路径开 + 未投入观测预览 + VOFA 原始角/关节速度 + 切大机器（作者 2026-09-23：可以，你先改，改完先测试所有观测输入是否跟训练端一致）

| 文件 | 改动 |
| --- | --- |
| `imcalib/task/robot_control.c` | `torque_output_enabled` 1 → **0**（全链零力矩）；`rl_control.infer_enable` 0 → **1**（走推理路径） |
| `imcalib/user-lib/machine_config.h` | **`MACHINE_DEFAULT` `MACHINE_ID_LOCAL` → `MACHINE_ID_CHUANLIANTUI`**（作者确认；networkzn1 对应大机器，小机器 `.rl` 未配置）。机器表本身一个数没动 |
| `imcalib/task/task_policy.c` | 新增 `RL_Observation_Preview()`：RL 未投入时仍按同一套映射/镜像构建观测，只供 VOFA，**不推历史**（投入沿历史照旧从首帧重填）；IMU 或两腿无效时观测清零。`RL_Command_From_Rc()` 挪到投入判断前 |
| `imcalib/task/task_comm.c` | RL 模式 VOFA：ch10~13 固件原始 thigh / vs（左、右）；ch15~20 由网络动作**直接替换**为观测关节速度（作者定不加宏切换）；ch6 加 bit6 观测有效、bit7 `.rl` 已配置 |
| `md/RL_OVERVIEW.md` | §3.5 投入行、§3.6 总开关、§五 ①②、§8.2 开关默认值、§8.3 VOFA |

**离线一致性核对（Python，已数值核验）**：训练侧不重写——从 `sim2sim/mj_sim2sim_ct.py` 源码用 ast 抽出原函数 `build_obs` / `quat_rotate_inverse` / `wrap_to_pi` 与常量直接执行；固件侧把 `rl_observation.c` 的重力、缩放、减默认角、裁剪、历史推入与 `Angle_Wrap_180` 逐行移植为 float32。20000 组随机状态（含超 ±100 裁剪）逐项最大差：角速度 6e−8、重力 5e−7、指令 8e−7、关节角 2e−7、关节速度 2e−7、上次动作+裁剪 4e−6、历史 FIFO（首帧 ×5、丢最旧追加末尾）0、角度环绕 2e−6，全部 float32 舍入量级。镜像：四元数 x/z 取反后重力恰为 `(gx, −gy, gz)`，差 0。固件起立指令 `[0, 0, 0.20]` → obs 6~8 = `[0, 0, 1.0]`。
**未覆盖（只能台架）**：`.rl` 符号/零点、`.imu` 取轴、镜像左右归属、电机速度 vs 训练 500 Hz 位置差分的噪声延迟差异。

**核对**：AC5 编译 `task_policy.c` / `task_comm.c` / `robot_control.c` / `machine_config.c` / `rl_torque.c` / `task_actuation.c` **0 err 0 warn**（`-o` 到临时目录，作者 `.obj` 未动）。未链接、未下载、未上机。

**未动物理量**：`.rl.sign/.zero`、`.imu`、`dm_sign`、`dji_sign`、`dm_zero`、`leg_off_phi0` 全未改。另：变更 98（`.rl` 填候选 A）账本里没有单独条目，只在 RL_OVERVIEW 提到，待补。

---

## 变更 100 · 切大机器（作者 2026-09-26：「我要从小机器切换为大机器」→「你改」）

| 文件 | 改动 |
| --- | --- |
| `imcalib/user-lib/machine_config.h:19` | `MACHINE_DEFAULT`：`MACHINE_ID_SMALL_WHEELLEG` → `MACHINE_ID_BIG_WHEELLEG`。该行工作区里已由作者改好（mtime 15:42，AI 未重复改动），本批只做文档同步与编译核验 |
| `md/AGENTS.md` | 关键约束「控制频率」：当前默认改注大机器 500 Hz；顺手去掉该行已不存在的「手动遥操」 |
| `md/RL_OVERVIEW.md` | §一 频率行、§四 差异表注、§8.1 末段：默认机器改为大机器 |
| `md/LQR_PLAN.md` | 头部「当前机器」段：默认已切大机器、`lqr_configured=0` → 左拨杆中位判失能，要跑 LQR 先切回小机器 |

**行为变化（全部随机器表/宏自动切换，其余代码一行未动）**
- 控制环：TIM6 Period 999 → 1999（1 kHz → **500 Hz**），`MACHINE_CTRL_DT` 0.001 → 0.002（`tim.c:223`、`robot_control.h:18`）。
- VOFA 口：UART8 → **USART1**（`Vofa_send.h:16`）—— 上位机接线要跟着换。
- ⚠ **更正（同批自查）**：FDCAN 数据段**不随机器切换**。`main.c:74-94` 的 `Machine_Apply_Fdcan_Data_Timing*` 整段注释、无调用点，`MACHINE_FDCAN13_DATA_*` 不生效；三路名义段一律取 `fdcan.c` 的 3/5/2（1 Mbps），FDCAN1 = FD_BRS、FDCAN2/3 = classic，固件只发经典帧。本条先前的"数据段 3/5/2 → 1/4/1"写法是 AI 误读，已更正；`md/AGENTS.md` 的 FDCAN 条目一并更正。
- 电机/几何/量程/零点/IMU/`.rl` 全改读大机器表：J8009P ×4 全在 FDCAN1、M3508（15.5）在 FDCAN3、lu 0.21 / lg 0.25、腿长 0.14~0.34、`dm_trq_clamp` 40 / `dji_trq_clamp` 3.9。
- 门控：`lqr_configured` 1 → 0（左拨杆中位 = 失能）；`.rl.configured` 0 → 1（RL 整链开，候选 A；左拨杆上位 + 右中位即进入推理路径）。
- ⚠ `torque_output_enabled` 初值仍为 **1**（`robot_control.c:48`，作者定为有意设计）→ 上电后拨杆到 RL 挡 + 右中位就会出力。第一次台架建议先用调试器把它写 0（非 static 全局，直接 Watch 改，不落盘）。

**输入 / 输出 / 调用链**：`machine_config.h:19 MACHINE_DEFAULT` → `machine_config.c:96 machine = &machine_table[MACHINE_DEFAULT]` → 全工程只读 `machine->`（频率 / 总线 / VOFA 口 / 量程限幅 / 腿几何 / `.imu` / `.rl`）；无运行时切换接口（`Machine_Id()` 只读）。

**核对**：AC5 全量编译（按 `build/CtrBoard-H7_ALL/compile_commands.json` 逐条 armcc，`-o` 全部指到临时目录，作者 `.obj` 未动）：默认（大机器）**95 文件 0 fail / 0 warn**；追加 `-DMACHINE_DEFAULT=1`（小机器分支，附录 A 第 2 条）**95 文件 0 fail / 0 warn**。另：作者 eIDE 侧重新链接通过（`build/CtrBoard-H7_ALL/compiler.log`：Program Size Code=100112 / RO=159884 / RW=4328 / ZI=52472，`.axf` / `.hex` 已重出）。未下载、未上机。

**未动物理量**：`.rl.sign/.zero`、`.imu`、`dm_sign`、`dji_sign`、`dm_zero`、`leg_off_phi0`、`dm_*_max`、`mirror`、`+LEG_PI` 一个数没动。另：工作区里 `machine_config.c` 大机器 `.eul_src` 行尾多一个空格（作者工作区改动），未处理。

**待台架**
① **IMU（优先）**：低头 30° 时 `euler_rad[0]` / `gyro_rad_s[1]` 响应、`obs[3]` 应 ≈ +0.5。AI 离线枚举（24 安装 × 24 模块四元数约定 × 2 方向 × 2 速率 = 2304；放宽到含镜像的 48×48×4 = 9216）**没有找到**能让当前 `.imu.quat_src={1,0,2}` 与 `.gyr_sign={1,1,-1}` 同时成立的物理解释，疑为多一次 X/Y 通道交换（去掉该交换后恰有唯一自洽解：绕 Z 180° 安装 + 模块共轭/速率取负约定，且同时解释表中三组 {1,1,−1} 符号）。**AI 未改任何物理量**，等台架读数定。
② **`.rl` 候选 A 复核**：默认站姿 `thigh ≈ 2.54 / vs ≈ 2.99`（读成 ≈0.60 / 0.16 即候选 B，停下重填）。
③ **acc 极性**：静止时看哪一轴 ≈ ±1 G（`.acc_sign` 的 x 与 gyro/euler 不同源）。
④ **DM**：用达妙上位机读 J8009P 的 pos/vel/trq 与 `dm_*_max` 比对；CAN 超时保护 20~50 ms 逐台设（变更 76 遗留）。

---

## 变更 101 · 🐞 RX 放行 FD 帧：达妙离线 / 失能不了 的根因（作者 2026-09-26：跟最新分支 little-wheelleg 做比较，找出为什么那一份没办法正常失能电机、电机是离线状态的原因 →「我切换回这一份了，你直接修改」）

**现象**：本分支上四台 DM 全离线（`online_mask` bit2~5 = 0，只有 IMU + 两轮在线），整车"失能不了"（电机保持最后状态，而 VOFA 显示未使能）。

**根因（两分支对比可证）**：本分支的 `HAL_FDCAN_RxFifo0Callback()` 比 main 多一句 `rx_header.FDFormat != FDCAN_CLASSIC_CAN → continue`。达妙跑 **FD、1M 仲裁 / 4M 数据**，反馈帧就是 FD+BRS 帧 → 硬件收到（`alive_cnt` 照涨）但被这句丢掉 → `Dm_Read()` 不执行 → `rx_seen` 恒 0 → `Dm_Is_Online()` 恒 false → 四台全离线。连带效应：`Dm_Disable_Watchdog()` 只对"在线且 err_raw==1"重发（离线直接跳过，失能帧丢了没有兜底），`Dm_Is_Enabled()` 又加了"离线一律显示未使能"，于是"失能不了"在界面上根本看不出来。

| 文件 | 改动 |
| --- | --- |
| `imcalib/user-lib/can_bus.c` | RX 校验去掉 `FDFormat != FDCAN_CLASSIC_CAN` 一条，保留 标准帧 / 数据帧 / 8 字节 三条；加一行注释（FD 帧也放行）。RX FIFO 元素本来就是 8 字节，FD 帧 DLC=8 编码与经典帧一致，不必改 |
| `md/AGENTS.md` | FDCAN 条目补一句"收不挑帧格式（经典 + FD 都收），只发是经典帧" |

**输入 / 输出 / 调用链**：`HAL_FDCAN_RxFifo0Callback()` → 校验 → 路由按 ID 匹配 → `Dm_Read()` → `dm_motor_feedback[i]`（`rx_seen = 1`、`raw_pending = 1`）→ `Dm_Parse()`（commTask）→ `motor_state.dm.*` → VOFA 在线掩码 / `Dm_Is_Online()` / 使能与失能看门狗。

**核对**：AC5 按 `build/CtrBoard-H7_ALL/compile_commands.json` 全量编译 **92 文件 0 fail / 1 warn**（`-o` 全部到临时目录，作者 `.obj` 未动）；唯一告警是 `task_comm.c:198` 的 `uint8_t i; 声明未使用`——本分支既有、与本次改动无关，未动。未链接、未下载、未上机。

**未动物理量**：`.imu` / `.rl` / `dm_sign` / `dji_sign` / `dm_zero` / 量程 / 镜像一律未动；**发送模板也没动**（`FDFormat = FDCAN_CLASSIC_CAN`、`BRS_OFF`，仍是经典帧发出去）。

**待台架**
① `online_mask` bit2~5 应变 1；`Can_Bus_Rx_Count(1)` 在涨、`Can_Bus_Last_Rx_Id(1)` = 0x11/0x13/0x12/0x14。
② 拨杆上→中使能、下→失能：DM 应能正常使能/失能。若"失能不了"仍在 → 去达妙上位机给四台设 CAN TIMEOUT 20~50 ms（变更 76 遗留，主控被拔/发不出去时靠它兜底）。
③ **若 ① 仍是 0**（电机压根不回）→ 说明电机不认经典帧，下一步才是把 FDCAN1 的发送模板改成 FD+BRS（`FDFormat = FDCAN_FD_CAN`、`BitRateSwitch = FDCAN_BRS_ON`，要**按总线**区分，DJI 那条 FDCAN3 保持经典）；那属于第二个变量，等作者点头再动。

---

## 变更 102 · 删 `leg_solver` 的 `mirror` 字段（作者 2026-09-27：「至于 mirror 删了吧，我可以在机器配置表改极性，没必要多一个这种一样性质的东西」）

**性质**：属 §0.1 红线物理量项，作者明确指示删除；改动**行为等价**——全树 `mirror` 只有 `robot_control.c:55/:60` 两处赋值（均为 +1），`Leg_Init()` 不设默认，无任何 `-1` 路径，几何里所有 `mirror ×` 都是恒等。

| 文件 | 改动 |
| --- | --- |
| `imcalib/Algorithm/leg_solver.h` | 删 `leg_config_t.mirror`（原 :24）、`leg_solver_cache_t.mirror`（原 :60）；`qf/qb` 注释「前髋(镜像后)」→「前髋角」 |
| `imcalib/Algorithm/leg_solver.c` | 删 `cache->mirror` 赋值（原 :72）；`qf/qb/vf/vb` 去掉 `mirror ×`（原 :75-78）；`virtual_shank_angle`（原 :126）、`d_virtual_shank_angle`（原 :180）去掉 `mirror ×` |
| `imcalib/task/robot_control.c` | 删 `leg_l/leg_r.config.mirror = 1`（原 :55/:60） |
| `md/RL_OVERVIEW.md` | :54、:114 去掉 `config.mirror` 表述 |
| `md/IO_CHAINS.md` | 求解公式块（:432 附近）改 `qf = hip_f`；配置说明行更新、参数表删 mirror 行、`thigh_angle` 行更新 |
| `md/LQR_PLAN.md` | :202、:237 去掉 mirror 表述 |

**输入 / 输出 / 调用链**：不变。`task_comm.c:48-58` 组装 `hip_f/hip_b`（含 `+LEG_PI`）→ `Leg_Solve()` → 几何 `qf = hip_f`（原 `mirror × hip_f`）→ 输出字段与消费端（`task_policy.c` RL_Joint_Map、`rl_torque.c`、`lqr_balance.c` / `leg_balance.c`）一字未动。

**物理量现状**：右腿极性仍只在两处——`dm.c` 反馈 `dm_sign.fb`（右前/右后 = −1）与输出 `dm_sign.out`；`machine_config.c:18` 未动。`md/AGENTS.md:21` 红线清单仍列 `mirror`（保留该表述，镜像概念本身仍归作者台架）。

**核对**：AC5 按 `build/CtrBoard-H7_ALL/compile_commands.json` 全量编译 **92 文件 0 fail / 1 warn**（`-o` 全部到临时目录，作者 `.obj` 未动）；唯一告警仍是 `task_comm.c:198` 的 `uint8_t i; 声明未使用`（本分支既有，与本次无关）。未链接、未下载、未上机。

**待台架**：默认站姿读 `thigh ≈ 2.54 / vs ≈ 2.99`、两腿镜像对称姿态 `hip_f_l ≈ hip_f_r`——与改动前一致即通过。

---

## 变更 103 · policyTask 改 TIM6 节拍分频：严格 100 Hz（作者 2026-09-27：「按分频方案改动」）

**问题**：policyTask 原来 `osDelay(10)`，循环体（观测 + 推理，`run_us` ≈ 500~700 µs）在延时之前 → 实际周期 ≈ 10.6 ms → ≈ 94 Hz；动作被 PD 保持 5.3 拍，训练是严格 5 拍。

**方案（分频）**：策略节拍从同一个 TIM6 控制节拍分出来——TIM6 ISR 里每 `MACHINE_POLICY_DIV` 拍释放一次 `policy_tick_sem`，policyTask 等该信号量再跑循环体。与 500 Hz 控制拍严格锁相，动作固定保持 5 拍（大机）。

| 文件 | 改动 |
| --- | --- |
| `imcalib/user-lib/machine_config.h` | 两份表各加 `MACHINE_POLICY_DIV`（大机 5u / 小机 10u），紧邻 `MACHINE_TIM6_PERIOD` / `MACHINE_CTRL_DT` |
| `imcalib/task/robot_control.c/h` | 新增 `policy_tick_sem`（def + `Robot_Control_Init()` 创建，初值 1）；新增 `Policy_Tick_Div()`（`++div_cnt >= MACHINE_POLICY_DIV` 归零并释放策略节拍，计数在函数内静态）；头文件加 extern 与原型 |
| `Core/Src/main.c` | TIM6 回调（USER CODE 区）在释放 `ctrl_tick_sem` 之后调 `Policy_Tick_Div()` |
| `Core/Src/freertos.c` | `policyTask_Entry` 的 `osDelay(10)` → `osSemaphoreWait(policy_tick_sem_handle, osWaitForever)`，先等节拍再跑 `ctrl_task_body()` |
| `md/RL_OVERVIEW.md` | 总述、任务表、速查同步为"TIM6 分频信号量" |
| `md/AGENTS.md` | 「控制频率」条目补 policyTask 分频说明 |

**输入 / 输出 / 调用链**：`TIM6 ISR` →（每拍）`ctrl_tick_sem` → actuationTask；同一 ISR 每 5 拍 → `policy_tick_sem` → policyTask → `ctrl_task_body()` → 观测 + `RL_Policy_Run()` → `action_state` → actuationTask 从下一拍起消费，固定保持 5 拍。

**核对**：AC5 全量编译 **92 文件 0 fail / 1 warn**（唯一告警仍是 `task_comm.c:198` 既有未用变量；`-o` 全部到临时目录，作者 `.obj` 未动）。未链接、未下载、未上机。

**待台架**：`rl_control.policy.run_ok` 每 10 s 应涨 **1000**（原来 ≈ 940）；动作保持固定 5 拍。

---

## 变更 104 · 移植整机诊断遥测 S2R1（gap 测试）：自包含 `imcalib/Telemetry/`（作者 2026-09-27：「以不耦合当前结构体、完全新建文件夹存放所有文件的形式来实现代码功能移植」）

**来源**：`main-new` @ `fc9df2c`（`1614827` 新增协议 + `32ac274` + `fc9df2c` 改遥测口），本次只搬 S2R1 遥测链路，**不搬**机型表/极性（§0.1 红线）、不搬 fdcan/ioc/模型签名等无关改动。

**设计（与上游实现的差别）**：上游把钩子插进 8 个任务文件、给 6 个既有结构体加字段（`imu_state.rx_us/seq`、`motor_state.*.rx_us`、`action_state.policy_seq/session`、`rl_torque_trace_t`、`dm/dji` 的 `decoded_rx_*`、`dm_last_submit_mask`）。本分支改为**去耦采样**：新目录只读既有全局量，自己维护序号与时间戳，事件用变化检测推断，现有代码只留 1 个挂点。

| 文件 | 改动 |
| --- | --- |
| `imcalib/Telemetry/s2r_wire.c/h` | 新增（原样）：记录结构、24 槽优先队列、小端编码、CRC32 |
| `imcalib/Telemetry/s2r_telemetry.c/h` | 新增：成帧（META/EVENT/POLICY/CONTROL/IMU/HEALTH/HISTORY）、会话与标志、事件去重、META JSON、健康统计、UART DMA 泵；`S2R_Pump()` 首次调用自初始化（不改 `main.c`） |
| `imcalib/Telemetry/s2r_source.c/h` | 新增：采样适配层。只读 `imu_state`/`motor_state`/`leg_l/r`/`rl_control`/`action_state`/`input_command`/`robot_state`/`hi229_data`/`dm/dji_motor_feedback`/`machine`，用 `policy.run_ok+run_fail` 变化判推理、`hi229_data.ts`+`last_rx_tick` 判 IMU 新帧、`last_rx_tick` 判电机新帧；序号自维护 |
| `imcalib/task/task_comm.c` | **唯一任务层挂点**：`comm_task_body()` 末尾 `if (!S2R_Pump()) { Robot_Control_Send_Vofa(); }`（+1 行相对路径 include） |
| `MDK-ARM/CtrBoard-H7_ALL.uvprojx` | 新增 `imcalib/Telemetry` 组（3 个 .c）+ 头文件搜索路径 |
| `MDK-ARM/CtrBoard-H7_ALL.sct` | `RW_IRAM2` 固定 `* (.s2r_dma)`，DMA 缓冲不落 DTCM |
| `.eide/eide.yml` | `srcDirs` 与包含路径加 `imcalib/Telemetry` |
| `imcalib/Telemetry/s2r_build_info.h` | 由 `tools/s2r_build_info.py` 生成（源集 SHA256 + 基准 commit + 模型签名 + 机器表 SHA256） |
| `tools/s2r_capture.py`、`tools/s2r_build_info.py` | 新增：被动接收/离线解码；构建指纹生成与 `--check` |
| `tests/test_s2r.py`、`tests/s2r_wire_test.c`、`tests/s2r_host_test.c`、`tests/fixtures/s2r_frames.json` | 新增：8 项协议一致性测试 + C 编码器交叉解码 + 替身生命周期/DMA/META 测试 |
| `md/sim2real_serial_protocol.md`、`md/sim2real_serial_capture.md` | 新增：协议布局与接线/采集/验收 |
| `Core/Src/freertos.c` | 仅修 `policyTask_Entry` 里 `/  }` 语法错误（a824da1 遗留，1 字符；不修则整个工程编译不过） |

**相对上游的字段降级（META 的 `unavailable`/`derived` 已声明，不伪造数据）**：

| 字段 | 本分支取值 |
| --- | --- |
| `action_raw`、`tau_virtual_raw_fw`、`gas_tau_shank_fw`、`tau_motor_unclipped` | NaN（内部裁前量/补偿中间量不出 `rl_torque`） |
| `motor_send_ok_mask`、`can_enqueue_us` | 0（驱动提交状态与入队时刻未暴露） |
| `current_motor` | NaN（电流无反馈） |
| `t_infer_start/end_us`、`control_exec_us` | 等于采样时刻 / 0（未插桩计时） |
| `motor_rx_us`、`imu_rx_us` | 采样首次见到新帧的时刻（commTask 1 kHz 量化，不是 CAN/UART 到达时刻） |
| `motor_clamp_or_mask` | 请求饱和推导（`|request| ≥ 0.999×限幅`），不是驱动内部限幅标志 |

**输入 / 输出 / 调用链**：`TIM6 → ctrl_tick_sem → actuationTask`（写 `rl_control.torque_state`/`rl_output_*`）+ `policy_tick_sem → policyTask`（写 `rl_control.observation`/`policy`、`action_state`）+ `imuTask`（写 `imu_state`/`hi229_data`）→ 全部只读 ← `comm_task_body → S2R_Pump → S2R_Source_Tick`（采样/成帧）→ 队列 → `VOFA_UART` DMA（`MACHINE_VOFA_PORT`，大机 USART1 1152000 8N1）。同口不再发 32 路 VOFA；`s2r_diagnostic_requested=0` 且失能、无会话时才退回旧 VOFA。

**核对**：Keil AC5（UV4 `-b`）全量编译 + 链接 **0 error**；告警 2 条均为既有（`ws2812.c` 文件末尾无换行、`task_comm.c` 未用变量 `i`）。尺寸 `Code=113132 RO=163408 RW=5420 ZI=93508`；`RW_IRAM1`(DTCM)=0、`RW_IRAM2`(AXI)=0x18270/0x50000；`dma_buffer` @ `0x24001100`（AXI SRAM）。三个新 .c 与 `tests/s2r_host_test.c`、`tests/s2r_wire_test.c` 单文件 armcc 编译 0 警告。`python tests/test_s2r.py` → 协议 8 项全过（`NativeChecks` 需主机 gcc，本机没有，未执行）。

**待台架**：烧录后按 `md/sim2real_serial_capture.md` §3 采集；HEALTH 的实际周期/峰值、丢帧、DMA 忙计数、boot_id 复位改变、会话分段与 `used_policy_seq` 绑定待实测。

**台架往返（2026-09-27）**：为查「左上+右中整机不动」临时把 `S2R_DIAGNOSTIC_DEFAULT` 置 `0`（上电发旧 VOFA），用 Vofa+ 读状态位定位到 **ch0 电机位（6 台电机全离线）→ `FAULT_MOTOR(0x04)` → 使能被拒 → `rl_engaged` 恒 0**（模型/机器表正常：ch2=193）。排查结束后**已改回 `1`**（上电由 S2R1 占口，开始 gap 采集），并重新生成指纹 + 双工程重建（Keil 0 error / 0 warning）。要看 VOFA 时临时改 `0` 重编译，或调试器写 `s2r_diagnostic_requested=0`（失能、无会话时生效）。

**链路实测（同日，档 0 采 10 s）**：`"S2R1"` 帧头出现 1337 次/10 s（固件发送正常，≈134 帧/s），但只有 275 帧通过 CRC、`garbage_bytes` 278 kB、`seq_gaps` 1293；相邻帧头间距仅 192~326 B（CONTROL 完整应 496 B）→ **链路整块丢字节约一半**，吞吐 33.6 kB/s（固件侧待机需 ≈58 kB/s）。采集口是 **`PowerDebugger Tx Serial Port`（VID 303A, Espressif）**，属带内部转发的调试器/无线串口桥，不是直通 USB‑TTL；同一条桥在 VOFA 只有 ~6.25 kB/s 时数据干净、速率一升就丢，与实际一致。**结论：不是固件问题，是桥的吞吐天花板。** 为此在 `s2r_telemetry.h` 增加 `S2R_RATE_LOW` 档位（CONTROL/POLICY/IMU/HEALTH/HISTORY/META 周期全部改为档位宏；限流时该次推理不成帧（`policy_pending`），序号不推进以免污染 `seq_gaps`；META 增 `rate_profile`/`rates_hz`/`history_period_us` 并随档位变化）。

**档位定稿（同日）**：档 1（≈9.5 kB/s）待机 40 s 实测零丢帧（`crc/format/seq_gaps` 全 0、`drop_total` 0、帧率 20.2/s 与档位吻合；`control_count` 与 `imu_age` 口径修正后分别为 2/窗口、~15 µs），但相对该桥天花板（6.25 kB/s 干净 / 33.6 kB/s 丢半）已无余量，投入段还要加 POLICY/HISTORY/事件。故重排档位并**默认改档 2（稳健，≈4.3 kB/s）**：0 = 原设定 58/85 kB/s；1 = 低速 9.5 kB/s；2 = 稳健 4.3 kB/s（CONTROL/POLICY 5 Hz、IMU 2 Hz、HEALTH 1 Hz、HISTORY 0.2 Hz、META 20 s）；3 = 极低 1.6 kB/s。波特率维持 1152000（两侧一致，降波特率只能缓解突发、不能提高吞吐，且牵动 CubeMX 与 Vofa+ 侧）。Keil 复核 **0 error / 0 warning**（`Code=113620`）。机器人尚不能起立站稳，本轮采集目标改为"链路 + 语义"（待机 / 投入 / 退出 段的 POLICY↔CONTROL 绑定、力矩方向、观测自洽），动力学对比留待能站立或换适配器后。

**短窗录制（同日，按协议 §10 第一条实现）**：为在不换硬件的前提下拿到 md 速率的数据，新增"板端短窗录制 + 慢速导出"：调试器写 `s2r_record_requested=1` → 按 **md 的 100 Hz**（`S2R_RECORD_PERIOD_US`）从同一采样快照生成 **CONTROL 完整快照**（452 B 载荷 + 生成时刻/标志/会话）存进 192 KB 片内缓存（紧凑格式 `u16 len|u8 type|u8 rsv|u32 flags|u32 session|u64 t_us|payload`，**≈4.1 s / 417 帧**，满即停并报 EVENT(10,0)），随后按 `S2R_DUMP_PERIOD_US`=150 ms（≈3.1 kB/s）逐条回放导出；`s2r_replay_requested=1` 可把同一轮再回放一次补丢帧；`s2r_record_state`(0/1/2)、`s2r_record_frames` 只读可见。回放帧**沿用协议原布局**：序号/CRC 在入队时重新生成、`t_us` 保持录制时刻、flags 加 `REPLAYING(16384)`，故**上位机与解析脚本零改动**；只新增 EVENT 码 9/10/11 与两个 flags 位（协议文档 §8.2/§13.1 已补）。录制与回放期间**实时 CONTROL 暂停**（避免挤链路），POLICY/IMU/HEALTH/META 照常，总占用 ≈3.5~4 kB/s。

**顺带修掉一个 DMA 隐患**：新增 192 KB 缓存后链接器把部分 .bss 挪进了 DTCM，`Vofa_send.o` 的 `buf`（UART TX DMA 读）落到 `0x2000aac0`（DTCM，H7 的 DMA 访问不到）→ 已由 `MDK-ARM/CtrBoard-H7_ALL.sct` 显式把 `* (.s2r_dma)`、`vofa_send.o (+RW +ZI)`、`uart_idle.o (+RW +ZI)` 固定在 RW_IRAM2(AXI)；复核 map：`dma_buffer`@0x240001c0、`buf`@0x24001d80、`dbus/debug/hi229_rx`@0x2400xxxx、`record_buffer`@0x2400b728(196608 B) 全部在 AXI，USB 缓冲（`dma_enable=DISABLE`，CPU 访问）留在 DTCM 无风险。内存占用：`RW_IRAM1`(DTCM)=50,720 B、`RW_IRAM2`(AXI)=247,384 / 327,680 B；**Keil 全量编译+链接 0 error / 0 warning**（`Code=114580 ZI=292616`），eIDE 侧同源重建通过。

**全自动触发 + POLICY_ACTIVE 修复（同日）**：① 作者无法在线调试（怕调试器停 CPU 危险），故录制改为**默认全自动**：`S2R_RECORD_AUTO=1` 时"投入建会话"即开录，"失能"后再录 `S2R_RECORD_TAIL_US`(200 ms) 尾巴即停并自动回放导出，全程不需调试器；调试器变量保留为手动覆盖（手动/自动用 `record_manual` 区分，避免自动录制被"请求位为 0"误停）；新增 EVENT 9 reason 1(投入自动)/EVENT 10 reason 3(会话结束自动)。② 台架投入轮实测（16:32 版固件）暴露移植遗漏：CONTROL 帧的 `S2R_ACTIVE`(POLICY_ACTIVE) 从未置位 → 上位机 `valid_policy_segment` 恒 false；已按原实现补回（本拍 `rl_valid && used_policy_seq && STARTED && 输出使能 && 电机使能` 时置位，`status_flags()` 在故障/失能时自动清除）。同轮实测的有效结论：RL 投入成功（EVENT 1→2×3）、POLICY `policy_seq` 1…14 连续、CONTROL `used_policy_seq` = 1/4/8/12 与 POLICY 一一对应（**策略↔执行绑定成立**）、`tau_motor_request` 非零（13.3/−1.3/−7.7 Nm）；随后 `EVENT 4 detail=2 = FAULT_RC` 为作者"关遥控收尾"所致（非故障）。另发现 Keil 周期性刷新 Watch 会抢无线调试口带宽 → 同轮出现 7 CRC 错误 / 24 缺口（此前各轮均 0），已在文档建议关闭 View→Periodic Window Update。Keil 复核 **0 error / 0 warning**（`Code=114896`），hex 16:43，指纹 `83cc75e1…`。

**自动录制首轮台架验证 + 两处修复（同日，16:50/16:52）**：首轮自动录制实测（`data/s2r_rec_05`）：投入 → `EVENT 9 reason 1` 自动开录 → 录满 `EVENT 10 reason 0 detail=416`（≈4.16 s / 100 Hz）✓，回放帧带 `ACTIVE`（`flags=18685`）、`used_policy_seq`=6/8/10/13 与 POLICY `policy_seq` 对应、`tau_motor_request` 非零 ✓ —— **机制可用**。但暴露两问题：① **录制窗口内混入 22 帧实时 CONTROL**（4.35 s ÷ 档 2 的 200 ms ≈ 21.7）：抑制条件写成"仅本拍要存缓存时不发"，其余拍仍按档位发 → 改为 `record_state != 0` 时一律不发实时 CONTROL；回放期间同时停 POLICY（那时 obs 是待机数据、无用且挤链路）。② **回放段丢 95%**（416 帧只收到 6 帧，`crc_errors 140`/`garbage 26.6 kB`/`seq_gaps 379`）：按原始字节切片统计，77.5 s 前**完全干净**（含机器人带电动作那 4 s，零错误），**78.7 s 开始导出那一刻起**垃圾/CRC/缺口才开始暴涨 → 判定为导出时帧间无空隙、队列积压时固件以线速连推多帧致串口桥缓冲溢出，而非电机干扰或持续带宽不足。修复：发送泵加**帧间强制空隙** `S2R_TX_GAP_US`(1500 µs，按帧长预估发完时刻后再等空隙，瞬时上限压到 ~85 kB/s)。两轮均 Keil 全量 0 error / 0 warning；hex 16:50(指纹 `ce9a862b…`)、16:52(指纹 `5bff26d4…`)。


**同日追加（作者 2026-09-27：「直接把 vofa 通道替换回旧的看一下为什么」）**：恢复 `task_comm.c::Robot_Control_Send_Vofa()` 里被 a824da1 注释掉的 **RL 布局**（ch3~6 下发力矩、ch7/8 轮指令、ch9~12 实测 DM 力矩、ch13~16 观测四腿角、ch17~22 六关节速度、ch23~28 上次动作、ch29/30 轮电流 raw、ch31 `ctrl_fault`；LQR 布局整段留在注释里备查），函数上方通道注释同步更新，`task_comm.c` 的未用变量告警随之消失。另在 `s2r_telemetry.h` 增 `S2R_DIAGNOSTIC_DEFAULT`（默认 `1` = 上电由 S2R1 占口；改 `0` 重编译即上电发旧 VOFA，台架调试用），`s2r_diagnostic_requested` 初值改用它。Keil AC5 全量编译+链接复核：**0 error / 0 warning**（`Code=113488`）。

---

## 附录 A · 每次改完必须跑的核对

1. 全量编译：按 `build/CtrBoard-H7_ALL/compile_commands.json` 逐条执行 armcc 命令（`-o` 指到临时目录即可）→ 要求 `0 fail / 0 warn`。
2. 另一台机器分支也要能编：在同样命令后追加 `-DMACHINE_CHUANLIANTUI=1`，至少覆盖 `dji.c`、`dm.c`。
3. `grep` 本次改动的宏/函数名，确认没有残留旧引用、没有死链。
4. 逐项对照本节记录的"输入 / 输出 / 调用链"，确认代码与文档一致。
5. 把不确定的写成"待台架"，不要写成"已完成"。

---

## 附录 B · 已知取舍与风险

| 项 | 内容 | 处理 |
| --- | --- | --- |
| 读参窗口丢帧 | 启动期可能吃掉 1 帧正常反馈 | 仅启动期、电机未使能，接受 |
| 启动延时 | 自检最多 +160 ms | 接受；失败不阻塞 |
| 满量程不一致 | ~~`P_MAX` 12.5 而实际 ±π~~ → 变更 23 已改为 ±π | 已解决；VMAX/TMAX 由作者上位机核对为 45/54 |
| 时钟调用周期 | Tick 必须 ≥ 每 7.81 s 一次 | 目前唯一挂在 1 kHz TIM6 上，满足 |
| 时钟溢出 | `cyc×1000` 约 9 小时上限 | 单次实验远小于该时长 |
| 力矩记录点 | 现在仍是"命令值"，量化后回算在步 5 | 见计划 §10.2 |
| 腿几何已进配置表 | ~~切机器时 `robot_control.c:43-63` 必须手改~~ → 变更 20 后只改 `MACHINE_DEFAULT` | 已完成 |
| 腿偏置为换算值 | 大机器偏置按参考固件表换算，未经卷尺/角度计验证 | 待台架标定；现象离谱就重标 |
