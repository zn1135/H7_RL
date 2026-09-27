# RL 部署总览 — 代码链路 & 进度

## 当前重构约定

- 用户后续自行调整遥控映射；临时 action 测试代码已删除，不再加入额外测试控制链路。
- 任务代码迁至 `imcalib/task/`；IMU、策略、执行、通信各自独立文件。
- `task/inc/robot_control.h` 统一声明共享状态和跨任务接口，不再保留分散的 `task_*.h`。
- Keil 与 eIDE 已更新任务源目录和 `imcalib/task/inc` include 路径。
- 已阅读姿态解算、`leg_solver`、观测构建；`rl_policy` 留待结合训练同学的模型参数共同检查。
- `rl_torque` 保持动作、PID、雅可比和电机输出的直接链路。PID 直接逐个 `PID_struct_init`，不使用参数指针同步、循环初始化或隐式重配。
- RL 力矩链按气弹簧两端点几何计算小腿被动力矩，左右补偿符号在机器表中独立配置，默认关闭；不加入斜率限制或额外控制策略。
- 多次计算保持单一中间量和短表达式；注释简短。循环索引在 `for` 内定义。
- 2026-09-22 起只有一个模型 `networkzn1`（chuanliantui 起立策略）；推理路径由 `machine->rl.configured` 门控直接运行（`rl_control.infer_enable` 开关与旧手动遥操路径已删除）；训练关节与固件关节的映射在机器表 `.rl`，未配置整链门控关。接入说明与训练侧待提供清单见 **§八**。

> 最后更新：2026-09-25
> 参考实车：XYEGA_RM2026_WheelLeg_Infatry_RLdeploy（复旦 EGA 2026 国赛上场版）

---

## 一、整体思路

轮腿平衡步兵的 RL 控制本质是一个 **sim-to-real** 流程：

1. **训练端**（IsaacLab / MuJoCo）产出 ONNX 策略模型
2. **板端**（STM32H723 + CubeAI）实时推理，输出 6 维动作
3. **执行层**把动作经 PD + 雅可比映射为 6 个电机力矩

整个链路在 actuationTask 控制环里跑（频率随机器：当前 `MACHINE_DEFAULT` 是大机，500 Hz 与训练 PD 内环同频；小机 1 kHz。见 `machine_config.h` 的 `MACHINE_TIM6_PERIOD` / `MACHINE_CTRL_DT`，变更 96 / 100），RL 推理 100 Hz（policyTask 由同一 TIM6 节拍按 `MACHINE_POLICY_DIV` 分频唤醒：大机 5 分频、小机 10 分频，严格锁相，变更 103；执行任务每拍复用最新动作）。

**数据流一句话**：
```
IMU(四元数+陀螺仪) + 电机编码器(关节角) + DJI轮速 + 遥控指令
    → 五连杆运动学 → 25维观测 → 5帧历史堆叠(125维)
    → CubeAI推理 → 6维动作 → PID → 雅可比映射 → 6个电机力矩
```

## 极性与坐标定义（已确认，禁止擅自修改）

### HI229 IMU

| 数据 | X | Y | Z | 说明 |
|------|---|---|---|------|
| 加速度 `accel_g` | -1 | +1 | -1 | 原始加速度映射到机体坐标；静止重力方向为负 |
| 角速度 `gyro_dps` | -1 | +1 | -1 | 映射后用于 RL 角速度观测 |
| 欧拉角 | -1 | +1 | -1 | 对应 Roll / Pitch / Yaw |
| 四元数虚部 | -1 | +1 | -1 | 对应 qx / qy / qz，实部 qw 不变 |

- HI229 姿态链路直接采用已映射的参考四元数；原始加速度不参与当前四元数生成。加速度极性只影响 `accel_g`、`accel_normed` 与调试输出，未来若启用加速度融合时必须重新复核。
- 投影重力使用 `g_body = R^T * [0,0,-1]`，只由输出四元数计算，不跟随原始加速度极性直接翻转。当前已验证水平静止、俯仰、横滚和偏航行为正确，禁止额外取反。

### DM 髋电机与五连杆

- DM 左前、左后髋 `feedback_sign=+1`；右前、右后髋 `feedback_sign=-1`。驱动层先统一右侧角度、角速度和力矩反馈到逻辑机体坐标。
- 几何输入中，前髋 `hip_f = DM 反馈角 + π + offset_f`，后髋 `hip_b = DM 反馈角 + offset_b`。左右腿均使用相同前后映射，五连杆层不做二次镜像或交换电机（`config.mirror` 字段已删，变更 102）。
- `l0` 增大表示伸腿；`phi0` 前摆为正、后摆为负；`virtual_shank = wrap(phi_a - qf - π/2)`，采用相对大腿的 EGA 虚拟小腿定义。
- 逻辑髋电机力矩到实体输出时，右侧 DM 只在 `dm.c` 驱动边界取反一次；禁止在解算器、测试模式或任务层再次取反。

### DJI 轮电机与遥控

- DJI 左轮 `feedback_sign=+1`，右轮 `feedback_sign=-1`；轮速和多圈角已验证：机器人前进方向为正，后退方向为负。
- 轮电机输出极性由 `dji.c` 按配置表的 `output_sign` 统一处理（与 `feedback_sign` 同号）；调用方不要再取反
- 遥控接收方向已验证；普通 RL 指令缩放以 `rl_policy.h` 的 `RL_CMD_*` 为准（已按训练域填，见 §3.5）。历史：手动遥操时代曾统一缩放到 `±0.1` 用于安全测试——该缩放随手动路径删除，不再是当前值。

以上定义已经过当前机械安装的腿部输入、腿长、腿角、虚拟小腿、速度/雅可比、虚拟力矩映射和实体运动联调确认。若物理行为再次异常，先检查反馈/下发链路和报文状态，不得直接改几何符号。

---

## 二、任务架构

5 个 FreeRTOS 任务，职责分明：

| 任务 | 频率 | 节拍方式 | 职责 |
|------|------|----------|------|
| `actuationTask` | 小机 1kHz / 大机 500Hz | TIM6 信号量（硬实时） | 策略仲裁（LQR / RL）→ 力矩计算 → CAN 下发 |
| `policyTask` | 100Hz | TIM6 节拍 `MACHINE_POLICY_DIV` 分频信号量（变更 103） | 观测构建 → CubeAI 推理 → 写 action_state |
| `imuTask` | 1kHz | osDelay(1ms) | HI229 新帧解析 → 姿态更新 → 写 imu_state |
| `commTask` | 1kHz | osDelay | DM/DJI/DR16 解析 → 状态更新 → 在线检测 → 故障位 → VOFA（或 S2R1 诊断遥测 `S2R_Pump()`，二者同口互斥） |
| `defaultTask` | - | - | USB 初始化（保留） |

**单写者模型**：每个共享状态只有一个任务写，32bit 对齐 float 在 M7 上读写原子，无需锁。

`freertos.c` 只维护任务初始化、循环和节拍等待。共享状态和公共控制接口位于 `robot_control.c/h`，各 `task_*.c` 实现对应任务的单周期逻辑。

**数据流向**：
```
[ISR] FDCAN → dm/dji raw_pending
[ISR] UART  → hi229_rx.flag / dbus_rx.flag
[ISR] TIM6  → ctrl_tick_sem (控制环信号量, 小机 1kHz / 大机 500Hz)

imuTask     → imu_state {quat, eul, gyr, acc, online}
commTask    → motor_state/leg_state; ctrl_fault; VOFA
policyTask  → action_state {a[6], last_ok_tick, updated, rl_ready}
actuationTask → torque_output_t → DM/DJI 力矩 → CAN
```

---

## 三、RL 部署链路详解

### 3.1 五连杆运动学 (`leg_solver`)

**输入**：左右各 2 个 DM 髋电机的角度 + 角速度（hip_f=前髋，hip_b=后髋）。前髋的几何零位比 DM 反馈零位多 π，使"前后上连杆反向水平"的最短腿姿态对应虚拟腿竖直。腿长和腿角由前后髋共同决定，不能把两台电机简称为"大腿/小腿电机"。

**计算**：
- 闭链几何求解足端 P → 腿长 l0、腿摆角 phi0（前摆为正，含零点偏置）
- 大腿角 `thigh_angle = Leg_Wrap(qf)`（前髋上连杆角，去镜像后与 hip_f 一致）
- 虚拟小腿角 `virtual_shank = wrap(phi_a - qf - π/2)`（相对前髋电机）
- 雅可比：`point_jac`（足端直角坐标）、`leg_jac`（极坐标）、`vshank_jac`（虚拟小腿）、`force_map`（力域转换）

实现分为闭链几何、速度与雅可比、力矩映射三层，`Leg_Solve()` 仅负责按顺序调用三层。

**根因修复（2026-09-17）**：`thigh_angle` 原来错误使用 `phi_a + π/2`（下连杆绝对角），后髋运动会干扰大腿角。修正为 `Leg_Wrap(cache->qf)`，即前髋上连杆角，只有前髋动才改变。已实机验证。

**极性**：DM/DJI 驱动反馈层统一到机体坐标系，右前髋、右后髋和右轮的物理角度/速度取反；五连杆输入不再重复做右腿镜像（原 `config.mirror` 字段已删，变更 102，几何层恒等）。

**输出**：`leg_output_t` 含 thigh_angle/l0/phi0/virtual_shank/各雅可比/force_map/valid

VOFA 正常控制帧为 32 通道；帧内容按当前测试阶段切换（当前为 LQR 观测帧，RL 调试打包在 `task_comm.c` 内已注释留档），不再维护通道 Markdown，当前布局直接以 `task_comm.c::Robot_Control_Send_Vofa()` 上方注释为准。

DM 反馈层已对右侧电机取反（`feedback_sign`），力矩下发按 `output_sign` 在 `dm.c` 边界取反，使逻辑侧正力矩与左右实体电机的正运动方向一致。

旧 F/T 与手动 action 测试入口已删除，执行链只保留策略动作→虚拟关节 PID→雅可比→实际电机。

### 3.2 观测构建 (`rl_observation`)

25 维观测 + 5 帧历史 = 125 维输入

| 索引 | 内容 | 缩放 |
|------|------|------|
| 0-2 | 陀螺仪角速度 (rad/s) | × 0.25 |
| 3-5 | 投影重力 (机体坐标系) | × 1.0 |
| 6-8 | 指令 [vx, yaw_rate, height] | × [2.0, 0.25, 5.0] |
| 9-12 | 训练关节角 − 默认角 [lf0, lf1, rf0, rf1]，默认角 [−0.06, 0.10, 0.06, −0.10] | × 1.0 |
| 13-18 | 训练关节速度 (6 维含轮子) | × 0.05 |
| 19-24 | 上步动作 (训练空间原值) | × 1.0 |

整体裁剪到 ±100。缩放与默认角来自训练侧《ONNX 导出说明》（2026-09-22），宏在 `rl_observation.h`，`RL_Observation_Param_Init()` 填入并置 `configured=1`。

投影重力：`g_body = R^T * [0,0,-1]^T`，用四元数旋转计算。

历史堆叠：循环左移，`[t-4, t-3, t-2, t-1, t]` 五帧；首帧重复 5 次填满，末帧含本次观测。

**关节映射**（`task_policy.c::RL_Joint_Map()`）：训练用串联代理关节 lf0/lf1（lf1 不是实物电机），固件给的是前髋大腿角 `thigh_angle` 与 EGA 虚拟小腿角 `virtual_shank_angle`。观测关节 = `sign × wrap(固件角 − zero)`，速度乘同一 `sign`；`sign[6]`/`zero[4]` 在机器表 `machine->rl`，**大机器表已按候选 A 填写并置 `configured=1`（变更 98，数值以 `machine_config.c` 为准、待作者台架复核，见 §8.1）；小机器表未配置（全 0、`configured=0`）**。轮速按物理左右交叉取源（左轮槽取 DJI RGT 索引，见 `task_policy.c` 注释）。未配置时观测无效、不推理、零力矩。

### 3.3 推理 (`rl_policy`)

只有一个 CubeAI 模型：`networkzn1`（训练侧 chuanliantui_standup `model_6000_h723.onnx`，X-CUBE-AI 10.2.0 / ST Edge AI Core 2.2.0 生成，2026-09-22）。

| 张量 | 形状 | 含义 |
|------|------|------|
| 输入 0 `observations` | [1, 25] | 当前观测 |
| 输入 1 `observation_history` | [1, 125] | 5 帧历史 |
| 输出 0 `actions` | [1, 6] | 动作均值（训练空间） |
| 输出 1 `latent` | [1, 3] | 编码器输出，只存进 `rl_policy_t.latent` 供调试 |

权重 152 KB flash、activation 1.1 KB、43k MACC；`rl_policy.c` 静态检查 25/125/6/3，激活缓存静态分配（`AI_ALIGNED(4)`），无堆内存，CRC 外设时钟在 `RL_Policy_Init()` 使能。`RL_Policy_Run()` 记录成功/失败次数与耗时（`run_ok`/`run_fail`/`run_us`，DWT 计时）。

旧的 stable/pin/upstairs/jump 四模型（EGA 参考车）已从两套工程与 `X-CUBE-AI/App` 删除；`rl_model_t` 只剩 `RL_MODEL_STANDUP`。

### 3.4 力矩执行 (`rl_torque`)

6 维动作 → 6 个电机力矩的完整转换：

```
1. 动作 (固件关节空间; task_policy 把训练动作乘 .rl.sign 得到)
2. PD 控制 (actuationTask 控制环; 大机 500 Hz 同训练内环):
   腿关节(4维): 目标 = act × 0.5 + dof_pos, dof_pos = zero + sign × 训练默认角 (RL_Torque_Param_Init 按机器表算)
                tau_v = Kp × wrap(目标 − q) − Kd × q̇   (D 项用关节速度, 同仿真 PD; 不再用 pid 的 Δe 微分)
   轮子(2维):   目标速度 = clamp(act × 10, ±20 rad/s) (训练侧尺度; RL_TQ_WHEEL_VEL_MAX, rl_torque.c:10); tau_v = Kp_w × (目标 − 轮速)
3. 虚拟→实际 (vshank_jac):
   tau_front_hip = tau_thigh + tau_shank × vshank_jac[1]
   tau_rear_hip  = tau_shank × vshank_jac[0]
4. 输出限幅: 机器表 dm_trq_clamp / dji_trq_clamp
```

（轮目标速度 `±20 rad/s` 限幅**已实现**（2026-09-25）：`rl_torque.c` 的 `RL_TQ_WHEEL_VEL_MAX`（:10）对 `vel_ref` 两端限幅（:227-230）。边界语义：`|a_wheel| > 2` 时目标速度饱和于 ±20 rad/s，而 `last_action` 仍记录未饱和的动作（仅受 `RL_ACTION_CLIP` 裁剪，`task_policy.c:184-187`）——该语义待训练侧确认。）

力矩输出结构 `torque_output_t` 将 DM 与 DJI 分离（已重构）：
- `dm[DM_MOTOR_NUM]` — 4 个髋关节力矩，按 DM 驱动索引（F_LFT/B_LFT/F_RGT/B_RGT）
- `dji[DJI_MOTOR_NUM]` — 2 个轮子力矩，按 DJI 驱动索引（WHEEL_LFT/WHEEL_RGT）
- 右轮极性在 `dji.c` 驱动边界按 `output_sign` 处理，调用方不再取反（`tau_v[VJ_R_WHEEL]` 直接传）
- `task_actuation.c::output_dispatch()` 调用 `Dm_Send_Torque(torque.dm)` + `Dji_Send_Wheel_Torque(...)` 下发；左右轮槽位在分发边界交叉（右槽进左轮、左槽进右轮），对应物理左右轮反馈源交叉

PID 参数按模型存表，具体数值以 `RL_Torque_Param_Init()` 为准。控制器在总初始化和模型切换时逐个调用 `PID_struct_init`，不做运行时参数同步。气弹簧端点几何来自训练仓 `sim2sim/chuanliantui.xml`，`gas_spring_force_n[2]` 表示左右轴向推力，`gas_comp_sign[2]` 的 0 关闭、+1 叠加同向被动力矩、-1 抵消被动力矩；符号及端点安装一致性待台架确认。两份机器表当前 `gas_comp_sign` 均为 0（默认关闭），补偿仅在 `.rl` 已配置且符号非 0 时生效。补偿进入虚拟小腿力矩，再经原雅可比映射到四髋，当前不包含输出斜率限制。

**当前 PD 参数（以代码为准，来自训练仓库 `chuanliantui_config.py` control 段）**：腿 Kp 10 / Kd 1.0，轮速度增益 0.1（训练 damping），虚拟关节力矩先裁到训练上限 40 / 3.9 N·m（Isaac 在映射前裁），再经机器表 `dm_trq_clamp` / `dji_trq_clamp` 限幅。训练 PD 公式 `τ = Kp(目标 − q) + Kd(目标速度 − q̇)`，腿目标速度 0、轮 Kp 0，与固件实现一致。

**已修正**：DJI 力矩常数的减速比因子已修——`per_raw` 按 `machine->dji_gear_ratio` 缩放（见 `dji.c` 的 `Dji_Torque_To_Current`）。剩余：**Kt 绝对值仍待台架实测**（悬臂挂砝码/弹簧秤法）。型号/刻度/减速比已集中到 `imcalib/user-lib/machine_config.h`。

### 3.5 遥控映射 (`task_policy`)

历史（**已移除**）：手动遥操模式曾是默认路径（`rl_control.infer_enable = 0`），遥控器直接控制关节偏移：

| 通道 | 输入 | 映射 |
|------|------|------|
| ch3（左Y） | 大腿偏移 | ×4.0 叠加到 base_action |
| wheel（拨轮） | 小腿偏移 | ×4.0 叠加到 base_action |
| ch1（右Y） | 轮子速度 | ×4.0 直接赋值（宽死区100） |
| ch0（右X） | yaw 指令 | ×REMOTE_COMMAND_SCALE → obs |

使能边沿锁存当前关节角为 base_action，后续摇杆在此基础上偏移。该路径连同 `rl_control.infer_enable`、`MANUAL_ACTION_SCALE` / `REMOTE_COMMAND_SCALE`、base_action 锁存已从代码删除，上表仅作历史记录。

当前 `task_policy.c` 只保留**推理路径**（`RL_Infer_Body()`，100 Hz，`machine->rl.configured` 时直接运行，无运行时开关）：

| 步骤 | 内容 |
|------|------|
| 投入判定 | 读 `output_task_rl_engaged()`：左拨杆上 + 右拨杆中 + 电机使能 + 遥控在线（拨杆语义仍只在 `task_actuation.c`）。未投入：清历史、发零动作、`rl_ready=0`；观测仍照算一份预览供 VOFA（不进历史，变更 99） |
| 指令 | 右摇杆 Y → vx × `RL_CMD_VX_MAX`；右摇杆 X → yaw_rate = −yaw × `RL_CMD_YAW_MAX`（右推为负，同 LQR）；拨轮 → 高度在 `[RL_CMD_HEIGHT_MIN, MAX]` 线性。起立策略训练域 vx [0,0]、yaw [0,0]、高度固定 0.20 m（`chuanliantui_standup_config.py`），所以宏为 0 / 0 / 0.20，**RL 模式下摇杆和拨轮无效** |
| 观测 | `RL_Control_Update_Observation()`：源无效或 `.rl` 未配置 → 从头预热、零动作 |
| 预热 | 投入后前 `RL_WARMUP_STEPS`（10 步 = 0.1 s）发零动作（此间 `rl_ready=1` 但动作为零），PD 与历史照跑。训练是"首次任一轮接触力 > 1 N 后的下一策略步才推理"，实机轮本来就在地上，只留短预热 |
| 推理 | `RL_Policy_Run()` → 训练动作 → 裁剪 `RL_ACTION_CLIP` = 100（训练 clip_actions）→ 存 last_action（训练 obs 里也是 clip 后的动作）→ 乘 `.rl.sign` 变固件动作 → 发布，`rl_ready=1` |
| 失败 | 推理失败：发零动作、`rl_ready=0`（零力矩），`run_fail++`（低 8 位进 VOFA ch2 状态位）；失败时同步将 `last_action` 记为零动作（2026-09-25）。执行侧同语义：`solve_rl()` 不再无条件置 `valid=1`，`RL_Torque_Compute()` 返回 0 时置 `torque->valid=0` 直接返回，由 `output_dispatch` 走零力矩（`Dm_Send_Zero` + `Dji_All_Stop`），成功才 `valid=1`（`imcalib/task/task_actuation.c:148`） |

`task_actuation.c::solve_rl()` 的出力前提：遥控使能 + 电机使能 + 两腿 valid + `rl_ready`，且 `RL_Torque_Compute()` 返回成功（失败即 `valid=0` 零力矩，见上）。policyTask 每 10 ms 发布一次动作（未投入时也发零动作保持新鲜）；`FAULT_ACTION` 只在左拨杆上位（RL 挡）时判定，两条判据（`task_comm.c::Robot_Fault_Update`）：①动作心跳过期（`action_state` ≥100 ms 不新鲜/停发）；②已投入（`output_task_rl_engaged()`）但 `action_state.rl_ready == 0` 持续 ≥100 ms，即观测无效/推理失败持续（2026-09-25 新增）。

### 3.6 使能与安全 (`task_comm`)

**使能状态机**（`Robot_Enable_Update()`，commTask 1 kHz）：
- 遥控 s1 中/上 = 使能请求
- s1 下 / 遥控离线 / 任一故障 / 翻倒 = 失能请求
- FAULT_ACTION 两条判据（仅左拨杆上位/RL 挡判定，`Robot_Fault_Update()`）→ 置故障 → 失能：①动作心跳过期(>100ms 不新鲜/停发)；②已投入（`output_task_rl_engaged()`，RL 挡+右中位+电机使能）但 `action_state.rl_ready==0` 持续 ≥100ms，即观测无效/推理失败持续（2026-09-25 新增）
- 使能沿：`motor_enabled=1`、发 `Dm_All_Enable()`；失能沿：`motor_enabled=0`、发 `Dji_All_Stop()` + `Dm_All_Disable()`
- 两个方向都有 100 ms 看门狗兜底：使能期间对仍失能的电机重发使能，失能期间对仍使能的电机重发失能（`Dm_Enable_Watchdog` / `Dm_Disable_Watchdog`）
- 2026-09-21 修复：变更 60 接看门狗时把失能分支写成了不可达代码，使能后再也退不出来（拨杆下位、遥控离线、翻倒都不失能），见账本变更 75

**故障位**：`FAULT_IMU | FAULT_RC | FAULT_MOTOR | FAULT_CAN | FAULT_ACTION`

**翻倒**：|pitch| > 1.4rad 置 fallen，< 1.0rad 回正（回差）

**总开关**：`torque_output_enabled`（`robot_control.c::Robot_Control_Init()` 初值 **1**，作者确认为有意设计（2026-09-25））。置 0 时 RL/LQR 链路第一步只看 VOFA 不出力。

`torque_output_enabled=0` 时执行任务保持 DJI 零电流和 DM 零力矩；遥控、动作、IMU、CAN、电机在线与翻倒保护仍有效。

### 3.7 LQR 平衡模式

actuationTask 策略仲裁：**左拨杆中位 = LQR，上位 = RL 推理，下位 = 失能；右拨杆中位才投入当前模式输出**。机器表未配置的模式（`lqr_configured` / `rl.configured` 为 0）直接判失能。

手动腿测和 Sysid 测试路径已在 2026-09-24 清理；`LQR_PLAN.md` 中的对应章节保留为历史记录。

LQR 链路（`lqr_balance.c` + `leg_balance.c`）与 RL 控制逻辑分开，只在 `task_actuation.c` 的策略分支交汇，彼此不直接调用。完整设计、参数来源、台架验证顺序与遗留项见 **[LQR_PLAN.md](LQR_PLAN.md)**。

要点速记：
- LQR 是**第一套真正能站的自动控制器**（RL 推理路径已接入，待台架）
- 遥控在 LQR 模式下换语义：右摇杆 X=转向，右摇杆 Y=前后速度，拨轮=升降
- LQR 投入查 `motor_enabled && imu_state.online && leg_l.valid && leg_r.valid`（`base_action_locked` 概念已随手动路径删除）
- LQR 不满足条件时直接零力矩，**不自动降级**到别的策略
- `lqr_debug` 可在调试器 Watch 中独立开关轮、髋、腿长 PID 并调整轮/髋限幅；不占 VOFA 通道
- LQR 当前保留腿长/横滚 PID，防劈叉 PID 已移除；腿长区间按本机自标，符号/零点待台架，见 `LQR_PLAN.md` §六 ①

---

## 四、当前进度 — 已完成 vs 待完成

### 当前完成情况

| 模块 | 文件 | 状态 |
|------|------|------|
| FDCAN 总线 | can_bus.c/h | ✅ 路由注册 + 批量接收 + bus-off 恢复 + RX 看门狗 |
| DM 电机 | dm.c/h | ✅ MIT 协议 + 解码 + 在线检测 + 多圈计数 |
| 机器配置表 | machine_config.c/h | ✅ 新增，两份表 + 运行时切换（M3508+J8009P / M2006+J4310） |
| 单调 ns 时钟 | mono_ns.c/h | ✅ 新增，DWT CYCCNT + 1kHz 周期扩展 |
| DJI 轮电机 | dji.c/h | ✅ 电流控制 + 解码 + 在线检测 + 减速比修正 |
| UART 底层 | uart_idle.c/h | ✅ IDLE+DMA Circular |
| DR16 遥控 | dr16.c/h | ✅ 解析 + 实测正常 |
| HI229 IMU | hi229.c/h | ✅ 通信 + 数据提取 |
| 姿态解算 | Attitude_Algorithm.c/h | ✅ Mahony + HI229 融合 |
| Vofa 调试 | Vofa_send.c/h | ✅ JustFloat DMA 发送；通道布局以代码为准 |
| 五连杆 | leg_solver.c/h | ✅ 几何/腿长/腿角/虚拟小腿/雅可比/force_map/极性，含 thigh_angle 根因修复，已上机验证 |
| RL 观测 | rl_observation.c/h | ✅ 缩放/默认角已按训练侧填（configured=1）+ ±100 裁剪；🟡 关节映射 `.rl` 大机器已按候选 A 填（configured=1），待台架核对 |
| CubeAI 推理 | rl_policy.c/h | ✅ 单模型 networkzn1（2 输入 2 输出）+ 耗时/成败计数；✅ 已接进 policyTask 推理路径（`machine->rl.configured` 门控）；🟡 待台架 |
| 力矩执行 | rl_torque.c/h | ✅ PID + 雅可比映射 + DM/DJI 分离输出 + 轮子 PID；已上机验证 |
| 任务框架 | task/robot_control.c + task_*.c | ✅ 4 任务体、共享状态、使能机、故障门、VOFA 32ch（上限 32）；已上机验证 |
| 遥控映射 | task_policy.c | ✅ 推理路径（投入 → 预热 → 推理 → 映射发布）；手动遥操路径已删除（历史） |
| 力矩下发 | task_actuation.c | ✅ torque_output_t 直接下发 DM+DJI，左右轮槽位在分发边界交叉 |
| 离线测试 | tests/offline_test.c | ⚠️ 已移除（文件已不存在，历史记录） |

### 待实测 / 待配置

| 项目 | 位置 | 说明 | 优先级 |
|------|------|------|--------|
| **关节映射 `.rl`** | `machine_config.c` | ✅ 已按候选 A 填（变更 98，作者定，`configured=1`）；🟡 台架核对默认站姿 ch11 / ch13 读数与 ch21~24 ≈ 0（ch 号为历史 RL 帧布局，当前 VOFA 帧见 `task_comm.c` 注释） | 🟡 P0 |
| **PD 增益 / 力矩上限** | `rl_torque.c::RL_Torque_Param_Init()` | ✅ 已按训练仓库填：Kp 10 / Kd 1 / 轮 0.1，虚拟力矩上限 40 / 3.9 | 🟢 |
| **指令 / 预热 / 动作裁剪** | `rl_policy.h` | ✅ 已按训练仓库填：vx、yaw 0，高度 0.20，预热 10 步，clip 100；高度 0.20 需训练侧确认该 checkpoint 课程已解锁（解锁前是 0.30） | 🟡 P1 |
| **DJI 力矩常数** | `dji.h DJI_NM_FULL_*` + `dji_gear_ratio` | ✅ 减速比因子已修正；Kt 绝对值待实测（悬臂挂砝码法） | 🟢 P2 |
| **遥控缩放（手动路径）** | `task_policy.c` | ⚠️ 已随手动遥操删除（`MANUAL_ACTION_SCALE` / `REMOTE_COMMAND_SCALE` 不存在，历史记录） | — |
| **跌倒恢复** | 未实现 | 只有翻倒标志，无自动起身 FSM | 🟡 P2 |
| **轮子电机毛刺** | DJI M2006 | 即使无 D 项也抖，可能需死区或低通滤波 | 🟡 P2 |

### ⚠️ 与参考实车的差异

| 项目 | 参考实车 | 本项目 | 影响 |
|------|----------|--------|------|
| IMU | BMI088 板载 SPI | HI229 外挂串口 | 姿态源不同，四元数约定可能需调整 |
| 髋电机 | DM8009P (54Nm) | DM J4310 (10Nm) | 力矩限幅不同，需确认是否够用 |
| 轮电机 | M3508 (16.33减速比) | M2006 (36减速比) | 减速比因子已按实机 gear_ratio 缩放；Kt 绝对值待实测 |
| 功率控制 | 超电 + RLS 自适应 | 无 | 暂无功率限制 |
| 大型自起 | LargeRecover FSM | 无 | 只有翻倒标志，无自动恢复 |
| 小陀螺动作延迟 | Spin 模式延迟 1 周期 | 无 | Spin 策略效果可能不同 |
| ToF 跳跃触发 | ToF 测距触发跳跃 | 无 | Jump 策略手动触发 |
| D-Cache | 开启 + Clean 处理 | 已开启；VOFA DMA 发送前 Clean | 继续保持 DMA 缓冲一致性 |
| FPU Error | 关闭 | 未确认 | 需在 CubeMX 中关闭 |

注：上表按小机器对比（写表时的默认机器；`MACHINE_DEFAULT` 自 2026-09-26 起已切大机器，变更 100）；RL 模型对应的大机器电机/减速比与参考实车同级，具体型号、刻度与减速比以 `machine_config.c` 为准、待台架复核。

---

## 五、实测打开顺序

Leg_Solve 当前已完成以下验证：

```text
输入极性与前髋 +π 几何零位
→ 腿长与虚拟腿摆角
→ 虚拟小腿角
→ thigh_angle 根因修复 (phi_a → qf)
→ 速度与解析雅可比
→ 虚拟 F/T 到实际髋电机力矩
→ 左右腿实体输出极性
→ 轮子极性与减速比修正
→ torque_output_t DM/DJI 分离重构
→ 低力矩上机运动（手动遥操模式）
```

后续 RL 整链路仍按以下顺序进行，任何一步失败则停止：

（下列 ch 号为历史 RL 调试帧布局；当前 VOFA 帧是 LQR 观测布局，以 `task_comm.c::Robot_Control_Send_Vofa()` 上方注释为准。RL 信号现经调试器 Watch 读取：`rl_control.observation.obs` / `.last_action`、`rl_output_dm_cmd_nm` / `rl_output_wheel_cmd_nm`。）

```
① .rl 已按候选 A 填 (变更 98); 台架核对 (左拨杆上, 不投入即可, 变更 99 起有观测预览):
   默认站姿 ch10/ch12 thigh ≈ 2.54, ch11/ch13 vs ≈ 2.99, ch21~24 ≈ 0
   → 数值明显不符 (如 thigh ≈ 0.6) 就是候选 B, 停下来重填
   → 手扳看极性: ch3~5 重力, ch7~9 角速度, ch15~20 关节速度, ch21~24 关节角

② 总输出关 (torque_output_enabled 置 0; 初值现为 1), 左上 + 右中投入 (推理路径无 infer_enable, .rl 配置即运行)
   → VOFA ch6 状态位就绪、ch31 耗时, ch21~24 映射零点合理 (网络动作已不上 VOFA, 要看在调试器 Watch last_action)
   → 低头 ch3 为正; 身体前倾时轮动作 (ch17/ch20) 应朝前追车身

③ 填 PD 增益, 仍总输出关
   → ch25~30 力矩方向: 腿目标偏离时力矩指向目标, 轮力矩方向同动作

④ 架空开总输出, 小限幅
   → PD 保持默认姿态, 摇杆指令不发散

⑤ 下地
   → 起立 / 站立; 逐步放开限幅与指令范围
```

---

## 六、踩坑备忘（来自参考实车）

1. **CubeMX FPU Error**：关掉，否则 CubeAI 跑着跑着就进异常中断
2. **CubeMX 时间配置**：每次点开 CubeAI 栏目后弹窗选 **No**，否则时钟被改
3. **syscalls.c 被删**：CubeAI 启用后重新 generate 会删此文件，需提前重命名备份
4. **D-Cache 与 DMA**：CubeAI 可能自动开启 D-Cache，导致 DMA 读旧数据
5. **电机偏置**：必须在 SolidWorks 中测量，实机零点→策略零点的偏置角
6. **欠压保护**：8009P 最好用 V3 版本（12V 以下才进保护），J4310 需确认保护电压
7. **DM/DJI 混用隐患**：不要把 DM 和 DJI 混合成同一个 motor 数组，索引混淆会导致力矩写错电机（已踩坑，已重构为 `torque_output_t` 分离）

---

## 七、关键约束速查

- **时钟**：HSE 24MHz → PLL → SYSCLK 550MHz
- **FDCAN**：1Mbps 仲裁段 = Prescaler=3, Seg1=5, Seg2=2（`fdcan.c` 三路一致；FDCAN1/3 数据段时序随机器，FDCAN2 保持 CubeMX）
- **BMI088**（本项目未使用，用 HI229）：驱动输出已是 rad/s 和 g
- **Mahony**：无 acc_trust 门控，无输出限幅
- **串口**：IDLE+DMA Circular，不使用 Resync
- **推理频率**：100Hz（policyTask 由 TIM6 节拍 `MACHINE_POLICY_DIV` 分频信号量驱动、与控制拍锁相，变更 103；执行任务小机 1 kHz / 大机 500 Hz，每拍用最新动作）
- **观测维度**：25 + 125(历史) = 150
- **动作维度**：6，训练空间 [lf0, lf1, lfwheel, rf0, rf1, rfwheel]，乘 `.rl.sign` 变固件动作（左大腿, 左虚拟小腿, 左轮, 右大腿, 右虚拟小腿, 右轮）
- **物理通道**：DM×4（左前/左后/右前/右后髋） + DJI×2（左轮/右轮）
- **力矩限幅**：机器表 `dm_trq_clamp` / `dji_trq_clamp`
- **DJI 减速比**：M2006 = 36，反馈 rpm 是转子转速，÷36 才是输出轴（大机器 M3508 总减速比以 `machine_config.c` 为准）

---

## 八、networkzn1 接入（2026-09-22，账本变更 95）

### 8.1 训练侧定义（2026-09-23 从训练仓库 `wheelleg-reinforcement-learning-26_wheelleg` 逐项核对）

| 项 | 训练侧定义 | 固件处置 |
|----|-----------|---------|
| 关节顺序 / 轴 | `[lf0, lf1, lfwheel, rf0, rf1, rfwheel]`；左侧三轴 `(0,−1,0)`，右侧 `(0,+1,0)`，右腿镜像（FK 里右腿取 −q，默认角左右反号） | 观测 / 动作顺序一致 |
| 串联代理 | 大腿 l1 = 0.21（髋→膝）、小腿 l2 = 0.25（膝→轮心）；FK 零位偏置 0.664720554 / 1.626002937；URDF 零姿态大腿指向 −x 下方 38.1°、轮心在髋正下 0.3175 m | 与固件 `leg_lu` / `leg_lg` 相同 |
| 实机机构 | 第二台髋电机经 0.1134 曲柄 → 0.135 连杆 → 三角块 → 0.0966 推杆 → 小腿摇臂驱动膝；0.1134 / 0.135 正好是 0.21 / 0.25 的 0.54 倍，是缩比联动 | 用 URDF 几何数值扫描工作区间 4415 个姿态：轮心与固件对称五连杆最大差 2e−11 m。**固件五连杆解算对大机器成立**，`virtual_shank_angle` 就是训练 lf1（差一个常数和符号） |
| 默认角 | lf0 −0.06 / lf1 0.10 / rf0 0.06 / rf1 −0.10（微蹲，轮心在髋正下，L0 ≈ 0.30） | 已填 |
| 观测 | 25 维同 §3.2；`dof_vel` 训练用 500 Hz 位置差分，固件用电机反馈速度（噪声与延迟特性不同，待观察） | 已填 |
| PD | Kp 10 / Kd 1.0（腿），轮 Kd 0.1；`τ = Kp(目标 − q) + Kd(目标速度 − q̇)`；虚拟关节力矩上限 40 / 40 / 3.9 | 已填（变更 97） |
| 动作 | 腿 ×0.5 + 默认角，轮 ×10；clip_actions 100；obs 里的 last_action 是 clip 后的动作 | 已填 |
| 指令 | 起立训练域 vx [0,0]、yaw [0,0]、高度 0.20 m（课程解锁后；解锁前 0.30）；sim2sim 回放用 `--cmd_vx 0 --cmd_height 0.20` | 已填 0 / 0 / 0.20；**checkpoint 是否解锁需训练侧确认** |
| 起立接管 | 初态 0.15 m 地面后摆，lf0 = ±11 rad（Isaac 不 wrap；sim2sim 回放 wrap 到 ±π 即 −1.566）；首次任一轮接触力 > 1 N 后的下一策略步才推理，之前零动作 + PD | 预热 10 步；固件角度 wrap ±π 与 sim2sim 一致；起立本身以训练侧 sim2sim 结果为准，先测站立 |
| 坐标 | URDF 机体系，z 上；重力投影 = quat_rotate_inverse(q, [0,0,−1])；角速度机体系 | 与固件一致，前提是下面的 x 方向判定 |

**关节映射（2026-09-23 作者定：按候选 A 填表，变更 98；`.rl` 已置 `configured=1`，台架仍按下面方法核对）**

训练关节与固件角只差符号和常数。固件角在"x 前、y 下"平面里量，训练 FK 在"x 后、z 下"平面里量。

- **候选 A**：固件 +x（作者定义的前进方向）= URDF +x。
  `lf0 = −(thigh_L − 2.476872)`，`lf1 = −(vs_L − 3.086386)`，`rf0 = +(thigh_R − 2.476872)`，`rf1 = +(vs_R − 3.086386)`；轮：lfwheel = −左轮速（训练左轮正向 = 向后滚），rfwheel = +右轮速。
  即 `.rl.sign = {−1, −1, −1, +1, +1, +1}`，`.rl.zero = {2.476872, 3.086386, 2.476872, 3.086386}`。默认站姿下固件应读到 **thigh ≈ 2.54 rad、vs ≈ 2.99 rad**（VOFA ch11 / ch13）。这也是固件"前髋 +π、前上连杆指向后方"约定对应的情形。
- **候选 B**：固件 +x = URDF −x（训练的"前"是实机车尾）。
  `lf0 = +(thigh_L − 0.664721)`，`lf1 = +(vs_L − 0.055207)`，右腿取反；轮：lfwheel = +左轮速，rfwheel = −右轮速；并且策略用的机体系要绕 z 转 180°，gyro x/y 与重力 x/y 取反，`.rl` 要再加帧符号字段。默认站姿下固件读到 **thigh ≈ 0.60 rad、vs ≈ 0.16 rad**。
- **台架判定**：站到默认姿态（轮心在髋正下、微蹲），看 ch11 / ch13 落在哪一组，一眼分清。
- **左右归属（已处理，不必等 CAD）**：URDF 里 `lf0` 在 y = −0.179，候选 A 下 "lf" 是 URDF 的右侧腿。机器左右对称时，"右腿喂 lf 槽、IMU 原样"与"左腿喂 lf 槽、策略机体系绕 x-z 面镜像"是同一策略的镜像部署，效果等价。历史：固件曾取后者，由 `task_policy.c` 的 `RL_FRAME_MIRROR_Y = 1` 实现，推理路径把 gyro x/z、四元数 x/z、偏航指令取反，左腿仍进 lf 槽，VOFA 的"左"仍是左。**该宏已删除**：当前推理路径无帧镜像开关，左右/极性以机器表 `.rl` 的 `sign` 与 `machine_config.c` 为准、待作者台架复核（左右轮另有物理交叉取源/下发，见 `task_policy.c` / `task_actuation.c`）。CAD 答复只在机器明显不对称时才有意义。
- 训练默认角左右反号只是因为右腿轴反向，物理姿态左右对称；映射后固件左右腿的 `dof_pos` 都是 thigh 2.5369 / vs 2.9864。

`MACHINE_DEFAULT` 已于 2026-09-26 切到 `MACHINE_ID_BIG_WHEELLEG`（变更 100，宏名以 `machine_config.h` 为准），RL 整链门控随之打开；小机器上跑这个策略没有意义。大机器 `.imu` 的 `quat_src` 与角速度通道是否对应有未确认疑点，见变更 100「待台架 ①」。大机器 `.imu` 表仍是"照抄原宏、待实测"，②那一步一起看。

### 8.2 运行时开关（原"两个运行时开关"）

| 开关 | 位置 | 含义 |
|------|------|------|
| `torque_output_enabled` | `robot_control.c::Robot_Control_Init()` 初始化 **1**（作者确认为有意设计，2026-09-25） | 总输出开关；置 0 时只观测不出力 |
| `rl_control.infer_enable` | ⚠️ **已删除**（历史：`robot_control.c` 初始化 1，0 = 旧手动遥操、1 = 推理路径） | 推理路径现由 `machine->rl.configured` 门控直接运行 |
| `RL_FRAME_MIRROR_Y` | ⚠️ **已删除**（历史：`task_policy.c` 宏，默认 1，左腿喂 lf 槽的镜像配套） | 见 §8.1 左右归属 |

### 8.3 VOFA

当前 VOFA 帧为 LQR 观测布局，**以 `task_comm.c::Robot_Control_Send_Vofa()` 上方注释为准**：ch0 在线掩码、ch1 状态位、ch2 RL 状态位、ch3~12 LQR 状态 x[0..9]、ch13~22 目标 target[0..9]、ch23~24 实测腿长、ch25~26 腿长目标、ch27~30 LQR 输出 u[0..3]、ch31 故障位；每两次 commTask 周期发一帧。未投入时观测照算预览。

历史（RL 调试帧，`task_comm.c` 内已注释留档）：RL 模式（推理路径且左拨杆上位）曾复用 LQR 无意义的通道，下标不动：ch3~5 投影重力、ch6 RL 状态位、ch7~9 观测角速度（策略机体系，已镜像、×0.25）、ch10~13 固件原始 thigh / vs、ch15~20 观测关节速度（×0.05）、ch21~24 观测关节角、ch25~30 力矩命令（总输出关也有值）、ch31 推理耗时。
