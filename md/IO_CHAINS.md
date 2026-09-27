# 输入输出链路总览

> 2026-09-24：Sysid、手动腿测、仅偏航测试已删除。下文若有旧测试阶段描述，以当前代码为准。

> 最后更新：2026-09-25
> 本文档记录各传感器/执行器的完整数据链路，从硬件到消费端。
> 物理量（极性/零点/轴向/量程刻度/mirror/符号项）一律以 `machine_config.c` 当前机器表为准、待作者台架复核；文中数值只作历史记录。

---

## 1. IMU（HI229 姿态传感器）

```
HI229 模块 (UART7 921600bps)
    │  IDLE+DMA 接收
    ▼
hi229.c: HI229_Process()
    帧头搜索(0x5A 0xA5) → CRC 校验 → 解析 payload
    写入 hi229_data_t:
      eul[3]   raw 欧拉角 deg
      gyr[3]   raw 角速度 deg/s
      acc[3]   raw 加速度 G
      quat[4]  raw 四元数
      ts       模块时间戳 ms
      online   100ms 无帧→清零
    │
    │  HI229_Snapshot()
    ▼
task_imu.c: imu_task_body() @ 约 1kHz (imuTask osDelay(1))
    去重(ts 与 last_timestamp_ms 相同则丢) → 按机器表取轴 + 乘符号 → 单位转换 → 写入 imu_state:
      quat[4]       [0] 原样; [1..3] = quat_sign[i] × raw.quat[quat_src[i]+1] → 归一化
                    (输出 X/Y/Z 各自先经 quat_src 选模块通道, 再乘 quat_sign, task_imu.c)
      euler_deg[3]  按机体 俯仰/横滚/偏航 序: eul_sign[i] × raw.eul[eul_src[i]]
      euler_rad[3]  euler_deg × DEG2RAD (Attitude_Update)
      gyro_rad_s[3] gyr_sign × raw × DEG2RAD (只做符号; 存序为机体 X/Y/Z)
      acc_g[3]      acc_sign × raw (G, 只做符号)
      online        HI229_Online() 且四元数归一化成功
    │
    ▼
消费端:
  task_policy.c:
    imu_state.online       → 观测有效门控
    imu_state.gyro_rad_s   → obs.gyro
    imu_state.quat         → obs.gravity (RL_Observation_Project_Gravity)
  task_comm.c:
    imu_state.online       → FAULT_IMU
    imu_state.euler_rad[ATTITUDE_PITCH] → |pitch| 翻倒检测
  lqr_balance.c:
    euler_rad[PITCH/ROLL/YAW], gyro_rad_s[1]/[2] → 俯仰、横滚、偏航角、俯仰/偏航角速度
    quat + acc_g → 前向加速度 (LQR_Accel_Forward, 速度卡尔曼输入)
```

**IMU 安装极性在机器表（`machine_config.c` 的 `.imu`，2026-09-21 从 `hi229.h` 全局宏搬入）：**

> ⚠ 下表数值为 2026-09-21 文档记录（历史快照），与当前 `machine_config.c` 机器表数值不一致。物理量一律**以 `machine_config.c` 为准、待作者台架复核**，本文不改写数值。

| 字段 | 含义 | 小机器（2026-09-21 文档记录，非当前值） |
|------|------|:---:|
| eul_src[3] | 机体 俯仰/横滚/偏航 各取模块哪一路（0 Roll / 1 Pitch / 2 Yaw） | {1, 0, 2}（轴不换） |
| eul_sign[3] | 欧拉角符号，同序 | {+1, −1, −1} |
| gyr_sign[3] | 角速度符号，模块 X/Y/Z | {−1, +1, −1} |
| acc_sign[3] | 加速度符号，模块 X/Y/Z | {−1, +1, −1} |
| quat_src[3] | 四元数输出 X/Y/Z 各取模块哪一路 | 见 machine_config.c |
| quat_sign[3] | 四元数 X/Y/Z 符号（quat_src 之后独立施加） | {−1, +1, −1} |

四组符号对应"模块绕 Y 轴装反 180°"，彼此一致（历史说明；与当前机器表不一致处以代码为准、待台架复核）。大机器表为独立一组值（见 `machine_config.c`），与早期"暂填同值"的记录不同，待作者确认。注意 `imu_state` 欧拉角按 `ATTITUDE_PITCH=0 / ROLL=1 / YAW=2` 存，角速度按机体 X/Y/Z 存，两者顺序不同。

---

## 2. DM 关节电机（达妙 J4310 MIT 协议）

```
DM 电机 × 4 (小机器 J4310: FDCAN1 左腿, FDCAN3 右腿; 型号/总线以 machine_config.c 的 dm_bus 为准)
    │  CAN 中断接收
    ▼
dm.c: Dm_Read() — 中断回调
    8 字节 → 存入 dm_motor_feedback[i].raw_data
    置 raw_pending = 1
    │
    │  comm_task 调用
    ▼
dm.c: Dm_Parse()
    raw_data 解码:
      err_raw    = data[0]>>4
      motor_id   = data[0]&0x0F
      angle_raw  = data[1]<<8 | data[2]         (16bit)
      vel_raw    = data[3]<<4 | data[4]>>4      (12bit)
      trq_raw    = (data[4]&0xF)<<8 | data[5]   (12bit)
      temp_mos   = data[6]
      temp_rotor = data[7]
    if dm_sign[i].fb < 0: angle_raw = CPR-1-angle_raw, vel/trq raw = FIELD_MAX-raw (原始域镜像, 非取反)
    pos_rad   = Dm_Uint_To_Float(angle_raw, -dm_pos_max, +dm_pos_max, 16)   /* 纯解码角（只乘过反馈极性） */
    pos_zero_rad = pos_rad + machine->dm_zero[i]             /* 加零点偏置后的关节角 */
    vel_rad_s = Dm_Uint_To_Float(vel_raw, -dm_vel_max, +dm_vel_max, 12)
    trq_nm    = Dm_Uint_To_Float(trq_raw, -dm_trq_max, +dm_trq_max, 12)   /* 量化刻度以 machine_config.c 的 dm_*_max 为准 */
    Dm_Update_Angle() → angle_total (计圈)
    │
    │  Motor_State_Update() @ task_comm.c
    ▼
motor_state.dm:
  pos_rad[i]     → 原始解码角（调试器 Watch 可用）
  pos_zero_rad[i] → leg.input.hip_f / hip_b (五连杆输入，零点后值)
  vel_rad_s[i]   → leg.input.d_hip_f / d_hip_b
  online[i]      → 故障检测
    │
    │  RL: rl_torque.c / LQR: leg_balance.c → torque_output_t
    ▼
输出: torque_output_t → task_actuation.c::output_dispatch (全文件唯一下发点)
    torque.dm[0..3] → Dm_Send_Torque(torque.dm):
      if dm_sign[i].out < 0: command_torque = -torque[i]
      trq_raw = Dm_Float_To_Uint(command_torque, ±machine->dm_trq_max, 12)
        (早期文档此处写死 ±10，已过时；反/正量化满量程一律以机器表 dm_trq_max 为准)
      Dm_Mit_Control(i, FIELD_MAX, FIELD_MAX, 0, 0, trq_raw)
        → 纯力矩模式 (kp=0, kd=0)
        → CAN 8 字节打包 → control_id
```

`err_raw` 同时参与使能状态：`0`=失能、`1`=使能、`0x8~0xE`=故障。`Robot_Enable_Update()` 在总使能期间调用 `Dm_Enable_Watchdog()`，对在线且 `err_raw=0` 的每台电机按独立 100 ms 计时重发 `DM_CMD_ENABLE`；总失能期间调用 `Dm_Disable_Watchdog()`，对在线且 `err_raw=1` 的电机重发 `DM_CMD_DISABLE`；`Dm_Has_Fault()` 并入 `FAULT_MOTOR`。

**电机映射:**

| 索引 | 位置 | CAN | feedback_id | control_id |
|:----:|------|-----|:-----------:|:----------:|
| 0 | F_LFT 左前髋 | FDCAN1 | 0x11 | 0x01 |
| 1 | B_LFT 左后髋 | FDCAN1 | 0x13 | 0x03 |
| 2 | F_RGT 右前髋 | FDCAN3 | 0x12 | 0x02 |
| 3 | B_RGT 右后髋 | FDCAN3 | 0x14 | 0x04 |

**极性不在驱动表里**：按机器存在 `machine_config.c` 的 `dm_sign[4]` / `dji_sign[2]`，每项是 `{反馈, 输出}` 一对。
**总线同理**：`dm_bus[4]`（1/2/3 = FDCANx）—— 大机器四台全在 FDCAN1，小机器左腿 1 / 右腿 3。

**关键函数:**

| 函数 | 作用 |
|------|------|
| Dm_Init() | 注册 CAN 回调 |
| Dm_Read() | 中断存 raw_data |
| Dm_Parse() | 解码 raw→物理量 |
| Dm_Is_Online() | 10ms 超时检测 |
| Dm_Is_Enabled() | `err_raw==1` 使能检测 |
| Dm_Has_Fault() | `err_raw` 在 `0x8~0xE` 的故障检测 |
| Dm_Enable_Watchdog() | 总使能期间，在线且失能态的电机每 100ms 重发使能 |
| Dm_Disable_Watchdog() | 总失能期间，在线且使能态的电机每 100ms 重发失能（丢帧兜底） |
| Dm_All_Enable() | 全部使能 |
| Dm_All_Disable() | 全部失能 |
| Dm_Send_Zero() | 零力矩 |
| Dm_Send_Torque() | 发送力矩数组 |

**ERR 状态码（不是故障位）**：`0` 失能 / `1` 使能 / `8` 过压 / `9` 欠压 / `A` 过流 / `B` MOS 过温 / `C` 线圈过温 / `D` 通信丢失 / `E` 过载。
判据是"落在 8~E 内"，不能写成 `err != 0`。

**满量程**：`dm_pos_max` / `dm_vel_max` / `dm_trq_max` 是电机的量化刻度（出厂预设 ±12.5 / ±45 / ±54，可在上位机改），
必须与 `machine_config.c` 中当前机器的 `dm_*_max` 一致，否则角度与力矩整列都错；文中具体数值只作历史记录，以机器表为准、待台架复核。
核对方式：用达妙上位机读一次并与配置表比对（固件不做读参）。

---

## 3. DJI 轮电机（M2006 电流协议）

```
DJI 轮电机 × 2 (小机器 M2006 @ FDCAN2, 共享 control_id=0x200; 大机器 M3508, 总线以 machine_config.c 的 dji_bus 为准)
    │  CAN 中断接收
    ▼
dji.c: Dji_Read() — 中断回调
    8 字节 → 存入 dji_motor_feedback[i].raw_data
    置 raw_pending = 1
    │
    │  comm_task 调用
    ▼
dji.c: Dji_Parse()
    raw_data 解码:
      angle_raw   = data[0]<<8 | data[1]    (uint16)
      vel_raw     = data[2]<<8 | data[3]    (int16)
      current_raw = data[4]<<8 | data[5]    (int16)
      temp_raw    = data[6]                 (int8)
    if dji_sign[i].fb<0: angle=CPR-angle(≠0 时), vel=-vel, current=-current
    Dji_Circle_Calculate() → angle_total (计圈)
    Dji_Update_Physical():
      angle_rad       = count × (2π/8192 / dji_gear_ratio)
      angle_total_rad = angle_total × (同上)
      vel_rad_s       = rpm × (2π/60 / dji_gear_ratio)
        (Dji_Encoder_To_Rad / Dji_Rpm_To_Rad_S: 已除总减速比, 输出轴口径)
    │
    │  Motor_State_Update() @ task_comm.c
    ▼
motor_state.dji:
  vel_rad_s[i]       → obs.joint_vel (轮速度)
                       → wheel_vel (LQR 速度估计 / RL 轮速)
  online[i]          → 故障检测
    │
    │  RL: rl_torque.c / LQR: leg_balance.c → torque_output_t
    ▼
输出: torque_output_t → task_actuation.c::output_dispatch
    torque.dji[DJI_MOTOR_*] → Dji_Send_Wheel_Torque(left_nm, right_nm)
      (RL 路径左右轮槽对调: 实际传 torque.dji[RGT], torque.dji[LFT]，
       台架反馈核对的轮左右交叉, 见 task_actuation.c::output_dispatch, 待作者台架确认)
      输出极性: 力矩 × dji_sign[i].out 后才换算电流 (dji.c 驱动边界)
      Dji_Torque_To_Current():
        raw = torque_nm / per_raw, clamp ±满 raw
        per_raw = 满电流堵转力矩/满 raw × (dji_gear_ratio / 标准减速比)
        (M2006 在标准减速比下等效 0.00018 Nm/raw、±10000；随 dji_gear_ratio 缩放, 以 dji.c/dji.h 为准)
      wheel_current[0..3] → Dji_Send_Current(总线, 0x200, current)
        → 8 字节: 4×int16 大端打包
    Dji_All_Stop():
      → 4 通道全 0 电流
```

**电机映射:**

| 索引 | 位置 | CAN | feedback_id | control_id |
|:----:|------|-----|:-----------:|:----------:|
| 0 | 左轮 | FDCAN2 | 0x201 | 0x200 |
| 1 | 右轮 | FDCAN2 | 0x202 | 0x200 |

型号与总减速比按机器取（`machine_config.c` 的 `dji_type` / `dji_gear_ratio`）：本机 M2006 / chuanliantui M3508。
表结构与 DM 完全一致（`motor_cfg_t`，见 `can_bus.h`）；极性同样在 `machine_config.c` 的 `dji_sign[2]`。上表 CAN 列为小机器接法，总线以 `machine->dji_bus` 为准。

**关键函数:**

| 函数 | 作用 |
|------|------|
| Dji_Init() | 注册 CAN 回调 |
| Dji_Read() | 中断存 raw_data |
| Dji_Parse() | 解码 raw→物理量 |
| Dji_Circle_Calculate() | 编码器计圈 |
| Dji_Update_Physical() | 编码→弧度, rpm→rad/s |
| Dji_Torque_To_Current() | Nm→raw current |
| Dji_Send_Wheel_Torque() | 左右轮力矩发送 |
| Dji_All_Stop() | 零电流停机 |
| Dji_Is_Online() | 10ms 超时检测 |

---

## 4. DR16 遥控器（DBUS 协议）

```
DR16 接收机 (UART9 DBUS, 100kbps)
    │  IDLE+DMA 接收, 18 字节/帧
    ▼
dr16.c: DR16_Process()
    帧长校验(18) → DR16_Parse():
      ch0    = (buf[0..1] & 0x07FF) - 1024     摇杆右X
      ch1    = (buf[1..2] & 0x07FF) - 1024     摇杆右Y
      ch2    = (buf[2..4] & 0x07FF) - 1024     摇杆左X
      ch3    = (buf[4..5] & 0x07FF) - 1024     摇杆左Y
      s1     = (buf[5]>>4) & 0x0C >> 2         左拨杆
      s2     = (buf[5]>>4) & 0x03              右拨杆
      mx/my/mz = buf[6..11]                    鼠标轴
      ml/mr  = buf[12..13]                     鼠标键
      key    = buf[14..15]                     键盘
      wheel  = 1024 - buf[16..17]              左侧拨轮
    校验: ch0-3/wheel ∈ [-660,660], s1/s2 ∈ [1,3]
    写入 dr16 (dr16_t)
    │
    │  DR16_Snapshot()
    ▼
task_comm.c: Remote_Control_Update()
    DR16_Process() → DR16_Snapshot() → Rc_Command_Update() → rc_command (全局, 唯一解算点)
    │
    ├─ ch1 → rc_command.vel  (死区 10, 归一化 [-1,1])
    ├─ ch0 → rc_command.yaw  (死区 20, 解算时取负: rc_command.c)
    ├─ wheel → rc_command.len (死区 20)
    ├─ ch3 → rc_command.ang  (死区 20, 当前无消费端)
    ├─ s1 / s2 / online → rc_command
    ├─ rc_enable = online && ((s1==MID && lqr_configured) || (s1==UP && rl.configured))
    │              (task_comm.c; 早期文档记 "s1 != DOWN 即使能", 已按代码更正)
    └─ s1 → input_command.mode (写后无读点, 见 §5)
    (推理路径: RL_Command_From_Rc() → vx/yaw_rate/height 按 RL_CMD_* 范围, 转向再取负;
     关节经机器表 .rl 映射到训练关节)
    │
    │  消费端
    ▼
task_comm.c:
  Robot_Fault_Update():
    !DR16_Online() → FAULT_RC (另有 FAULT_IMU/CAN/MOTOR/ACTION)
  Robot_Fallen_Update():
    |pitch|>1.4 → fallen=1, <1.0 → 回正
  Robot_Enable_Update():
    enable_request = rc_enable && ctrl_fault==0 && !fallen
    使能沿 → Dm_All_Enable; 失能沿 → Dji_All_Stop + Dm_All_Disable
    其后使能期 Dm_Enable_Watchdog / 失能期 Dm_Disable_Watchdog
    (早期文档的 Robot_Control_Output() 已不存在, 安全收口在 Robot_Enable_Update + output_dispatch)

（历史/已移除）task_policy.c 手动遥操（左拨杆上位, ch3→thigh 偏移、wheel→shank 偏移、ch1→轮速）:
  已删除；Manual_Action_Apply / base_action 叠加路径均不存在。

lqr_balance.c (LQR, 左拨杆中位 + 右拨杆中位):
  ch1    → 前后速度, ch0 → 偏航角速度 (yaw_hold/yaw_rate_hold 默认开, 两偏航列默认参与——
           作者确认为有意设计, 2026-09-25), wheel → 腿长目标 (机器区间 ∩ K 表域)

task_policy.c (RL, 左拨杆上位 + 右拨杆中位):
  ch1/ch0/wheel → vx/yaw_rate/height (RL_CMD_* 范围, 转向取负; 起立策略当前 vx/yaw 量程为 0、高度固定, 以 rl_policy.h 为准)
```

**dr16_t 字段:**

| 字段 | 来源 | 范围 | 说明 |
|------|------|:----:|------|
| ch0 | 摇杆右X | ±660 | → rc_command.yaw（解算取负） |
| ch1 | 摇杆右Y | ±660 | → rc_command.vel 前进/后退 |
| ch2 | 摇杆左X | ±660 | (未用) |
| ch3 | 摇杆左Y | ±660 | → rc_command.ang（当前无消费端） |
| wheel | 左侧拨轮 | ±660 | → rc_command.len（LQR 腿长 / RL 高度） |
| s1 | 左拨杆 | 1/2/3 | 挡位/使能前置条件 |
| s2 | 右拨杆 | 1/2/3 | 中位投入当前模式；其他位零力矩 |
| mx/my/mz | 鼠标 | int16 | (未用) |
| ml/mr | 鼠标键 | 0/1 | (未用) |
| key | 键盘 | uint16 | (未用) |
| online | 50ms 超时 | bool | 在线 |

**拨杆语义:**

| 拨杆 | 值 | 作用 |
|:----:|:--:|------|
| s1 DOWN | 2 | 失能 (CTRL_STRATEGY_DISABLE, rc_enable=0) |
| s1 MID | 3 | 选 LQR（需 machine->lqr_configured 才 rc_enable） |
| s1 UP | 1 | 选 RL（需 machine->rl.configured 才 rc_enable） |
| s2 | — | 中位投入当前模式；其他位零力矩 |

**关键函数:**

| 函数 | 作用 |
|------|------|
| DR16_Init() | UART9+DMA 启动 |
| DR16_Process() | 解析一帧写入 dr16 |
| DR16_Online() | 50ms 超时检测 |
| DR16_Deadline() | 死区滤波 |
| DR16_Snapshot() | 返回 dr16 副本 |

---

## 5. 状态聚合层

三个聚合结构体：motor_state / robot_state 由 commTask 写入、其他任务只读；input_command 目前是死状态（见下）。

### motor_state — 电机状态

```
dm_motor_feedback[i] / dji_motor_feedback[i]  (驱动层解码)
    │
    │  Motor_State_Update() @ task_comm.c
    ▼
motor_state_t motor_state:
    dm.pos_rad[i]       ← dm_motor_feedback[i].pos_rad       （原始解码角）
    dm.pos_zero_rad[i]  ← dm_motor_feedback[i].pos_zero_rad  （零点后值）
    dm.vel_rad_s[i]     ← dm_motor_feedback[i].vel_rad_s
    dm.trq_nm[i]        ← dm_motor_feedback[i].trq_nm
    dm.last_rx_tick[i]  ← dm_motor_feedback[i].last_rx_tick
    dm.online[i]        ← Dm_Is_Online(i)
    dji.angle_rad[i]       ← dji_motor_feedback[i].angle_rad
    dji.angle_total_rad[i] ← dji_motor_feedback[i].angle_total_rad
    dji.vel_rad_s[i]       ← dji_motor_feedback[i].vel_rad_s
    dji.current_raw[i]     ← dji_motor_feedback[i].current_raw
    dji.last_rx_tick[i]    ← dji_motor_feedback[i].last_rx_tick
    dji.online[i]          ← Dji_Is_Online(i)
    timestamp_ms           ← HAL_GetTick()
    updated                = 1

消费端:
  task_comm.c:   dm/dji.online + Dm_Has_Fault → FAULT_MOTOR
  task_comm.c:   dm.pos_zero_rad/vel_rad_s → leg.input (髋关节映射，零点已在 dm.c 叠加, +π 在 Leg_State_Update 折算)
  task_actuation.c: dji.vel_rad_s → wheel_vel (LQR 速度估计; RL 轮速, 传入前左右槽对调)
  task_policy.c: dm/dji.online → RL_Motors_Online; dji.vel_rad_s → RL_Joint_Map 观测轮速 (左右槽交叉)
  (早期文档记 "task_policy.c: dm.pos_zero_rad/vel → obs.joint_pos/vel" 已不准确:
   obs 关节角/速度来自 leg_solver 输出经 RL_Joint_Map 映射, 见 §7)
```

### input_command — 遥控指令（当前为死状态：只写不读）

```
写点:
  task_policy.c:126-128  RL_Command_From_Rc() → vx_cmd / yaw_cmd / height_cmd (推理路径)
  task_comm.c:77         Remote_Control_Update() → mode = s1
  字段 thigh_delta_cmd[2] / shin_delta[2] 无任何读写 (早期手动遥操遗留)

读点: 无（全机无消费端）
  早期文档记的 "task_policy.c → obs.command[6..8]"、"Manual_Action_Apply (手动测试)"
  均已不存在：obs 的 command[3] 直接取 RL_Command_From_Rc 的局部数组，不经过 input_command。
  结构体仍留在 robot_control.h（未删）；是否保留待作者确认。
```

### robot_state — 机器人状态

```
DR16 / IMU / 故障状态
    │
    │  task_comm.c 各子函数写入
    ▼
robot_state_t robot_state:
    rc_enable    ← Remote_Control_Update()
                   DR16_Online() && ((s1==MID && machine->lqr_configured)
                                   || (s1==UP  && machine->rl.configured))
                   (早期文档记 "s1 != DOWN && online", 已按 task_comm.c:74-76 更正)
    fallen       ← Robot_Fallen_Update()
                   |pitch|>1.4 → 1, <1.0 → 0
    motor_enabled← Robot_Enable_Update()
                   rc_enable && ctrl_fault==0 && !fallen

消费端:
  task_actuation.c:
    motor_enabled && imu 在线 && 两腿有效  → LQR 投入前提 (lqr_engage_update)
    rc_enable && motor_enabled && 两腿有效 && action_state.rl_ready → RL 力矩计算
    未投入 / torque.valid=0 / 总开关关    → Dm_Send_Zero + Dji_All_Stop (output_dispatch)
  task_comm.c:
    motor_enabled → 使能沿 Dm_All_Enable / 失能沿 Dji_All_Stop + Dm_All_Disable, 以及使能看门狗切换
  (早期文档记的 "task_policy.c: motor_enabled → 锁存 base_action / Action_State_Clear" 已不存在;
   Action_State_Clear 现只在 Robot_Control_Init / RL_Control_Select_Model 调用)
```

---

## 6. 五连杆腿部解算（leg_solver）

```
dm.pos_zero_rad[0..3] (DM 电机编码器，零点后值)
    │
    │  Leg_State_Update() @ task_comm.c
    ▼
leg_l / leg_r (leg_state_t):
    input.hip_f   = dm.pos_zero_rad[F] + π                   前髋角 (零点已在 dm.c 叠加)
    input.hip_b   = dm.pos_zero_rad[B]                       后髋角 (零点已在 dm.c 叠加)
    input.d_hip_f = dm.vel_rad_s[F]                           前髋速度
    input.d_hip_b = dm.vel_rad_s[B]                           后髋速度
    (+π 折算在 task_comm.c::Leg_State_Update 输入端完成, leg_solver 不再加偏置;
      符号项 +LEG_PI 属物理量, 以代码为准)
    │
    │  Leg_Solve() @ task_comm.c
    ▼
三层求解 (三角函数为 CMSIS-DSP 查表 arm_sin/cos_f32, 同 Leg2; LEG_TRIG_LIBM=1 退回 libm):

① Leg_Solve_Geometry — 闭链几何
    qf = hip_f, qb = hip_b      (不镜像: 极性统一在 dm.c 反馈层)
    A = (lu·cos qf, lu·sin qf)    前杆端点
    B = (lu·cos qb, lu·sin qb)    后杆端点
    求 P 点 (两圆交点) → phi_a, phi_b
    输出:
      thigh_angle          = Leg_Wrap(qf)                        大腿角(前髋上连杆; 即 wrap(hip_f))
      virtual_leg_length   = |OP|                           虚拟腿长
      virtual_leg_angle    = Leg_Wrap(π/2 - atan2(y_p,x_p) + offset_phi0)  虚拟腿摆角
      virtual_shank_angle  = Leg_Wrap(phi_a - qf - π/2)   虚拟小腿角

② Leg_Solve_Velocity — 速度雅可比
    leg_jac[2][2]       → d_virtual_leg_length, d_virtual_leg_angle
    vshank_jac[2]       → d_virtual_shank_angle
    point_jac[2][2]     → 计算后无人消费 (死输出)

③ Leg_Solve_Force_Map — 力矩映射
    force_map = leg_jac 转置
    Leg_Force_Map_Forward(force, torque) → tau_f, tau_b
    │
    │  消费端
    ▼
task_policy.c (RL_Joint_Map):
  thigh_angle / virtual_shank_angle  → 观测 joint_pos (经机器表 .rl 的 sign/zero 映射, 见 §7)
  d_hip_f / d_virtual_shank_angle    → 观测 joint_vel
  (历史: thigh_angle 曾作手动遥操 base_action 基准, 已随手动遥操删除)
rl_torque.c:
  virtual_shank_angle    → q[1,4] (RL 关节位置)
  d_virtual_shank_angle  → qd[1,4] (RL 关节速度)
  thigh_angle            → q[0,3] (RL 关节位置)
  vshank_jac             → 力矩分解 tau_f/tau_b
  输出: torque_output_t {dm[4], dji[2]}
lqr_balance.c / leg_balance.c:
  virtual_leg_length, d_virtual_leg_length → 状态/速度估计 + 腿长 PID
  virtual_leg_angle, d_virtual_leg_angle   → 腿摆角世界系
  force_map → Leg_Force_Map_Forward → 前/后髋力矩
task_comm.c:
  virtual_leg_length 等 → VOFA 调试
```

**配置参数（`machine_config.c`）：**

`lu`、`lg`、`dm_zero`、`offset_phi0` 和机器腿长区间均已进入机器表；`robot_control.c` 初始化时只读取当前 `machine`（`lu`、`lg`、`dm_zero`、`offset_phi0` 和机器腿长区间均已进入机器表；`robot_control.c` 初始化时只读取当前 `machine`，几何层不做镜像——反馈极性统一在 `dm.c` 驱动层，`mirror` 字段已删，变更 102）。下表杆长为小机器值（历史记录），大机器见 `machine_config.c`。

| 参数 | 左腿 | 右腿 | 说明 |
|------|:----:|:----:|------|
| lu | 0.13087 | 0.13087 | 上杆长 (m) |
| lg | 0.15240 | 0.15240 | 下杆长 (m) |
| dm_zero | 见 machine_config.c | 见 machine_config.c | 电机零点 (rad)，dm.c 解码时叠加 |
| offset_phi0 | 见 machine_config.c | 见 machine_config.c | 虚拟腿摆角零位偏置 (rad)，只加在 `virtual_leg_angle` 上，不影响腿长、大腿角、小腿角、雅可比与角速度 |
**输出字段:**

| 字段 | 含义 | 用途 |
|------|------|------|
| thigh_angle | 大腿角 (qf = hip_f, hip_f 含 +π) | RL obs（RL_Joint_Map）+ rl_torque 腿关节 PD |
| virtual_leg_length | 虚拟腿长 \|OP\| | LQR 状态/速度估计 + 腿长 PID（leg_balance） |
| virtual_leg_angle | 虚拟腿摆角 (相对竖直) | LQR |
| virtual_shank_angle | 虚拟小腿角 (小腿相对大腿) | RL obs（RL_Joint_Map）+ rl_torque 腿关节 PD |
| d_virtual_leg_length | 虚拟腿长速度 | LQR 速度运动学 |
| d_virtual_leg_angle | 虚拟腿摆角速度 | LQR |
| d_virtual_shank_angle | 虚拟小腿角速度 | RL obs + rl_torque D 项 |
| vshank_jac[2] | 虚拟小腿雅可比（[0] 后髋, [1] 前髋） | rl_torque 力矩分解 |
| leg_jac[2][2] | 腿雅可比 | 解算内部（速度与力矩映射） |
| force_map[2][2] | 力矩映射矩阵 | Leg_Force_Map_Forward → 髋力矩（LQR 路径） |
| force_det / force_valid | 行列式 / 有效位 | 解算内部，force_valid 供 Force_Map_Forward 门控 |
| point_jac[2][2] | P 点雅可比 | **无人消费**（计算后死输出） |

**VOFA 观测：**32 路 JustFloat；当前布局与打包顺序以 `task_comm.c::Robot_Control_Send_Vofa()` 上方注释为准（"当前为 IMU 极性测试帧"等历史测试阶段描述已过时）。

---

## 7. RL 观测层（policyTask）

```
rc_command (遥控) → RL_Command_From_Rc() @ task_policy.c
    command[0] = vel × RL_CMD_VX_MAX
    command[1] = -yaw × RL_CMD_YAW_MAX        (转向取负, 同 LQR)
    command[2] = 拨轮 → [RL_CMD_HEIGHT_MIN..MAX]
    (起立策略当前 vx/yaw 量程为 0、高度固定, 以 rl_policy.h 的 RL_CMD_* 为准)
imu_state (IMU 姿态)
    gyro_rad_s[3]   → obs[0-2] × gyro_scale
    quat[4]         → obs[3-5]  投影重力 (RL_Observation_Project_Gravity)
关节来源 — RL_Joint_Map() @ task_policy.c:
    训练关节 = sign × Angle_Wrap_180(固件角 − zero)
    zero/sign 见 machine_config.c 的 .rl (sign 序 [左大腿 左小腿 左轮 右大腿 右小腿 右轮], zero 只有 4 个腿关节)
    joint_pos[0..3] = [左大腿(thigh_angle), 左小腿(virtual_shank_angle),
                       右大腿(thigh_angle), 右小腿(virtual_shank_angle)] 映射后 → obs[9-12] − dof_pos
    joint_vel[0..5] = sign[i] × [左大腿速度(d_hip_f), 左小腿速度, 左轮, 右大腿速度, 右小腿速度, 右轮]
                       → obs[13-18] × joint_vel_scale
    轮槽左右交叉: 左轮槽取 motor_state.dji.vel_rad_s[WHEEL_RGT], 右轮槽取 [WHEEL_LFT]
      (台架反馈核对结论, task_policy.c:58 注释; 待作者台架确认)
obs[19-24] = last_action[6] (上步训练空间动作, RL_Observation_Set_Last_Action 写入)
    │
    │  RL_Observation_Build() @ policyTask (task_policy.c)
    ▼
rl_observation_state_t:
    obs[25]                      当前观测帧 (含 last_action 段)
    history[125]                 5帧历史 (5×25, 循环左移; 首帧填满 5 帧同值后 history_ready=1;
                                 仅在已投入的 RL_Control_Update_Observation 推历史, 预览路径不推)
    last_action[6]               上步动作缓存
    │
    │  CubeAI 推理 (rl_policy.c::RL_Policy_Run)
    ▼
网络输入 (两路张量): obs[25] + history[125]
    (早期文档 "obs+history+last_action=156 维" 有误: last_action 不是独立输入, 已含在 obs[19-24])
网络输出 (两路张量): action[6] + latent[3]
    │
    │  RL_Torque_Compute() → rl_torque.c
    ▼
力矩分解 (见 rl_torque 链路)
```

**观测维度**（缩放与默认角以 `rl_observation.h` 的 `RL_OBS_*` 为准，整体裁剪 `RL_OBS_CLIP`）:

| 索引 | 字段 | 来源 | 缩放 |
|:----:|------|------|------|
| 0-2 | gyro | imu_state.gyro_rad_s | gyro_scale |
| 3-5 | gravity | RL_Observation_Project_Gravity(quat) | — |
| 6-8 | command | RL_Command_From_Rc 的 command[0..2] | command_scale |
| 9 | l_thigh | RL_Joint_Map(joint_pos[0]) − dof_pos | — |
| 10 | l_shank | RL_Joint_Map(joint_pos[1]) − dof_pos | — |
| 11 | r_thigh | RL_Joint_Map(joint_pos[2]) − dof_pos | — |
| 12 | r_shank | RL_Joint_Map(joint_pos[3]) − dof_pos | — |
| 13 | l_thigh_vel | leg_l.input.d_hip_f × sign | joint_vel_scale |
| 14 | l_shank_vel | leg_l.output.d_virtual_shank_angle × sign | joint_vel_scale |
| 15 | l_wheel_vel | dji.vel_rad_s[WHEEL_RGT]（交叉）× sign | joint_vel_scale |
| 16 | r_thigh_vel | leg_r.input.d_hip_f × sign | joint_vel_scale |
| 17 | r_shank_vel | leg_r.output.d_virtual_shank_angle × sign | joint_vel_scale |
| 18 | r_wheel_vel | dji.vel_rad_s[WHEEL_LFT]（交叉）× sign | joint_vel_scale |
| 19-24 | last_action | 上步动作（训练空间） | — |

---

## 全局数据流

```
┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐
│  HI229   │  │  DM ×4   │  │  DJI ×2  │  │   DR16   │
│  UART7   │  │ FDCAN1/3 │  │  FDCAN2  │  │  UART9   │
└────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘
     │             │             │             │
     ▼             ▼             ▼             ▼
 imu_state    motor_state_t (聚合)        input_command_t (只写不读, 见 §5)
              dm.pos_rad/vel/trq          robot_state_t
              dji.vel_rad_s/online
                   │
           ┌───────┴───────┐
           ▼               ▼
     leg_l/leg_r     ┌───────────┐
     Leg_Solve()     │  RL 推理  │
     五连杆解算      │ CubeAI    │
           │         └─────┬─────┘
           │               │
           ▼               ▼
     ┌───────────┐  ┌────────────┐
     │   LQR     │  │ rl_torque  │
     │ 平衡控制  │  │ 力矩分解   │
     └─────┬─────┘  └─────┬──────┘
           │               │
           ▼               ▼
      ┌─────────────────────────┐
      │     task_actuation      │
      │  DM/DJI 力矩下发        │
      └─────────────────────────┘
```

---

## 8. LQR 平衡链路（task_actuation 内，@1kHz）

左拨杆中位 = 选 LQR（需 `machine->lqr_configured`），右拨杆中位 = 投入出力（其他位零力矩）。全部计算在 `actuationTask` 里完成，只读其它任务的共享状态。

```
imu_state (pitch/roll/yaw/gyro)    leg_l / leg_r (Leg_Solve 输出)
motor_state.dji.vel_rad_s          rc_command (commTask 已解算)
        │                                  │
        ▼                                  │
  LQR_State_Update()   每拍必算(限 lqr_configured 机器), 不看挡位   │
  x[10] 状态组装 + 速度运动学 + 位移积分     │
  → lqr_state.valid                        │
        │            ── 以下只在左中+右中且投入后 ──
        │                                  ▼
        │                          LQR_Target_Update()
        │                          target[10] + 腿长目标
        └──────────────┬───────────────────┘
                       ▼
              LQR_Control_Update()   (valid 才算)
              腿长变化>0.5mm → LQR_K_WBR(h_l,h_r) 求 40 个增益
              u[i] = Σ K[i][j]·(target[j] − x[j])   → [T_wl,T_wr,T_bl,T_br]
                       │
                       ▼
              Leg_Balance_Compute()
              腿长PID + 横滚PID → 足端力 F；LQR 髋扭矩取反得到 Tp
              Leg_Force_Map_Forward(&leg, F, Tp) → 前/后髋力矩
              lqr_debug 通道门 + 限幅 → torque_output_t
                       │
                       ▼
        Dm_Send_Torque() + Dji_Send_Wheel_Torque()
```

**分层**（作者 2026-09-21 定）：解算与估计每拍都算（同 RL 观测），控制律只在对应挡位算，出力只在使能 + 解算有效 + 投入时发；投入瞬间清位移积分并锁腿长/朝向/速度目标（`LQR_Enable_Latch`），低通滤波器常跑不复位。

**状态索引**：`[s, ds, φ, dφ, θ_ll, dθ_ll, θ_lr, dθ_lr, θ_b, dθ_b]`，与数学建模一致；φ / dφ / s 三列由 `lqr_debug.yaw_hold / yaw_rate_hold / pos_hold` 开关，三者 `LQR_Init` 默认均为开（偏航两列默认参与；作者确认为有意设计，2026-09-25）。
**俯仰角**：`pitch = euler[PITCH]`，同一 pitch 进 θ_b 与两腿世界系摆角；当前无额外 `pitch_off`。
**位移积分**：速度目标非 0 清零撤防；目标回 0 且 |ds| < `lqr_debug.pos_arm_vel` 才启动（默认 0 = 立即），`lqr_state.pos_armed` 可看。
**腿摆角世界系**：`−virtual_leg_angle + pitch`；**角速度**同理 `−d_virtual_leg_angle + omg_pitch`。
（本工程解算腿角前摆为正，数学模型 θ_ll 前摆为负，**整体取反后再加 pitch**；髋扭矩同步取反，详见 [LQR_PLAN.md](LQR_PLAN.md) §3.1）
**速度**：`ω_轮·machine->wheel_r + L·dθ·cosθ + dL·sinθ`，`ω_轮 = ω_电机 + vel_leg_comp_sign·dθ_leg + pitch_comp_sign·ω_pitch`（两个符号都是 `lqr_debug` 台架 A/B 字段，默认 −1 / −1 保持现状）；后接低通（α=0.3）或卡尔曼，`vel_src` 选（`LQR_Init` 默认卡尔曼），前向加速度乘 `acc_fwd_sign`。
**腿长限制**：机器表工作区间与 K 表拟合域 0.13~0.23 m 的交集（`LQR_Len_Range`，只夹拨轮目标）；文中"小机器为 0.13~0.21 m"为早期记录，与当前 `machine_config.c` 小机器腿长区间不一致——以代码为准、待作者台架复核，数值本文不改。
**调试门**：`lqr_debug` 可分别关闭轮、髋、腿长 PID 的最终输出并调整限幅；关闭通道时 PID 仍持续计算。
**符号责任**：反馈极性按 `dm_sign/dji_sign` 的 `.fb` 在驱动解码时统一到机体坐标；输出极性按 `.out` 在驱动下发时统一处理（`dm.c` / `dji.c`），调用方不要取反。详见 [LQR_PLAN.md](LQR_PLAN.md)。

---

## 2026-09-27：整机诊断遥测 (S2R1, gap 测试)

`imcalib/Telemetry/` 是**自包含模块**：只读既有全局量，不改任何现有结构体；任务层只有一个挂点。协议布局见 [sim2real_serial_protocol.md](sim2real_serial_protocol.md)，接线/采集/验收见 [sim2real_serial_capture.md](sim2real_serial_capture.md)。

```
imuTask    → imu_state / hi229_data          ─┐
policyTask → rl_control.observation/policy    │  只读
             action_state / input_command     ├─► s2r_source.c  (采样 + 变化检测)
actuationTask → rl_control.torque_state       │        │
                rl_output_dm/wheel_cmd_nm     ─┘        ▼
commTask → S2R_Pump() ─► s2r_telemetry.c (成帧/CRC/队列) ─► VOFA_UART DMA 1152000 8N1
             └ 返回 0 时才发旧 32 路 VOFA (s2r_diagnostic_requested=0 且失能、无会话)
```

- **采样层 `s2r_source.c`**：`policy.run_ok+run_fail` 变化 → 一次推理；`hi229_data.ts`/`last_rx_tick` 变化 → IMU 新帧；`dm/dji_motor_feedback[].last_rx_tick` 变化 → 电机新帧。序号（`state/imu/policy/control`）与时间戳全部由模块自维护。
- **成帧层 `s2r_telemetry.c`**：POLICY 每次推理、CONTROL 100 Hz（10 ms 抽样）、IMU 最快 50 Hz、HISTORY 2 Hz、HEALTH 10 Hz、META 分片限速 300 ms；队列 24 槽，控制侧不等串口。
- **链路口径**（本分支未插桩处一律写 NaN/0，不伪造）：`action_raw`、`tau_virtual_raw_fw`、`gas_tau_shank_fw`、`tau_motor_unclipped`、`current_motor` = NaN；`motor_send_ok_mask`/`can_enqueue_us` = 0；`*_rx_us` 是 commTask 首次见到新帧的时刻（1 ms 量化）；`motor_clamp_or_mask` 由请求饱和推导。META 的 `unavailable` / `derived` 两栏即这份清单。
- **回退**：调试器写 `s2r_diagnostic_requested=0`（或把 `s2r_telemetry.h` 的 `S2R_DIAGNOSTIC_DEFAULT` 改成 0 重编译），只有失能、无会话、UART 就绪时才切回旧 VOFA；切回后 ch0/ch1/ch2 是状态位，ch3~31 是 RL 布局（下发力矩/实测力矩/观测/上次动作/轮电流），ch31 = `ctrl_fault`。`s2r_init_error` bit0 = 启动 RNG/boot_id 失败，bit1 = META 缓冲溢出。
- **构建**：两份工程都已登记 `imcalib/Telemetry`；`.sct` 把 `.s2r_dma` 固定在 AXI SRAM（当前 `dma_buffer` @ `0x24001100`）。改源码后先 `python tools/s2r_build_info.py` 再编译。
