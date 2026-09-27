# DBUS 遥控器解析

> 配合文件：[`dr16.h`](../imcalib/user-lib/dr16.h)、[`dr16.c`](../imcalib/user-lib/dr16.c)

---

## 1. 协议

DR16 遥控器 DBUS 协议，18 字节定长帧，100kbps（标准 DBUS）。

---

## 2. 数据结构

```c
typedef struct {
    int16_t  ch0;          /* 遥杆右X */
    int16_t  ch1;          /* 遥杆右Y */
    int16_t  ch2;          /* 遥杆左X */
    int16_t  ch3;          /* 遥杆左Y */
    int16_t  wheel;        /* 左侧拨轮 */
    uint8_t  s1;           /* 左拨杆 */
    uint8_t  s2;           /* 右拨杆 */
    int16_t  mx;           /* 鼠标X */
    int16_t  my;           /* 鼠标Y */
    int16_t  mz;           /* 鼠标Z */
    uint8_t  ml;           /* 鼠标左键 */
    uint8_t  mr;           /* 鼠标右键 */
    uint16_t key;          /* 键盘按键 */
    bool     online;       /* 在线标志 */
    uint32_t last_rx_tick; /* 最后接收时间戳 */
} dr16_t;
```

遥杆范围：-660 ~ +660，中值 0
拨轮范围：-660 ~ +660，中值 0

---

## 3. 使用方式

```c
#include "dr16.h"

/* 任务循环中 */
DR16_Process();

/* 读取数据 */
if (dr16.s1 == DR16_SW_DOWN) { ... }
int16_t ch0 = dr16.ch0;

/* 在线检测 */
if (DR16_Online()) { ... }

/* 快照 (避免跨帧混值) */
dr16_t remote = DR16_Snapshot();

/* 死区滤波 */
int16_t value = DR16_Deadline(raw, 20);
```

---

## 4. 内部流程

```
dbus_rx.flag == 1 ?
    ↓ yes
帧长校验 (>=18，尾部多余字节忽略)
    ↓
DR16_Parse: 解析 18 字节 → dr16_t
    ↓
范围校验: ch0-3/wheel ∈ [-660,660], s1/s2 ∈ [1,3]
    ↓
写入 dr16 结构体 + 更新 last_rx_tick
```

---

## 5. 通道映射

| 字段 | 解析位置 | 范围 | 用途 |
|------|:--------:|:----:|------|
| ch0 | buf[0..1] | ±660 | yaw 指令 → `rc_command.yaw`（LQR 偏航 / RL `command[1]`） |
| ch1 | buf[1..2] | ±660 | 前进速度 → `rc_command.vel`（LQR 速度 / RL `command[0]`） |
| ch2 | buf[2..4] | ±660 | **无消费**（仅解析 + 范围校验） |
| ch3 | buf[4..5] | ±660 | → `rc_command.ang`（**当前无消费**） |
| wheel | buf[16..17] | ±660 | 腿长 / 高度 → `rc_command.len`（LQR 腿长积分 / RL `command[2]`） |
| s1 | (buf[5]>>6)&3 | 1/2/3 | 挡位选择 + 使能（§6） |
| s2 | (buf[5]>>4)&3 | 1/2/3 | 投入出力（§6） |
| mx/my/mz | buf[6..11] | int16 | (未用) |
| ml/mr | buf[12..13] | 0/1 | (未用) |
| key | buf[14..15] | uint16 | (未用) |

> 各通道取位与符号细节以 `dr16.c:28-45` 为准。

---

## 6. 拨杆语义

| 拨杆 | 值 | 宏 | 作用 |
|:----:|:--:|:--:|------|
| s1 DOWN | 2 | `DR16_SW_LEFT_DOWN` | 失能 → `CTRL_STRATEGY_DISABLE`（`task_actuation.c:84-85`） |
| s1 MID | 3 | `DR16_SW_LEFT_MID` | 选 LQR（需 `machine->lqr_configured`，`task_actuation.c:80-81`）；计入 `rc_enable`（`strategy_rc_enable()`，`task_actuation.c`） |
| s1 UP | 1 | `DR16_SW_LEFT_UP` | 选 RL（需 `machine->rl.configured`，`task_actuation.c:82-83`）；计入 `rc_enable`（`strategy_rc_enable()`，`task_actuation.c`）；RL 挡（`ctrl_strategy == CTRL_STRATEGY_RL`）动作过期或已投入但动作持续不可用（`rl_ready` 持续 0）≥100ms → `FAULT_ACTION`（`Robot_Fault_Update()`，2026-09-25） |
| s2 MID | 3 | `DR16_SW_RIGHT_MID` | 投入出力（LQR：`task_actuation.c:180`；RL：`task_actuation.c:193`） |
| s2 UP/DOWN | 1/2 | `DR16_SW_RIGHT_UP/DOWN` | 已选模式但零力矩（不投入） |

> 离线 / 使能判定为 0（`strategy_rc_enable()`）→ `CTRL_STRATEGY_DISABLE`（`strategy_from_remote()` 先判 `robot_state.rc_enable`，`task_actuation.c`）；LQR 投入还依赖电机使能 + IMU 在线 + 两腿有效（`task_actuation.c:94-96`）。

---

## 7. 遥控指令映射

摇杆只在一处解算：`commTask` → `Remote_Control_Update()` (`task_comm.c:67`) → `Rc_Command_Update()` (`user-lib/rc_command.c`) 填全局 `rc_command`，各链路只读。

**`rc_command_t`** (死区 + 限幅 ±660 → [-1, 1], `rc_command.c:4-18`):

| 字段 | 通道 | 死区 | 含义 |
|------|------|:----:|------|
| `vel` | ch1 右摇杆 Y | 10 | 前进 |
| `yaw` | ch0 右摇杆 X | 20 | 转向 |
| `len` | wheel 拨轮 | 20 | 腿长 / 高度 |
| `ang` | ch3 左摇杆 Y | 20 | 摆角 / 大腿（**当前无消费**） |
| `s1` / `s2` / `online` | 拨杆 / 在线 | — | 仲裁用 |

死区常量 `RC_DEADBAND_VEL/YAW/LEN/ANG` 见 `rc_command.h:7-10`。

**消费端**（当前）:

| 链路 | 位置 | 用法 |
|------|------|------|
| LQR | `lqr_balance.c:141` `LQR_Target_Update()`（`task_actuation.c:124` 调用） | `vel × LQR_RC_VEL_MAX` 速度目标（`lqr_balance.h:41`，现值 1.2 m/s，可再经 `lqr_debug.vel_ramp` 斜坡）；`−yaw × LQR_RC_YAW_MAX` 偏航角速度目标 + 摇杆有输入时锁当前朝向（`lqr_balance.h:42`，现值 5 rad/s）；`len × LQR_RC_LEN_RATE × dt` 积分成腿长目标、夹在腿长工作区间（`lqr_balance.h:43`，现值 0.3 m/s）；`ang` 无消费 |
| RL 推理指令 | `task_policy.c:120` `RL_Command_From_Rc()`（`RL_Infer_Body()` 每拍调用，`task_policy.c:154`） | `command[0] ← vel × RL_CMD_VX_MAX`、`command[1] ← −yaw × RL_CMD_YAW_MAX`（右推为负，同 LQR）、`command[2] ← len` 线性到 `[RL_CMD_HEIGHT_MIN, RL_CMD_HEIGHT_MAX]`；宏在 `rl_policy.h:10-13`，数值以代码为准、待训练侧（代码注释标注为起立策略训练域，拨轮暂无效） |
| RL 观测 | `task_policy.c:71/92` → `rl_observation.c::RL_Observation_Build()` | `command[3]` 进观测，再乘 `RL_OBS_CMD_*_SCALE`（`rl_observation.h:13-15`） |
| 挡位 / 投入 | `task_actuation.c:72` `strategy_from_remote()` + `:176-204` | `s1`：中 = LQR、上 = RL、下 / 离线 = 失能 → `ctrl_strategy`；`s2` 中位 = 投入出力（LQR `:180`、RL `:193`），其他位 = 已选模式但零力矩 |
| 使能 / 故障 | `strategy_rc_enable()`（`task_actuation.c`） | online + 左中且 `lqr_configured` / 左上且 `rl.configured` → `robot_state.rc_enable`（`task_comm.c`）；RL 挡（`ctrl_strategy == CTRL_STRATEGY_RL`）动作过期或已投入但动作持续不可用（`rl_ready` 持续 0）≥100ms → `FAULT_ACTION`（`Robot_Fault_Update()`，2026-09-25 收敛为唯一函数） |

**历史 / 已移除**（旧路径，当前代码已无）:

| 链路 | 位置 | 用法 |
|------|------|------|
| ~~RL 观测（旧）~~ | ~~`task_policy.c::Remote_Command_Apply()`~~ | `vx_cmd ← vel`、`yaw_cmd ← yaw`、`height_cmd ← len`，各 × `REMOTE_COMMAND_SCALE`（旧手动遥操路径，git `3a943aa` 及以前） |
| ~~RL 手动偏移（旧）~~ | 同上 | `thigh ← ang × 4`、`shank ← len × 4`、`wheel ← vel × 4` 叠加 base action（`MANUAL_ACTION_SCALE`，git `3a943aa`） |
| ~~LQR 手动腿测（旧）~~ | — | 曾记"手动时 `ang × 0.5 rad` 摆角"；当前 `ang` 仅在 `rc_command.c` 赋值，无任何消费（与早期文档记录不同，待作者确认） |

---

## 8. 关键常量

| 常量 | 值 | 说明 |
|------|:--:|------|
| `DR16_FRAME_LEN` | 18 | 帧长度（`dr16.h:7`） |
| `DR16_OFFLINE_MS` | 50 | 超时 (ms)（`dr16.h:8`） |
| `DR16_CH_LIMIT` | 660 | 通道最大绝对值（`dr16.h:9`） |
| `DR16_SW_UP` | 1 | 拨杆上位（`dr16.h:11`） |
| `DR16_SW_MID` | 3 | 拨杆中位（`dr16.h:12`） |
| `DR16_SW_DOWN` | 2 | 拨杆下位（`dr16.h:13`） |
| `REMOTE_COMMAND_SCALE` | 3.0f | **历史 / 已移除**：RL 指令缩放（旧 `rl_policy.h`，git `3a943aa`）；当前代码无此宏，RL 缩放改用 `RL_CMD_*`（`rl_policy.h:10-13`） |

---

## 9. 关键函数

| 函数 | 作用 |
|------|------|
| `DR16_Init()` | UART9+DMA 启动（`dr16.c:65`，`main.c:157` 调用） |
| `DR16_Process()` | 解析一帧写入 dr16（`dr16.c:71`，`task_comm.c:71` 每拍调用） |
| `DR16_Online()` | 50ms 超时检测，超时清零 `dr16`（`dr16.c:84`；`task_comm.c:106/195`） |
| `DR16_Deadline()` | 死区滤波（`dr16.c:98`；唯一调用点 `rc_command.c:8` 归一化死区） |
| `DR16_Snapshot()` | 返回 dr16 副本（`dr16.c:106`；唯一使用点 `task_comm.c:72`）。LQR/RL/仲裁不得自行调用；注释称"正在接收则跳过"，当前实现两分支均直接返回（与注释意图不同，待作者确认） |
