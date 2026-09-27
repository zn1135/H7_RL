# chuanliantui 轮与闭链腿系统辨识交接

> 大机器测试专用，`SYSID_ENABLE=1` 时生效。

> 给下位机：采原始数据；给算法：在 MuJoCo 回放同一力矩输入，拟合 sim2real。
> 轮：DJI M3508 + C620 + 自制减速箱。腿：四台 DM8009P、五连杆闭链。

## 1. 只做两件事

1. **试验 A：单轮**。左右轮分别架空，记录 C620 原始命令和反馈；用 M3508 P19 理论曲线按本机传动比生成初始速度—力矩包络，再用真机数据拟合摩擦、阻尼和延迟。
2. **试验 B：完整闭链腿**。四台 DM8009P 全部使用**力矩模式**。记录真实电机力矩，以及左右腿的腿长、腿倾角；MuJoCo 也用力矩驱动并比较这四个任务坐标。

两包绝不混写。训练串联代理的 `lf1/rf1` 不是实物独立电机，不能记录为真实电机，也不能直接下发。

## 2. 安全门槛

- 机器人刚性架空、轮离地、急停当天验证、运动平面无人。
- `manifest.yaml` 必须填写并签字：四个 DM8009P 的力矩/速度/位置/温度限制，及轮端允许电流。未填写不得执行。
- C620 轮端测试上限为 **±15 A = ±12288 raw**；这不是 DM8009P 的许可，也不是轮端 N·m。
- 过流、过温、通信异常、限位、异常振动或急停：立刻置零；保留 CSV，标记该 run 无效。

## 3. 试验 A：单轮

每次仅测 `lfwheel` 或 `rfwheel`，另一轮和四个腿电机均为零命令。

```text
data/sysid/chuanliantui/<date>/wheel-<motor>-<run_id>/
  c620_command_raw.csv
  c620_feedback_raw.csv
  manifest.yaml
  README.md
  checksum.sha256
```

### 原始 CSV

`c620_feedback_raw.csv`：每收到一帧 C620 反馈写一行。

```text
run_id,test_id,phase,t_fb_can_rx_ns,can_id,motor_name,ecd_raw,speed_rpm,torque_current_raw,temperature_c
```

| 字段 | 含义 |
| --- | --- |
| `run_id,test_id,phase` | 本次单轮执行、用例、当前阶段；一个 run 中只变 `phase`。 |
| `t_fb_can_rx_ns` | 主控收到此反馈帧的单调时钟纳秒。 |
| `can_id,motor_name` | C620 CAN ID 及被测轮名。 |
| `ecd_raw,speed_rpm` | M3508 电机侧编码器码和电机转速；换算轮端需使用实际总传动比。 |
| `torque_current_raw,temperature_c` | C620 反馈的原始力矩电流码及温度。原始电流码不能直接写成轮端 N·m。 |

`c620_command_raw.csv`：每成功发送一帧 C620 命令写一行。

```text
run_id,test_id,phase,t_cmd_can_tx_ns,sequence_id,can_id,cmd_test_wheel_raw
```

`cmd_test_wheel_raw = round(I_A × 819.2)`。`t_cmd_can_tx_ns` 是 CAN 发送完成时刻，不是 C620 内部 FOC 生效时刻；C620 不提供后者。

### 用例

| 用例 | 命令与时长 |
| --- | --- |
| `baseline_sign` | `0 A` 5 s → `+0.5 A` 1 s → `0 A` 2 s → `-0.5 A` 1 s → `0 A` 5 s。 |
| `stiction` | `0,+0.25,+0.5,+0.75,+1,+1.25,+1.5,+2,+2.5,+3,+4,+5,+6,+8,+10,+12,+15 A`，每级 1 s；负向单独 run。首次持续转动后回零。 |
| `plateau` | `±1,±1.5,±2,±2.5,±3,±4,±6,±8,±10,±12,±15 A`；每个电流单独 run：零 2 s → 平台 5 s → 零 2 s。 |
| `step` | `±1,±2,±5,±10,±15 A`；每个幅值单独 run：零 2 s → 阶跃 0.5 s → 零 3 s，重复三次。 |
| `holdout_step` | 重做一个 `step`，算法侧不得用来拟合。 |

### 初始轮包络

华南虎历史 CSV 的每行是 M3508 P19 **理论电机轴**工作点，不是真机 CAN/测功数据。只使用电机轴列，按 chuanliantui 的参数映射：

```text
omega_wheel = rpm_motor * 2*pi / 60 / G_total
tau_wheel   = tau_motor_theory * G_total * eta_total
```

`G_total` 必须是“电机轴到轮输出轴”的实际总传动比，含 P19 内置减速与自制减速箱；`eta_total` 必须是保守总效率。不得抄华南虎的 `268/17`、`5.5 N·m` 或 `eta=1`。

算法交付：`wheel_torque_speed_envelope_theory.csv`、`wheel_current_speed_envelope.csv`。前者仅是仿真初值；真机 `stiction/plateau/step` 用于校正摩擦、延迟、力矩倍率。若有测功机，才额外交付 `wheel_torque_speed_envelope_measured.csv`。

## 4. 试验 B：力矩输入，腿长/腿倾角输出

本试验不使用位置 `reference.csv`，不使用位置 PD 纠正轨迹。进入记录阶段后四台 DM8009P 均为 `torque/Nm`。

```text
data/sysid/chuanliantui/<date>/joint-torque-<run_id>/
  joint_torque_snapshot.csv
  torque_program.yaml
  kinematics_manifest.yaml
  manifest.yaml
  README.md
  checksum.sha256
```

### `joint_torque_snapshot.csv`

每成功发送一次四路腿力矩命令写一行；发送频率不低于 200 Hz，推荐 500 Hz。电机名固定使用 URDF 中的真实主动轴名 `lf0`、`lf00`、`rf0`、`rf00`。

```text
t_cmd_can_tx_ns,
tau_lf0_Nm,tau_lf00_Nm,leg_length_left_m,leg_pitch_left_rad,
tau_rf0_Nm,tau_rf00_Nm,leg_length_right_m,leg_pitch_right_rad
```

| 字段 | 每一行的含义 |
| --- | --- |
| `t_cmd_can_tx_ns` | 本行四路力矩命令成功送上 CAN 总线的时刻，单位为主控单调时钟 ns。 |
| `tau_lf0_Nm,tau_lf00_Nm` | 本行已限幅、实际发送给左腿两个真实 DM8009P 的力矩，单位 N·m。 |
| `leg_length_left_m,leg_pitch_left_rad` | 命令发送时，按最新左腿电机角经真实 FK 得到的左腿长和左腿倾角。 |
| `tau_rf0_Nm,tau_rf00_Nm` | 本行已限幅、实际发送给右腿两个真实 DM8009P 的力矩，单位 N·m。 |
| `leg_length_right_m,leg_pitch_right_rad` | 命令发送时，按最新右腿电机角经真实 FK 得到的右腿长和右腿倾角。 |

### 用例和步骤

`torque_program.yaml` 写四路实际力矩序列、周期、签字安全上限和 FK 版本。DM8009P 真机许可力矩不在仓库中，文档不填假数字。

| 用例 | 阶段 | 目的 |
| --- | --- | --- |
| `torque_baseline` | `baseline_start → zero_hold → baseline_end` | 全零力矩时的腿长、腿倾角和力矩零漂。 |
| `torque_step` | `zero_before → excite → zero_after` | 小幅批准力矩阶跃的延迟、超调和阻尼。 |
| `torque_chirp` 或 `torque_prbs` | `zero_before → excite → zero_after` | 频率/宽频响应。 |
| `holdout_torque_*` | 同原用例 | 留出验证，不参与拟合。 |

执行：架空和 FK 静态核对完成后，先记录四路 `0 N·m` 至少 5 s；仅按 `torque_program.yaml` 激励；结束立即四路置零并再记录至少 5 s。每个用例三次有效 run，加一次 holdout。发生任何故障立即置零。

## 5. MuJoCo 如何回放

真机和 MuJoCo 电机安装位置不同，**不能**同名电机直接复制力矩。两边通过共同腿任务坐标 `s=[leg_length, leg_pitch]` 传递功率：

```text
q_motor = f(s)
J = dq_motor / ds
tau_leg_real = J_real(q_real)^T * tau_motor_real
J_model(q_model)^T * tau_motor_model = tau_leg_real
```

MuJoCo 输入 `tau_motor_model`，腿位置 PD 对该 run 关闭，轮力矩为零。仿真输出同定义的左右腿长、腿倾角及导数，与真机 CSV 对齐。

拟合：力矩倍率、力矩延迟/一阶滞后、`armature`、`damping`、`frictionloss` 和传动弹性。不要用位置 PD 或闭链 equality 的 `solref/solimp` 掩盖力矩误差。

只有 `FK_real`、`J_real`、电机力矩方向/单位、`FK_model`、`J_model` 都验证后，才允许回放。需要的 `kinematics_manifest.yaml` 至少含：两电机基座坐标、连杆长度/销轴拓扑、电机零位/正方向、腿长参考点、腿倾角零位和正方向。

## 6. 交付检查

- [ ] 轮端：两份原始 C620 CSV、`manifest.yaml`、理论包络和留出 run。
- [ ] 关节：`joint_torque_snapshot.csv`、`torque_program.yaml`、`kinematics_manifest.yaml`、`manifest.yaml` 和留出 run。
- [ ] 关节 CSV 每一行只有 CAN 发送时间戳、四路 URDF 实际电机力矩、左右腿长和左右腿倾角。
- [ ] MuJoCo 用映射后的力矩比较腿长/腿倾角，不把真实电机位置或同名电机力矩直接当作模型输入。

原 `data/sysid/chuanliantui/reference/pose_*_v1/model_reference.csv` 仅保留为模型闭链位置映射检查，禁止下发真机，也不作为试验 B 输入。
