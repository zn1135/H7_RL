# H7 板载 USB CDC：关节映射与力矩辨识

本页是关节 USB 试验流程。`md/sysid/` 中的旧方案仅供追溯，旧 `imcalib/Sysid/` 固件模块已移除。本流程不使用 S2R 或 VOFA 帧控制电机；USB 上的 `JID1` 是独立的双向协议。当前测试固件上电将 VOFA 留在 UART8，USB CDC 交给 `JID1`。切换到 USB 遥测前须先失能、退出物理许可挡位并停止关节采样流；USB 遥测占口时 `JID1` 不接收命令、不能使能关节输出，物理许可挡位仍保持原有电机失能门控。

## 状态与安全门

- 机器：大轮腿，四个真实 DM 电机依次为 `front_left`、`rear_left`、`front_right`、`rear_right`。训练侧 `lf1/rf1` 是闭链解算的虚拟膝角，并非独立电机。轮电机在 USB 台架模式恒为零命令。
- 物理许可：机身刚性固定、腿自由、轮离地、急停经当日检查。遥控 **左上＋右下** 选 USB 测试；左下失能。此挡位默认仍失能，USB 不能绕过遥控。测试时任何拨杆变化、USB 断开、故障、CAN 反馈失效、位置／速度越限、命令 30 ms 未更新，都将零力矩并锁止。必须先拨左下，再回到测试挡位并重新 ARM。
- `imcalib/Telemetry/joint_usb_limits.h` 当前 `JOINT_USB_LIMITS_APPROVED=0`，四路力矩／速度／角度上限全零；非零输出**不可用**。作者台架测定四电机的许可力矩、最大速度以及 `pos_zero_rad` 坐标下的位置区间，并核对 MIT 实际刻度后，才可单独确认并配置该文件。训练策略的 40 N·m 限幅、MIT 的 54 N·m 量化满量程和旧文档里的数值都不是本试验许可值。USB 命令不能改板端限值。
- 测试模式保留原 `task_actuation.c` 唯一 CAN 下发出口。任何时候最多一个 `dm[]` 非零，其他三个 DM 为零，DJI 两轮为零。STOP、中止、物理失能后不自动切回 RL/LQR。

## 先验证 USB（全程零出力）

1. 接板载 USB 数据口，找 `STM32 Virtual ComPort` 对应的 `/dev/ttyACM*`；不要选调试器 USB 转串口口。CDC 报告的 1152000 line coding 不决定 USB 线速。
2. 安装上位机 `pyserial`，在 H7 仓库运行：

   ```bash
   python3 tools/joint_usb_capture.py --port /dev/ttyACM0 --mode echo --out data/joint_usb/echo-001
   ```

   期望 STATUS 返回、100/100 个 32 字节回显一致。此前板载 USB CDC 曾可枚举但 PC 接收为 0 字节；编译通过不等于此项通过。若失败，保持失能，用调试器观察 `joint_usb_cdc_debug` 中 `rx_packets/tx_submit/tx_busy/tx_done`，再查 USB IRQ 和主机原始收包，不转入力矩阶段。
3. 左拨杆下位，运行手动采集：

   ```bash
   python3 tools/joint_usb_capture.py --port /dev/ttyACM0 --mode map --phase-seconds 6 --out data/joint_usb/map-001
   ```

   先给实体支路贴正方向标记；脚本依次提示沿该方向缓慢摆动并保持，再返回初始位。它实时打印四个电机原始角、速度、命令与反馈力矩，以及左右腿长、摆角和故障，保存 `usb_raw.bin`、`joint_samples.csv`、`mapping_spans.json`、`manifest.json`。闭链会联动，报告只列带符号位移和变化幅度；物理电机标签、转动方向、零位及腿长要由现场逐项确认。不要仅凭最大变化列自动改 `dm_sign/dm_zero/rl.sign/rl.zero`。

2026-09-30 在电机失能、限值未批准状态完成 USB CDC 双向链路验收：COM70 的 STATUS 返回源码指纹且与本次构建一致，100/100 次 32 字节 ECHO 一致；`--mode passive --duration 5` 收到 2500 条快照（500 Hz），主机序号缺口 0、CRC 错帧 0、板端累计丢弃 1 条旧快照，四路命令力矩均为 0。原始字节、`joint_samples.csv` 与 manifest 位于 `build/jid1-usb-20260930/passive-01/`。手动摆腿映射与非零命令仍未测试。

## 后续非零辨识（待限值确认）

先确认 USB 回显和失能映射，再由作者写入四路限值、重新编译烧录。现场再次固定机身并检查急停。左上＋右下后用脚本发送 ARM；每个命令只有 `motor_index` 与该电机力矩 N·m，100 Hz 逐点发送，H7 在 500 Hz 控制拍保持最新有效值。超时 30 ms 即中止。仅小幅正负阶跃开始，随后才做扫频；每个用例留独立重复 run，不把 holdout 用于拟合。

```bash
python3 tools/joint_usb_program.py --motor 0 --kind step --amplitude-nm <已批准幅值> --out /tmp/joint-step.json
python3 tools/joint_usb_capture.py --port /dev/ttyACM0 --mode run --program /tmp/joint-step.json --out data/joint_usb/step-front-left-001
```

出现拒绝码、故障或异常运动，脚本发送 STOP，板端自身也执行超时/故障锁止。采集记录 USB 和板端丢帧；坏帧、单次样本间隔超过 8 ms 或丢帧超过 1% 的 run 不参与拟合。保存失败 run 的原始文件并标为无效。

## `JID1` 协议 v1

所有整数和 float32 为小端。帧为 `magic="JID1"(4), version=1(u8), type(u8), payload_len(u16), seq(u32), payload, CRC32(u32)`；CRC 使用 IEEE/zlib，覆盖 CRC 字段之前的全部字节；payload 最长 192 字节。接收回调只复制至有界队列，`commTask` 校验与解析；无 CRC、版本、长度或序号有效性时不接受控制。RX 溢出会中止试验。

| 主机 type | payload | 返回 |
|---|---|---|
| 1 STATUS | 空 | `0x82 STATE`，含固件源码 SHA、机型、门控、故障及四路限值 |
| 2 ARM | 空 | `0x81 REPLY`；限值未批准时拒绝 |
| 3 SET | `motor_index:u8, torque_nm:f32` | `0x81 REPLY`；超限/无效即中止 |
| 4 STOP | 空 | `0x81 REPLY`，锁止并清零 |
| 5 ECHO | 最长 32 字节 | `0x85 ECHO_REPLY`，原样回传 |
| 6 STREAM | `0` 停、`1` 开 | `0x81 REPLY`；采集脚本运行期间才发快照 |

`0x83 SAMPLE` 每控制拍生成，USB 忙时可丢旧快照并累计 `dropped_samples`；包括 MCU `t_sample_ns`、最近有效命令末字节进入 CDC 接收回调时的 `t_usb_rx_ns`、本拍四路 CAN **排队前** `t_can_queue_ns`、与已解析角度/速度同一帧的 CAN IRQ 接收时刻 `t_can_rx_*_ns`，以及序号、状态、四路命令/反馈角度/速度/力矩和左右腿几何。新 CAN 报文尚未解析时，不提前更新这一帧的反馈时间。`t_can_queue_ns` 不是总线上发完、更不是电机内部生效时刻。PC `host_recv_ns` 只用于检测主机侧拥塞，不用于板端动力学时间基准。

## 离线交付训练仓

先经作者台架确认实体电机与模型 `lf0/lf00/rf0/rf00` 的一一对应、符号和零点，写映射 JSON。未确认时 `approved` 保持 `false`，离线脚本拒绝拟合：

```json
{
  "approved": false,
  "motor_to_model": {
    "front_left":  {"joint": "lf0",  "sign": null, "offset_rad": null},
    "rear_left":   {"joint": "lf00", "sign": null, "offset_rad": null},
    "front_right": {"joint": "rf0",  "sign": null, "offset_rad": null},
    "rear_right":  {"joint": "rf00", "sign": null, "offset_rad": null}
  }
}
```

`joint_samples.csv` 和 `manifest.json` 可交给训练仓 `sim2sim/fit_joint_usb.py`。它把基座在内存中固定、重放已排队的真实电机力矩，拟合全局延迟、力矩倍率、主动轴阻尼与摩擦倍率，输出拟合与留出对照 CSV/PNG 和报告；不写回 H7 物理表或训练模型。模型中的气弹簧为每侧恒定 150 N，仅作为现有 XML 假设。若初始闭链装配分支或静态角度对不上，先修正台架映射记录，不继续优化动力学参数。
