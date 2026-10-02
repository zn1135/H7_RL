# HPI1：上位机策略推理与 H7 实时执行

此功能默认关闭。训练仓的 `sim2sim/host_policy_usb.py` 用完整
chuanliantui `model_*.pt` 在电脑上推理；H7 仍从真机 IMU、闭链解算和电机反馈
构造 25 维观测与 125 维历史，以 100 Hz 送往电脑。电脑回 6 维**训练空间**动作，
H7 用机器表中已配置的映射转为固件空间，沿现有 `RL_Torque_Compute`、
`output_dispatch` 和 CAN 驱动以 500 Hz 执行。电脑不直接发送电机力矩。

## 物理许可与故障行为

1. 左拨杆下位、电机失能时，电脑发 `HELLO` 才能建立 HPI1 会话。此时 H7
   锁住普通板端 RL，**不会**因拨杆改变自动切回板端推理。
2. 电脑程序须显式传 `--enable-output` 才会发 `ARM`。只有左拨杆上位、右拨杆中位、
   遥控/IMU/腿解算/六电机反馈有效且无故障时，H7 接受 `ARM`。上位机未返回首拍
   动作前，执行层保持零动作。物理急停与原有总出力开关仍有效。
3. 超过 30 ms 没有有效动作、USB 断开、输入帧/动作异常、反馈失效、拨杆改变或
   板端故障时，H7 清动作并锁止，电机按现有失能路径关断。锁止后不会自动恢复；
   左拨杆下位且电机失能时发 `STOP` 才释放会话。
4. HPI1 与 USB `JID1`、USB 端 VOFA/S2R 遥测互斥。HPI1 占 USB CDC；若要同时
   采集 S2R1，必须使用独立的 UART8 遥测链路。原 JID1 单电机台架许可和限值不变。

MCU 重启会清除 HPI1 会话锁；重启前及重新接线时将左拨杆保持下位，避免固件按
原有遥控模式启动板端策略。

这些是源码中的门控设计，**尚未由真机验证**。首次验收从失能通信开始，随后按
固定机身、轮离地、急停可及的台架流程核对观测、动作、极性、时序和锁止；
物理量由现场人员实测，不按仿真推断修改。

## 帧格式

所有整数与 float32 为小端。帧结构：`"HPI1"(4), version=1(u8),
type(u8), payload_len(u16), frame_seq(u32), payload, CRC32(u32)`。
CRC32 使用 IEEE/zlib，覆盖帧头与 payload。最大 payload 为 616 字节。
USB 回调只入队，`commTask` 校验长度、CRC、会话和序号。

| 方向 | type | payload | 作用 |
|---|---:|---|---|
| PC→H7 | `1 HELLO` | 空 | 失能时建立会话；返回 STATUS |
| PC→H7 | `2 ARM` | `session_id:u32` | 请求投入；返回 ACK |
| PC→H7 | `3 ACTION` | `session_id:u32, input_seq:u32, action:f32[6]` | 回对应输入的训练空间动作；返回 ACK |
| PC→H7 | `4 STOP` | 空 | 清动作并锁止；左拨杆下位且电机失能时释放会话 |
| H7→PC | `0x81 STATUS` | `session:u32, locked:u8, armed:u8, obs:u16, history:u16, action:u16, fault:u32, source_sha256:ascii[64]` | 接口和源码身份 |
| H7→PC | `0x82 INPUT` | `session:u32, input_seq:u32, sample_us:u64, obs:f32[25], history:f32[125]` | 每个策略拍发送当前真实输入 |
| H7→PC | `0x83 ACK` | `code:u8, session:u32, applied_input_seq:u32` | 命令结果 |

`INPUT` 的历史末帧应等于本拍观测。板端拒绝重复、未来或落后超过一拍的动作；
延迟观测不得当成新观测使用。`sample_us` 是板端观测采样时间；电脑单调时间只用于
测自身耗时，两个时钟未经同步不能直接相减。

## 运行与数据

在训练仓中使用其 `.env.local` 的 Isaac Gym Python 环境：

```bash
"$WHEELLEGGED_PYTHON" sim2sim/host_policy_usb.py \
  --port <H7板载USB CDC串口> \
  --checkpoint <完整的25维model_*.pt> \
  --out <新目录>
```

上述命令只探测接口，不 ARM。现场完成台架与安全检查后，显式追加
`--enable-output --duration-s <秒>` 才会等待拨杆许可并运行。电脑保存
`host_policy.csv`（输入序号、板端采样时刻、电脑收/发时刻、推理耗时、动作）与
`host_report.json`。板端同时用 UART8 S2R1 采集 `POLICY/CONTROL/IMU`，
再使用训练仓 `sim2sim/s2r_offline.py` 做闭链 MuJoCo 离线对照。

上位机与板端模型必须事先核对权重来源、观测接口和模型校验值；协议目前只报告
固件源码指纹，不能自动证明两端权重相同。HPI1 的 30 ms 锁止是故障门，不表示
通信+执行延迟已满足训练分布。联机延迟、抖动和整机行为均待台架及真机验收。
