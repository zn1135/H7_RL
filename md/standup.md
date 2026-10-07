# 独立自起模块

`imcalib/Algorithm/standup.c/h` 是唯一自起模块，沿原执行任务1kHz运行；不新增任务、反馈解算或通信入口。当前包含正立准备及俯仰翻倒／仰卧恢复第一轮试验；明显侧躺或混合大侧偏只分类后停止，尚未执行侧躺扶正。

## 公共输入与参数

自起只读执行层从 `imu_state`、`leg_l`、`leg_r` 复制的同一拍快照；公共腿解算、轴映射、零点与极性保持。世界 `pitch_world/pitch_world_valid` 只判断是否需要自起、允许准备范围及回正到位；自起控制摆角改为公共几何腿角 `virtual_leg_angle`，摆角速度为对应 `d_virtual_leg_angle`，不再减原pitch／gyro。角度误差用最短路径环绕，没有额外反馈反号。

全局 `standup_param` 是一套公共自起参数，直接初始化，不再有大小机数组、machine_id选择、configured标志或ctx中的参数副本。两个机型共用该对象；原机器配置仍提供腿长辅助PD、站立目标、支撑前馈、机械范围及平台电机限幅，这些属于既有平台数据，本轮没有改它们。

参数在 `standup.c` 顶部可改，也可失能时通过Watch修改 `standup_param`，退出再投入后重新建立PID历史。长度增益读取当前 `machine->lqr.leg_len` 的P/D和输出上限，不新增自起腿长增益。自起自己保存PID实例和历史，避免影响正常平衡。所有实例初始化Ki=0，积分限幅0。

入口现在只检查指针、IMU在线／世界pitch有效和两腿解算／力映射有效标志，不再逐项检查机器常量或PID系数；参数由配置者保证合法。计算角度／速度、F/Tp和最终输出的异常清零检查保留。

自起参数数值以standup.c顶部为准；摆角外环／内环P由作者持续调试，当前为42／10，I和D均为0。速度目标上限仍单独由angle_speed_max约束。自起Tp使用独立tp_max（当前40N·m），内环同时取机器上限的较小值；气弹簧补偿后的四髋最终力矩仍使用machine->dm_trq_clamp，正常LQR限幅不改。LQR运行时lqr_debug.trq_max_hip仍是原算法的独立调试覆盖，自起不引用LQR模块。触发、到位、超时和目标斜坡沿用原参数，未因合并力矩限幅而改动。

## 状态与目标分发

| phase | 意义 |
|---|---|
| 0 IDLE | 首次投入统一准备后摆；许可不足时need=1、不出力 |
| 1 RETRACT | 第一步：长度PD直接给短腿目标，两腿都≤retract_ready_len后立即切入摆腿，Tp与两轮为零 |
| 2 SWING | 第二步：摆角位置外环＋速度内环对齐原站立摆角，长度PD继续向原站立长度控制，支撑前馈渐入，轮零 |
| 3 DONE | 准备条件持续满足，允许原LQR投入；先渐变最终输出，随后正常平衡 |
| 4 FAILED | 第一条错误锁存、清输出，退出投入位置后才能重试 |
| 5 REAR | 上绕后摆，保持进入腿长、连续几何角路径、轮零和支撑零 |
| 6 EXTEND | 先伸腿，保持进入摆角，腿长目标extend_len，轮零和支撑零 |

当前流程为 `IDLE → EXTEND → REAR → RETRACT → SWING → DONE`，phase为 `0 → 6 → 5 → 1 → 2 → 3`。已伸足可跳过EXTEND，已在后摆窗口可跳过REAR；原RETRACT的0.19m摆腿切入门槛保留。收腿PID目标 `retract_len` 保持0.15m；新增 `retract_ready_len` 是切入摆腿门槛，当前0.19m。两腿都≤门槛就立即开始摆腿，不再要求长度误差±length_tol、伸缩速度低、目标长度到位或额外保持100ms。进入时已经满足门槛可直接SWING；摆腿期间长度目标也保持retract_len；接入后LQR仍使用各机器原站立长度。数值以源码为准。

到位统一由Standup_Ready判断：EXTEND两腿实际长度≥有效extend_len−extend_tol并保持stable_time；REAR两腿沿选定路径的连续几何角都到rear_goal±rear_tol并保持stable_time后转段；RETRACT保持两腿实际长度≤0.19m切入摆腿；SWING只检查左右实际／指令腿长误差≤0.04m、摆角误差≤angle_tol（作者当前改为0.3rad），并连续保持stable_time（当前100ms）。不再检查世界pitch角度、pitch角速度、腿长／摆角速度或支撑渐入完成。支撑仍按原控制过程渐入，但不是交接门槛。

自起没有独立pitch纠偏环；自起摆角环已不使用pitch／gyro作坐标换算或速度反馈；原LQR仍按原定义使用它们。世界pitch仍用于自起触发、准备姿态保护和公共翻倒保护，输入有效标志保留。本轮只简化交接到位判定，不改反馈坐标或公共保护。支撑角度窗口仍为STANDUP_SUPPORT_ANGLE_RANGE=0.6rad，支撑力计算不改。

`phase` 是主状态源，删除active／done等重复字段；need保留，因为等待许可时也要记录需求。elapsed、stable、trigger_elapsed分别记录阶段超时、到位保持和再次触发保持。正常平衡中只有遥控速度／转向／腿长命令均为0且偏差持续超过trigger_time，才再次准备。

## 控制与力矩输出

当前函数职责：`Standup_PID_Init`只初始化六个PD实例，在模块初始化和开始一次自起时调用；`Standup_Stage_Update`只判需求、阶段／到位保持及超时；`Standup_Target_Update`只根据阶段建立／更新长度和摆角目标及支撑渐入；`Standup_PID_Calculate`只计算F/Tp和PD历史；`Standup_Torque_Output`只调用公共映射、补偿与限幅。主Update按上述顺序调用，不再混入PID参数初始化，不使用goto或压成一行的if/for。阶段判断现在在本拍控制计算前，转段相对旧末尾判定可能相差一个控制拍，保持时间及阈值不变。

长度使用原 `pid_calc` 和机器辅助PD参数；初始化／复位后的首拍用本拍目标和反馈预置历史，消除清零历史带来的D跳变，之后仍按原误差差分D计算。此机制不改正常平衡辅助环，也不新增积分。

摆角位置外环输出rad/s速度目标，先限速；速度内环直接读取公共解算几何摆角速度，输出N·m的Tp并限幅。两环首拍均预置历史，防止后续调入D后产生初始化冲击。长度目标在自起期间直接给retract_len（当前0.15m），len_rate字段和腿长斜坡已移除；SWING首拍直接将angle_cmd设为本机leg_trim并保持，不再执行摆角目标斜坡，swing_rate字段已移除。目标零偏保持，准备摆角反馈改为几何坐标，PID历史只属于自起。

`F/Tp → Leg_Force_Map_Forward → Gas_Spring_Apply一次 → 最终关节限幅 → prepare`。公共映射及弹簧实现未改；删除旧独立len_kp/len_kd/force_max、单层swing_kp/swing_kd、额外body_kp和目标速度派生数组。

## 接入原LQR

`Robot_Control_Init` 调用无机器参数的 `Standup_Init`。执行层仍由左中＋右中选择这条路径；原 `lqr_engage_update`、`solve_lqr`、RL与唯一 `output_dispatch` 函数体保持。

DONE之前，Standup_Update输出准备力矩；本拍刚完成仍发最后一拍准备输出。后续DONE拍由执行层先调用原投入／求解生成有效LQR候选，再交给同一个Standup_Update内部做短输出渐变，不再暴露Standup_Handoff接口。首次投入不再直接判断已站立并跳过准备，而是统一确认后摆位置；关闭standup仍沿原LQR路径。正常平衡不强制重复后摆，仍按原Standup_Need和trigger_time触发新的准备周期。

交接只混合两个已包含补偿的最终候选：关节 `(1-blend)*prepare+blend*balance`，轮输出 `blend*balance`；交接期间根据当前反馈刷新准备控制，blend=1后停止准备计算，不重复补偿。故障或LQR候选无效同拍清输出并锁存。再次触发时，执行层本拍算出的平衡候选被准备输出覆盖，唯一分发口只发送最终选择的一份。

无效世界pitch／IMU离线归硬FAULT_IMU。有效pitch超过1.4rad仍置fallen、小于1.0rad解除；正常LQR／RL据此停止，自起模式开启recovery_enabled时按新恢复状态执行，硬故障保持停止。自起自身失败保持零力矩，不另发失能命令。反馈恢复不清锁存；有效遥控退出右中、左下或切模式清本次状态。

## 观测与验证

当前作者已将初始化默认设为 `standup_control.enabled=1`，左中＋右中进入自起／LQR选择；失能时设为0可回到原平衡路径。Watch看 `standup_param`、`standup_control.phase/fault/need`、length_cmd、angle_cmd、angle_speed_cmd、force、tp、support、blend以及三组PID历史。旧active/done/param/handoff字段已移除，状态码也已更新。

普通VOFA沿用Robot_Control_Send_Vofa发送39通道，现完整记录控制摆角／速度、目标、F/Tp、支撑、交接、pitch及最终髋／轮命令。完整通道定义见 [VOFA全过程](vofa_policy_trace.md)。旧离线留证通道已替换，但原内部锁存继续工作。数据用于比较响应和阶段时限，不直接据图形观感调参。

`Standup_Fail` 保留第一次失败的fault、elapsed、stable及最后一拍控制量，prepare仍立即清零；FAILED期间不会继续计时或计算。退出投入位置会Reset，清这些记录，因此采集要包含退出前的失败帧。phase=4、fault=2且elapsed接近3000/4000ms分别对应收腿／摆腿超时，当前阈值以源码为准。若在线掩码ch0／故障掩码ch2同时变化，结合fault区分自起超时与外部失能。公开模块接口仍是Init、Reset、Fail、Update四个。

脚本反馈测试验证两机前／后摆腿、两阶段超时、持稳条件丢失、速度限幅与超速制动、首拍PD历史、Ki为0、输入无效和人工重试、原LQR交接及旧路径。编译／逻辑测试不代表实机负载、接地、动态起立、1ms预算或原左髋反馈中断原因已验证。

参考采用Wheelleg-big-lhx的两步准备思想和摆角P/P串级初值；未照抄其角度零点／镜像符号／固定bench K。该参考user_lib.c的PidCalc使用位置／角度外环到速度内环；当前Leg3_v2自救分支为空。


## 参考摆角双环比较

Wheelleg-big-lhx的IMCALIB/Tool/user_lib.c第178～181行：位置外环P14/I0/D0、MaxOutput200；速度内环P3/I0/D0、MaxOutput25。PidCalc第219～223行将外环pos_out传给内环，stand_step=1时StandFun直接将Target_Leg_Angle置0。当前P由作者改为42／10，也直接给本机站立目标；速度目标上限仍保留本机设定，静止时Tp约为内环P乘受限速度目标，并不自动达到力矩上限。参考位置反馈是几何腿角减π/2，速度反馈含其原pitch构造和滤波，当前只采用几何角准备控制的思路，不照抄π/2零点、速度融合或电机符号。

参考的摆角两环D均为0，内环速度反馈本身提供阻尼；当前未饱和时相当于Tp=外环P×内环P×摆角误差−内环P×摆角速度。腿长参考采用位置600/0/28000到速度1.1/0/0的串级，另有50N收腿偏置；当前自起仍复用本机单层长度PD，未在本轮修改。Leg3_v2的SELF_RESCUE分支为空，不作为可执行自起PID参数来源。


## 本轮采集参数快照（2026-10-05）

本轮仅改VOFA，不调整这些参数。若后续源码／Watch调参，CSV须附当时参数，不能沿用这份快照。ch36/37记录实际PID实例P；退出复位后可为0。

```c
standup_param_t standup_param = {
    .recovery_settle_time = 0.2f, .recovery_tuck_time = 0.1f,
    .recovery_len = 0.16f, .recovery_support_len = 0.20f,
    .recovery_len_rate = 0.3f, .recovery_angle_rate = 2.0f,
    .recovery_force_max = 150.0f,
    .recovery_support_pitch = 0.8f, .recovery_ready_pitch = 0.35f,
    .recovery_ready_leg = 0.7f, .recovery_timeout = 4.0f,
    .recovery_stall_time = 0.3f, .recovery_retry_max = 2u,
    .extend_len = 0.30f, .extend_tol = 0.01f, .extend_timeout = 4.0f,
    .rear_angle = -1.5f, .rear_tol = 0.3f, .rear_timeout = 4.0f,
    .rear_rate = 4.8f,
    .retract_len = 0.15f, .retract_ready_len = 0.19f,
    .angle_pos_kp = 25.0f, .angle_pos_kd = 20.0f,
    .angle_speed_kp = 5.0f, .angle_speed_kd = 5.0f,
    .angle_speed_max = 10.0f,
    .tp_max = 40.0f,
    .roll_force_max = 40.0f, .roll_force_rate = 10.0f,
    .roll_ready = 0.3f,
    .trigger_angle = 50.0f * LEG_PI / 180.0f,
    .trigger_pitch = 40.0f * LEG_PI / 180.0f,
    .angle_tol = 0.3f, .length_tol = 0.04f,
    .pitch_max = 1.0f, .roll_max = 1.2f,
    .stable_time = 0.1f, .support_time = 0.3f,
    .blend_time = 0.35f, .trigger_time = 0.1f,
    .timeout = {3.0f, 4.0f},
};
```

大机器现行LQR／辅助参数：

```c
        .lqr = {
            .dt = 0.001f,
            .leg_len_init = {0.15f, 0.15f},
            .leg_trim = {-0.06f, -0.06f},
            .pitch_trim = 0.0f, .pos_target = 0.0f,
            .vel_max = 3.0f, .yaw_max = 5.0f, .len_rate = 0.3f,
            .vel_ramp = 5.0f, .pos_arm_vel = 0.0f,
            .lpf_alpha = {0.3f, 0.3f, 0.3f},
            .kf_p0 = 0.1f, .kf_q = 0.007f, .kf_r = 0.01f, .kf_p_max = 0.5f,
            .leg_len = {{800.0f, 0.0f, 25000.0f, 5000.0f, 0.0f},
                        {800.0f, 0.0f, 25000.0f, 5000.0f, 0.0f}},
            .roll = {500.0f, 0.0f, 100.0f, 5000.0f, 0.0f},
            .support_force = {20.534f * 9.81f * 0.5f, 20.534f * 9.81f * 0.5f},
            .vel_src = 1u, .yaw_hold = 1u, .yaw_rate_hold = 1u,
            .pos_hold = 1u, .wheel_enable = 1u, .hip_enable = 1u,
            .len_pid_enable = 1u,
        },

```

力矩上限54N·m、轮上限约4.844N·m、轮半径0.0525m；自起与LQR共用机器表和Gas_Spring_Apply。K表头快照：

```text
表号   : big_wheelleg-sjtu5-20261004-1753
Q      : diag([100      1   4000      1    600     10    600     10  60000      1])
R      : diag([100  100    1    1])
网格   : lL, lR = 0.15:0.01:0.31 (17x17), Ts = 0.001, c2d ZOH + dlqr, 腿数据取行 interp
拟合   : poly33: poly22 + p30*lL^3 + p21*lL^2*lR + p12*lL*lR^2 + p03*lR^3
```

直接进入LQR的现行原因：Standup_Need仅按两腿摆角偏差和世界pitch判断，没有腿长、机体高度或接地判断。即使腿压在机身下面，只要角度偏差未到触发阈值，就进入DONE且blend=1。当前采集会记录这一分支，尚未修改触发条件。


## 直接长度目标

RETRACT／SWING从首拍直接给左右length_cmd=retract_len（当前0.15m），最终到位判定也以该目标为准，不再引用len_rate或做长度目标斜坡。DONE保持最后自起目标供输出渐变；完全接入后按各机器原LQR腿长目标控制。首次长度PD仍预置误差历史，避免额外D初始化冲击，但P仍立即响应完整目标误差。


## 几何摆角准备与bench

本轮Standup_Need的摆角触发、Standup_Ready的到位、目标首拍初始化、角度／速度双环和支撑权重统一使用公共几何腿角；IMU继续只用于世界pitch触发／输入有效性／姿态保护。原LQR世界系腿角和滤波角速度不改。自起目标继续用本机leg_trim（大机−0.06rad），未改零点／极性或公共几何解算。

参考bench是stand摆腿到位后的短时平衡状态，立即用固定10维状态反馈同时控制轮和虚拟腿Tp，含pitch和pitch角速度，运行50周期后切wheelleg正常K表；不是单独的pitch PID或本机台架测试开关。当前仍保留原DONE渐变到本机LQR，没有新增参考固定K阶段。


## 当前中断诊断页

当前39通道VOFA已替换为四髋反馈年龄／错误码和FDCAN1 FIFO／错误计数／协议状态页，保留阶段、姿态、长度、几何角与最终髋命令；旧全过程映射为历史。参见vofa_policy_trace.md当前表。本轮控制逻辑和参数均未修改。


## 后摆预先位置

本轮作者指定rear_angle=−1.3rad、rear_tol=0.3rad，故双腿几何角需在[−1.6,−1.0]rad内。首次左中右中统一走这套准备，不能再因前方／压腿姿态与站立角接近而直接LQR。REAR使用现有摆角双环，保留进入时的左右长度目标，支撑和轮输出零，仍经过一次公共气弹簧补偿和原最终分发。到位保持原stable_time（当前100ms），Rear超时rear_timeout（初值4s）清零并锁存，外部CAN门控／姿态保护仍生效。

转入RETRACT／SWING后才直接给retract_len=0.15m；转段预置长度／角度PD历史，避免从保持长度跳到收腿目标时增加D初始化冲击。原pitch／roll保护、增益、0.19m摆腿切入门槛、原收腿／摆腿超时和LQR渐变都保持。单机已经后摆到位可直接从IDLE进入原收腿／摆腿，已有正常LQR只有原再次自起条件持续成立才重新准备。

诊断页ch3=5表示后摆，ch36/37是实际几何角，ch27/28是腿长，ch21～24为最终髋命令，ch7～10与CAN字段继续定位掉线。新增后摆不等于修复已确认的CAN ACK中断。


## 先伸腿和上绕路径（当前）

作者确认正立前侧约+0.5rad时，增大几何角经过+π／−π到−1.3rad是从机体上方绕过，进一步要求先伸到0.30m。首次投入先EXTEND（phase6），用原长度PD直接给extend_len，按机械范围夹紧；大机有效目标0.30m、到位下界0.29m，保持原100ms。此时几何摆角目标保持进入值，轮和支撑为零；伸腿超时extend_timeout初值4s。小机有效目标夹到其原机械上限，不改变机器表。

伸腿到位后REAR（phase5）保持当时长度，再上绕。规划把当前几何角／后摆目标减去同一入口world pitch，映射到[0,2π)后选择不跨世界向下切点的路径；世界上方位于该区间中央。进入时一次锁定rear_goal，过程中不随pitch波动重新选路。正立前侧的−1.3rad目标等效为约4.983rad，倒置时路径的选择随重力方向改变，但现有fallen及pitch／roll保护不放开，未支持倒置实机出力。

rear_position通过每拍Wrap(本拍几何角−上一拍几何角)累计展开，后摆位置PID临时angle_wrap=0，避免把大于π的目标误差重新缩成最短路。到位按展开角检查，沿错误下绕方向到达同一个环绕角不能提前通过。退出REAR恢复angle_wrap=1并预置PID历史，原收腿／摆腿、支撑、LQR渐变与公共极性／零点不改。新增轨迹不等于CAN中断已修复，原CAN诊断页继续记录。


## 自起专用Tp限幅（当前）

作者要求采用Leg3恢复的40N·m限幅，新增tp_max=40。速度内环MaxOutput取min(tp_max,机器限幅)，计算后的Tp再按tp_max裁剪，因此伸腿保持角、上绕、摆正和交接中的准备候选均受此约束；收腿Tp原本为0。40是虚拟腿Tp，不限制腿长控制F，雅可比分解／弹簧补偿后的单电机仍限54（大机）。原LQR和输出混合不改，交接期LQR候选不受这个自起专用限制。

两份参考摆角参数：Leg3_v2当前app_recovery是单层位置PD300/0/1500、合成Tp限40，角目标每拍增加约0.00424rad；Wheelleg-big-lhx stand双环外14/0/0、速度目标上限200，内3/0/0、输出上限25。其init粗复位速度PID9/0.003/0、输出上限20，速度目标常见±4。当前双环25/0/0→5/0/0、速度目标上限10，保持不变。各参考坐标、补偿和D算法不能只按P数值直接替换。


## 上绕角目标斜坡（当前）

作者要求仿照Leg3绕腿目标斜坡，新增rear_rate=4.239rad/s，对应参考每毫秒0.00314×1.35rad。Rear路径终点rear_goal仍按原入口world pitch锁定，方向和连续角反馈不改；初始化angle_cmd为实测几何角，每拍最多rear_rate×dt向展开终点推进，不一次跳到4.983rad。只改变REAR，原伸腿／收腿直接目标、SWING直接站立摆角、原补偿和交接保持。

Rear到位同时要求展开实际角及推进后的目标角都距终点≤rear_tol，再保持原stable_time，防止目标尚未推进到终点就提前切段。作者当前将tp_max调为30N·m、速度目标上限调为6rad/s，本轮保留，不恢复40／10。正立从+0.5上绕到−1.3等效终点约4.983，目标推进约1.06秒（不代表实际到位时间）；可在Watch看angle_cmd／rear_position／rear_goal，CAN诊断页ch36/37为环绕几何反馈。


## 双腿可支撑时的roll恢复（当前）

作者确认常见姿态是明显侧偏但两腿仍能支撑。本轮在准备控制中增加独立roll PD，参数读取本机原LQR roll的P/D、强制Ki0，首拍预置历史；目标为原控制坐标roll=0（Euler[0]）。差动力方向沿用正常Leg_Balance：左F加、右F减，不改电机极性／零点。

新增roll_force_max40N、roll_force_rate200N/s，先裁剪／渐变PD差动力，再乘roll_weight=min(max(cos(左几何角−world pitch),0),max(cos(右几何角−world pitch),0))。它只是腿方向权重，不是接地传感器；腿朝机体上方时为0，避免上绕时施加同样的差动力。roll_force是限速后的请求，roll_applied是实际加入左右F的差动力；两者单位N，不是虚拟Tp或单电机N·m。原雅可比、弹簧补偿及最终限幅保持。

准备roll保护范围扩大为roll_max1.2rad（约69°），并新增roll_ready0.3rad作为SWING到LQR的到位要求，与原长度／摆角条件共同保持100ms；pitch交接门槛仍未恢复，pitch触发／保护及fallen门控保留。这版不执行90°侧躺或倒置恢复，不绕过CAN／电机硬故障。正常LQR仍使用原roll环，与自起PID历史独立。

作者已将VOFA改回LQR状态页。现新增空闲ch33原roll、ch34 roll_applied（N）、ch35方向权重、ch36 phase、ch37阶段ms、ch38 fault；原LQR四输出为ch23～26，修正原6次读取4维数组越界。其他现行状态／目标／腿长／几何角布局保留，当前不是旧CAN39通道诊断页。


## 2026-10-06 · 翻倒恢复第一版（当前）

作者授权先收敛重复门控，再加入翻倒恢复。通信层汇总硬故障，world pitch无效／非有限属于FAULT_IMU；fallen只保持有效姿态的大俯仰检测，不再兼作输入失效标志。执行层Robot_Control_Enable_Allowed统一判使能，Control_Frame_Read每控制拍复制同一IMU、两腿、遥控和轮速，并生成drive／normal／recovery许可。正常LQR、RL仍不在fallen状态输出；LQR且自起／recovery_enabled开启时允许因姿态fallen进入恢复，CAN／电机／IMU／遥控硬故障仍拒绝。最终输出额外读取最新ctrl_fault／总开关一次，统一检查六路力矩有限性；不重新扫描设备。

固定K表／机器几何／周期匹配的LQR_Gain_Compatible首次检查后缓存；纯LQR_Gain_Check接口保留，动态K求值结果仍逐次校验。机器与选中表是编译期只读配置；更换表／机器须重新编译重启。参数指针、雅可比有效和计算新结果的检查仍按模块契约保留，没有把数值异常当成姿态恢复。

姿态从现有映射四元数计算upright（正立+1、倒置−1）和侧向重力幅值side，不改输入极性。较大俯仰／仰卧进入恢复；侧向分量过大仍标记SIDE，但不以此直接锁存停止；正立侧偏沿原自起路径并行纠偏，upright<0则进入翻倒恢复。未实现独立90°侧躺扶正动作。倒置Euler roll可能换分支，不用该值直接中断俯仰翻身；大姿态阶段也不套用正常Euler roll差动力。

| phase | 动作 | 转段 |
|---|---|---|
| 7 SETTLE | 目标追平实测、受限补偿／角速度阻尼；轮零 | 默认0.2s后TUCK |
| 8 TUCK | 保持进入几何角，长度目标限速向recovery_len推进（现为伸腿目标） | 按recovery_tuck_time后FLIP，不等待长度到位 |
| 9 FLIP | 两腿沿几何正方向、连续角目标2rad/s推进，继续缓慢调到短腿目标 | 身体进入支撑窗口、两腿世界角都在窗口，保持后SUPPORT |
| 10 SUPPORT | 扫腿停止，几何目标限速归到站立目标，长度目标缓慢向0.20m建立支撑 | 世界pitch≤0.35rad、roll≤原roll_ready、两腿世界摆角≤0.7rad保持后转原RETRACT／SWING |


FLIP进入支撑窗口使用pitch≤0.8rad、roll≤0.3rad、两腿世界角≤0.7rad和upright>0；有效保持期间冻结扫腿目标，避免继续推过窗口。机械卡住只有在角目标误差>0.5rad、反馈角速度<0.1rad/s且回正程度没有改善时计时，0.3s后回TUCK重贴实测再尝试。目标累计扫满2π或每阶段超过4s也有限重试，默认两次重试；不是无限加力。硬故障期间不重试；反馈恢复且许可连续成立后才允许有限重启。重试耗尽或阶段配置错误仍需退出投入位置复位。

新恢复阶段虚拟控制F另限±150N，Tp沿原40N·m，最终电机原限幅；气弹簧补偿仍只加一次。roll纠偏在各阶段并行计算，只在upright>0时应用，沿用腿支撑方向权重；倒置时不使用存在分支歧义的Euler roll差动力。长度／角目标每阶段贴实测并预置D历史。正常正立的6→5→1→2→3流程与用户当前25/20→5/5、后摆−1.5、4.8rad/s、roll力速率10N/s均保持。

翻身后不重走伸0.30／上绕，直接原收腿／摆正，随后原0.35s输出渐变接LQR；接管事件重置受翻身拖地污染的LQR估计器及目标，保存并恢复lqr_debug运行时设置，不改变其符号／调试限幅。recovery_enabled默认1，失能时设0可退回原翻倒失能路径；普通standup.enabled开关保留。

当前39通道：ch27为姿态分类（0正常、1翻倒、2侧躺、3非法），ch28为upright；ch36 phase可见7～10，ch37阶段ms、ch38 fault；原几何角ch31/32、实际长度ch29/30、roll及差动力ch33/34保持。现LQR状态页在恢复时属于持续估计／旧候选，不能把ch23～26当作恢复实际力矩，目标与F/Tp可Watch读取standup_control。

逻辑测试只用脚本反馈推动姿态窗口，确认模式／阶段／故障与输出规则，不代表机体已经物理翻回。真实碰撞、侧躺、单向扫腿效果和控制预算仍待台架。


## 2026-10-06 并行纠偏与有限重试

不新增阶段或VOFA通道。DONE（含渐入）中的腿摆角、世界pitch或roll持续超出对应trigger阈值后重新自起，取消运动命令必须归零的限制。恢复路径轮组输出为零，运动命令不参与准备阶段；正常LQR输入、K表与物理映射不改。roll仍通过现有左右F差动力和统一雅可比分解，腿未到后点也计算；支撑权重为零时不强行产生纠偏力，不能保证无接触侧躺回正。

FAILED中输入有效且permit连续成立retry_wait后，重贴实测目标、预置PID历史并重新按当前姿态选路径。每轮成功平衡前共用recovery_retry_max预算，跨Reset和转段保留；超时、短暂输入中断或门控中断可有限重试，BAD_CONFIG与耗尽预算保持停止。硬故障仍由唯一公共许可禁止输出，故障消失后才计等待时间；失败等待期间统一零输出，不清除驱动故障。正常DONE渐入完成且姿态无恢复请求时清除重试计数。allow_restart接口参数保留兼容，当前不再限制姿态恢复。

并行纠偏幅值／速率保持作者现值，本次未调PID。仅脚本反馈和编译核验，侧偏纠正能力、碰撞与翻倒动作须实测。


2026-10-06 参数排版：standup_param分组排列，每项单行标注单位。逐项核对保持文件最新字段和值，不改控制逻辑。


2026-10-06 速度观测互锁：只有LQR投入且自起DONE（或自起模块关闭）时更新速度KF和打滑确认；自起中暂停惯性预测及轮速修正，避免悬空轮速污染交接。交接使用零速先验而非空转轮速；这是受限先验，不保证机体仍滑动时立即准确。100ms只屏蔽打滑标志，观测降权始终生效。原恢复LQR全状态初始化后再应用同样交接规则。


2026-10-06 解耦：速度互锁转到task_actuation的Control_State_Update，独立slip.c实现融合、标志及重置。LQR仅组装原观测与消费任务赋入的速度，旧LQR_Velocity_Mode接口已移除；交接含义不变。


2026-10-07：作者观察短腿不能形成有效撑翻力臂，指定翻倒恢复recovery_len改为0.30m，recovery_tuck_time注释改为伸腿。字段与阶段名TUCK保留兼容；定时转段不变，伸腿0.1s后开始扫腿，FLIP中长度继续限速伸向0.30m，并非实测伸到0.30m才扫腿。SUPPORT仍采用原recovery_support_len，正立自起retract_len与其余参数未改。


2026-10-07：按作者要求移除recovery_len_rate。SETTLE仍追平实测；伸腿准备/TUCK与FLIP直接给机械区间夹紧后的recovery_len，SUPPORT直接给recovery_support_len。摆角目标限速、0.1s定时转段、PD历史预置和原限幅保持，目标直给不代表实测伸到位才扫腿。


2026-10-07：按作者确认移除SUPPORT阶段与recovery_support_len/recovery_support_pitch。FLIP在原回正窗口（世界pitch<=0.8rad、正立侧、roll和世界腿角到位）持稳后直接按实测腿长/后点选择原EXTEND/REAR/RETRACT/SWING；保留recovered事件，正常LQR接入仍重置估计。唯一回正pitch参数recovery_ready_pitch沿用原FLIP的0.8rad窗口，不再额外等待旧SUPPORT的0.35rad。无0.20m过渡目标；普通起立retract_len保持原值。


2026-10-07：翻身退出不再要求左右腿世界角接近平衡方向，删除recovery_ready_leg。仅正立侧、世界pitch回正窗口、原roll_ready与stable_time确认；退出后原正常起立流程按实际腿位置选择准备动作。后点rear_angle及公共几何解算未改。
