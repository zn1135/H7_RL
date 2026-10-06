# 独立自起模块

`imcalib/Algorithm/standup.c/h` 是唯一自起模块，沿原执行任务1kHz运行；不新增任务、反馈解算或通信入口。范围仍是机体基本水平、腿在前／后方的起立，完整倒置／侧翻恢复未开放。

## 公共输入与参数

自起直接只读 `imu_state`、`leg_l`、`leg_r`；公共腿解算、轴映射、零点与极性保持。世界 `pitch_world/pitch_world_valid` 只判断是否需要自起、允许准备范围及回正到位；自起控制摆角改为公共几何腿角 `virtual_leg_angle`，摆角速度为对应 `d_virtual_leg_angle`，不再减原pitch／gyro。角度误差用最短路径环绕，没有额外反馈反号。

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

世界pitch无效／IMU离线或超过1.4rad仍由原公共fallen门控失能，小于1.0rad解除，迟滞保留；自起没有绕过保护。自起自身失败保持零力矩，不另发失能命令。反馈恢复不清锁存；有效遥控退出右中、左下或切模式清本次状态。

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
    .extend_len = 0.30f, .extend_tol = 0.01f, .extend_timeout = 4.0f,
    .rear_angle = -1.3f, .rear_tol = 0.3f, .rear_timeout = 4.0f,
    .rear_rate = 4.239f,
    .retract_len = 0.15f, .retract_ready_len = 0.19f,
    .angle_pos_kp = 25.0f, .angle_pos_kd = 0.0f,
    .angle_speed_kp = 5.0f, .angle_speed_kd = 0.0f,
    .angle_speed_max = 10.0f,
    .tp_max = 40.0f,
    .trigger_angle = 50.0f * LEG_PI / 180.0f,
    .trigger_pitch = 40.0f * LEG_PI / 180.0f,
    .angle_tol = 0.3f, .length_tol = 0.04f,
    .pitch_max = 1.0f, .roll_max = 0.75f,
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
