# Leg3 气弹簧补偿

2026-10-02：作者确认 Leg3_v2 是同一机器的另一套控制方案，授权直接使用其气弹簧参数。已删除线性缩放、旧端点模型、多模式状态、参数锁存和专用 VOFA 页；不移植 Leg3 的 PID、LQR、重力补偿或状态机。

## 两个接口

- `float Leg_SpringF(float L0)`：输入实测腿长 m，返回沿伸腿方向的弹簧等效力 N。使用 Leg3 的原 C 常量及 CMSIS 三角/开方计算，模型为 `Fs = 150 N * ds/dL`；挂点几何和大小腿参数直接采用同机来源。有限输入按原函数钳位到 0.05～0.45 m；NaN/Inf 返回 NaN，避免伪造有效反馈。
- `Gas_Spring_Apply(left, right, base_dm, raw_dm)`：把左右 `−Leg_SpringF(L)` 经本机 `Leg_Force_Map_Forward(F,0)` 转为四髋增量，执行 `raw_dm = base_dm + delta_dm`。只补 F，不增加 Tp；没有内部限幅或电机发送。支持原地数组；启用时任一腿解算/映射无效或出现非有限量，则返回 0 并将四个输出全部清零。

两个控制器均为“原基础髋力矩 → 调用 Apply → 原电机限幅 → 原分发”。两轮控制保持原值，LQR 的 `lb->F` 恢复记录基础控制力；JID1 不经过补偿。

## 开关与机型

`gas_spring.h` 的 `GAS_SPRING_COMP_ENABLE` 默认 0，保持此前的关闭状态；需要投入时设为 1 后重新编译。只有大机器配置使用这套同机模型，小机器即使宏为 1 也保持原基础力矩。关闭时 Apply 直接复制有限基础力矩，不增加力域几何要求。

不再存在 `gas_spring.requested`、`scale`、`gain_n_m` 或补偿页 `-2031`。原普通 VOFA 和策略追踪保持原样。机器表中遗留的 `gas_spring_force_n/gas_comp_sign` 不再参与这个计算，未改动其数值或其他物理量。

## 来源与核验

来源：作者提供的 `Leg3_v2/Wheel_Leg/Code/Math/Leg_SpringF.c`，可读参数和推导在 `Leg3_v2/Matlab/Tool/Leg_SpringF.m`。模型正值代表弹簧助伸，适配层沿用其减去 Fs 的补偿语义；现有电机极性、零点与量程均保持原值。

`tests/test_gas_spring.py` 核对模型与适配层；`tests/test_gas_spring_integration.py` 执行真实 RL/LQR/PID/分发，覆盖大小机和开关组合、仅补 F、合成后限幅及错误零输出。源码对拍使用相同 libm 数学替身，不等于 MCU 时序或实机验证。编译结果见变更记录；本次未烧录。
