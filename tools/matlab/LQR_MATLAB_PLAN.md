# LQR 增益表 MATLAB 管线 — 计划（tools/matlab/）

> 状态：**阶段 0 已完成（2026-09-22，变更 85），同日精简为四个代码文件（变更 87、88：常改 / 不常改 / 不该改分开），阶段 1 未开工**。运行方式见 §十二。
> 作者决策（2026-09-21）：新建一套完整管线，不改 `leg_matlab_script`；先用 Leg2 模型复现现表并调参验证可用，再换成自研 15 方程模型；参数用"机器配置表"，小机器/大机器一处切换，与板上 `machine_config.c` 同思路。
> 文中所有数值都是 2026-09-21 的快照，以对应脚本/代码为准。

---

## 一、目标与边界

**目标**：让"改 Q/R → 出 K 表 → 核对 → 上板"这条路一条命令跑完、可复现、可追溯，并且能在两台机器、两个动力学模型之间切换。

**边界**：
- 产出物只有一个：与现 `imcalib/Algorithm/lqr_gain_table.c` **同签名、同下标**的 C 文件。板上求值器 `LQR_Control_Update()`、拟合域宏 `LQR_K_LEN_MIN/MAX` 在阶段 0~2 一行不动。
- 本管线不碰极性、零点、轴向、量程（AGENTS §0.1）。它输出的是增益，输入的机械参数里"待实测"项只能作者定。
- `leg_matlab_script`（大机器旧脚本）和 `Leg2_v1/轮腿上交建模MATLAB`（上交原件）只作来源与对照，不改、不删。

---

## 二、背景：现表从哪来，两份 MATLAB 差在哪

### 2.1 板上现表的来源

`imcalib/Algorithm/lqr_gain_table.c` 的生成时间 2026-07-27 15:02:48，Leg2 `LQR_K_WBR.m` 15:02:39，`WBR_modeling.mlx` 保存于同日 15:02。三者时间连续，**现表 = `WBR_modeling.mlx` 用其文件内那组 Q/R 生成**：

```
Q = diag([16000 1200 1000 870 2500 365 2500 365 10500 2000])   % s ds phi dphi th_ll dth_ll th_lr dth_lr th_b dth_b
R = diag([5480 5480 650 650])                                   % T_wl T_wr T_bl T_br
```

PSO 脚本里的 `Q0=[6000 2000 1 800 …]` 是另一组基线，没上过板。

### 2.2 两份 MATLAB 的主要区别

| 项 | `leg_matlab_script`（自研，大机器） | Leg2 `轮腿上交建模MATLAB`（小机器，现表来源） |
|---|---|---|
| 物理模型 | 15 条牛顿-欧拉方程消元成 5 条，含机体质心偏置 `l_c/phi_c`、腿质心离轴 `delta`（`eq2ss.m`） | 上交论文 (3.11)~(3.15) 直接写好的 5 条线性方程，腿质心假设在腿轴线上，无 `phi_c` |
| 状态/输入顺序 | `[s ds phi dphi th_ll dth_ll th_lr dth_lr th_b dth_b]`、`[T_wl T_wr T_bl T_br]` | 同 |
| 机器参数 | `R_w 0.06`、`R_l 0.221`、`m_b 20`、`I_b 0.5+m_b*x_c^2` 等；腿数据 9 点 0.11~0.33 m 带 `delta` 列 | `R_w 0.04`、`R_l 0.3459`、`l_c 0.0501`、`m_b 5.25`；腿数据 11 点 0.13~0.23 m 无 `delta` |
| 腿参数进模型 | poly3 拟合后按腿长求值；`lw_y + lb_y = l` 逐点成立，几何自洽 | 直接按网格下标取 `Leg_data(il,:)`；`leg_fit.mlx` 的 poly4 拟合没被主循环使用；表内 `l_wl + l_bl ≠ L0`（0.13 m 时和为 0.216），0.23 行 `l_wl` 倒退 |
| 腿长网格 | 二维 0.11:0.01:0.33，529 点 | 二维 0.13:0.01:0.23，121 点 |
| A/B 数值化 | 每格点 `subs`+`vpa`，很慢 | mlx 也是 `subs`；PSO v2 改为 `matlabFunction` 缓存 `AB_WBR_gen.m`，121 点秒级 |
| 离散化/求解 | `c2d` Ts=0.001 + `dlqr` | 同（PSO 调参用连续 `lqr`） |
| Q/R | `[600 1000 5000 80 15 2 15 2 90000 500]` / `[50 50 1 1]` | 见 §2.1 |
| K 拟合 | `poly33`，10 项/元素 | `poly22`，6 项/元素 |
| 输出 | `wholebody.hpp` 数组 `kMatureLqrPoly_whole[4][10][10]` + `A_L/B_L/U_d` 表，板上不认 | `matlabFunction` → `LQR_K_WBR.m` → Coder → `LQR_K_WBR.c`，接口 `LQR_K_WBR(lL,lR,K_sym[40])` |
| 附加工具 | 无 | PSO v1/v2（v2 带 ZOH+一拍延迟+饱和+电机滞后的"真实层"仿真与极点整形）、VMC 三函数、`check.slx` |
| 已知小毛病 | `lqr_numeric.m` 的 `Ts = meta.step` 取的是腿长步距（未用到但误导）；`g=9.8` | `R_w_ac=0.03` 组是被注释掉的另一台车；`leg_fit.mlx` 内还有第三套 0.06~0.23 腿数据，与 mlx 用的不是一回事 |

**符号约定两边一致**（读式子得出，阶段 2 用数值比对兜底）：`phi = -R_w/(2R_l)(th_wl-th_wr)+…` 左转为正；`S_b` 中 `l*sin(th)` 与 `l_c*sin(th_b+phi_c)` 同号，即"θ 正 = 机体相对轮向前"，对应板上 `x[th_ll] = -解算摆角 + pitch`、俯仰低头为正。

### 2.3 为什么不能直接拿一份改参数

- 拿 Leg2 mlx 改参数：参数、Q/R、推导、导出混在一个 mlx 里，无法分机器、无法复现"某张表对应哪组参数"，且腿数据按下标取、不能换网格。
- 拿自研脚本换参数：模型不同、腿数据定义不同、输出格式板上不认，三处都要动，出问题无法归因。
- **Leg2 的 `Leg_data` 不能喂 15 方程模型**：Leg2 模型里 `l_wl/l_bl` 只以乘积或单独项出现，表内不自洽它自己不报错；15 方程模型会把 `l_wl + l_bl ≠ l` 解释成腿质心偏离轴线几十度，A/B 全错。

---

## 三、总体路线（作者定）

```
阶段 0  搭管线：Leg2 5 方程模型 + 小机器表 + mlx 那组 Q/R  →  复现现表（240 系数逐个对上）
阶段 1  用它调参：whatif + 真实层仿真 → 出表 → 核对 → 上板 → 记账；验证"可用"
阶段 2  接自研 15 方程模型：同参数比 A/B、比 K 趋势，过了才允许上板
阶段 3  文档收口
```

一次只换一个变量：阶段 0/1 不换模型，阶段 2 不换 Q/R。

---

## 四、目录与文件职责

放在本仓库 `CtrBoard-H7_ALL/tools/matlab/`（与 `tools/sysid_export.py` 同级）。放仓库里的理由：生成的 C 直接落到 `imcalib/Algorithm/`，git 把"哪张表 ↔ 哪组 Q/R ↔ 哪版固件"一起记住。

2026-09-22 作者定的分法（变更 86~88）：**按"多久改一次"分文件**，常改的一个、不常改的一个、验证过不该改的一个、物理模型一个。

```
tools/matlab/
├── LQR_MATLAB_PLAN.md      ← 本文件
├── run_all.m               ← 常改: §1 选机器 / 模型 / 拟合阶次 / 是否自动部署, §2 Q/R (数值原样进 C 文件头, 无标签), 然后一行调 build_gain_table
├── machine_table.m         ← 不常改: 两张机器表 (local / chuanliantui), 与 machine_config.c 对应; 末尾自检 + 待实测清单
├── build_gain_table.m      ← 验证过不改 (变更 85 复现): scan_grid 网格 → c2d(ZOH) → dlqr; fit_K / eval_K_poly poly22|33 最小二乘;
│                              check_closed_loop 全网格离散闭环谱半径 < 1; emit_gain_table 写 C; 通过且 CFG.deploy 则 copyfile 覆盖 imcalib/Algorithm/lqr_gain_table.c; 报告 diary 到 output/report_latest.txt
├── model_AB.m              ← 物理模型: [A,B] = model_AB(model, m, lL, lR); sjtu5 参数向量 / 腿质心取行 + Leg2 5 方程符号推导
│                              build_sjtu5 (首次约 13 s, 缓存后秒级); 阶段 2 的 newton15 在这里加 case
├── cache/                  ← AB_sjtu5_gen.m (matlabFunction 生成物)、scan_<机器>_<模型>.mat; 已进 .gitignore
└── output/
    ├── lqr_gain_table.c    ← 产物; 闭环通过时已自动复制到 imcalib/Algorithm/ (CFG.deploy=true), 整项目编译即可
    ├── report_latest.txt   ← 最近一次输出的副本 (diary: 命令行窗口照常显示, 同时抄一份; 每次覆盖)
    └── report_<表号>.txt   ← 只有拷上板的表才另存一份 (手动复制 report_latest), 随表号记进 sysid-change-map.md
```

已删（2026-09-22，作者定"对比过一次就够"）：与 Leg2 原表的复现对照、与板上 `machine_config.c` 的字段比对、AC5 单文件编译检查、按名字取表的 `machine_load`、`ref/` 对照件。阶段 0 的对照证据留在变更 85 与 `output/report_local-sjtu5-20260922-0945.txt`。**代价**：板上机器表（轮径、杆长、区间、限幅）改了，`machine_table.m` 要手动跟着改，没有脚本再替你查。

---

## 五、关键设计

### 5.1 机器配置表（与 `machine_config.c` 同思路）

两张机器表是 `machine_table.m` 里的两个 case，`run_all.m` §1 `CFG.machine` 选哪台。字段分四组，每个数值字段可挂 `status`：`'实测'` / `'Leg2'` / `'旧脚本'` / `'待实测'`。导出 C 的文件头会列出所有 `'待实测'` 项。

| 组 | 字段 | 小机器 `local` 来源 | 大机器 `chuanliantui` 来源 |
|---|---|---|---|
| 机体 | `g, wheel_r, half_track(R_l), l_c, m_w, m_l, m_b, I_w, I_b, I_z` | Leg2 mlx 生效组：0.04 / 0.3459 / 0.0501 / 0.13463 / 0.949 / 5.25 / 7.3294e-5 / 0.028669861 / 0.084144108 | `leg_param.m`：0.06 / 0.221 / sqrt(0.015²+0.01²) / 0.3 / 2.0 / 20 / 0.008 / 0.5+20·0.015² / 0.7；全部 `'旧脚本'` 或 `'待实测'` |
| 腿几何 | `leg.lu, leg.lg, leg.len_min, leg.len_max` | 对齐 `machine_config.c`：0.13087 / 0.15240 / 0.13 / 0.23（作者 2026-09-22 把区间从 0.09~0.21 改成 0.13~0.23） | 0.21 / 0.25 / 0.14 / 0.34 |
| 腿质心表 | `leg.data_sjtu5 = [L0 l_wl l_bl I_ll]`；`leg.data_newton15 = [l lw_y lb_y delta Ileg]` | sjtu5 抄 Leg2 `Leg_data` 11 点；newton15 **待作者定**（推荐按五连杆各杆质量 + 解算几何算，或 CAD 导出） | newton15 抄 `leg_param.m` 9 点；sjtu5 由 newton15 换算（`l_wl = sqrt(lw_y²+delta²)` 等） |
| 控制约束 | `ctrl.Ts, ctrl.T_wheel_max, ctrl.T_hip_max, ctrl.grid` | 0.001 / 1.8（`dji_trq_clamp`）/ 10（`dm_trq_clamp`）/ 0.13:0.01:0.23 | 0.001 / 4.8 / 20 / 待定 |

**与板上重叠、必须一致的字段**：`wheel_r`、`leg.lu`、`leg.lg`、`leg.len_min/max`、`T_wheel_max`、`T_hip_max`、`Ts`（= `CTRL_DT`）。这些字段板上改了要手动同步到 `machine_table.m`（比对脚本已删）。其余质量、惯量板上没有，只在 MATLAB 表维护。

**K 表域**：`ctrl.grid` 的首尾就是 `LQR_K_LEN_MIN/MAX`。阶段 0~2 固定 0.13~0.23 与板上宏一致；要换域先改宏，属代码改动，单独授权。

### 5.2 模型接口

```matlab
[A, B] = model_AB(model_name, m, lL, lR)   % A 10x10, B 10x4，连续时间
```

- 内部找 `cache/AB_<model>_gen.m`，没有就先跑 `model_AB.m` 里的 `build_<model>()` 生成（`matlabFunction`，参数向量顺序写死并注释）。
- 腿长 → 腿质心参数由 `model_AB.m` 里各模型自己的 `*_param_vec()` 负责（含取行 / 插值），外面只给 `lL, lR`。
- 加第三个模型 = `model_AB.m` 加一个 case 和一个 `build_*()`。

### 5.3 导出格式契约（与板上一致，逐字对齐）

- 签名：`void LQR_K_WBR(float lL, float lR, float K_sym[40])`
- 下标：`K_sym[状态*4 + 输出]`，状态 0..9 按 §2.2 顺序，输出 0..3 = T_wl T_wr T_bl T_br。`lqr_balance.c` 按 `K_sym[j*4+i]` 取成 `K[i][j]`，**不许换序**。
- 多项式：poly22，`K = p00 + p10*lL + p01*lR + p20*lL² + p11*lL*lR + p02*lR²`，用 `float` 字面量：先舍入到 single 再以 `%.9g` 打印加 `F` 后缀（与 Coder 一致，C 端解析回同一个 float）。多项式求值写在生成的 C 函数体内，板上只调 `LQR_K_WBR()`，**换阶次不需要改板上代码**。
- 文件头注释：机器名、模型名、Q/R、网格、Ts、拟合阶次、日期、表号、待实测项清单。
- include 行与现文件一致：`#include "lqr_gain_table.h"`；头文件不重新生成。

### 5.4 核对（只剩一项）

| 检查 | 判据 | 为什么留 |
|---|---|---|
| 闭环谱半径 `build_gain_table.m` 的 `check_closed_loop()` | 拟合后的 K 装回离散模型，全部 121 格点 `max|eig(Ad − Bd·K)| < 1`；同时打印拟合残差 | 唯一能在烧板前告诉你"这组 Q/R 出的表会不会站不住"的检查，零点几秒 |

未通过时 C 文件照样写出，但控制台和文件头都标"**未通过, 不要上板**"。

已删的检查及原因（作者 2026-09-22 定）：对照 Leg2 原表（阶段 0 已证 3.5e-11，后续调参 K 本来就不同）；对照板上 `machine_config.c`（对比过一次就够）；AC5 单文件编译（整项目编译时同样会发现）。

### 5.5 调参流程（阶段 1 起固定）

```
1. run_all.m §2 改 Q/R (不用起标签, 数值原样写进 C 文件头)
2. 跑 run_all → 命令行窗口看: 闭环谱半径 < 1, 标称腿长 K 没跑飞, 末行 "已覆盖 imcalib/Algorithm/lqr_gain_table.c"
   (未通过 → 板上文件不动, 末行 "不要上板")
3. 整项目编译、上板
4. sysid-change-map.md 记: 表号 (C 文件头第一行) + Q/R + 现象; 想留证据把 output/report_latest.txt 另存 report_<表号>.txt
```

表号 = 机器-模型-时间。CFG.deploy=false 时回到手动拷贝。`.h` 从不需要重生成（签名不变）。

---

## 六、分阶段任务与验收

### 阶段 0 · 搭管线并复现现表

固定：模型 `sjtu5`，机器 `local`，Q/R = §2.1 那组，网格 0.13:0.01:0.23，腿数据按最近行取（与 mlx 完全一致）。

任务：
1. `config/`：四个机器/模型文件 + `lqr_weights.m` + `machine_load.m`。（当日布局，变更 87 后已全部并进 `run_all.m`）
2. `model/sjtu5/`：搬 Leg2 5 条方程，`matlabFunction` 缓存。
3. `design/`：`scan_grid.m`、`fit_K.m`。
4. `emit/emit_gain_table.m`。
5. `check/` 四个脚本；`ref/` 放 Leg2 `LQR_K_WBR.m` 副本。
6. `run_all.m` 串起来，一条命令跑完。
7. `.gitignore` 加 `tools/matlab/cache/`。

验收：
- `check_vs_ref` 240 系数逐个对上。（该检查已于变更 87 删除，证据留在 `output/report_local-sjtu5-20260922-0945.txt`）
- 其余三个 check 通过。
- 板上代码零改动。

**结果（2026-09-22，表号 local-sjtu5-20260922-0945，变更 85；当日文件布局见变更 85，之后已精简为两个文件）**：A/B 与 Leg2 `AB_WBR_gen` 相对差 0；拟合 K 与 `ref/LQR_K_WBR.m` 在 121 个网格点最大相对差 3.5e-11、7 个非网格/非对称点 1.3e-11；生成的 C 与板上 C 在 21×21 点 × 40 元素上最大相对差 1.6e-8（float 舍入）；拟合 K 闭环谱半径最大 0.998929（折算 −1.07 1/s）；`check_machine_config`、`check_c_compile`（AC5 0 err 0 warn）通过。板上代码零改动。顺带看到：现表 poly22 对 dlqr 真值的拟合残差最大 0.026（相对 1.5%，位移→右髋一项），是现表自带的误差。

### 阶段 1 · 用它调参，验证可用

任务：
1. `sim_real()`（搬 PSO v2 `sim_wbr_real`，参数取机器表 `ctrl.*`，延迟拍数默认 1，电机滞后默认 0.003 s 可关）——放进 `whatif.m` 作局部函数。
2. `whatif.m`（第三个代码文件）：输入 Q/R 缩放或直接给 Q/R，输出标称腿长 K、连续/离散极点表、四个初值场景的响应图（俯仰 5°、0.1 m/s+3°、双腿 2°、8°+0.3 m/s）。
3. 表号机制已在 `run_all` 里（表号 = 机器-模型-时间，Q/R 数值在 C 文件头）。

验收：
- 至少一轮"改 Q/R → 上板 → 现象变化能归因"，记进 `sysid-change-map.md`。
- 绕圈问题的处理顺序：先用 VOFA ch3−ch4（偏航角误差是否收敛）、ch27（扶住不动偏航角速度是否为零）排除"环没起作用"和"零偏"，再动偏航角/角速度两列的 Q。

### 阶段 2 · 接自研 15 方程模型

前置：作者定小机器 newton15 格式腿质心表的来源（§5.1）。

任务：
1. `model_AB.m` 加 `build_newton15()`：搬 `eq2ss.m` 全部推导（15 方程 → 消元 → 线性化 → `A_orig/B_u/B_x` → `T_acc` → 拼 10 阶），`matlabFunction` 缓存。
2. `newton15_param_vec()`：断言 `lw_y + lb_y == l`（容差 1e-6）。
3. `check_model_pair.m`（临时脚本，比完可删）：同机器、`phi_c=0`、`delta=0`，比 sjtu5 与 newton15 的 A/B：符号全同，量级差异逐元素归到上交模型的简化项，写进 report。
4. 同 Q/R 出两张 K 表，比符号趋势与量级。

验收：
- A/B 比对符号全同；K 趋势一致。
- 两条都过，`run_all.m` §1 才允许切到 `newton15` 出表上板；上板仍只换一个文件。

### 阶段 3 · 文档收口

- 新建 `md/LQR_MATLAB.md`：目录、机器表字段、模型接口、导出契约、调参流程、"为什么"。
- `md/LQR_PLAN.md` §七替换成指向它；§1.1 "小机器重算增益必须使用 mlx" 改为指向本管线。
- `md/AGENTS.md` 文件树加 `tools/matlab/`。
- 本文件 §十 进度表补齐。

---

## 七、明确不做的

- 不改 `lqr_balance.c`、`LQR_K_LEN_MIN/MAX`；换域是单独一次授权。升 poly33 只需 `run_all.m` §1 `CFG.fit_order = 3`，板上无需改（多项式在生成的 C 里）。
- 不把 Leg2 `Leg_data` 直接喂 15 方程模型。
- 阶段 0/1 不换模型，阶段 2 不换 Q/R。
- 不删、不改 `leg_matlab_script` 与 Leg2 原件。
- 不用 MATLAB Coder；不生成 `A_L/B_L/U_d` 表（板上无人用）。
- 不在本管线里做 PSO 自动整定（PSO v2 的仿真器可以搬，搜索本身等手动调参跑通后再议）。

---

## 八、已知风险与取舍

| 项 | 风险 | 处理 |
|---|---|---|
| Leg2 `Leg_data` 几何不自洽 | sjtu5 模型下不报错，但它描述的"腿"和实物有出入 | 阶段 0/1 照用以复现现表；阶段 2 用自洽表，比对时把差异归因 |
| 腿数据按最近行取 vs 插值 | mlx 是按下标取；换插值 K 会微变 | 阶段 0 用最近行保证复现；之后可切插值，作为独立变量记录 |
| poly22 残差 | 现表本身对 dlqr 真值的拟合残差就有 max 0.026（相对 1.5%）；新 Q/R 下可能更大 | 报告里打印；不够就 `CFG.fit_order = 3`，板上无需改 |
| 大机器参数 | 多数是旧脚本值、未实测 | 表内 `status='待实测'`，导出文件头列出；大机器出表只作预研 |
| 板上机器表与 MATLAB 表漂移 | 比对脚本已删（作者定），轮径 / 杆长 / 区间 / 限幅两边改了没人查 | 改 `machine_config.c` 这几项时同步改 `machine_table.m`；`LQR_PLAN.md` 决策 8 提醒 |
| MATLAB 版本 | 本机 R2024b，Control / Symbolic 工具箱都有 | `fit_K` 用最小二乘反斜杠不依赖 Curve Fitting；`build_sjtu5` 首次需 Symbolic（约 13 s，之后走缓存）；`dlqr/c2d` 需 Control |

---

## 九、待作者决定

| # | 事项 | 推荐 | 何时需要 |
|---|---|---|---|
| 1 | 小机器 newton15 格式腿质心表来源 | 按五连杆各杆质量 + 解算几何算（自洽、可复现）；有 CAD 就用 CAD | 阶段 2 前 |
| 2 | 腿数据按最近行取还是插值 | 阶段 0 最近行，阶段 1 起可切插值 | 阶段 1 |
| 3 | 大机器 K 表域 `ctrl.grid` | 待 `leg_len_min/max` 实测后定 | 大机器出表前 |

---

## 十、进度

| 阶段 | 状态 | 日期 | 备注 |
|---|---|---|---|
| 计划 | ✅ 本文件 | 2026-09-21 | 作者确认路线：新建目录、先 Leg2 模型、再 15 方程、机器表可切换 |
| 0 搭管线复现现表 | ✅ 完成 | 2026-09-22 | 五项核对全过，数值见 §六 阶段 0 结果（变更 85）；同日三轮精简，最终 `run_all` / `machine_table` / `build_gain_table` / `model_AB` 四个文件、只留闭环检查（变更 86~88），每轮重跑生成的 40 行 K 逐字节相同 |
| 1 调参验证可用 | ⚪ 未开工 | | |
| 2 接 15 方程模型 | ⚪ 未开工 | | 前置：§九 #1 |
| 3 文档收口 | ⚪ 未开工 | | |

---

## 十一、来源文件索引

| 用途 | 路径 |
|---|---|
| 现表（板上） | `CtrBoard-H7_ALL/imcalib/Algorithm/lqr_gain_table.c/.h` |
| 板上求值与状态定义 | `CtrBoard-H7_ALL/imcalib/Algorithm/lqr_balance.c/.h` |
| 板上机器表 | `CtrBoard-H7_ALL/imcalib/user-lib/machine_config.c/.h` |
| Leg2 建模主脚本 | `Leg2_v1(1)/Leg2_v1/轮腿上交建模MATLAB/WBR_modeling.mlx`（zip 内 `matlab/document.xml` 的 CDATA） |
| Leg2 A/B 缓存写法 | 同目录 `AB_WBR_gen.m`、`PSO_LQR_Tuning_v2.m` Section 2 |
| Leg2 真实层仿真 | `PSO_LQR_Tuning_v2.m` 的 `sim_wbr_real` |
| Leg2 原表 MATLAB 版 | 同目录 `LQR_K_WBR.m` |
| 自研 15 方程推导 | `leg_matlab_script/eq2ss.m` |
| 自研大机器参数 | `leg_matlab_script/leg_param.m` |
| 自研扫描/拟合/导出 | `leg_matlab_script/leg_scan.m`、`lqr_numeric.m` |

---

## 十二、运行方式

MATLAB 桌面：`cd tools/matlab`，敲 `run_all`。命令行（不开桌面；控制台里的中文会因编码显示成乱码，看 `-logfile` 或 `output/report_latest.txt`）：

```
"D:\matlab\bin\matlab.exe" -wait -batch "cd('<仓库>/tools/matlab'); ok = run_all(); exit(double(~ok))" -logfile "<仓库>/tools/matlab/run_all.log"
```

- 退出码 0 = 闭环检查通过。`run_all('chuanliantui')` 临时换机器。
- 首次运行 `build_sjtu5` 做符号推导约 13 s；之后 `cache/AB_sjtu5_gen.m` 存在，全程约 3 s。桌面里跑，所有输出都在命令行窗口，`report_latest.txt` 只是副本。
- 改 Q/R：`run_all.m` §2，不用起标签，数值原样写进 C 文件头，账本靠表号 + 文件头对表。
- 换机器 / 模型 / 拟合阶次：`run_all.m` §1。
- 板上机器表改了轮径、杆长、区间、限幅：手动同步 `machine_table.m`。
- 上板：闭环通过后把 `output/lqr_gain_table.c` 拷到 `imcalib/Algorithm/` 覆盖，整项目编译；同时把 `output/report_latest.txt` 另存为 `report_<表号>.txt`，`sysid-change-map.md` 记"表号 + Q/R + 现象"。
