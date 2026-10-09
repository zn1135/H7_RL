function ok = run_all(machine_name)
%RUN_ALL  LQR 增益表入口: 选机器 / 模型 → Q/R → build_gain_table 出表 → 生成对应机器候选表
%   run_all                  用 §1 的选择
%   run_all('big_wheelleg')  临时换机器
% 产物: output/lqr_gain_small.c / lqr_gain_big.c  → 闭环检查通过且 CFG.deploy=true 时自动覆盖 imcalib/Algorithm/lqr_gain_<机器>.c,
%                                   之后整项目编译上板 (.h 不用动, 函数签名从未变过)
%       output/report_small/big_latest.txt → 各机器报告；report_latest.txt 为最近一次报告
% 日常只改本文件: §1 选择、§2 Q/R (数值会原样写进 C 文件头, 表号 = 机器-模型-时间)
% 机械参数在 machine_table.m; 扫描 / 拟合 / 闭环检查 / 写 C / 部署在 build_gain_table.m; 动力学在 model_AB.m
% 状态序 x = [s ds phi dphi th_ll dth_ll th_lr dth_lr th_b dth_b], 输出序 u = [T_wl T_wr T_bl T_br]

%% ============ §1 选择 ============
CFG.machine   = 'big_wheelleg';    % small_wheelleg 小机器 | big_wheelleg 大机器
CFG.model     = 'sjtu5';    % 'sjtu5' Leg2 5 方程线性模型 (板上现表来源) | 'newton15' 自研 15 方程 (阶段 2)
CFG.fit_order = 2;          % 2 = poly22 (板上现表格式) | 3 = poly33 (残差不够时用; 多项式在生成的 C 里, 板上无需改)
CFG.deploy    = true;      % 显式开启后仅部署对应机器表
if nargin >= 1 && ~isempty(machine_name), CFG.machine = machine_name; end

%% ============ §2 Q/R  (数值原样写进 C 文件头, 不需要另起标签) ============
% Q = diag(q): 状态误差平方的权重, 顺序对应 x; R = diag(r): 力矩平方的权重, 顺序对应 u。
% q(1)  s       髋轴中点前进位移 [m]
% q(2)  ds      髋轴中点前进速度 [m/s]
% q(3)  phi     偏航角 [rad]
% q(4)  dphi    偏航角速度 [rad/s]
% q(5)  th_ll   左虚拟腿世界系前摆角 [rad]
% q(6)  dth_ll  左虚拟腿世界系摆角速度 [rad/s]
% q(7)  th_lr   右虚拟腿世界系前摆角 [rad]
% q(8)  dth_lr  右虚拟腿世界系摆角速度 [rad/s]
% q(9)  th_b    机身俯仰角 pitch [rad]
% q(10) dth_b   机身俯仰角速度 [rad/s]
% r(1)  T_wl    左轮力矩 [N*m]
% r(2)  T_wr    右轮力矩 [N*m]
% r(3)  T_bl    左虚拟髋力矩 [N*m], 经力映射分配给左腿两个关节电机
% r(4)  T_br    右虚拟髋力矩 [N*m], 经力映射分配给右腿两个关节电机
% 固件腿状态: th_l/r = virtual_leg_angle - pitch, dth_l/r = d_virtual_leg_angle - pitch_rate。
% 其余条件相同时, 增大某项 q 更重视该状态误差, 增大某项 r 更惩罚对应力矩。
switch CFG.machine
    case 'small_wheelleg'
        q = [16000, 3000, 1000, 870, 11000,  250,  11000,  250, 7000, 500];   % Leg2 mlx 生效组 = 板上现表
        r = [5480, 5480, 650, 650];                                          % T_wl T_wr T_bl T_br

% 状态序 x = [s ds phi dphi th_ll dth_ll th_lr dth_lr th_b dth_b], 输出序 u = [T_wl T_wr T_bl T_br]

    case 'big_wheelleg'
        % hx
        % q = [300, 1, 4000, 1, 600, 10, 600, 10, 55000, 1];             % Leg3 候选权重
        % r = [100, 100, 1, 1];  
        % kp
        % cfg.Q = diag([600,  1000, 25000,   19000,     3200,        750,      3200,   750,     25000,   5000]);
        % cfg.R = diag([430.0,  430.0,   500.0,    500.0]);
        q = [600,  800, 30,   100,     100,        210,      100,   210,    8000,   10];            
        r = [100.0,  100.0,   10.0,    10.0];                                           
        CFG.fit_order = 3;
    otherwise
        error('run_all:machine', '未知机器 "%s" (small_wheelleg / big_wheelleg)', CFG.machine);
end

%% ============ 出表 ============
addpath(fileparts(mfilename('fullpath')));
ok = build_gain_table(CFG, q, r);
end
