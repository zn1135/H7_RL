function ok = run_all(machine_name)
%RUN_ALL  LQR 增益表入口: 选机器 / 模型 → Q/R → build_gain_table 出表 → 通过则自动覆盖板上 C
%   run_all                  用 §1 的选择
%   run_all('big_wheelleg')  临时换机器
% 产物: output/lqr_gain_table.c  → 闭环检查通过且 CFG.deploy=true 时自动覆盖 imcalib/Algorithm/lqr_gain_table.c,
%                                   之后整项目编译上板 (.h 不用动, 函数签名从未变过)
%       output/report_latest.txt → 本次全部输出 (每次覆盖; 拷上板的表手动另存 report_<表号>.txt)
% 日常只改本文件: §1 选择、§2 Q/R (数值会原样写进 C 文件头, 表号 = 机器-模型-时间)
% 机械参数在 machine_table.m; 扫描 / 拟合 / 闭环检查 / 写 C / 部署在 build_gain_table.m; 动力学在 model_AB.m
% 状态序 x = [s ds phi dphi th_ll dth_ll th_lr dth_lr th_b dth_b], 输出序 u = [T_wl T_wr T_bl T_br]

%% ============ §1 选择 ============
CFG.machine   = 'small_wheelleg';    % small_wheelleg 小机器 | big_wheelleg 大机器
CFG.model     = 'sjtu5';    % 'sjtu5' Leg2 5 方程线性模型 (板上现表来源) | 'newton15' 自研 15 方程 (阶段 2)
CFG.fit_order = 2;          % 2 = poly22 (板上现表格式) | 3 = poly33 (残差不够时用; 多项式在生成的 C 里, 板上无需改)
CFG.deploy    = true;       % true = 闭环检查通过后自动覆盖 imcalib/Algorithm/lqr_gain_table.c (未通过不动板上文件)
if nargin >= 1 && ~isempty(machine_name), CFG.machine = machine_name; end

%% ============ §2 Q/R  (数值原样写进 C 文件头, 不需要另起标签) ============
switch CFG.machine
    case 'small_wheelleg'
        %    位移   速度  偏航角     左摆角         右摆角       pitch
        %      s     ds   phi  dphi th_ll dth_ll th_lr dth_lr th_b  dth_b
        q = [16000, 1200, 1000, 870, 11000,  250,  11000,  250, 7000, 500];   % Leg2 mlx 生效组 = 板上现表
        r = [5480, 5480, 650, 650];                                          % T_wl T_wr T_bl T_br
    case 'big_wheelleg'
        q = [600, 1000, 5000, 80, 15, 2, 15, 2, 90000, 500];                 % 旧脚本 lqr_numeric.m, 未上过板
        r = [50, 50, 1, 1];
    otherwise
        error('run_all:machine', '未知机器 "%s" (small_wheelleg / big_wheelleg)', CFG.machine);
end

%% ============ 出表 ============
addpath(fileparts(mfilename('fullpath')));
ok = build_gain_table(CFG, q, r);
end
