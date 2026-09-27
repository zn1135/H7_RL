function m = machine_table(name)
%MACHINE_TABLE 机器参数表 — 与 machine_config.c 两份表对应, 不常改
%   m = machine_table('small_wheelleg') | machine_table('big_wheelleg')
% 板上 machine_config.c 改了轮径 / 杆长 / 腿长区间 / 限幅, 这里要手动跟着改 (比对脚本已删)。
% status: 实测 / Leg2 / 旧脚本 / 待实测 ('待实测' 会打进导出 C 的文件头)
switch name
    case 'small_wheelleg'   % ---------- 小轮腿 M2006/J4310 = machine_config.c [MACHINE_ID_SMALL_WHEELLEG] ----------
        m.name = 'small_wheelleg';  m.c_id = 'MACHINE_ID_SMALL_WHEELLEG';
        % 机体 (Leg2 WBR_modeling.mlx 生效组, 板上现表就是这组生成的)
        m.body.g          = 9.81;
        m.body.wheel_r    = 0.04;           % 驱动轮半径 [m]          = .wheel_r
        m.body.half_track = 0.3459;         % 两轮中心距/2 [m]        (模型 R_l)
        m.body.l_c        = 0.0501;         % 机体质心到髋轴 [m]
        m.body.m_w        = 0.13463;        % 单轮质量 [kg]
        m.body.m_l        = 0.949;          % 单腿质量 [kg]
        m.body.m_b        = 5.25;           % 机体质量 [kg]
        m.body.I_w        = 0.000073294;    % 轮转动惯量 [kg m^2]
        m.body.I_b        = 0.028669861;    % 机体俯仰惯量 [kg m^2]
        m.body.I_z        = 0.084144108;    % 整机偏航惯量 [kg m^2]
        % 腿几何 (= machine_config.c)
        m.leg.lu = 0.13087;  m.leg.lg = 0.15240;         % 上杆 / 下杆 [m]
        m.leg.len_min = 0.13;  m.leg.len_max = 0.23;     % 机器腿长区间 [m] (作者 2026-09-22 定)
        % 腿质心表 sjtu5 格式 [L0 l_wl l_bl I_ll]: 腿长 / 轮轴到腿质心 / 髋到腿质心 / 腿惯量
        % (Leg2 Leg_data 11 点; l_wl + l_bl ≠ L0, 与 sjtu5 模型自洽但不能喂 newton15)
        m.leg.data_sjtu5 = [
            0.13, 0.13301, 0.08290, 0.011777385;
            0.14, 0.13348, 0.08814, 0.012884469;
            0.15, 0.13403, 0.09345, 0.014075467;
            0.16, 0.13465, 0.09884, 0.015350323;
            0.17, 0.13534, 0.10427, 0.016709084;
            0.18, 0.13610, 0.10975, 0.018151806;
            0.19, 0.13694, 0.11527, 0.019678555;
            0.20, 0.13784, 0.12083, 0.021289413;
            0.21, 0.13880, 0.12641, 0.022984478;
            0.22, 0.13984, 0.13202, 0.024763878;
            0.23, 0.13889, 0.13465, 0.025659060 ];
        m.leg.data_newton15 = [];           % [l lw_y lb_y delta Ileg], lw_y + lb_y = l — 待作者定 (阶段 2)
        m.leg.row_mode = 'nearest';         % 'nearest' 与 mlx 一致按最近行取 | 'interp' pchip 插值
        % 控制约束
        m.ctrl.Ts          = 0.001;         % = CTRL_DT
        m.ctrl.T_wheel_max = 1.8;           % = .dji_trq_clamp  (阶段 1 仿真饱和用)
        m.ctrl.T_hip_max   = 10.0;          % = .dm_trq_clamp
        m.ctrl.grid        = 0.13:0.01:0.23;% K 表拟合域 = LQR_K_LEN_MIN/MAX
        m.status = {
            'body.*',            'Leg2';
            'leg.lu/lg/len_*',   'machine_config.c';
            'leg.data_sjtu5',    'Leg2';
            'leg.data_newton15', '待实测 (阶段 2 前作者定)';
            'ctrl.*',            'machine_config.c / lqr_balance.h' };

    case 'big_wheelleg'   % ---------- 大轮腿 = machine_config.c [MACHINE_ID_BIG_WHEELLEG]; 机体全是旧脚本值, 只作预研 ----------
        m.name = 'big_wheelleg';  m.c_id = 'MACHINE_ID_BIG_WHEELLEG';
        m.body.g          = 9.81;
        m.body.wheel_r    = 0.04;                     % 板上占位 (旧脚本 0.06) — 待实测
        m.body.half_track = 0.221;
        m.body.l_c        = sqrt(0.015^2 + 0.01^2);   % 旧脚本 x_c=-0.015, z_c=0.01
        m.body.m_w = 0.3;  m.body.m_l = 2.0;  m.body.m_b = 20.0;
        m.body.I_w = 0.008;  m.body.I_b = 0.5 + 20.0 * 0.015^2;  m.body.I_z = 0.7;
        m.leg.lu = 0.21;  m.leg.lg = 0.25;
        m.leg.len_min = 0.14;  m.leg.len_max = 0.34;
        % newton15 格式 [l lw_y lb_y delta Ileg] (旧脚本 9 点, lw_y + lb_y = l 逐点成立); sjtu5 格式由它换算
        D = [
            0.11, 0.09, 0.02, -0.066, 0.021;
            0.13, 0.10, 0.03, -0.067, 0.022;
            0.15, 0.11, 0.04, -0.067, 0.023;
            0.18, 0.12, 0.06, -0.066, 0.025;
            0.21, 0.13, 0.08, -0.064, 0.026;
            0.24, 0.15, 0.09, -0.062, 0.029;
            0.27, 0.16, 0.11, -0.059, 0.031;
            0.30, 0.18, 0.12, -0.055, 0.034;
            0.33, 0.20, 0.13, -0.051, 0.036 ];
        m.leg.data_newton15 = D;
        m.leg.data_sjtu5 = [D(:, 1), sqrt(D(:, 2).^2 + D(:, 4).^2), sqrt(D(:, 3).^2 + D(:, 4).^2), D(:, 5)];
        m.leg.row_mode = 'interp';                    % 表只有 9 点, 网格 1 cm, 只能插值
        m.ctrl.Ts          = 0.001;
        m.ctrl.T_wheel_max = 4.8;  m.ctrl.T_hip_max = 20.0;
        m.ctrl.grid        = 0.14:0.01:0.33;          % 待定: 板上 LQR_K_LEN_MIN/MAX 现为小机器的 0.13/0.23
        m.status = {
            'body.*',            '待实测 (旧脚本 leg_param.m)';
            'leg.lu/lg/len_*',   'machine_config.c';
            'leg.data_newton15', '待实测 (旧脚本 9 点)';
            'ctrl.grid',         '待实测 (区间实测后定)' };

    otherwise
        error('machine_table:unknown', '未知机器 "%s" (small_wheelleg / big_wheelleg)', name);
end

% 自检 + 待实测清单
g = m.ctrl.grid;  D = m.leg.data_sjtu5;
assert(m.leg.len_min < m.leg.len_max, 'leg.len_min 必须小于 len_max');
assert(all(diff(g) > 0) && all(diff(D(:, 1)) > 0), 'grid / 腿数据第 1 列必须递增');
assert(g(1) >= D(1, 1) - 1e-9 && g(end) <= D(end, 1) + 1e-9, ...
    'grid [%.3f %.3f] 超出腿数据覆盖 [%.3f %.3f]', g(1), g(end), D(1, 1), D(end, 1));
m.pending = {};
for k = 1:size(m.status, 1)
    if contains(m.status{k, 2}, '待实测')
        m.pending{end + 1} = sprintf('%s(%s)', m.status{k, 1}, m.status{k, 2}); %#ok<AGROW>
    end
end
end
