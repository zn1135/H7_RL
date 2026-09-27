function [A, B] = model_AB(model, m, lL, lR)
%MODEL_AB 动力学模型统一接口: 连续时间 A (10x10), B (10x4)
%   [A, B] = model_AB('sjtu5', m, lL, lR)
% 首次调用做符号推导并缓存到 cache/AB_<model>_gen.m (约 13 s), 之后直接用缓存。
% 加新模型 = 加一个 case + 一个 build_* 局部函数。阶段 2 的 'newton15' 在这里接。
% 状态序 x = [s ds phi dphi th_ll dth_ll th_lr dth_lr th_b dth_b], 输出序 u = [T_wl T_wr T_bl T_br]
here      = fileparts(mfilename('fullpath'));
cache_dir = fullfile(here, 'cache');
switch lower(model)
    case 'sjtu5'
        gen = fullfile(cache_dir, 'AB_sjtu5_gen.m');
        if ~exist(gen, 'file'), build_sjtu5(gen); end
        if exist('AB_sjtu5_gen', 'file') ~= 2, addpath(cache_dir); rehash; end
        [A, B] = AB_sjtu5_gen(sjtu5_param_vec(m, lL, lR));
    case 'newton15'
        error('model_AB:notyet', 'newton15 在阶段 2 接入 (见 LQR_MATLAB_PLAN.md §六)');
    otherwise
        error('model_AB:unknown', '未知模型 "%s"', model);
end
end

%% ---- sjtu5: 机器表 + 腿长 → 18 维参数向量 (顺序与 build_sjtu5 一致) ----
function p = sjtu5_param_vec(m, lL, lR)
% [R_w, R_l, l_l, l_r, l_wl, l_wr, l_bl, l_br, l_c, m_w, m_l, m_b, I_w, I_ll, I_lr, I_b, I_z, g]
rowL = leg_row(m, lL);  rowR = leg_row(m, lR);   % [l_wl l_bl I_ll]
b = m.body;
p = [b.wheel_r, b.half_track, lL, lR, rowL(1), rowR(1), rowL(2), rowR(2), b.l_c, ...
     b.m_w, b.m_l, b.m_b, b.I_w, rowL(3), rowR(3), b.I_b, b.I_z, b.g];
end

function row = leg_row(m, L)
% 腿长 L → leg.data_sjtu5 的 [l_wl l_bl I_ll]; 'nearest' 与 Leg2 mlx 按下标取一致, 'interp' 为 pchip
D = m.leg.data_sjtu5;
switch m.leg.row_mode
    case 'nearest'
        [~, k] = min(abs(D(:, 1) - L));  row = D(k, 2:4);
    case 'interp'
        row = interp1(D(:, 1), D(:, 2:4), L, 'pchip');
        assert(all(isfinite(row)), '腿长 %.4f 超出腿数据范围 [%.3f, %.3f]', L, D(1, 1), D(end, 1));
    otherwise
        error('model_AB:row_mode', '未知 row_mode "%s"', m.leg.row_mode);
end
end

%% ---- sjtu5: Leg2 上交 5 方程线性模型 → 符号 A/B → matlabFunction 缓存 (只跑一次) ----
function build_sjtu5(gen_file)
% 方程 (19)-(23) 与 Leg2 WBR_modeling.mlx 逐字相同; A/B 拼装同 mlx
assert(~isempty(which('syms')), 'build_sjtu5 需要 Symbolic Math Toolbox');
fprintf('build_sjtu5: 符号推导并生成 %s ...\n', gen_file);
t0 = tic;
syms R_w R_l l_l l_r l_wl l_wr l_bl l_br l_c m_w m_l m_b I_w I_ll I_lr I_b I_z g
syms ddtheta_wl ddtheta_wr ddtheta_ll ddtheta_lr ddtheta_b
syms T_wl T_wr T_bl T_br
syms theta_ll theta_lr theta_b

eqn1 = (I_w*l_l/R_w + m_w*R_w*l_l + m_l*R_w*l_bl)*ddtheta_wl ...
     + (m_l*l_wl*l_bl - I_ll)*ddtheta_ll ...
     + (m_l*l_wl + m_b*l_l/2)*g*theta_ll ...
     + T_bl - T_wl*(1 + l_l/R_w) == 0;
eqn2 = (I_w*l_r/R_w + m_w*R_w*l_r + m_l*R_w*l_br)*ddtheta_wr ...
     + (m_l*l_wr*l_br - I_lr)*ddtheta_lr ...
     + (m_l*l_wr + m_b*l_r/2)*g*theta_lr ...
     + T_br - T_wr*(1 + l_r/R_w) == 0;
eqn3 = -(m_w*R_w^2 + I_w + m_l*R_w^2 + m_b*R_w^2/2)*ddtheta_wl ...
     - (m_w*R_w^2 + I_w + m_l*R_w^2 + m_b*R_w^2/2)*ddtheta_wr ...
     - (m_l*R_w*l_wl + m_b*R_w*l_l/2)*ddtheta_ll ...
     - (m_l*R_w*l_wr + m_b*R_w*l_r/2)*ddtheta_lr ...
     + T_wl + T_wr == 0;
eqn4 = (m_w*R_w*l_c + I_w*l_c/R_w + m_l*R_w*l_c)*ddtheta_wl ...
     + (m_w*R_w*l_c + I_w*l_c/R_w + m_l*R_w*l_c)*ddtheta_wr ...
     + m_l*l_wl*l_c*ddtheta_ll + m_l*l_wr*l_c*ddtheta_lr ...
     - I_b*ddtheta_b + m_b*g*l_c*theta_b ...
     - (T_wl + T_wr)*l_c/R_w - (T_bl + T_br) == 0;
eqn5 = ((I_z*R_w)/(2*R_l) + I_w*R_l/R_w)*ddtheta_wl ...
     - ((I_z*R_w)/(2*R_l) + I_w*R_l/R_w)*ddtheta_wr ...
     + (I_z*l_l)/(2*R_l)*ddtheta_ll - (I_z*l_r)/(2*R_l)*ddtheta_lr ...
     - T_wl*R_l/R_w + T_wr*R_l/R_w == 0;

sols = solve([eqn1, eqn2, eqn3, eqn4, eqn5], [ddtheta_wl, ddtheta_wr, ddtheta_ll, ddtheta_lr, ddtheta_b]);
dd  = [sols.ddtheta_wl, sols.ddtheta_wr, sols.ddtheta_ll, sols.ddtheta_lr, sols.ddtheta_b];
J_A = jacobian(dd, [theta_ll, theta_lr, theta_b]);
J_B = jacobian(dd, [T_wl, T_wr, T_bl, T_br]);

A_sym = sym(zeros(10, 10));  B_sym = sym(zeros(10, 4));
for rr = 1:2:9, A_sym(rr, rr + 1) = 1; end
col = [5, 7, 9];
A_sym(2,  col) = R_w * (J_A(1, :) + J_A(2, :)) / 2;
A_sym(4,  col) = (R_w * (-J_A(1, :) + J_A(2, :))) / (2*R_l) - (l_l * J_A(3, :)) / (2*R_l) + (l_r * J_A(4, :)) / (2*R_l);
A_sym(6,  col) = J_A(3, :);
A_sym(8,  col) = J_A(4, :);
A_sym(10, col) = J_A(5, :);
for h = 1:4
    B_sym(2,  h) = R_w * (J_B(1, h) + J_B(2, h)) / 2;
    B_sym(4,  h) = (R_w * (-J_B(1, h) + J_B(2, h))) / (2*R_l) - (l_l * J_B(3, h)) / (2*R_l) + (l_r * J_B(4, h)) / (2*R_l);
    B_sym(6,  h) = J_B(3, h);
    B_sym(8,  h) = J_B(4, h);
    B_sym(10, h) = J_B(5, h);
end
pvec = [R_w, R_l, l_l, l_r, l_wl, l_wr, l_bl, l_br, l_c, m_w, m_l, m_b, I_w, I_ll, I_lr, I_b, I_z, g];

gen_dir = fileparts(gen_file);
if ~exist(gen_dir, 'dir'), mkdir(gen_dir); end
matlabFunction(A_sym, B_sym, 'File', gen_file, 'Vars', {pvec}, 'Outputs', {'A', 'B'});
addpath(gen_dir);  rehash;
fprintf('build_sjtu5: 完成, 耗时 %.1f s\n', toc(t0));
end
