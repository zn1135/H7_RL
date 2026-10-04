function ok = build_gain_table(CFG, q, r)
%BUILD_GAIN_TABLE 机器表 + Q/R → 腿长网格 c2d + dlqr → 多项式拟合 → 闭环检查 → 写 C → (通过) 覆盖板上文件
%   ok = build_gain_table(CFG, q, r)     由 run_all 调用; 中间计算已验证 (变更 85), 不需要改
%   CFG.machine / CFG.model / CFG.fit_order / CFG.deploy 见 run_all §1; q, r 为 Q/R 对角线 (原样写进 C 文件头)
% 产物: output/lqr_gain_<机器>.c, output/report_<机器>_latest.txt, cache/scan_<机器>_<模型>.mat;
%       CFG.deploy 且闭环通过 → 复制对应机器表，公共入口不覆盖
here      = fileparts(mfilename('fullpath'));
repo_root = fileparts(fileparts(here));
out_dir   = fullfile(here, 'output');
cache_dir = fullfile(here, 'cache');
if ~exist(out_dir, 'dir'),   mkdir(out_dir);   end
if ~exist(cache_dir, 'dir'), mkdir(cache_dir); end

m = machine_table(CFG.machine);
assert(strcmp(m.ctrl.state_schema, 'hip'), '固件表必须采用公共髋轴状态');
Q = diag(q);  R = diag(r);
table_id = sprintf('%s-%s-%s', m.name, CFG.model, char(datetime('now', 'Format', 'yyyyMMdd-HHmm')));
report   = fullfile(out_dir, ['report_' m.suffix '_latest.txt']);
if exist(report, 'file'), delete(report); end
diary(report);                                        % 命令行窗口照常显示, 同时抄一份进文件
cleanup = onCleanup(@() diary('off')); %#ok<NASGU>

fprintf('==== 表号 %s ====\n', table_id);
fprintf('机器 %s (%s)   模型 %s   拟合 poly%d%d\n', m.name, m.c_id, CFG.model, CFG.fit_order, CFG.fit_order);
fprintf('Q = diag([%s])\nR = diag([%s])\n', num2str(q), num2str(r));
fprintf('网格 %.2f:%.2f:%.2f (%d 点)   Ts = %g   腿数据取行 %s\n', ...
    m.ctrl.grid(1), m.ctrl.grid(2) - m.ctrl.grid(1), m.ctrl.grid(end), numel(m.ctrl.grid), m.ctrl.Ts, m.leg.row_mode);
if ~isempty(m.pending), fprintf('待实测项: %s\n', strjoin(m.pending, ', ')); end

S  = scan_grid(CFG.model, m, Q, R);                   % 1. 网格: A/B → c2d → dlqr
F  = fit_K(S, CFG.fit_order);                          % 2. 拟合
fprintf('fit_K: poly%d%d, 拟合残差 max|K_fit-K_dlqr| = %.3g (相对 %.3g)\n', ...
    F.order, F.order, max(F.resid_max(:)), max(F.resid_rel(:)));
[ok, validation] = check_closed_loop(S, F, CFG.model, m);                          % 3. 闭环谱半径
print_K_nominal(F, S);
c_file = fullfile(out_dir, ['lqr_gain_' m.suffix '.c']);        % 4. 写 C
emit_gain_table(F, m, CFG, q, r, c_file, table_id, repo_root, ok);
save(fullfile(cache_dir, sprintf('scan_%s_%s.mat', m.name, CFG.model)), 'S', 'F', 'm', 'Q', 'R', 'table_id', 'validation');

board_c = fullfile(repo_root, 'imcalib', 'Algorithm', ['lqr_gain_' m.suffix '.c']);
if ok && isfield(CFG, 'deploy') && CFG.deploy         % 5. 部署: 闭环通过才覆盖板上文件 (.h 不动)
    copyfile(c_file, board_c);
    verdict = sprintf('闭环检查通过, 已覆盖 %s → 整项目编译后上板', board_c);
elseif ok
    verdict = sprintf('闭环检查通过, 未部署 (CFG.deploy=false), 手动拷到 %s', board_c);
else
    verdict = '**闭环检查未通过, 板上文件未动, 不要上板**';
end
fprintf('\n==== %s ====\n产物: %s\n报告: %s\n', verdict, c_file, report);
clear cleanup;
copyfile(report, fullfile(out_dir, 'report_latest.txt'));
end

%% ============ 网格扫描: A/B → c2d(ZOH) → dlqr ============
function S = scan_grid(model, m, Q, R)
grid = m.ctrl.grid(:)';  n = numel(grid);  Ts = m.ctrl.Ts;
S.grid = grid;  S.Ts = Ts; S.Q = Q;
S.Ad = zeros(10, 10, n, n);  S.Bd = zeros(10, 4, n, n);
S.K  = nan(4, 10, n, n);     S.ok = false(n, n);
t0 = tic;
for il = 1:n
    for ir = 1:n
        [A, B] = model_AB(model, m, grid(il), grid(ir));
        [Ad, Bd] = c2d(A, B, Ts);                      % 与 Leg2 mlx 同一调用
        S.Ad(:, :, il, ir) = Ad;  S.Bd(:, :, il, ir) = Bd;
        try
            S.K(:, :, il, ir) = dlqr(Ad, Bd, Q, R);
            S.ok(il, ir) = true;
        catch ME
            warning('build_gain_table:dlqr', 'dlqr 失败 lL=%.3f lR=%.3f: %s', grid(il), grid(ir), ME.message);
        end
    end
end
kn = ceil(n / 2);
[A, B] = model_AB(model, m, grid(kn), grid(kn));
fprintf('scan_grid: %dx%d 点, dlqr 成功 %d/%d, 标称 L=%.2f 可控秩 %d/10, 耗时 %.1f s\n', ...
    n, n, nnz(S.ok), n*n, grid(kn), rank(ctrb(A, B)), toc(t0));
end

%% ============ K 拟合: 二元多项式最小二乘 (基函数顺序与 Leg2 poly22 相同) ============
function F = fit_K(S, order)
[LL, LR] = ndgrid(S.grid, S.grid);
x = LL(:);  y = LR(:);
switch order
    case 2, X = [ones(size(x)), x, y, x.^2, x.*y, y.^2];
    case 3, X = [ones(size(x)), x, y, x.^2, x.*y, y.^2, x.^3, x.^2.*y, x.*y.^2, y.^3];
    otherwise, error('build_gain_table:order', 'fit_order 只支持 2 或 3');
end
F.order = order;  F.grid = S.grid;
F.C = zeros(4, 10, size(X, 2));  F.resid_max = zeros(4, 10);  F.resid_rel = zeros(4, 10);
for i = 1:4
    for j = 1:10
        z  = reshape(S.K(i, j, :, :), [], 1);          % il 变快, 与 LL(:) 一致
        ok = isfinite(z);
        assert(nnz(ok) >= size(X, 2), 'K(%d,%d) 有效点不够拟合', i, j);
        c = X(ok, :) \ z(ok);
        F.C(i, j, :) = c;
        rr = X(ok, :) * c - z(ok);
        F.resid_max(i, j) = max(abs(rr));
        F.resid_rel(i, j) = max(abs(rr)) / max(max(abs(z(ok))), 1e-12);
    end
end
end

function K = eval_K_poly(F, lL, lR)
switch F.order
    case 2, b = [1; lL; lR; lL^2; lL*lR; lR^2];
    case 3, b = [1; lL; lR; lL^2; lL*lR; lR^2; lL^3; lL^2*lR; lL*lR^2; lR^3];
end
K = reshape(reshape(F.C, 40, []) * b, 4, 10);
end

%% ============ 闭环谱半径: 拟合后的 K 装回离散模型, 全网格 max|eig(Ad - Bd K)| < 1 才稳 ============
function [ok, V] = check_closed_loop(S, F, model, m)
% 检查实际 float32 系数和运算，以及网格之间的模型。
F.C = double(single(F.C));
grid = linspace(S.grid(1), S.grid(end), 2*numel(S.grid)-1);
n = numel(grid); rho = nan(n); delay_rho = nan(n);
for il = 1:n
    for ir = 1:n
        [A,B] = model_AB(model, m, grid(il), grid(ir));
        [Ad,Bd] = c2d(A,B,S.Ts);
        Kf = eval_K_float(F, grid(il), grid(ir));
        rho(il,ir) = max(abs(eig(Ad-Bd*Kf)));
        delay_rho(il,ir) = max(abs(eig([Ad,-Bd; Kf,zeros(4)])));
    end
end
V.rho_max = max(rho(:)); V.delay_rho_max = max(delay_rho(:));
V.grid = grid;
% 冻结腿长的线性饱和仿真，不能替代实机。
kn = ceil(numel(S.grid)/2);
Ad = S.Ad(:,:,kn,kn); Bd = S.Bd(:,:,kn,kn);
K = eval_K_float(F,S.grid(kn),S.grid(kn));
x = zeros(10,1); x(9) = 0.05; start = norm(sqrt(diag(S.Q)).*x);
limits = [m.ctrl.T_wheel_max;m.ctrl.T_wheel_max;m.ctrl.T_hip_max;m.ctrl.T_hip_max];
V.saturated_steps = 0;
for t = 1:round(10/S.Ts)
    raw = -K*x;
    u = min(max(raw,-limits),limits);
    V.saturated_steps = V.saturated_steps + any(raw ~= u);
    x = Ad*x + Bd*u;
end
V.final_ratio = norm(sqrt(diag(S.Q)).*x)/start;
ok = all(S.ok(:)) && all(isfinite(rho(:))) && all(rho(:)<1) ...
    && all(isfinite(delay_rho(:))) && all(delay_rho(:)<1) ...
    && isfinite(V.final_ratio) && V.final_ratio<0.2;
fprintf('float32 密网格 %dx%d: rho=%.9g, 一拍延迟 rho=%.9g\n',n,n,V.rho_max,V.delay_rho_max);
fprintf('10s 饱和线性仿真: 加权状态末值比例 %.9g, 饱和 %d 拍\n',V.final_ratio,V.saturated_steps);
if ok, fprintf('闭环检查: 通过\n'); else, fprintf('闭环检查: 未通过\n'); end
end

function K = eval_K_float(F,lL,lR)
x=single(lL); y=single(lR); t2=x*x; t3=y*y; t4=x*y;
b=single([1;x;y;t2;t4;t3]);
if F.order==3, b=[b;t2*x;t2*y;x*t3;t3*y]; end
K=zeros(4,10);
for i=1:4
    for j=1:10
        c=single(squeeze(F.C(i,j,:))); z=c(1);
        for k=2:numel(c), z=z+c(k)*b(k); end
        K(i,j)=double(z);
    end
end
end

function print_K_nominal(F, S)
kn = ceil(numel(S.grid) / 2);
fprintf('标称 lL=lR=%.2f 的拟合 K (行 T_wl T_wr T_bl T_br, 列 s ds phi dphi th_ll dth_ll th_lr dth_lr th_b dth_b):\n', S.grid(kn));
disp(eval_K_poly(F, S.grid(kn), S.grid(kn)));
end

%% ============ 写 C: 与板上 lqr_gain_table.c 同签名同下标 K_sym[状态*4 + 输出] ============
function emit_gain_table(F, m, CFG, q, r, out_file, table_id, repo_root, ok_cl)
assert(all(isfinite(F.C(:))), '拟合系数含 NaN/Inf, 拒绝导出');
git_hash = '';
[rc, h] = system(sprintf('git -C "%s" rev-parse --short HEAD', repo_root));
if rc == 0, git_hash = strtrim(h); end
if isempty(m.pending), pend_str = '(无)'; else, pend_str = strjoin(m.pending, '; '); end
if ok_cl, cl_str = '通过'; else, cl_str = '**未通过, 不要上板**'; end
state_names = {'s', 'ds', 'phi', 'dphi', 'th_ll', 'dth_ll', 'th_lr', 'dth_lr', 'th_b', 'dth_b'};
out_names   = {'T_wl', 'T_wr', 'T_bl', 'T_br'};
switch F.order
    case 2, poly_str = 'poly22: K = p00 + p10*lL + p01*lR + p20*lL^2 + p11*lL*lR + p02*lR^2';
            vars = {'', 'lL', 'lR', 't2', 't4', 't3'};
    case 3, poly_str = 'poly33: poly22 + p30*lL^3 + p21*lL^2*lR + p12*lL*lR^2 + p03*lR^3';
            vars = {'', 'lL', 'lR', 't2', 't4', 't3', 't5', 't6', 't7', 't8'};
end

fid = fopen(out_file, 'w', 'n', 'UTF-8');
assert(fid > 0, '无法写 %s', out_file);
c = onCleanup(@() fclose(fid));
w = @(varargin) fprintf(fid, varargin{:});
w('/*\n * File: generated machine gain\n *\n');
w(' * WBR LQR 最优反馈增益 K(lL, lR) —— 由 tools/matlab/run_all.m 生成, 勿手改。\n *\n');
w(' * 表号   : %s\n', table_id);
w(' * 生成   : %s   git %s\n', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm')), git_hash);
w(' * 机器   : %s (%s)\n', m.name, m.c_id);
w(' * 模型   : %s\n', CFG.model);
w(' * Q      : diag([%s])\n', num2str(q));
w(' * R      : diag([%s])\n', num2str(r));
w(' * 网格   : lL, lR = %.2f:%.2f:%.2f (%dx%d), Ts = %g, c2d ZOH + dlqr, 腿数据取行 %s\n', ...
    F.grid(1), F.grid(2) - F.grid(1), F.grid(end), numel(F.grid), numel(F.grid), m.ctrl.Ts, m.leg.row_mode);
w(' * 拟合   : %s\n', poly_str);
w(' * 残差   : max|K_fit - K_dlqr| = %.3g (相对 %.3g)\n', max(F.resid_max(:)), max(F.resid_rel(:)));
w(' * 闭环   : %s\n', cl_str);
w(' * 待实测 : %s\n *\n', pend_str);
w(' * 状态序 x = [s ds phi dphi th_ll dth_ll th_lr dth_lr th_b dth_b]\n');
w(' * s/ds = 髋轴中点位置/速度，已从上交轮轴状态做坐标变换；腿摆角/角速度前摆为正。\n');
w(' * 输出序 u = [T_wl T_wr T_bl T_br]\n');
w(' * 用法   : LQR_K_WBR(lL, lR, K_sym) -> K_sym[状态*4 + 输出] (lqr_balance.c 按此取 K[输出][状态])\n */\n\n');
w('#include "lqr_gain_table.h"\n\n');
w('static void eval_%s(float lL, float lR, float K_sym[40])\n{\n', m.suffix);
w('  float t2;\n  float t3;\n  float t4;\n');
if F.order == 3, w('  float t5;\n  float t6;\n  float t7;\n  float t8;\n'); end
w('  t2 = lL * lL;\n  t3 = lR * lR;\n  t4 = lL * lR;\n');
if F.order == 3, w('  t5 = t2 * lL;\n  t6 = t2 * lR;\n  t7 = lL * t3;\n  t8 = t3 * lR;\n'); end
for j = 1:10
    w('  /* %s */\n', state_names{j});
    for i = 1:4
        cvec = squeeze(F.C(i, j, :));
        expr = fmt_float(cvec(1));
        for k = 2:numel(cvec)
            if cvec(k) >= 0, sgn = '+'; else, sgn = '-'; end
            expr = sprintf('%s %s %s * %s', expr, sgn, fmt_float(abs(cvec(k))), vars{k});
        end
        w('  K_sym[%2d] = %s;   /* %s */\n', (j - 1) * 4 + (i - 1), expr, out_names{i});
    end
end
w('}\n\n');
w('const lqr_gain_desc_t lqr_gain_%s = {\n',m.suffix);
w('    .table_id = "%s", .model = "%s",\n',table_id,CFG.model);
w('    .machine_id = %s, .state_schema = LQR_STATE_HIP_FRONT_V2,\n',m.c_id);
w('    .input_schema = LQR_INPUT_WHEELS_HIPS_V1, .checked = %du,\n',ok_cl);
w('    .dt = %s, .len_min = %s, .len_max = %s,\n',fmt_float(m.ctrl.Ts),fmt_float(F.grid(1)),fmt_float(F.grid(end)));
w('    .wheel_r = %s, .leg_lu = %s, .leg_lg = %s,\n',fmt_float(m.body.wheel_r),fmt_float(m.leg.lu),fmt_float(m.leg.lg));
w('    .x_eq = {0}, .u_eq = {0}, .eval = eval_%s,\n};\n',m.suffix);
fprintf('emit: 已写 %s\n', out_file);
end

function s = fmt_float(v)
% 先舍入到 single 再以 9 位有效数字打印: C 端解析回来就是同一个 float
s = sprintf('%.9g', double(single(v)));
if isempty(regexp(s, '[.eE]', 'once')), s = [s '.0']; end
s = [s 'F'];
end
