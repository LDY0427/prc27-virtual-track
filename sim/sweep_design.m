function W = sweep_design(P, outdir)
% SWEEP_DESIGN  설계·세팅 값을 하나씩 바꿔 가속 성능이 어떻게 달라지는지 비교한다 (수정 방안 검토용).
%
%   W = sweep_design(prc27_params(), 'out')
%
% 바꿔 보는 것 (나머지는 기본값 고정):
%   종감속비, 변속 회전수, 발진 회전수, 차량 질량, 공기저항(CdA), 후륜 하중 배분,
%   그리고 결과 신뢰도를 보기 위한 엔진 회전관성 ±50%
% 지표: 75 m 시간(건조), 0→60 km/h 시간, 예상 최고속, 50 km/h에서 2단 회전수
% 모두 가정값이 섞인 모델 결과다. '어느 쪽으로 얼마나 민감한가'를 보는 용도이고 절대값 비교용이 아니다.
if nargin < 2, outdir = 'out'; end
base = run1(P);
W.base = base;
W.final = sweep(P, 'final', 3.4:0.2:5.0, @(Q, x) setf(Q, 'drv', 'final', x));
W.shift = sweep(P, 'shift_rpm', 9000:250:10500, @(Q, x) setf(Q, 'drv', 'shift_rpm', x));
W.launch = sweep(P, 'launch_rpm', 5000:1000:9000, @(Q, x) setf(Q, 'eng', 'launch_rpm', x));
W.mass = sweep(P, 'mass_kg', P.veh.mass_kg + [-20 -10 0 10], @(Q, x) setf(Q, 'veh', 'mass_kg', x));
W.cda = sweep(P, 'CdA_m2', P.veh.CdA_m2 * [0.8 0.9 1.0 1.1], @(Q, x) setf(Q, 'veh', 'CdA_m2', x));
W.rear = sweep(P, 'rear_frac', [0.45 0.48 0.52 0.55], @(Q, x) setf(Q, 'veh', 'rear_frac', x));
W.inertia = sweep(P, 'J_eng', P.eng.J_kgm2 * [0.5 1.0 1.5], @(Q, x) setf(Q, 'eng', 'J_kgm2', x));

% ---------- 그림 ----------
h = prc_fig(1100, 700);
pl = {'final', 'final drive ratio'; 'shift', 'shift rpm'; 'launch', 'launch rpm'; ...
      'mass', 'vehicle mass [kg]'; 'cda', 'CdA [m^2]'; 'rear', 'rear weight fraction'};
for k = 1:size(pl, 1)
  Z = W.(pl{k, 1});
  subplot(2, 3, k); hold on; grid on;
  plot(Z.x, 1000*(Z.t75 - base.t75), 'o-'); plot(Z.x, 1000*(Z.t60 - base.t60), 's--');
  plot([min(Z.x) max(Z.x)], [0 0], 'k:');
  xlabel(pl{k, 2}); ylabel('change vs baseline [ms]  (- = faster)');
  if k == 1, legend('75 m', '0-60 km/h', 'Location', 'northeast'); end
  title(pl{k, 2});
end
prc_savefig(h, fullfile(outdir, 'sweep_design.png'));

% ---------- 보고서 ----------
Lr = {'# 설계·세팅 비교 시뮬레이션 (가정값 포함)', '', ...
  sprintf('기본값: 종감속 %.2f, 변속 %d rpm, 발진 %d rpm, %d kg, CdA %.2f, 후륜 %.0f%% → 75 m %.2f s, 0→60 %.2f s, 최고속 %.0f km/h', ...
          P.drv.final, P.drv.shift_rpm, P.eng.launch_rpm, P.veh.mass_kg, P.veh.CdA_m2, 100*P.veh.rear_frac, base.t75, base.t60, base.vmax), ''};
Lr = [Lr, table_lines(W.final, '종감속비', '%.1f'), table_lines(W.shift, '변속 rpm', '%.0f'), ...
      table_lines(W.launch, '발진 rpm', '%.0f'), table_lines(W.mass, '질량 kg', '%.0f'), ...
      table_lines(W.cda, 'CdA m²', '%.2f'), table_lines(W.rear, '후륜 하중 비율', '%.2f'), ...
      table_lines(W.inertia, '엔진 회전관성 kg·m² (신뢰도 확인용)', '%.3f'), {'![sweep](sweep_design.png)'}];
prc_write_lines(fullfile(outdir, 'sweep_design_report.md'), Lr);
end

function Q = setf(Q, a, b, x)
Q.(a).(b) = x;
end

function r = run1(Q)
R = sim_longitudinal(Q, struct('tcs_on', false, 'mu_scale', 1.0, 't_end', 8));
r.t75 = R.t75;
k = find(R.v_kmh >= 60, 1); r.t60 = NaN; if ~isempty(k), r.t60 = R.t(k) - R.opt.tps_t0; end
r.max_slip = max(R.slip_meas);
D = sim_virtual_dyno(Q);
vt = nan(1, numel(D.ratios));
for g = 1:numel(D.ratios)
  Fr = interp1(D.v_res_kmh, D.F_res_N, D.v_kmh(:, g));
  k = find(D.F_N(:, g) < Fr, 1);
  if isempty(k), vt(g) = D.v_max_gear_kmh(g); else, vt(g) = D.v_kmh(k, g); end
end
r.vmax = max(vt);
r.rpm50_g2 = 50/3.6 / Q.tire.r_m * D.ratios(2) * 60/(2*pi);
end

function Z = sweep(P, name, xs, setter)
Z.name = name; Z.x = xs(:);
n = numel(xs);
Z.t75 = nan(n, 1); Z.t60 = nan(n, 1); Z.vmax = nan(n, 1); Z.rpm50 = nan(n, 1); Z.slip = nan(n, 1);
for i = 1:n
  r = run1(setter(P, xs(i)));
  Z.t75(i) = r.t75; Z.t60(i) = r.t60; Z.vmax(i) = r.vmax; Z.rpm50(i) = r.rpm50_g2; Z.slip(i) = r.max_slip;
end
[~, Z.best] = min(Z.t75);
end

function L = table_lines(Z, label, fmt)
L = {['## ' label], '', ['| ' label ' | 75 m s | 0→60 km/h s | 최고속 km/h | 50 km/h 2단 rpm | 최대 슬립 |'], ...
     '|---:|---:|---:|---:|---:|---:|'};
for i = 1:numel(Z.x)
  b = ''; if i == Z.best, b = ' ◀ 75 m 최소'; end
  L{end+1} = sprintf(['| ' fmt ' | %.3f | %.3f | %.0f | %.0f | %.3f |%s'], Z.x(i), Z.t75(i), Z.t60(i), Z.vmax(i), Z.rpm50(i), Z.slip(i), b); %#ok<AGROW>
end
L{end+1} = '';
end
