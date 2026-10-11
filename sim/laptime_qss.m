function Q = laptime_qss(P, R, opt)
% LAPTIME_QSS  준정상상태(점질량 + GGV) 랩타임 시뮬레이션. 설계값 비교·운전자 목표 속도용.
%
%   Q = laptime_qss(P, vt_road('autocross'))
%   Q.v_ms(s), Q.t_s, Q.lap_time_s, Q.ax, Q.ay, Q.limit (1 코너, 2 가속, 3 제동)
%
% 속도별 한계(GGV):
%   ay_max(v) = mu_y · (g + Fdown(v)/m)
%   ax_acc(v) = min( (F_tract(v) − F_res(v))/m ,  mu_x·(후륜 수직하중 + 하중이동)/m )
%   ax_brk(v) = mu_x·(g + Fdown/m) + F_res/m
%   코너와 동시 가·감속은 마찰 타원: ax_avail = ax_max·sqrt(1 − (ay/ay_max)²)
% 코너 속도 상한 v = sqrt(ay_max/|kappa|)를 앞으로(가속 한계)·뒤로(제동 한계) 훑어 v(s)를 만든다.
% 변속 시간·과도 응답·운전자 지연은 없다 → 실제보다 빠른 상한값.
if nargin < 3, opt = struct(); end
opt = prc_opts(opt, struct('mu_y', P.tire.mu, 'mu_x', P.tire.mu, 'v0', 0, 'laps', 1, 'closed_ss', R.closed));
V = P.veh; g = V.g; m = V.mass_kg;
D = sim_virtual_dyno(P);
Fenv = interp1_env(D);
% 1단 최저 속도(아이들/발진 rpm) 아래는 클러치 미끄럼으로 1단 발진 구동력을 낸다고 본다
v_launch = P.eng.launch_rpm / D.ratios(1) * 2*pi*P.tire.r_m / 60 * 3.6;
F_launch = interp1(D.v_kmh(:, 1), D.F_N(:, 1), v_launch);
Fenv(D.v_res_kmh < v_launch) = max(Fenv(D.v_res_kmh < v_launch), F_launch);
Ftr = @(v) max(interp1(D.v_res_kmh, Fenv, v*3.6, 'linear', 0), 0);   % 최대 구동력(단 선택 포함)
Fres = @(v) 0.5*V.rho*V.CdA_m2*v.^2 + V.Crr*m*g;
Fdn = @(v) 0.5*V.rho*V.ClA_m2*v.^2;
% 횡 한계: 타이어 하중 민감도와 횡 하중이동을 넣으면 유효 마찰이 줄어든다
%   축 횡력 = (mu0 − ls·dF)(F/2 + dF) + (mu0 + ls·dF)(F/2 − dF) = mu0·F − 2·ls·dF²,  dF = m·ay·h·분담/윤거
T = P.tire; h_ = V.cg_h_m; rsd = V.roll_stiff_dist_f;
ay_max = @(v) ay_eff(v, m, g, Fdn(v), V.aero_rear_frac, V.rear_frac, T.mu_y0, T.load_sens, T.Fz_ref_N, h_, rsd, V.track_f_m, V.track_r_m, opt.mu_y / P.tire.mu);
ax_brk = @(v) opt.mu_x * (g + Fdn(v)/m) + Fres(v)/m;
h = V.cg_h_m; L = V.wheelbase_m;
% 후륜 접지 한계 (하중이동 포함, 정상 가속): Fx = mu(Fz_r0 + Fx h/L) → Fx = mu Fz_r0/(1 − mu h/L)
ax_grip = @(v) opt.mu_x * (m*g*V.rear_frac + Fdn(v)*V.aero_rear_frac) / (1 - opt.mu_x*h/L) / m;
ax_acc = @(v) max(min((Ftr(v) - Fres(v))/m, ax_grip(v)), 0);

s = R.s; k = abs(R.kappa); n = numel(s);
if opt.laps > 1 && R.closed
  s = [s; cumsum(repmat(diff(s), opt.laps - 1, 1)) + s(end)]; k = [k; repmat(k(2:end), opt.laps - 1, 1)]; n = numel(s);
end
% 1) 코너 상한
vc = inf(n, 1);
for i = 1:n
  if k(i) > 1e-6
    vv = 5;
    for it = 1:30, vv = sqrt(ay_max(vv) / k(i)); end     % 다운포스 때문에 반복
    vc(i) = vv;
  end
end
vc = min(vc, 60);                                           % 상한 216 km/h (수치 안정)
% 2) 앞으로: 가속 한계
vf = zeros(n, 1); vf(1) = min(opt.v0, vc(1));
if opt.closed_ss, vf(1) = vc(1); end
for i = 2:n
  dsi = s(i) - s(i-1); v = vf(i-1);
  ay = v^2 * k(i-1); r = min(ay / max(ay_max(v), 1e-6), 1);
  a = ax_acc(v) * sqrt(max(1 - r^2, 0));
  vf(i) = min(sqrt(max(v^2 + 2*a*dsi, 0)), vc(i));
end
% 3) 뒤로: 제동 한계
vb = vc; vb(n) = vf(n);
if opt.closed_ss, vb(n) = min(vc(n), vf(n)); end
for i = n-1:-1:1
  dsi = s(i+1) - s(i); v = vb(i+1);
  ay = v^2 * k(i+1); r = min(ay / max(ay_max(v), 1e-6), 1);
  a = ax_brk(v) * sqrt(max(1 - r^2, 0));
  vb(i) = min(sqrt(v^2 + 2*a*dsi), vc(i));
end
vq = min(vf, vb);
if opt.closed_ss
  % 폐곡선 정상 랩: 출발 속도를 끝 속도와 맞출 때까지 반복
  for it = 1:4
    vf(1) = vq(end);
    for i = 2:n
      dsi = s(i) - s(i-1); v = vf(i-1);
      ay = v^2 * k(i-1); r = min(ay / max(ay_max(v), 1e-6), 1);
      a = ax_acc(v) * sqrt(max(1 - r^2, 0));
      vf(i) = min(sqrt(max(v^2 + 2*a*dsi, 0)), vc(i));
    end
    vq = min(vf, vb);
  end
end
vq = max(vq, 0.5);
if ~opt.closed_ss, vq(1) = max(opt.v0, 0.5); end
dt = diff(s) ./ ((vq(1:end-1) + vq(2:end)) / 2);
Q.s = s; Q.v_ms = vq; Q.v_kmh = vq * 3.6; Q.t_s = [0; cumsum(dt)];
Q.lap_time_s = Q.t_s(end);
Q.ay_g = vq.^2 .* k / g;
Q.ax_g = [0; diff(vq.^2) ./ (2*diff(s))] / g;
Q.limit = 2 * ones(n, 1); Q.limit(vq >= vc - 1e-6) = 1; Q.limit(vb < vf - 1e-6 & vq < vc - 1e-6) = 3;
Q.v_corner = vc;
Q.ggv = struct('v_kmh', (0:5:120)', 'ay_max_g', ay_max((0:5:120)'/3.6)/g, ...
               'ax_acc_g', ax_acc((0:5:120)'/3.6)/g, 'ax_brk_g', ax_brk((0:5:120)'/3.6)/g);
for q = 1:numel(R.marks)
  Q.marks.(R.marks(q).name) = interp1(s(1:numel(R.s)), Q.t_s(1:numel(R.s)), R.marks(q).s);
end
end

function ay = ay_eff(v, m, g, Fdn, arf, rf, mu0, ls, Fref, h, rsd, tf, tr, scale)
% 횡가속 한계를 반복으로 푼다 (하중이동이 ay에 의존)
Ff = m*g*(1 - rf) + Fdn*(1 - arf); Fr = m*g*rf + Fdn*arf;   % 정적 축하중
ay = mu0 * g * ones(size(Fdn));
for it = 1:12
  dFf = m*ay*h*rsd/tf; dFr = m*ay*h*(1-rsd)/tr;
  muf = mu0 - ls*((Ff/2) - Fref);  mur = mu0 - ls*((Fr/2) - Fref);     % 축 평균 하중에서의 마찰
  Fy = (muf.*Ff - 2*ls*dFf.^2) + (mur.*Fr - 2*ls*dFr.^2);
  ay_new = max(Fy, 0.2*m*g) / m;
  ay = 0.5*ay + 0.5*ay_new;
end
ay = ay * scale;
end

function F = interp1_env(D)
% 단별 구동력 곡선의 상한(변속 완벽) → D.v_res_kmh 축으로
F = zeros(size(D.v_res_kmh));
for g = 1:size(D.v_kmh, 2)
  f = interp1(D.v_kmh(:, g), D.F_N(:, g), D.v_res_kmh, 'linear', 0);
  F = max(F, f);
end
end
