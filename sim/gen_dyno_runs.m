function [F, truth] = gen_dyno_runs(P, db, opt)
% GEN_DYNO_RUNS  로드 다이노·코스트다운 시험 더미 로그.
%
%   [F, truth] = gen_dyno_runs(P, db, struct('out_csv', 'out/dummy_dyno.csv'))
%
% 시험 절차 (실제로 할 순서와 같다):
%   평지 정차 5 s (IMU 영점) → [ 출발 → 3단 3500 rpm 정속 → 3단 풀스로틀 당기기(10200 rpm까지)
%   → 80 km/h까지 가속 → 중립 타력 주행 80→15 km/h(코스트다운) → 정지 ] × 왕복 × opt.n_rep
%   같은 길을 양방향으로 달려 경사(opt.grade, 한 방향 오르막)를 상쇄한다.
%
% '정답' 엔진은 P의 가정 곡선과 일부러 다르게 만들었다(5000 rpm 부근 토크 골짜기).
% 분석이 가정값이 아니라 로그에서 정답을 다시 찾는지 확인하려는 것이다.
if nargin < 3, opt = struct(); end
opt = prc_opts(opt, struct('seed', 2, 'out_csv', '', 'grade', 0.008, 'gear', 3, 'n_rep', 2, ...
                           'trq_tab', [17 24 30 30.5 33 36.5 36 34.2 32.5 29.5 25]));
rng(opt.seed);
Pt = P;
Pt.eng.trq_tab = opt.trq_tab;                                     % 정답 토크 (P.eng.rpm_tab 축)
CdA_t = 2.30; Crr_t = 0.018;                                       % 정답 저항
V = P.veh; g = V.g; m = V.mass_kg; r = P.tire.r_m; eta = P.drv.eta;
Jw4 = 4 * P.tire.J_wheel_kgm2; Je = P.eng.J_kgm2;
ratios = P.drv.primary * P.drv.gears * P.drv.final;
i3 = ratios(opt.gear);
dt = 1e-3;
Fres = @(v) Crr_t * (m*g + 0.5*V.rho*V.ClA_m2*v.^2) + 0.5*V.rho*CdA_t*v.^2;

parts = {seg_idle(5, P)};
for rep = 1:opt.n_rep
  for dir = [1 -1]
    gr = dir * opt.grade;
    vs = 3500 / i3 / 60 * 2*pi*r;
    A = seg_kin(0, vs, 0.3*g, P);
    B = seg_blank(1000); B.v(:) = vs; B.w(:) = vs; B.gear(:) = opt.gear; B.rpm(:) = 3500; B.tps(:) = 0.25;
    % --- 풀스로틀 당기기 (정답 엔진, 경사 포함) ---
    meq = m + Jw4/r^2 + Je*i3^2*eta/r^2;
    v = vs; vv = []; aa = [];
    while v / r * i3 * 60/(2*pi) < 10200 && numel(vv) < 20000
      rpm = v / r * i3 * 60/(2*pi);
      Te = interp1(Pt.eng.rpm_tab, Pt.eng.trq_tab, min(max(rpm, 1500), 10500));
      a = (Te*i3*eta/r - Fres(v) - m*g*sin(gr)) / meq;
      vv(end+1, 1) = v; aa(end+1, 1) = a; %#ok<AGROW>
      v = v + a*dt;
    end
    C = seg_blank(numel(vv));
    C.v = vv; C.w = repmat(vv, 1, 4); C.ax = aa; C.tps(:) = 1; C.gear(:) = opt.gear;
    C.rpm = vv / r * i3 * 60/(2*pi);
    D = seg_kin(v, 80/3.6, 0.3*g, P, struct('tps', 0.7, 'rpm_max', 9500));
    % --- 코스트다운 (중립) ---
    meq0 = m + Jw4/r^2;
    v = 80/3.6; vv = []; aa = [];
    while v > 15/3.6
      a = (-Fres(v) - m*g*sin(gr)) / meq0;
      vv(end+1, 1) = v; aa(end+1, 1) = a; %#ok<AGROW>
      v = v + a*dt;
    end
    E = seg_blank(numel(vv));
    E.v = vv; E.w = repmat(vv, 1, 4); E.ax = aa; E.rpm(:) = P.eng.idle_rpm;   % gear 0, tps 0
    G = seg_kin(v, 0, 0.3*g, P, struct('pF', 10));
    H = seg_idle(2, P);
    seg = {A, B, C, D, E, G, H};
    for k = 1:numel(seg), seg{k}.grade(:) = gr; end
    parts = [parts, seg]; %#ok<AGROW>
  end
end
X = seg_cat(parts{:});
[F, aux] = prc_make_frames(P, db, X, struct('drop_rate', 0.001));
truth.P_true = Pt;
truth.rpm_tab = Pt.eng.rpm_tab; truth.trq_tab = Pt.eng.trq_tab;
truth.CdA_eff = CdA_t + Crr_t * V.ClA_m2;   % 다운포스로 늘어난 구름저항이 v^2 항에 섞인다
truth.CdA = CdA_t; truth.Crr = Crr_t; truth.grade = opt.grade;
truth.imu_mount_deg = aux.opt.imu_mount_deg; truth.gyro_bias_dps = aux.opt.gyro_bias_dps;
truth.X = X;
if ~isempty(opt.out_csv)
  prc_log_write_csv(opt.out_csv, F, sprintf('dummy road-dyno/coastdown seed=%d (SIMULATED DATA)', opt.seed));
end
end
