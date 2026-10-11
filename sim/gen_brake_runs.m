function [F, truth] = gen_brake_runs(P, db, opt)
% GEN_BRAKE_RUNS  제동 시험 더미 로그: 40 km/h에서 압력을 단계별로 올려 가며 급제동.
%
%   [F, truth] = gen_brake_runs(P, db, struct('out_csv', 'out/dummy_brake.csv'))
%
% 바퀴 4개를 각각 계산한다: 제동 토크 = 계수 × 회로 압력(전 pF, 후 0.6·pF),
% 수직하중 = 정적 + 감속에 따른 전후 하중 이동, 타이어 = 매직포뮬러.
% 바퀴 회전이 0이 되면 잠김. FL 제동 계수를 4% 크게 해서 FL이 먼저 잠기게 했다(정답 확인용).
if nargin < 3, opt = struct(); end
opt = prc_opts(opt, struct('seed', 5, 'out_csv', '', 'p_list', [20 35 50 65 85], ...
                           'v0_kmh', 40, 'rear_ratio', 0.6, 'fl_gain', 1.04));
rng(opt.seed);
V = P.veh; g = V.g; m = V.mass_kg; r = P.tire.r_m; J = P.tire.J_wheel_kgm2;
kb = [P.brk.k_front_Nm_bar*opt.fl_gain, P.brk.k_front_Nm_bar, P.brk.k_rear_Nm_bar, P.brk.k_rear_Nm_bar];
Fz_s = m*g * [(1-V.rear_frac)/2 (1-V.rear_frac)/2 V.rear_frac/2 V.rear_frac/2];
dts = 1e-4; dec = 10;
parts = {seg_idle(5, P)};
ev = struct('p', {}, 't_lock', {}, 'p_lock', {}, 'stop_m', {}, 'v0', {}, 't_on', {});
t_now = 5;
for e = 1:numel(opt.p_list)
  pT = opt.p_list(e);
  A = seg_kin(0, opt.v0_kmh/3.6, 0.5*g, P, struct('tps', 0.8));
  B = seg_blank(500); B.v(:) = opt.v0_kmh/3.6; B.w(:) = opt.v0_kmh/3.6; B.tps(:) = 0.2;
  [B.gear, B.rpm] = seg_gear_rpm(B.v, P);
  t_on = t_now + (numel(A.v) + numel(B.v)) * 1e-3;     % 제동 시작 시각
  % --- 제동 (4륜, 1e-4 s) ---
  v = opt.v0_kmh/3.6; om = v/r * ones(1, 4); a = 0; x = 0; tt = 0;
  rec = zeros(0, 8); tl = nan(1, 4); pl = nan(1, 4); k = 0;
  while v > 0.3
    p = pT * min(1, tt / 0.15);
    pw = p * [1 1 opt.rear_ratio opt.rear_ratio];
    dFz = m * (-a) * V.cg_h_m / V.wheelbase_m;
    Fz = Fz_s + [dFz/2 dFz/2 -dFz/2 -dFz/2] + 0.5*V.rho*V.ClA_m2*v^2/4;
    kap = (om*r - v) / max(v, 1);
    Fx = prc_tire_fx(P, kap, Fz);
    a = (sum(Fx) - 0.5*V.rho*V.CdA_m2*v^2) / m;
    om = max(om + dts * (-kb.*pw - Fx*r) / J, 0);
    v = max(v + a*dts, 0); x = x + v*dts; tt = tt + dts;
    lk = om*r < 0.5*v & v > 5/3.6 & isnan(tl);
    tl(lk) = tt; pl(lk) = pw(lk);
    k = k + 1;
    if mod(k, dec) == 0, rec(end+1, :) = [v, om*r, a, p, p*opt.rear_ratio]; end %#ok<AGROW>
  end
  C = seg_blank(size(rec, 1));
  C.v = rec(:, 1); C.w = rec(:, 2:5); C.ax = rec(:, 6); C.pF = rec(:, 7); C.pR = rec(:, 8);
  C.rpm(:) = P.eng.idle_rpm;
  D = seg_blank(800); D.pF(:) = pT; D.pR(:) = pT*opt.rear_ratio; D.rpm(:) = P.eng.idle_rpm;  % 정지 후 잠시 유지
  E = seg_idle(2, P);
  seg = {A, B, C, D, E};
  for q = 1:numel(seg), t_now = t_now + numel(seg{q}.v) * 1e-3; end
  parts = [parts, seg]; %#ok<AGROW>
  ev(end+1) = struct('p', pT, 't_lock', tl, 'p_lock', pl, 'stop_m', x, 'v0', opt.v0_kmh, 't_on', t_on); %#ok<AGROW>
end
X = seg_cat(parts{:});
[F, aux] = prc_make_frames(P, db, X, struct('drop_rate', 0.001));
truth.events = ev;
truth.imu_mount_deg = aux.opt.imu_mount_deg;
truth.X = X;
if ~isempty(opt.out_csv)
  prc_log_write_csv(opt.out_csv, F, sprintf('dummy brake test seed=%d (SIMULATED DATA)', opt.seed));
end
end
