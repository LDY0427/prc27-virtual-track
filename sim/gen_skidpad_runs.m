function [F, truth] = gen_skidpad_runs(P, db, opt)
% GEN_SKIDPAD_RUNS  조향 특성 시험 더미 로그: 일정 반경(스키드패드 R 9.125 m)에서 속도를 천천히 올린다.
%
%   [F, truth] = gen_skidpad_runs(P, db, struct('out_csv', 'out/dummy_skidpad.csv'))
%
% 절차: 정차 5 s → [ 출발 → 15 km/h에서 진입 → 15→42 km/h를 30 s 동안 서서히 → 탈출 → 정지 ]
%       × 좌회전·우회전
% 정답 조향 특성 (선형 자전거 모델 + 한계 근처 비선형):
%   전륜 조향각 = 축거/반경 + K_us·|ay| + 4.0·(|ay| − 1.1)^2 (|ay| > 1.1 g)   [deg, ay in g]
%   K_us = opt.K_us (언더스티어 기울기, 기본 1.2 deg/g)
if nargin < 3, opt = struct(); end
opt = prc_opts(opt, struct('seed', 4, 'out_csv', '', 'K_us', 1.2, 'R', 9.125, ...
                           'v_lo', 15, 'v_hi', 42, 'T_ramp', 30));
rng(opt.seed);
g = P.veh.g; L = P.veh.wheelbase_m; tf = P.veh.track_f_m; tr = P.veh.track_r_m;
dt = 1e-3;
parts = {seg_idle(5, P)};
for dir = [1 -1]
  A = seg_kin(0, opt.v_lo/3.6, 0.3*g, P);
  n1 = round(1.5/dt); n2 = round(opt.T_ramp/dt);
  v = [opt.v_lo/3.6 * ones(n1, 1); linspace(opt.v_lo, opt.v_hi, n2)'/3.6; opt.v_hi/3.6 * ones(n1, 1)];
  s = [linspace(0, 1, n1)'; ones(n2, 1); linspace(1, 0, n1)'];
  k = dir * s / opt.R;                                   % 곡률 (좌회전 +)
  B = seg_blank(numel(v));
  B.v = v; B.ax = [0; diff(v)] / dt;
  B.ay = v.^2 .* k; B.yaw = v .* k * 180/pi;
  B.w = [v.*(1 - k*tf/2), v.*(1 + k*tf/2), v.*(1 - k*tr/2), v.*(1 + k*tr/2)];
  ayg = abs(B.ay) / g;
  dw = L * k * 180/pi + sign(k) .* (opt.K_us * ayg + 4.0 * max(ayg - 1.1, 0).^2);
  B.steer = interp1(P.steer.wheel_tab, P.steer.swa_tab, dw, 'linear', 'extrap');
  B.tps(:) = 0.3;
  [B.gear, B.rpm] = seg_gear_rpm(v, P);
  C = seg_kin(opt.v_hi/3.6, 0, 0.4*g, P, struct('pF', 15));
  D = seg_idle(2, P);
  parts = [parts, {A, B, C, D}]; %#ok<AGROW>
end
X = seg_cat(parts{:});
[F, aux] = prc_make_frames(P, db, X, struct('drop_rate', 0.001));
truth.K_us = opt.K_us; truth.R = opt.R;
truth.roll_grad_deg_g = P.sim.roll_grad_deg_g;
truth.imu_mount_deg = aux.opt.imu_mount_deg; truth.gyro_bias_dps = aux.opt.gyro_bias_dps;
truth.X = X;
if ~isempty(opt.out_csv)
  prc_log_write_csv(opt.out_csv, F, sprintf('dummy skidpad steer test seed=%d (SIMULATED DATA)', opt.seed));
end
end
