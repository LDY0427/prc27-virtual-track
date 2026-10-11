function [F, truth] = gen_dummy_session(P, db, opt)
% GEN_DUMMY_SESSION  가상 주행 1회분의 원본 CAN 로그(더미 데이터)를 만든다.
%
%   [F, truth] = gen_dummy_session(P, db, struct('out_csv', 'out/dummy_session.csv'))
%
% 주행 순서 (약 30 s):
%   A 정차 아이들 5 s → B 발진 가속(sim_longitudinal, TCS ON) 5 s → C 급제동 →
%   D 스키드패드형 좌회전(R 9.125 m, 35 km/h) → E 감속·정지 → 아이들
%
% 센서 경로를 실제처럼 거친다:
%   바퀴 회전 → 톤휠 에지(지터 포함) → 에지 주기로 휠속도 계산(prc_wss_from_edges)
%   → 좌우 비교·기준 차속·슬립(prc_vehicle_ref) → TCS 로직(prc_tcs_replay)
%   → 각 신호를 DBC로 인코딩 → 원본 CAN 프레임
%
% 일부러 넣는 고장 (opt.faults = true):
%   - FL 휠센서 신호 1.0 s 끊김 (코너링 중)
%   - DAQ_Shock 프레임 60 ms 연속 누락
%   - 전체 프레임 무작위 0.3% 누락
%   - NTC T3 단선 (전 구간 무효)
% → analyze_session / tests가 이것을 찾아내는지 확인한다.
%
% opt: seed(1), mu_scale(0.7, 젖은 노면 가정), faults(true), out_csv('')
if nargin < 3, opt = struct(); end
opt = prc_opts(opt, struct('seed', 1, 'mu_scale', 0.7, 'faults', true, 'out_csv', ''));
rng(opt.seed);
dt = 1e-3; g = P.veh.g; r = P.tire.r_m;
ratios = P.drv.primary * P.drv.gears * P.drv.final;
seg = struct();

% ---------- A. 아이들 ----------
A = seg_const(5.0, dt, 0, P.eng.idle_rpm, 0);
% ---------- B. 발진 가속 ----------
L = sim_longitudinal(P, struct('tcs_on', true, 'mu_scale', opt.mu_scale, 't_end', 5, ...
                               'dt', 5e-4, 'dec', 2));
B = blank(numel(L.t));
B.v = L.v_kmh / 3.6; B.w = [B.v B.v L.vr_kmh/3.6 L.vr_kmh/3.6];
B.rpm = L.rpm; B.tps = L.tps; B.gear = L.gear; B.ax = L.a_g * g; B.cut = L.cut_act;
% ---------- C. 급제동 → 35 km/h ----------
C = seg_brake(B.v(end), 35/3.6, 1.1*g, 0.15, 45, dt, ratios, r, P);
% ---------- D. 좌회전 ----------
D = seg_corner(35/3.6, 9.125, 6.5, 95, 0.6, dt, ratios, r, P);
% ---------- E. 감속·정지 + 아이들 ----------
E1 = seg_brake(35/3.6, 0, 0.4*g, 0.2, 12, dt, ratios, r, P);
E2 = seg_const(4.0, dt, 0, P.eng.idle_rpm, 0);

parts = {A, B, C, D, E1, E2};
names = {'idle', 'launch', 'brake', 'corner', 'stop', 'idle2'};
X = blank(0); t0 = 0;
for k = 1:numel(parts)
  n = numel(parts{k}.v);
  seg.(names{k}) = [t0, t0 + (n-1)*dt];
  t0 = t0 + n*dt;
  X = cat_seg(X, parts{k});
end
N = numel(X.v);
t = (0:N-1)' * dt;

% ---------- 센서·CAN 프레임 (공용 센서 모델) ----------
X.t = t;
fl_drop = [seg.corner(1) + 2.0, seg.corner(1) + 3.0];
shock_burst = [12.000, 12.060];
fo = struct();
if opt.faults
  fo = struct('fl_dropout', fl_drop, 'shock_burst', shock_burst, 'drop_rate', 0.003, 't3_open', true);
end
[F, aux] = prc_make_frames(P, db, X, fo);

truth = struct('t', t, 'v_kmh', 3.6*X.v, 'w_kmh', 3.6*X.w, 'rpm', X.rpm, 'gear', X.gear, ...
  'tps', X.tps, 'cut', X.cut, 'ax_g', X.ax/g, 'ay_g', X.ay/g, 'yaw', X.yaw, 'steer', X.steer, 'seg', seg, ...
  'launch', L, 'tw', aux.tw, 'wss_kmh', aux.wss_kmh, 'ref', aux.ref, 'tcs', aux.tcs);
truth.faults = struct('enabled', opt.faults, 'fl_dropout', fl_drop, 'shock_burst', shock_burst, ...
  'dropped_ids', aux.dropped_ids, 't3_open', opt.faults);
truth.imu_mount_deg = aux.opt.imu_mount_deg; truth.gyro_bias_dps = aux.opt.gyro_bias_dps;
truth.mu_scale = opt.mu_scale;
if ~isempty(opt.out_csv)
  prc_log_write_csv(opt.out_csv, F, sprintf('dummy session seed=%d mu_scale=%.2f faults=%d (SIMULATED DATA)', ...
                    opt.seed, opt.mu_scale, opt.faults));
end
end

% ======================= 구간 생성 =======================
function S = blank(n)
z = zeros(n, 1);
S = struct('v', z, 'w', zeros(n, 4), 'rpm', z, 'tps', z, 'gear', z, 'ax', z, 'ay', z, ...
           'yaw', z, 'steer', z, 'pF', z, 'pR', z, 'cut', z);
end

function X = cat_seg(X, S)
f = fieldnames(X);
for i = 1:numel(f), X.(f{i}) = [X.(f{i}); S.(f{i})]; end
end

function S = seg_const(T, dt, v, rpm, gear)
n = round(T / dt);
S = blank(n);
S.v(:) = v; S.w(:) = v; S.gear(:) = gear;
S.rpm = rpm + 25 * filter(0.05, [1 -0.95], randn(n, 1)) * 4;
end

function [gear, rpm] = gear_for_speed(v, ratios, r, P)
% 내리막 변속 가정: 9000 rpm을 넘지 않는 가장 낮은 단. 저속은 클러치 분리(아이들).
gear = zeros(size(v)); rpm = P.eng.idle_rpm * ones(size(v));
for i = 1:numel(v)
  rr = v(i) / r * ratios * 60 / (2*pi);
  k = find(rr <= 9000, 1);
  if isempty(k), k = numel(ratios); end
  if v(i) > 3 / 3.6 && rr(k) >= P.eng.idle_rpm
    gear(i) = k; rpm(i) = rr(k);
  elseif v(i) > 3 / 3.6
    gear(i) = 1;
  end
end
end

function S = seg_brake(v0, v1, decel, t_ramp, p_front, dt, ratios, r, P)
v = v0; a = 0; tt = 0; vv = []; aa = [];
while v > v1 + 1e-6
  a = -decel * min(1, tt / t_ramp);
  vv(end+1, 1) = v; aa(end+1, 1) = a; %#ok<AGROW>
  v = max(v + a * dt, v1); tt = tt + dt;
end
n = numel(vv);
S = blank(n);
S.v = vv; S.w = repmat(vv, 1, 4); S.ax = aa;
S.pF = p_front * (-aa / decel); S.pR = 0.6 * S.pF;
[S.gear, S.rpm] = gear_for_speed(vv, ratios, r, P);
end

function S = seg_corner(v, R, T, steer_max, t_ramp, dt, ratios, r, P)
n = round(T / dt);
tt = (0:n-1)' * dt;
s = min(1, min(tt / t_ramp, (T - tt) / t_ramp));
s = max(s, 0);
k = s / R;                                    % 곡률 1/m (좌회전 +)
S = blank(n);
S.v(:) = v;
S.w = [v*(1 - k*P.veh.track_f_m/2), v*(1 + k*P.veh.track_f_m/2), ...
       v*(1 - k*P.veh.track_r_m/2), v*(1 + k*P.veh.track_r_m/2)];
S.ay = v^2 * k; S.yaw = v * k * 180/pi; S.steer = steer_max * s;
S.tps(:) = 0.35;
[S.gear, S.rpm] = gear_for_speed(S.v, ratios, r, P);
end
