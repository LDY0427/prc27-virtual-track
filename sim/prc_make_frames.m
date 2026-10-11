function [F, aux] = prc_make_frames(P, db, X, opt)
% PRC_MAKE_FRAMES  차량 상태 시계열(정답) → 센서 측정 → 원본 CAN 프레임 (공용 센서 모델).
%
%   [F, aux] = prc_make_frames(P, db, X, opt)
%
% X (모두 N×1, 등간격 X.t):
%   v 차속 m/s, w N×4 바퀴 주변속도 m/s [FL FR RL RR], rpm, tps 0..1, gear, cut 0..1,
%   ax, ay 차체 가속도 m/s^2 (y 왼쪽 +), yaw deg/s (좌회전 +), steer 핸들각 deg,
%   pF, pR 제동압 bar.  선택: grade 노면 경사 rad (오르막 +), roll rad (오른쪽 내려감 +),
%   pitch rad (앞 들림 +). 없으면 P.sim의 롤·피치 기울기로 만든다. lambda (람다 센서 값, 없으면 TPS로 대충).
%
% 센서 모델:
%   휠속도  바퀴 회전 → 톤휠 에지(지터 2 µs) → 에지 주기 계산 (펌웨어 방식)
%   IMU     비력 = 가속도 + 중력 성분(경사·피치·롤) → 장착 오차 회전 → 영점 오차 + 잡음
%   나머지  샘플-홀드 + 잡음
%
% opt: fl_dropout [t0 t1] (FL 에지 제거), shock_burst [t0 t1], drop_rate (무작위 누락 비율),
%      t3_open (NTC T3 단선), imu_mount_deg, gyro_bias_dps, heading0_deg, clt0
if nargin < 4, opt = struct(); end
opt = prc_opts(opt, struct('fl_dropout', [], 'shock_burst', [], 'drop_rate', 0, 't3_open', false, ...
  'imu_mount_deg', P.imu_sim.mount_deg, 'gyro_bias_dps', P.imu_sim.gyro_bias_dps, ...
  'heading0_deg', 0, 'clt0', 82));
t = X.t(:); N = numel(t); dt = t(2) - t(1); T_end = t(end);
g = P.veh.g; r = P.tire.r_m;
z = zeros(N, 1);
if ~isfield(X, 'grade'), X.grade = z; end
if ~isfield(X, 'roll'),  X.roll  = P.sim.roll_grad_deg_g  * pi/180 * X.ay / g; end
if ~isfield(X, 'pitch'), X.pitch = P.sim.pitch_grad_deg_g * pi/180 * X.ax / g; end
if ~isfield(X, 'cut'),   X.cut = z; end
if ~isfield(X, 'lambda'), X.lambda = 1.0 - 0.12 * X.tps; end   % 람다 센서 값(지연 포함)을 주면 그대로 쓴다
if ~isfield(X, 'ign'), X.ign = P.ign.base(X.rpm, X.tps * 100); end   % 점화각(ECU 보고값)

% ---------- 휠속도 ----------
tw = (0.003:1/P.wss.rate_hz:T_end - 0.01)';
wkmh = zeros(numel(tw), 4);
for k = 1:4
  rev = cumsum(X.w(:, k)) * dt / (2*pi*r);
  te = prc_wss_edges(t, rev, P.wss.teeth);
  te = sort(te + 2e-6 * randn(size(te)));
  if k == 1 && ~isempty(opt.fl_dropout)
    te = te(te < opt.fl_dropout(1) | te > opt.fl_dropout(2));
  end
  wkmh(:, k) = prc_wss_from_edges(te, tw, P.wss.teeth, r, P.wss.timeout_s);
end
% 로거/ECU 쪽 TCS (정렬된 요레이트를 쓴다고 가정)
yaw_w = prc_hold(t, X.yaw, tw);
R = prc_vehicle_ref(wkmh, true(size(wkmh)), P, yaw_w);
Trep = prc_tcs_replay(R.slip, R.ref_ok & R.drv_ok, prc_hold(t, X.tps, tw), true, true, P);

% ---------- IMU ----------
th = X.grade + X.pitch;
f_b = [X.ax + g*sin(th), X.ay + g*sin(X.roll), g*cos(th).*cos(X.roll)] / g;     % g
rr = [0; diff(X.roll)] / dt * 180/pi; pr = [0; diff(X.pitch + X.grade)] / dt * 180/pi;
w_b = [rr, pr, X.yaw];                                                        % deg/s
Rm = prc_rotm(opt.imu_mount_deg);
f_s = (Rm' * f_b')';
w_s = (Rm' * w_b')' + repmat(opt.gyro_bias_dps(:)', N, 1);

% ---------- 기타 ----------
heading = opt.heading0_deg + cumsum(X.yaw) * dt;
px = cumsum(X.v .* cosd(heading)) * dt;
py = cumsum(X.v .* sind(heading)) * dt;
lat0 = 35.2300; lon0 = 129.0800;                    % 가상 기준 좌표
lat = lat0 + py / 111320;
lon = lon0 + px / (111320 * cosd(lat0));
MAP = 32 + 66 * X.tps;
CLT = opt.clt0 + 9 * t / T_end;
bump = filter(0.08, [1 -0.92], randn(N, 4));
axg = X.ax / g; ayg = X.ay / g;
shock = 30 + [ 8*(-axg) - 7*ayg, 8*(-axg) + 7*ayg, ...
               6*axg   - 7*ayg,  6*axg   + 7*ayg] + 0.4 * bump;

% ---------- 프레임 ----------
FR = struct('t', {}, 'id', {}, 'data', {});
h = @(x, tq) prc_hold(t, x, tq);
te50 = (0.001:0.02:T_end)'; n = numel(te50);
FR = prc_frames_add(FR, db, 'EMU_SIM_0', te50, struct( ...
  'RPM', h(X.rpm, te50) + 5*randn(n,1), 'TPS', 100*h(X.tps, te50), 'IAT', 27, ...
  'MAP', h(MAP, te50) + 0.5*randn(n,1), 'InjPw', 0.9 + 7*h(MAP, te50)/100));
FR = prc_frames_add(FR, db, 'EMU_SIM_1', te50 + 0.0005, struct( ...
  'CLT', h(CLT, te50) + 0.1*randn(n,1), 'Lambda', h(X.lambda, te50) + 0.01*randn(n,1), ...
  'IgnAngle', h(X.ign, te50), ...
  'Gear', h(X.gear, te50), 'Vbat', 13.9 + 0.05*randn(n,1), 'TcCutPct', 100*h(X.cut, te50)));
FR = prc_frames_add(FR, db, 'WSS_Speed', tw, struct('WssFL', wkmh(:,1), 'WssFR', wkmh(:,2), ...
  'WssRL', wkmh(:,3), 'WssRR', wkmh(:,4)));
FR = prc_frames_add(FR, db, 'WSS_TCS', tw + 0.0002, struct('WssValid', 15, 'TcsMode', Trep.mode, ...
  'TcsReason', Trep.reason, 'VehSpeedRef', R.v_ref, 'SlipDrive', R.slip, 'TcsCutReq', 100*Trep.cut));
tc = (0.005:0.01:T_end)'; n = numel(tc);
pF = max(h(X.pF, tc) + 0.3*randn(n,1), 0); pR = max(h(X.pR, tc) + 0.3*randn(n,1), 0);
FR = prc_frames_add(FR, db, 'DAQ_Chassis', tc, struct('SteerAngle', h(X.steer, tc) + 0.2*randn(n,1), ...
  'BrkPressF', pF, 'BrkPressR', pR, 'ChassisValid', 7, 'BrkLight', double(pF > 3)));
ts = (0.007:0.01:T_end)';
FR = prc_frames_add(FR, db, 'DAQ_Shock', ts, struct('ShockFL', h(shock(:,1), ts), 'ShockFR', h(shock(:,2), ts), ...
  'ShockRL', h(shock(:,3), ts), 'ShockRR', h(shock(:,4), ts), 'ShockValid', 15, 'ShockSat', 0));
ti = (0.011:0.02:T_end)'; n = numel(ti);
an = P.imu_sim.acc_noise_g; gn = P.imu_sim.gyro_noise_dps;
FR = prc_frames_add(FR, db, 'DAQ_IMU_Acc', ti, struct('AccX', h(f_s(:,1), ti) + an*randn(n,1), ...
  'AccY', h(f_s(:,2), ti) + an*randn(n,1), 'AccZ', h(f_s(:,3), ti) + an*randn(n,1), 'ImuStatus', 1));
FR = prc_frames_add(FR, db, 'DAQ_IMU_Gyro', ti + 0.0003, struct('GyroX', h(w_s(:,1), ti) + gn*randn(n,1), ...
  'GyroY', h(w_s(:,2), ti) + gn*randn(n,1), 'GyroZ', h(w_s(:,3), ti) + gn*randn(n,1), 'ImuStatus', 1));
tt = (0.013:0.1:T_end)'; n = numel(tt);
T3 = 60 * ones(n, 1); tv = 15;
if opt.t3_open, T3(:) = NaN; tv = 11; end
FR = prc_frames_add(FR, db, 'DAQ_Temp', tt, struct('T1', h(CLT, tt) + 1, 'T2', h(CLT, tt) - 11, ...
  'T3', T3, 'T4', 24 + 0.1*randn(n,1), 'TempValid', tv));
tg = (0.05:0.1:T_end)'; n = numel(tg);
FR = prc_frames_add(FR, db, 'DAQ_GNSS_Pos', tg, struct('Latitude', h(lat, tg), 'Longitude', h(lon, tg)));
FR = prc_frames_add(FR, db, 'DAQ_GNSS_Vel', tg + 0.0004, struct('GnssSpeed', max(3.6*h(X.v, tg) + 0.2*randn(n,1), 0), ...
  'GnssHeading', mod(90 - h(heading, tg), 360), 'NumSV', 12, 'FixType', 3, 'HAcc', 0.8));
tsx = (0.25:0.5:T_end)';
FR = prc_frames_add(FR, db, 'DAQ_Status', tsx, struct('DaqUptime', 180 + floor(tsx), 'LogState', 2, ...
  'DaqErrFlags', 0, 'SdFree', 87, 'DropCount', 0, 'DaqVbat', 13.8));

% ---------- 합치기·고장 ----------
tA = vertcat(FR.t); idA = vertcat(FR.id); dA = vertcat(FR.data);
keep = true(size(tA));
if ~isempty(opt.shock_burst)
  idS = prc_dbc_msg(db, 'DAQ_Shock').id;
  keep(idA == idS & tA >= opt.shock_burst(1) & tA < opt.shock_burst(2)) = false;
end
if opt.drop_rate > 0
  keep(rand(size(tA)) < opt.drop_rate) = false;
end
aux.dropped_ids = idA(~keep);
tA = tA(keep); idA = idA(keep); dA = dA(keep, :);
tA = tA + 3e-4 * rand(size(tA));                    % 수신 시각 지터
[tA, o] = sort(tA);
F.t_us = round(tA * 1e6); F.bus = ones(size(tA)); F.id = idA(o);
F.dlc = 8 * ones(size(tA)); F.data = dA(o, :);
aux.tw = tw; aux.wss_kmh = wkmh; aux.ref = R; aux.tcs = Trep;
aux.f_body_g = f_b; aux.Rm = Rm; aux.opt = opt;
end
