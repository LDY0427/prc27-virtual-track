function S = vt_run(P, road, opt)
% VT_RUN  가상 시험장 실행: 도로 + 준정상상태 목표 속도 + 운전자 + 2트랙 차량 (+ 센서 CAN 로그).
%
%   S = vt_run(P, 'skidpad')                      % 또는 vt_road(...) 구조체
%   S = vt_run(P, 'autocross', struct('laps', 2, 'tcs', true, 'log_csv', 'out/vt_autocross.csv'))
%
%   S.t, x, y, v_kmh, sw_deg, tps, pF, ax_g, ay_g, yaw_dps, slip4, Fz4, rpm, gear, e_lat, s
%   S.lap_time_s (폐곡선·accel·skidpad 계측 구간), S.qss (준정상상태 비교값), S.offtrack (코스 이탈 여부)
%   log_csv를 주면 prc_make_frames로 센서 CAN 로그를 만든다 → analyze_* 함수에 바로 넣을 수 있다.
if nargin < 3, opt = struct(); end
opt = prc_opts(opt, struct('laps', 1, 'tcs', false, 'log_csv', '', 'dt', 2e-3, 'v0', 0, 't_max', 120, 'mu_scale', 1.0, ...
                           'v_ramp', []));   % v_ramp = [시작 배율, 끝 배율, 램프 시간 s]: 운전자 목표 속도를 천천히 올림
if ischar(road), R = vt_road(road); else, R = road; end
P.tire.mu = P.tire.mu * opt.mu_scale; P.tire.mu_y0 = P.tire.mu_y0 * opt.mu_scale;
Q = laptime_qss(P, R, struct('v0', opt.v0, 'closed_ss', R.closed));
% 속도용 엔진 표 (루프 안 interp1 제거)
P.eng.grid_rpm = 0:1:(P.eng.rpm_limit + 2000);
P.eng.grid_wot = interp1(P.eng.rpm_tab, P.eng.trq_tab, min(max(P.eng.grid_rpm, P.eng.rpm_tab(1)), P.eng.rpm_tab(end)));
P.eng.grid_fric = P.eng.fric_a + P.eng.fric_b * P.eng.grid_rpm / 1000;
P.eng.grid_wot(P.eng.grid_rpm >= P.eng.rpm_limit) = -P.eng.grid_fric(P.eng.grid_rpm >= P.eng.rpm_limit);
dt = opt.dt; g = P.veh.g;
st = vt_vehicle_init(P, opt.v0, R.x(1), R.y(1), R.psi(1));
ds = struct('idx', 1, 's', 0, 'sw', 0, 'ie', 0, 'e_lat', 0, 'v_ref', 0);
nmax = round(opt.t_max / dt); dec = round(0.01 / dt); no = floor(nmax / dec);
f = {'t','x','y','v_kmh','sw_deg','tps','pF','ax_g','ay_g','yaw_dps','rpm','gear','e_lat','s','tcs_cut','v_ref_kmh'};
for k = 1:numel(f), S.(f{k}) = zeros(no, 1); end
S.slip4 = zeros(no, 4); S.Fz4 = zeros(no, 4); S.w4 = zeros(no, 4); S.alpha4 = zeros(no, 4);
tcs = prc_tcs_init(); cut = 0; nctl = round(P.tcs.Ts / dt);
io = 0; laps = 0; s_prev = 0; t_lap0 = 0; lap_times = []; done = false;
mark_t = struct();
for i = 1:nmax
  t = i * dt;
  if ~isempty(opt.v_ramp)
    P.driver.v_scale = opt.v_ramp(1) + (opt.v_ramp(2) - opt.v_ramp(1)) * min(t / opt.v_ramp(3), 1);
  end
  [u, ds] = vt_driver(ds, st, R, Q, P, dt);
  if opt.tcs && mod(i, nctl) == 0
    wk = st.om * P.tire.r_m * 3.6;
    Rr = prc_vehicle_ref(wk, true(1, 4), P, st.r * 180/pi);
    [tcs, o] = prc_tcs_step(tcs, struct('slip', Rr.slip, 'ok', true, 'tps', u.tps, 'sw', true, 'can_ok', true), P);
    cut = o.cut;
  end
  u.tcs_cut = cut;
  [st, o2] = vt_vehicle(st, u, P, dt);
  % 랩·마크
  if R.closed && ds.s < s_prev - R.length_m/2
    laps = laps + 1; lap_times(end+1) = t - t_lap0; t_lap0 = t; %#ok<AGROW>
    if laps >= opt.laps, done = true; end
  end
  for q = 1:numel(R.marks)
    nm = R.marks(q).name;
    if ~isfield(mark_t, nm) && ds.s >= R.marks(q).s && s_prev < R.marks(q).s, mark_t.(nm) = t; end
  end
  s_prev = ds.s;
  if ~R.closed && ds.s >= R.length_m - 1, done = true; end
  if mod(i, dec) == 0
    io = io + 1;
    S.t(io) = t; S.x(io) = st.x; S.y(io) = st.y; S.v_kmh(io) = st.vx * 3.6;
    S.sw_deg(io) = u.sw_deg; S.tps(io) = u.tps; S.pF(io) = u.pF_bar;
    S.ax_g(io) = st.ax / g; S.ay_g(io) = st.ay / g; S.yaw_dps(io) = st.r * 180/pi;
    S.rpm(io) = o2.rpm; S.gear(io) = o2.gear; S.e_lat(io) = ds.e_lat; S.s(io) = ds.s;
    S.tcs_cut(io) = cut; S.v_ref_kmh(io) = ds.v_ref * 3.6;
    S.slip4(io, :) = o2.kappa; S.Fz4(io, :) = o2.Fz; S.w4(io, :) = st.om * P.tire.r_m; S.alpha4(io, :) = o2.alpha * 180/pi;
  end
  if done, break; end
end
fn = fieldnames(S);
for k = 1:numel(fn), S.(fn{k}) = S.(fn{k})(1:io, :); end
S.road = R; S.qss = Q; S.marks_t = mark_t; S.laps = lap_times;
S.offtrack = max(abs(S.e_lat)) > R.width_m / 2;
S.e_lat_max = max(abs(S.e_lat));
if R.closed && ~isempty(lap_times), S.lap_time_s = lap_times(end);
elseif isfield(mark_t, 'timing_end') && isfield(mark_t, 'timing_start'), S.lap_time_s = mark_t.timing_end - mark_t.timing_start;
elseif isfield(mark_t, 'finish_75m'), S.lap_time_s = mark_t.finish_75m;
else, S.lap_time_s = NaN; end
if isfield(Q, 'marks') && isfield(Q.marks, 'timing_end'), S.qss_time_s = Q.marks.timing_end - Q.marks.timing_start;
elseif isfield(Q, 'marks') && isfield(Q.marks, 'finish_75m'), S.qss_time_s = Q.marks.finish_75m;
else, S.qss_time_s = Q.lap_time_s; end
% ---------- 센서 로그 ----------
if ~isempty(opt.log_csv)
  % 앞에 3 s 정차(IMU 정적 보정용)를 붙이고 1 ms로 보간해 센서 모델에 넣는다
  X = seg_blank(io);
  X.v = S.v_kmh / 3.6; X.w = S.w4; X.rpm = S.rpm; X.tps = S.tps; X.gear = S.gear;
  X.cut = S.tcs_cut; X.ax = S.ax_g * g; X.ay = S.ay_g * g; X.yaw = S.yaw_dps; X.steer = S.sw_deg;
  X.pF = S.pF; X.pR = S.pF * P.brk.rear_ratio;
  tq = (0:1e-3:(S.t(end) - S.t(1)))'; ts = S.t - S.t(1);
  fn2 = fieldnames(X);
  for k = 1:numel(fn2)
    if size(X.(fn2{k}), 1) == io, X.(fn2{k}) = interp1(ts, X.(fn2{k}), tq); end
  end
  X.rpm(isnan(X.rpm)) = P.eng.idle_rpm;
  X = seg_cat(seg_idle(6, P), X);
  db = prc_dbc_load(fullfile(fileparts(mfilename('fullpath')), '..', 'dbc', 'PRC27_sim_v0.1.dbc'));
  F = prc_make_frames(P, db, X, struct('drop_rate', 0));
  prc_log_write_csv(opt.log_csv, F, sprintf('virtual test track %s (SIMULATED DATA)', R.name));
  S.log_csv = opt.log_csv;
end
end
