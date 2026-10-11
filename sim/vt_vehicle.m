function [st, out] = vt_vehicle(st, u, P, dt)
% VT_VEHICLE  평면 2트랙 차량 모델 한 스텝 (CarMaker Vehicle의 역할, 단순화).
%
%   자유도: 차체 vx, vy, 요레이트 r (+ 위치 x, y, psi), 바퀴 회전 4개, 엔진·기어 상태
%   입력 u: sw_deg 핸들각, tps 0..1, pF_bar 전륜 제동압, tcs_cut 0..1 (TCS 토크 감소율)
%   조향: 애커만 LUT → 좌/우 바퀴각.  타이어: 횡 매직포뮬러(하중 민감도) + 종 prc_tire_fx, 마찰 타원 결합
%   수직하중: 정적 + 종 하중이동 + 축별 롤강성 분담에 따른 횡 하중이동 + 다운포스(앞/뒤 배분)
%   구동: 엔진 토크 → 기어·종감속 → 뒷바퀴 (open 차동 = 좌우 같은 토크, locked = 같은 속도)
%   없는 것: 서스펜션 운동학·캠버 변화·롤/피치 자유도·릴랙세이션 길이·브레이크 온도
%
%   st = vt_vehicle_init(P, v0) 로 초기 상태. out에 힘·슬립·수직하중·가속도.
V = P.veh; T = P.tire; E = P.eng; Dr = P.drv; B = P.brk;
g = V.g; m = V.mass_kg; L = V.wheelbase_m; rw = T.r_m;
a = L * V.rear_frac; b = L - a;                      % CG→앞축 a, CG→뒤축 b (앞 하중 b/L)
tf = V.track_f_m; tr = V.track_r_m;
ratios = Dr.primary * Dr.gears * Dr.final;

% ---------- 조향 ----------
[dL, dR] = prc_steer_lut(P, u.sw_deg);
delta = [dL dR 0 0] * pi/180;                        % FL FR RL RR
px = [a a -b -b]; py = [tf/2 -tf/2 tr/2 -tr/2];      % 바퀴 위치 (x 앞, y 왼쪽)

% ---------- 바퀴 중심 속도·슬립 ----------
vx = st.vx; vy = st.vy; r = st.r;
vwx = vx - r * py; vwy = vy + r * px;
vt_ = vwx .* cos(delta) + vwy .* sin(delta);         % 바퀴 진행 방향 속도
vn_ = -vwx .* sin(delta) + vwy .* cos(delta);        % 바퀴 측방 속도
alpha = atan2(-vn_, max(vt_, T.v_floor));                 % 슬립각 (좌회전 시 +)
kappa = (st.om * rw - vt_) ./ max(abs(vt_), T.v_floor);

% ---------- 수직하중 ----------
v = hypot(vx, vy); q = 0.5 * V.rho * v^2;
Fdn = q * V.ClA_m2; Fdn_f = Fdn * (1 - V.aero_rear_frac); Fdn_r = Fdn * V.aero_rear_frac;
ax_prev = st.ax; ay_prev = st.ay;
Wf = m*g*b/L - m*ax_prev*V.cg_h_m/L + Fdn_f;
Wr = m*g*a/L + m*ax_prev*V.cg_h_m/L + Fdn_r;
dFf = m*ay_prev*V.cg_h_m*V.roll_stiff_dist_f / tf;           % 좌회전(ay>0) → 오른쪽(바깥) 증가
dFr = m*ay_prev*V.cg_h_m*(1 - V.roll_stiff_dist_f) / tr;
Fz = max([Wf/2 - dFf, Wf/2 + dFf, Wr/2 - dFr, Wr/2 + dFr], 0);

% ---------- 구동·제동 토크 ----------
om_r = (st.om(3) + st.om(4)) / 2;
rpm_wheel = om_r * ratios(st.gear) * 60/(2*pi);
if ~st.locked && rpm_wheel >= E.launch_rpm, st.locked = true; end
if st.locked
  st.rpm = max(rpm_wheel, E.idle_rpm);
  if ~st.shifting && st.rpm >= Dr.shift_rpm && st.gear < numel(ratios), st.shifting = true; st.t_shift = 0; end
  if st.gear > 1 && st.rpm < 4000 && u.tps < 0.3 && ~st.shifting, st.gear = st.gear - 1; end   % 간단한 다운시프트
else
  st.rpm = E.launch_rpm;
end
if st.shifting
  st.t_shift = st.t_shift + dt;
  if st.t_shift >= Dr.shift_time_s, st.shifting = false; st.gear = st.gear + 1; end
end
if isfield(E, 'grid_wot')
  ir = min(round(st.rpm) + 1, numel(E.grid_rpm));
  Te = (u.tps * E.grid_wot(ir) - (1 - u.tps) * E.grid_fric(ir)) * (1 - u.tcs_cut);
else
  Te = prc_engine_torque(P, st.rpm, u.tps) * (1 - u.tcs_cut);
end
if st.shifting, Te = 0; end
T_axle = Te * ratios(st.gear) * Dr.eta;
Tb = u.pF_bar * [B.k_front_Nm_bar B.k_front_Nm_bar B.k_rear_Nm_bar*B.rear_ratio B.k_rear_Nm_bar*B.rear_ratio];
Tb = Tb .* sign(st.om + 1e-9);                       % 제동은 회전 반대
if strcmp(Dr.diff, 'locked')
  Td = [0 0 T_axle/2 T_axle/2];                      % 평균 속도로 묶는다 (아래)
else
  Td = [0 0 T_axle/2 T_axle/2];                      % open: 좌우 같은 토크
end

% ---------- 타이어 힘 ----------
mu_y = max(T.mu_y0 - T.load_sens * (Fz - T.Fz_ref_N), 0.3);
Calpha = T.Cy_per_N * Fz * 180/pi;                   % N/rad
By = Calpha ./ max(T.Cy * mu_y .* Fz, 1e-6);
Fy0 = mu_y .* Fz .* sin(T.Cy * atan(By .* alpha));
Fx0 = prc_tire_fx(P, kappa, Fz);
mu_x = T.mu;
% 마찰 타원: 종력 우선, 횡력은 남은 만큼
rho = min(abs(Fx0) ./ max(mu_x * Fz, 1e-6), 1);
Fy = Fy0 .* sqrt(1 - rho.^2);
Fx = Fx0;
Fxb = Fx .* cos(delta) - Fy .* sin(delta);
Fyb = Fx .* sin(delta) + Fy .* cos(delta);

% ---------- 차체 운동 ----------
Fdrag = q * V.CdA_m2 + (vx > 0.05) * V.Crr * m * g;
ax = (sum(Fxb) - Fdrag) / m + vy * r;
ay = sum(Fyb) / m - vx * r;
Mz = sum(px .* Fyb - py .* Fxb);
rdot = Mz / V.Izz_kgm2;
st.vx = max(st.vx + ax * dt, 0);
st.vy = st.vy + ay * dt;
st.r = st.r + rdot * dt;
if st.vx < 0.3, st.vy = 0; st.r = 0; end
st.psi = st.psi + st.r * dt;
st.x = st.x + (st.vx * cos(st.psi) - st.vy * sin(st.psi)) * dt;
st.y = st.y + (st.vx * sin(st.psi) + st.vy * cos(st.psi)) * dt;
st.ax = ax - vy * r; st.ay = ay + vx * r;            % 차체 좌표 가속도 (IMU가 재는 값)

% ---------- 바퀴 회전 ----------
J = T.J_wheel_kgm2 * ones(1, 4);
J(3:4) = J(3:4) + E.J_kgm2 * ratios(st.gear)^2 / 2;
dom = (Td - Tb - Fx * rw) ./ J;
st.om = max(st.om + dom * dt, 0);
if strcmp(Dr.diff, 'locked'), st.om(3:4) = mean(st.om(3:4)); end
st.rev = st.rev + st.om * dt / (2*pi);

out = struct('Fz', Fz, 'Fx', Fx, 'Fy', Fy, 'alpha', alpha, 'kappa', kappa, 'delta', delta, ...
             'ax', st.ax, 'ay', st.ay, 'rpm', st.rpm, 'gear', st.gear, 'Te', Te, 'v', v);
end
