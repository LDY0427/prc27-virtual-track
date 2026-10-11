function R = sim_longitudinal(P, opt)
% SIM_LONGITUDINAL  직선 가속(Acceleration 75 m) 종방향 시뮬레이션 + 휠스피드 TCS.
%
%   R = sim_longitudinal(P, struct('tcs_on', true, 'mu_scale', 0.7))
%
% 모델 (모두 단순화):
%   - 차체 1자유도(속도), 후륜 두 바퀴를 한 축으로 묶은 회전 1자유도 (스풀/LSD 가정)
%   - 전륜은 자유 회전(전륜 속도 = 차속)
%   - 발진: 바퀴 기준 엔진 회전수가 launch_rpm보다 낮은 동안은 launch_rpm의 토크를 낸다
%     (클러치 미끄럼 단순화). 엔진 관성은 처음부터 기어비 제곱으로 환산해 축 관성에 더한다
%     (클러치가 엔진을 끌어내리는 효과를 근사. 실제 클러치 조작 모델 아님)
%   - 변속: shift_rpm 도달 시 shift_time 동안 토크 0 후 다음 단
%   - 후륜 수직하중 = 정적 + 종방향 하중 이동 + 다운포스 후륜 분담
%   - TCS: P.tcs.Ts 주기로 슬립 계산 → 토크 감소율 → 지연(act_delay) + 1차 지연(torque_tau)
%
% opt 필드 (기본값):
%   tcs_on (true), mu_scale (1.0, 노면 마찰 배율), t_end (6 s), dist_m (75),
%   dt (5e-4 s), dec (2, 출력 간격 = dt*dec), tps_t0 (0.1 s, 풀스로틀 시작)
%
% 출력 R: t, x, v_kmh, vf_kmh(전륜), vr_kmh(후륜), rpm, gear, kappa, Fx, Fz_r,
%         cut_cmd, cut_act, tcs_mode, slip_meas, tps, a_g, rev_f, rev_r, t75, v75_kmh
if nargin < 2, opt = struct(); end
opt = prc_opts(opt, struct('tcs_on', true, 'mu_scale', 1.0, 't_end', 6, 'dist_m', 75, ...
                           'dt', 5e-4, 'dec', 2, 'tps_t0', 0.1));
P.tire.mu = P.tire.mu * opt.mu_scale;
V = P.veh; T = P.tire; E = P.eng; D = P.drv; C = P.tcs;
m = V.mass_kg; g = V.g; r = T.r_m; dt = opt.dt;
ratios = D.primary * D.gears * D.final;
% 속도용: 엔진 토크(WOT)와 마찰 토크를 1 rpm 간격 표로 미리 계산 (루프 안 interp1 호출 제거)
rpm_grid = 0:1:(E.rpm_limit + 2000);
Twot_tab = interp1(E.rpm_tab, E.trq_tab, min(max(rpm_grid, E.rpm_tab(1)), E.rpm_tab(end)));
Tfric_tab = E.fric_a + E.fric_b * rpm_grid / 1000;
Twot_tab(rpm_grid >= E.rpm_limit) = -Tfric_tab(rpm_grid >= E.rpm_limit);
nmax = numel(rpm_grid);
mu = T.mu; Bt = T.B; Ct = T.C; Et = T.E;

n = round(opt.t_end / dt);
no = floor(n / opt.dec);
f = {'t','x','v_kmh','vf_kmh','vr_kmh','rpm','gear','kappa','Fx','Fz_r', ...
     'cut_cmd','cut_act','tcs_mode','slip_meas','tps','a_g','rev_f','rev_r'};
for k = 1:numel(f), R.(f{k}) = zeros(no, 1); end

x = 0; v = 0; wr = 0; gear = 1; rpm = E.launch_rpm; locked = false;
shift_t0 = -inf; shifting = false;
cut_cmd = 0; cut_act = 0; a = 0; mode = 1; slip_meas = 0;
rev_f = 0; rev_r = 0; t75 = NaN; v75 = NaN;
nctl = max(1, round(C.Ts / dt));
ndel = max(0, round(C.act_delay_s / C.Ts));
dbuf = zeros(1, ndel + 1);
st = prc_tcs_init();
io = 0;

for i = 0:n-1
  t = i * dt;
  tps = double(t >= opt.tps_t0);

  % ---- TCS (제어 주기) ----
  if mod(i, nctl) == 0
    vf = v * 3.6; vr = wr * r * 3.6;
    in.slip = (vr - vf) / max(vf, C.v_floor_kmh);
    in.ok = true; in.tps = tps; in.sw = opt.tcs_on; in.can_ok = true;
    [st, o] = prc_tcs_step(st, in, P);
    dbuf = [dbuf(2:end) o.cut];
    cut_cmd = dbuf(1); mode = o.mode; slip_meas = in.slip;
  end
  cut_act = cut_act + (cut_cmd - cut_act) * dt / C.torque_tau_s;

  % ---- 변속 ----
  if shifting && t - shift_t0 >= D.shift_time_s
    shifting = false; gear = gear + 1;
  end
  ratio = ratios(gear);
  rpm_wheel = wr * ratio * 60 / (2*pi);
  if ~locked && rpm_wheel >= E.launch_rpm, locked = true; end
  if locked
    rpm = max(rpm_wheel, E.idle_rpm);
    if ~shifting && rpm >= D.shift_rpm && gear < numel(ratios)
      shifting = true; shift_t0 = t;
    end
  else
    rpm = E.launch_rpm;
  end

  % ---- 구동 토크 (prc_engine_torque와 같은 계산, 표 조회) ----
  ir = min(round(rpm) + 1, nmax);
  Te = tps * Twot_tab(ir) - (1 - tps) * Tfric_tab(ir);
  if shifting, Te = 0; end
  Te = Te * (1 - cut_act);
  T_axle = Te * ratio * D.eta;
  J = 2 * T.J_wheel_kgm2 + E.J_kgm2 * ratio^2;

  % ---- 타이어·차체 ----
  q = 0.5 * V.rho * v^2;
  Fz_r = m * g * V.rear_frac + m * a * V.cg_h_m / V.wheelbase_m + q * V.ClA_m2 * V.aero_rear_frac;
  kappa = (wr * r - v) / max(v, T.v_floor);
  Bk = Bt * kappa;
  Fx = Fz_r * mu * sin(Ct * atan(Bk - Et * (Bk - atan(Bk))));   % prc_tire_fx와 같은 식
  Fres = q * V.CdA_m2 + (v > 0.01) * V.Crr * m * g;
  a = (Fx - Fres) / m;
  if v <= 0 && a < 0, a = 0; end
  dw = (T_axle - Fx * r) / J;
  v = max(v + a * dt, 0);
  wr = max(wr + dw * dt, 0);
  x = x + v * dt;
  rev_f = rev_f + v / (2*pi*r) * dt;
  rev_r = rev_r + wr / (2*pi) * dt;
  if isnan(t75) && x >= opt.dist_m, t75 = t - opt.tps_t0; v75 = v * 3.6; end

  % ---- 기록 ----
  if mod(i + 1, opt.dec) == 0
    io = io + 1;
    R.t(io) = t + dt; R.x(io) = x; R.v_kmh(io) = v * 3.6;
    R.vf_kmh(io) = v * 3.6; R.vr_kmh(io) = wr * r * 3.6;
    R.rpm(io) = rpm; R.gear(io) = gear; R.kappa(io) = kappa; R.Fx(io) = Fx; R.Fz_r(io) = Fz_r;
    R.cut_cmd(io) = cut_cmd; R.cut_act(io) = cut_act; R.tcs_mode(io) = mode;
    R.slip_meas(io) = slip_meas; R.tps(io) = tps; R.a_g(io) = a / g;
    R.rev_f(io) = rev_f; R.rev_r(io) = rev_r;
  end
end
R.t75 = t75;      % 스로틀 시작부터 75 m 도달까지 s (못 가면 NaN)
R.v75_kmh = v75;
R.opt = opt;
end
