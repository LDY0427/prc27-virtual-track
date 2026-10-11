function A = analyze_road_dyno(logfile, P, db, outdir, truth)
% ANALYZE_ROAD_DYNO  주행 로그만으로 코스트다운(저항)과 로드 다이노(엔진 토크 곡선)를 추정한다.
%
%   A = analyze_road_dyno('out/dummy_dyno.csv', P, db, 'out')
%   A = analyze_road_dyno(..., truth)     % 더미 정답이 있으면 비교까지
%
% 1) IMU 장착 보정 (prc_imu_align)
% 2) 코스트다운: 기어 0·스로틀 0·제동 0·15 km/h 이상 구간
%      저항력 = −(질량 + 바퀴 회전관성 환산) × 가속도 = c0 + c2·v²
%      → c0 = 구름저항 (Crr·m·g),  c2 = ½ρ·CdA(+ 다운포스로 늘어난 구름저항)
%    방법 A: 휠속도 미분만 (양방향 평균으로 경사 상쇄)
%    방법 B: IMU 비력 사용 (m·f_x = m·a + m·g·sin경사 → 경사가 자동으로 들어감)
% 3) 로드 다이노: 풀스로틀·같은 단·핸들 직진 구간
%      바퀴 구동력 F = (m + 4Jw/r²)·a + c0 + c2·v² + m·g·sin경사
%      엔진 토크 T = F·r / (감속비·효율) + Je·감속비·a / r   (엔진 회전관성)
%    RPM 250 단위로 모아 평균
%
% 결과는 질량·회전관성·구동계 효율·감속비·굴림반경 입력값에 그대로 비례해 틀어진다.
% 엔진 관성 ±50% 민감도를 같이 보고한다.
if nargin < 4, outdir = 'out'; end
if nargin < 5, truth = []; end
V = P.veh; g = V.g; m = V.mass_kg; r = P.tire.r_m; eta = P.drv.eta;
Jw4 = 4 * P.tire.J_wheel_kgm2; Je = P.eng.J_kgm2;
ratios = P.drv.primary * P.drv.gears * P.drv.final;

F = prc_log_read_csv(logfile);
S = prc_log_decode(F, db);
cal = prc_imu_align(S, P);
fs = 100; dt = 1/fs;
[tu, Y] = prc_grid(S, fs, {'WSS_Speed','WssFL'; 'WSS_Speed','WssFR'; 'WSS_Speed','WssRL'; 'WSS_Speed','WssRR'; ...
  'EMU_SIM_0','RPM'; 'EMU_SIM_0','TPS'; 'EMU_SIM_1','Gear:hold'; 'DAQ_Chassis','SteerAngle'; ...
  'DAQ_Chassis','BrkPressF'; 'DAQ_IMU_Acc','AccX'; 'DAQ_IMU_Acc','AccY'; 'DAQ_IMU_Acc','AccZ'});
fb = prc_imu_apply(cal, [Y.AccX Y.AccY Y.AccZ]);
fx = fb(:, 1) * g;                                      % 차체 x 비력 m/s^2 (경사 포함)
v = mean([Y.WssFL Y.WssFR], 2) / 3.6;                   % 비구동 전륜 = 차속
a = prc_deriv(v, dt, 10);                               % 0.21 s 창
ok = ~isnan(a) & ~isnan(fx);

% ---------- 코스트다운 ----------
cm = ok & Y.Gear == 0 & Y.TPS < 2 & Y.BrkPressF < 1 & v > 15/3.6 & abs(Y.SteerAngle) < 10;
runs = prc_runs(cm, tu, 3);
cm(:) = false;
for k = 1:size(runs, 1), cm(runs(k,1)+30 : runs(k,2)-30) = true; end   % 앞뒤 0.3 s 버림
meq0 = m + Jw4/r^2;
X2 = [ones(sum(cm), 1), v(cm).^2];
FresA = -meq0 * a(cm);
FresB = -(m * fx(cm) + Jw4/r^2 * a(cm));
cA = X2 \ FresA; cB = X2 \ FresB;
A.coast.n_runs = size(runs, 1);
A.coast.c = [cA cB];                                    % [c0; c2] × [방법A 방법B]
A.coast.CdA = 2 * [cA(2) cB(2)] / V.rho;
A.coast.Crr = [cA(1) cB(1)] / (m * g);

% ---------- 로드 다이노 ----------
G = Y.Gear;
dm = ok & Y.TPS > 95 & G > 0 & abs(Y.SteerAngle) < 10 & v > 5/3.6;
runs = prc_runs(dm, tu, 1.0);
dm(:) = false;
for k = 1:size(runs, 1), dm(runs(k,1)+30 : runs(k,2)-10) = true; end
dm = dm & G == mode(G(dm));                             % 한 단만 사용
gsel = mode(G(dm)); i_g = ratios(gsel);
c0 = cB(1); c2 = cB(2);                                  % 저항은 IMU 방법 결과 사용
FA = meq0 * a + c0 + c2 * v.^2;                         % 방법 A: 경사 무시 (양방향 평균에 맡김)
FB = m * fx + Jw4/r^2 * a + c0 + c2 * v.^2;             % 방법 B: IMU로 경사 포함
TA = FA * r / (i_g * eta) + Je * i_g * a / r;
TB = FB * r / (i_g * eta) + Je * i_g * a / r;
TBlo = FB * r / (i_g * eta) + 0.5*Je * i_g * a / r;
TBhi = FB * r / (i_g * eta) + 1.5*Je * i_g * a / r;
rpm_v = v / r * i_g * 60/(2*pi);                         % 휠속도 기준 rpm (EMU rpm과 비교용)
edges = 3500:250:10250;
c = edges(1:end-1) + 125;
nb = numel(c);
A.dyno.rpm = c(:); A.dyno.TA = nan(nb,1); A.dyno.TB = nan(nb,1); A.dyno.n = zeros(nb,1);
A.dyno.TBlo = nan(nb,1); A.dyno.TBhi = nan(nb,1);
for b = 1:nb
  s = dm & Y.RPM >= edges(b) & Y.RPM < edges(b+1);
  A.dyno.n(b) = sum(s);
  if sum(s) >= 5
    A.dyno.TA(b) = mean(TA(s)); A.dyno.TB(b) = mean(TB(s));
    A.dyno.TBlo(b) = mean(TBlo(s)); A.dyno.TBhi(b) = mean(TBhi(s));
  end
end
A.dyno.kW = A.dyno.TB .* A.dyno.rpm * 2*pi/60 / 1000;
A.dyno.gear = gsel; A.dyno.n_pulls = size(runs, 1);
A.dyno.rpm_check = mean(abs(Y.RPM(dm) - rpm_v(dm)) ./ Y.RPM(dm)) * 100;   % 감속비·반경이 맞는지
A.imu = cal;
if ~isempty(truth)
  Tt = interp1(truth.rpm_tab, truth.trq_tab, A.dyno.rpm);
  sel = A.dyno.rpm >= 5000 & A.dyno.rpm <= 10000 & ~isnan(A.dyno.TB);
  A.dyno.errA_pct = mean(abs(A.dyno.TA(sel) - Tt(sel)) ./ Tt(sel)) * 100;
  A.dyno.errB_pct = mean(abs(A.dyno.TB(sel) - Tt(sel)) ./ Tt(sel)) * 100;
  A.dyno.T_true = Tt;
end

% ---------- 그림 ----------
h = prc_fig(1100, 450);
subplot(1,2,1); hold on; grid on;
plot(v(cm)*3.6, FresB, '.', 'MarkerSize', 3);
vv = (15:1:80)'/3.6;
plot(vv*3.6, cA(1) + cA(2)*vv.^2, 'k-', vv*3.6, cB(1) + cB(2)*vv.^2, 'r--');
if ~isempty(truth)
  plot(vv*3.6, truth.Crr*(m*g + 0.5*V.rho*V.ClA_m2*vv.^2) + 0.5*V.rho*truth.CdA*vv.^2, 'g:', 'LineWidth', 2);
  legend('samples (IMU)', 'fit A wheel-only', 'fit B IMU', 'truth', 'Location', 'northwest');
else
  legend('samples (IMU)', 'fit A wheel-only', 'fit B IMU', 'Location', 'northwest');
end
xlabel('speed [km/h]'); ylabel('resistance [N]'); title('Coastdown: drag + rolling');
subplot(1,2,2); hold on; grid on;
plot(A.dyno.rpm, A.dyno.TA, 'b.-', A.dyno.rpm, A.dyno.TB, 'r.-');
plot(A.dyno.rpm, A.dyno.TBlo, 'r:', A.dyno.rpm, A.dyno.TBhi, 'r:');
plot(P.eng.rpm_tab, P.eng.trq_tab, 'k--');
lg = {'est A wheel-only', 'est B IMU', 'B, Je -50%', 'B, Je +50%', 'assumed (params)'};
if ~isempty(truth), plot(truth.rpm_tab, truth.trq_tab, 'g-', 'LineWidth', 2); lg{end+1} = 'truth (dummy)'; end
xlim([3000 10500]); legend(lg, 'Location', 'south'); xlabel('engine speed [rpm]'); ylabel('torque [N m]');
title(sprintf('Road dyno, gear %d', gsel));
prc_savefig(h, fullfile(outdir, 'road_dyno.png'));

% ---------- 보고서 ----------
Lr = {'# 로드 다이노·코스트다운 분석 (가상 데이터)', '', ...
  sprintf('> 입력: `%s` | IMU 장착 추정 롤 %.2f° 피치 %.2f° 요 %.2f° (정지 %d·직진 %d 샘플)', ...
          logfile, cal.rpy_deg, cal.n_static, cal.n_straight), '', '## 코스트다운', '', ...
  sprintf('구간 %d개. 저항력 = c0 + c2·v²', A.coast.n_runs), '', ...
  '| 방법 | CdA m² (구름저항 증가분 포함) | Crr |', '|---|---:|---:|', ...
  sprintf('| A 휠속도만 | %.3f | %.4f |', A.coast.CdA(1), A.coast.Crr(1)), ...
  sprintf('| B IMU 경사 보정 | %.3f | %.4f |', A.coast.CdA(2), A.coast.Crr(2))};
if ~isempty(truth)
  Lr{end+1} = sprintf('| 정답(더미) | %.3f | %.4f |', truth.CdA_eff, truth.Crr);
end
Lr = [Lr, {'', sprintf('참고: params의 CdA(설계보고서 CFD 유도값) %.2f m²', V.CdA_m2), '', ...
  '## 로드 다이노', '', sprintf('%d단 풀스로틀 구간 %d개. EMU RPM과 휠속도로 계산한 RPM 차이 평균 %.2f %% (크면 감속비·굴림반경 입력이 틀린 것)', ...
  gsel, A.dyno.n_pulls, A.dyno.rpm_check), ''}];
Lr{end+1} = '| RPM | 토크 A N·m | 토크 B N·m | B 출력 kW | Je ±50% 범위 N·m | 샘플 |';
Lr{end+1} = '|---:|---:|---:|---:|---|---:|';
for b = 1:nb
  if A.dyno.n(b) >= 5
    Lr{end+1} = sprintf('| %d | %.1f | %.1f | %.1f | %.1f ~ %.1f | %d |', c(b), A.dyno.TA(b), A.dyno.TB(b), ...
                        A.dyno.kW(b), min(A.dyno.TBlo(b), A.dyno.TBhi(b)), max(A.dyno.TBlo(b), A.dyno.TBhi(b)), A.dyno.n(b)); %#ok<AGROW>
  end
end
if ~isempty(truth)
  Lr{end+1} = '';
  Lr{end+1} = sprintf('정답 대비 평균 오차(5000~10000 rpm): A %.1f %%, B %.1f %%', A.dyno.errA_pct, A.dyno.errB_pct);
end
Lr = [Lr, {'', '토크는 구동계 효율·질량·관성·굴림반경 입력값에 비례해 틀어진다. 절대값은 섀시 다이노 1회 대조 후 신뢰하고, 그 전에는 같은 조건 전후 비교(기어비·맵 변경)에 쓴다.', ...
  '', '![road dyno](road_dyno.png)'}];
prc_write_lines(fullfile(outdir, 'road_dyno_report.md'), Lr);
end
