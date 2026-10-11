function A = analyze_steering(logfile, P, db, outdir, truth)
% ANALYZE_STEERING  조향각(SAS)·요레이트·휠속도·IMU로 언더/오버스티어 특성과 롤 기울기를 구한다.
%
%   A = analyze_steering('out/dummy_skidpad.csv', P, db, 'out')
%
% 정상 선회 구간(핸들 20° 이상, 요레이트·속도 변화가 느림)에서:
%   선회 반경 R = v / 요레이트,  횡가속도 ay = v·요레이트 (운동학 값, 롤 영향 없음)
%   전륜 조향각 δ = 핸들각을 P.steer 표로 환산
%   δ − 축거/R  vs  ay  기울기 = 언더스티어 기울기 K_us (deg/g)
%     K_us > 0 언더스티어, = 0 뉴트럴, < 0 오버스티어
%   IMU 횡비력 − 운동학 ay = g·sin(롤)  →  롤 기울기 (deg/g)
%   요 게인 = 요레이트 / δ 를 속도별로 (뉴트럴이면 v/L)
% 선형 구간은 |ay| ≤ 1.0 g로 맞춘다. 그 이상은 한계 거동으로 따로 본다.
if nargin < 4, outdir = 'out'; end
if nargin < 5, truth = []; end
g = P.veh.g; L = P.veh.wheelbase_m;
F = prc_log_read_csv(logfile);
S = prc_log_decode(F, db);
cal = prc_imu_align(S, P);
fs = 100; dt = 1/fs;
[tu, Y] = prc_grid(S, fs, {'WSS_Speed','WssFL'; 'WSS_Speed','WssFR'; 'WSS_Speed','WssRL'; 'WSS_Speed','WssRR'; ...
  'DAQ_Chassis','SteerAngle'; 'DAQ_IMU_Acc','AccX'; 'DAQ_IMU_Acc','AccY'; 'DAQ_IMU_Acc','AccZ'; ...
  'DAQ_IMU_Gyro','GyroX'; 'DAQ_IMU_Gyro','GyroY'; 'DAQ_IMU_Gyro','GyroZ'});
[fb, wb] = prc_imu_apply(cal, [Y.AccX Y.AccY Y.AccZ], [Y.GyroX Y.GyroY Y.GyroZ]);
yaw = wb(:, 3);                                         % deg/s, 좌회전 +
v = mean([Y.WssFL Y.WssFR Y.WssRL Y.WssRR], 2) / 3.6;   % 안팎 평균 = 중심 속도
yr = yaw * pi/180;
ay_k = v .* yr / g;                                      % g
ay_i = fb(:, 2);                                         % IMU (롤 성분 포함)
delta = interp1(P.steer.swa_tab, P.steer.wheel_tab, Y.SteerAngle, 'linear', 'extrap');
Rr = v ./ yr;
under = delta - L ./ Rr * 180/pi;                        % deg
dyaw = prc_deriv(yaw, dt, 25);
st = abs(Y.SteerAngle) > 20 & v > 10/3.6 & abs(yaw) > 5 & abs(dyaw) < 3 & ~isnan(under + ay_i + dyaw);
A.n_steady = sum(st);
sg = sign(yaw);
lin = st & abs(ay_k) <= 1.0;
% 좌우 합쳐서 |ay| 대비 부호 맞춘 언더스티어 각
p = [abs(ay_k(lin)) ones(sum(lin), 1)] \ (sg(lin) .* under(lin));
A.K_us = p(1); A.K_us_offset = p(2);
for d = [1 -1]
  s = lin & sg == d;
  q = [abs(ay_k(s)) ones(sum(s), 1)] \ (d * under(s));
  if d > 0, A.K_us_left = q(1); else, A.K_us_right = q(1); end
end
roll = asind(max(min(ay_i - ay_k, 1), -1));             % deg (오른쪽 내려감 +, 좌회전 기준)
q = [ay_k(st) ones(sum(st), 1)] \ roll(st);
A.roll_grad = q(1);
A.ay_max = max(abs(ay_k(st)));
A.imu = cal;
% 한계 구간: |ay| > 1.0 g에서 직선 예측보다 더 필요한 조향
hi = st & abs(ay_k) > 1.0;
A.extra_at_max = NaN;
if any(hi)
  [~, k] = max(abs(ay_k) .* hi);
  A.extra_at_max = sg(k)*under(k) - (A.K_us*abs(ay_k(k)) + A.K_us_offset);
end

% ---------- 그림 ----------
h = prc_fig(1100, 450);
subplot(1,2,1); hold on; grid on;
plot(abs(ay_k(st & sg > 0)), under(st & sg > 0), '.', 'MarkerSize', 3);
plot(abs(ay_k(st & sg < 0)), -under(st & sg < 0), '.', 'MarkerSize', 3);
aa = (0:0.05:1.6)';
plot(aa, A.K_us*aa + A.K_us_offset, 'k--');
lg = {'left', 'right', sprintf('fit K_{us} = %.2f deg/g', A.K_us)};
if ~any(st & sg > 0), lg(1) = []; end
if ~any(st & sg < 0), lg(strcmp(lg, 'right')) = []; end
if ~isempty(truth)
  plot(aa, truth.K_us*aa + 4*max(aa-1.1, 0).^2, 'g-', 'LineWidth', 2); lg{end+1} = 'truth (dummy)';
end
legend(lg, 'Location', 'northwest'); xlabel('|a_y| [g]'); ylabel('delta - L/R [deg]');
title('Understeer gradient');
subplot(1,2,2); hold on; grid on;
gain = yaw ./ delta;                                     % (deg/s)/deg
vk = v * 3.6;
plot(vk(st), gain(st), '.', 'MarkerSize', 3);
vl = (10:1:45)'/3.6;
plot(vl*3.6, vl / L, 'k--');
vch2 = L / (A.K_us*pi/180 / g);                          % 특성속도^2 (언더스티어일 때)
lg2 = {'measured', 'neutral v/L'};
if A.K_us > 0, plot(vl*3.6, vl ./ (L + vl.^2 / vch2), 'r-'); lg2{end+1} = 'linear model with K_{us}'; end
legend(lg2, 'Location', 'northwest');
xlabel('speed [km/h]'); ylabel('yaw rate / wheel angle [1/s]'); title('Steady-state yaw gain');
prc_savefig(h, fullfile(outdir, 'steering.png'));

Lr = {'# 조향 특성 분석 (가상 데이터)', '', ...
  sprintf('> 입력: `%s` | IMU 장착 추정 롤 %.2f° 피치 %.2f° 요 %.2f°', logfile, cal.rpy_deg), '', ...
  sprintf('- 정상 선회 샘플 %d개, 최대 횡가속 %.2f g', A.n_steady, A.ay_max), ...
  sprintf('- 언더스티어 기울기 K_us = **%.2f deg/g** (좌 %.2f / 우 %.2f) — 양수 = 언더스티어', A.K_us, A.K_us_left, A.K_us_right), ...
  sprintf('- 롤 기울기(IMU 횡비력 − 운동학 횡가속) = %.2f deg/g', A.roll_grad), ...
  sprintf('- 최대 횡가속 지점에서 직선 예측보다 더 필요한 조향 %.2f deg (한계 언더스티어 징후)', A.extra_at_max)};
if ~isempty(truth)
  Lr{end+1} = sprintf('- 정답(더미): K_us %.2f deg/g, 롤 기울기 %.2f deg/g', truth.K_us, truth.roll_grad_deg_g);
end
Lr = [Lr, {'', '주의: 바퀴 조향각은 P.steer 표(지금은 조향비 5:1 가정)로 환산한다. 실제 표가 틀리면 K_us가 그대로 틀어진다.', ...
  'IMU 횡가속은 롤 때문에 실제보다 크게 읽힌다. 횡가속 기준값은 속도×요레이트를 쓴다.', '', '![steering](steering.png)'}];
prc_write_lines(fullfile(outdir, 'steering_report.md'), Lr);
end
