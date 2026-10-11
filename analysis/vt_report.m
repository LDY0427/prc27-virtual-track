function V = vt_report(P, db, outdir)
% VT_REPORT  가상 시험장 3종(가속·스키드패드·오토크로스) 실행 → 그림·보고서. 스키드패드 로그는 조향 분석기로 재검증.
%
%   V = vt_report(P, db, 'out')
%   V.accel, V.skid, V.autox : vt_run 결과,  V.steer : analyze_steering 결과 (스키드패드 센서 로그)
%   V.Kus_truth : 시뮬 내부값(핸들각→평균 바퀴각, 속도·요레이트)으로 직접 계산한 언더스티어 기울기
g = P.veh.g; L = P.veh.wheelbase_m;
V.accel = vt_run(P, 'accel');
V.skid = vt_run(P, 'skidpad');                                            % 한계 속도: 랩타임
% 조향 특성용: 8바퀴 동안 목표 속도를 35% → 100%로 올린다 (실차 시험 계획과 같은 절차)
V.skid_ramp = vt_run(P, vt_road('skidpad', struct('laps', 8)), struct('v_ramp', [0.35 1.0 45], 't_max', 90, ...
                     'log_csv', fullfile(outdir, 'vt_skidpad.csv')));
V.autox = vt_run(P, 'autocross', struct('laps', 2, 'log_csv', fullfile(outdir, 'vt_autocross.csv')));
% 조향 분석기로 K_us (센서 경로를 거친 로그) vs 시뮬 내부 정답
V.steer = analyze_steering(fullfile(outdir, 'vt_skidpad.csv'), P, db, outdir);
S = V.skid_ramp; sel = S.v_kmh > 12 & abs(S.yaw_dps) > 8 & S.t > 3;
[~, ~, dAvg] = prc_steer_lut(P, S.sw_deg(sel));
ayk = S.v_kmh(sel)/3.6 .* (S.yaw_dps(sel)*pi/180) / g;
Rr = (S.v_kmh(sel)/3.6) ./ (S.yaw_dps(sel)*pi/180);
und = dAvg - L ./ Rr * 180/pi;
lin = abs(ayk) <= 1.0 & abs(ayk) > 0.2;
V.Kus_n = sum(lin);
p = [abs(ayk(lin)) ones(sum(lin), 1)] \ (sign(ayk(lin)) .* und(lin));
V.Kus_truth = p(1);

% ---------- 그림 1: 코스 지도 + 속도 ----------
h = prc_fig(1150, 420);
runs = {V.skid, 'Skidpad (R 9.125 m)'; V.autox, 'Autocross loop (synthetic)'};
for k = 1:2
  S = runs{k, 1}; R = S.road;
  subplot(1, 2, k); hold on; axis equal; grid on;
  nrm = -sin(R.psi); nrm2 = cos(R.psi); w = R.width_m / 2;
  plot(R.x + w*nrm, R.y + w*nrm2, 'k-', R.x - w*nrm, R.y - w*nrm2, 'k-', 'LineWidth', 0.5);
  scatter(S.x, S.y, 6, S.v_kmh, 'filled'); colormap(jet); cb = colorbar; ylabel(cb, 'km/h');
  title(sprintf('%s  lap %.2f s (QSS %.2f s)', runs{k, 2}, S.lap_time_s, S.qss_time_s));
  xlabel('x [m]'); ylabel('y [m]');
end
prc_savefig(h, fullfile(outdir, 'vt_tracks.png'));

% ---------- 그림 2: 오토크로스 속도·g, GGV ----------
h = prc_fig(1150, 760);
S = V.autox; Q = S.qss; R = S.road;
last = S.s > 0 & S.t > S.t(end) - S.lap_time_s - 0.05;      % 마지막 랩
sl = S.s; sl(~last) = NaN; wrap = [false; diff(sl) < 0]; sl(wrap) = NaN;   % 랩 경계에서 선 끊기
vl_ = S.v_kmh; axl = S.ax_g; ayl = S.ay_g;
subplot(3, 2, [1 2]); hold on; grid on;
plot(Q.s(1:numel(R.s)), Q.v_kmh(1:numel(R.s)), 'k--', sl, vl_, 'b-');
legend('QSS upper bound', '2-track + driver', 'Location', 'south'); ylabel('km/h'); title('Autocross speed vs distance');
subplot(3, 2, [3 4]); hold on; grid on;
plot(sl, axl, sl, ayl); legend('a_x', 'a_y', 'Location', 'south'); ylabel('g'); xlabel('s [m]');
subplot(3, 2, 5); hold on; grid on; axis equal;
plot(S.ay_g, S.ax_g, '.', 'MarkerSize', 3);
th = linspace(0, 2*pi, 100); plot(1.5*cos(th), 1.5*sin(th), 'k:');
xlabel('a_y [g]'); ylabel('a_x [g]'); title('g-g diagram (dotted = mu 1.5 circle)'); xlim([-2 2]); ylim([-2 2]);
subplot(3, 2, 6); hold on; grid on;
plot(Q.ggv.v_kmh, Q.ggv.ay_max_g, Q.ggv.v_kmh, Q.ggv.ax_acc_g, Q.ggv.v_kmh, -Q.ggv.ax_brk_g);
legend('a_y max', 'a_x accel', 'a_x brake', 'Location', 'west'); xlabel('km/h'); ylabel('g'); title('GGV envelope used by QSS');
prc_savefig(h, fullfile(outdir, 'vt_autocross.png'));

% ---------- 그림 3: 스키드패드 조향·애커만 ----------
h = prc_fig(1150, 420);
S = V.skid; T = P.steer.lut;
subplot(1, 3, 1); hold on; grid on;
plot(T(:, 1), T(:, 2), 'o-', T(:, 1), T(:, 3), 's-', T(:, 1), (T(:, 2) + T(:, 3))/2, 'k--');
legend('outer', 'inner', 'average', 'Location', 'northwest'); xlabel('steering wheel [deg]'); ylabel('road wheel [deg]');
title('Ackermann LUT (PRC-26 report Fig.9)');
subplot(1, 3, 2); hold on; grid on;
plot(T(2:end, 1), P.steer.ackermann_pct(2:end), 'r.-'); ylabel('Ackermann [%]'); xlabel('steering wheel [deg]');
title('Ackermann % along LUT');
subplot(1, 3, 3); hold on; grid on;
S = V.skid_ramp;
plot(S.t, S.sw_deg, S.t, S.ay_g * 20, S.t, S.v_kmh);
legend('SW [deg]', 'a_y x20 [g]', 'v [km/h]', 'Location', 'southeast'); xlabel('t [s]');
title(sprintf('Skidpad trace, K_{us} analysis %.2f / truth %.2f deg/g', V.steer.K_us, V.Kus_truth));
prc_savefig(h, fullfile(outdir, 'vt_skidpad.png'));

% ---------- 보고서 ----------
Lr = {'# 가상 시험장 (CarMaker 대체) 결과 — 가상 데이터', '', ...
  '2트랙 평면 차량 + 경로/속도 추종 운전자 + 도로 + 센서 CAN 로그. 타이어 횡특성·요 관성·운전자 설정은 가정값.', '', ...
  '| 시험 | 2트랙+운전자 | 준정상상태 상한(QSS) | 보고서 목표 | 코스 이탈 | 최대 횡편차 m |', '|---|---:|---:|---:|:---:|---:|', ...
  sprintf('| 75 m 가속 | %.2f s | %.2f s | - | %d | %.2f |', V.accel.lap_time_s, V.accel.qss_time_s, V.accel.offtrack, V.accel.e_lat_max), ...
  sprintf('| 스키드패드 2바퀴 (R 9.125) | %.2f s (최대 %.2f g) | %.2f s | 9.7 s @1.56 g | %d | %.2f |', V.skid.lap_time_s, max(abs(V.skid.ay_g)), V.skid.qss_time_s, V.skid.offtrack, V.skid.e_lat_max), ...
  sprintf('| 오토크로스 루프 %.0f m | %.2f s | %.2f s | - | %d | %.2f |', V.autox.road.length_m, V.autox.lap_time_s, V.autox.qss_time_s, V.autox.offtrack, V.autox.e_lat_max), '', ...
  sprintf('- 스키드패드 로그를 센서 모델 → `analyze_steering`에 넣은 언더스티어 기울기 %.2f deg/g, 시뮬 내부값으로 직접 계산한 %.2f deg/g (코너링 계수 전후 동일 가정이라 선형 구간은 거의 중립, 하중이동·다운포스 배분으로 약간 언더).', V.steer.K_us, V.Kus_truth), ...
  sprintf('- 애커만 LUT: 핸들 ±%.0f° 기구 제한, 평균 조향비 %.2f, 애커만 %.0f~%.0f%%.', P.steer.sw_max_deg, mean(P.steer.ratio_tab(2:end)), min(P.steer.ackermann_pct(2:end)), max(P.steer.ackermann_pct(2:end))), ...
  '- QSS는 점질량 상한이라 항상 더 빠르다. 2트랙 결과와의 차이가 과도 응답·운전자 손실의 크기다.', '', ...
  '![tracks](vt_tracks.png)', '![autocross](vt_autocross.png)', '![skidpad](vt_skidpad.png)'};
prc_write_lines(fullfile(outdir, 'vt_report.md'), Lr);
end
