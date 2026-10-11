function [n_fail, log] = check_vt(V, P)
% CHECK_VT  가상 시험장 결과의 자기 일관성 확인.
log = {}; n_fail = 0;
% 조향 LUT
T = P.steer.lut;
[log, n_fail] = chk(log, n_fail, '애커만 LUT: 내륜각 ≥ 외륜각, 핸들각에 대해 단조 증가', all(T(2:end, 3) >= T(2:end, 2)) && all(diff(T(:, 2)) > 0) && all(diff(T(:, 3)) > 0));
sw = [-120 -60 -9 0 9 60 120]; [~, ~, dA] = prc_steer_lut(P, sw); back = prc_steer_inv(P, dA);
[log, n_fail] = chk(log, n_fail, sprintf('LUT 역함수 왕복 오차 %.3f°', max(abs(back - sw))), max(abs(back - sw)) < 1e-6);
[dL, dR] = prc_steer_lut(P, 90);
[log, n_fail] = chk(log, n_fail, sprintf('좌회전 90°: 왼쪽(내륜) %.1f° > 오른쪽(외륜) %.1f°', dL, dR), dL > dR);
% 가속
Rl = sim_longitudinal(P, struct('tcs_on', false, 'mu_scale', 1.0));
[log, n_fail] = chk(log, n_fail, sprintf('2트랙 75 m %.2f s vs 종방향 1자유도 %.2f s (0.4 s 이내)', V.accel.lap_time_s, Rl.t75), abs(V.accel.lap_time_s - Rl.t75) < 0.4);
[log, n_fail] = chk(log, n_fail, '직선 가속에서 QSS 상한 ≤ 2트랙 시간', V.accel.qss_time_s <= V.accel.lap_time_s);
% 스키드패드
[log, n_fail] = chk(log, n_fail, sprintf('스키드패드 코스 안 유지 (최대 횡편차 %.2f m < %.2f)', V.skid.e_lat_max, V.skid.road.width_m/2), ~V.skid.offtrack);
[log, n_fail] = chk(log, n_fail, sprintf('스키드패드 2바퀴 %.2f s, 보고서 목표 9.7 s의 ±25%% 안', V.skid.lap_time_s), abs(V.skid.lap_time_s/9.7 - 1) < 0.25);
[log, n_fail] = chk(log, n_fail, sprintf('스키드패드 최대 횡가속 %.2f g (1.2~1.7)', max(abs(V.skid.ay_g))), max(abs(V.skid.ay_g)) > 1.2 && max(abs(V.skid.ay_g)) < 1.7);
[log, n_fail] = chk(log, n_fail, sprintf('센서 로그 기반 K_us %.2f vs 시뮬 내부 %.2f deg/g (0.4 이내)', V.steer.K_us, V.Kus_truth), abs(V.steer.K_us - V.Kus_truth) < 0.4);
% 오토크로스
[log, n_fail] = chk(log, n_fail, sprintf('오토크로스 코스 안 유지 (최대 횡편차 %.2f m)', V.autox.e_lat_max), ~V.autox.offtrack);
[log, n_fail] = chk(log, n_fail, sprintf('오토크로스 2트랙 %.2f s ≥ QSS %.2f s, 차이 30%% 이내', V.autox.lap_time_s, V.autox.qss_time_s), ...
  V.autox.lap_time_s >= V.autox.qss_time_s && V.autox.lap_time_s / V.autox.qss_time_s < 1.3);
[log, n_fail] = chk(log, n_fail, '오토크로스 랩 2회 모두 기록', numel(V.autox.laps) == 2);
log{end+1} = sprintf('합계: %d개 중 실패 %d', numel(log), n_fail);
for i = 1:numel(log), fprintf('%s\n', log{i}); end
end

function [log, n_fail] = chk(log, n_fail, name, ok)
if ok, log{end+1} = ['PASS  ' name];
else, log{end+1} = ['FAIL  ' name]; n_fail = n_fail + 1; end
end
