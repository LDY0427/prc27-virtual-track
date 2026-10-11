function [n_fail, log] = check_test_runs(R)
% CHECK_TEST_RUNS  시험별 더미 로그 분석 결과가 더미에 심어 둔 정답을 다시 찾는지 확인한다.
%
%   R.dyno, R.dyno_truth, R.steer, R.steer_truth, R.brake, R.brake_truth,
%   R.session, R.session_truth  (run_all이 채운다)
% 허용 오차는 '센서 잡음·양자화가 있어도 이 정도는 맞아야 쓸 만하다'는 기준으로 정했다.
log = {}; n_fail = 0;

% ---- IMU 장착 보정 (다이노 로그) ----
cal = R.dyno.imu; tr = R.dyno_truth;
e = abs(cal.rpy_deg - tr.imu_mount_deg);
[log, n_fail] = chk(log, n_fail, sprintf('IMU 장착각 추정 오차 롤 %.2f° 피치 %.2f° 요 %.2f° (각 0.3° 이내)', e), all(e < 0.3));
eb = abs(cal.gyro_bias - tr.gyro_bias_dps);
[log, n_fail] = chk(log, n_fail, sprintf('자이로 영점 추정 오차 최대 %.2f deg/s (0.1 이내)', max(eb)), max(eb) < 0.1);

% ---- 코스트다운 ----
c = R.dyno.coast;
eC = abs(c.CdA(2) / tr.CdA_eff - 1) * 100; eR = abs(c.Crr(2) / tr.Crr - 1) * 100;
[log, n_fail] = chk(log, n_fail, sprintf('코스트다운 CdA %.3f (정답 %.3f, 오차 %.1f%%, 10%% 이내)', c.CdA(2), tr.CdA_eff, eC), eC < 10);
[log, n_fail] = chk(log, n_fail, sprintf('코스트다운 Crr %.4f (정답 %.4f, 오차 %.1f%%, 30%% 이내)', c.Crr(2), tr.Crr, eR), eR < 30);

% ---- 로드 다이노 ----
d = R.dyno.dyno;
[log, n_fail] = chk(log, n_fail, sprintf('로드 다이노 IMU 방법 토크 평균 오차 %.2f%% (5000~10000 rpm, 3%% 이내)', d.errB_pct), d.errB_pct < 3);
[log, n_fail] = chk(log, n_fail, sprintf('로드 다이노 휠속도만(양방향 평균) 오차 %.2f%% (5%% 이내)', d.errA_pct), d.errA_pct < 5);
[log, n_fail] = chk(log, n_fail, sprintf('EMU RPM과 휠속도 환산 RPM 차이 %.2f%% (1%% 이내 = 감속비 입력 일치)', d.rpm_check), d.rpm_check < 1);
sel = d.rpm >= 4500 & d.rpm <= 6000 & ~isnan(d.TB);
Tas = interp1(R.P.eng.rpm_tab, R.P.eng.trq_tab, d.rpm(sel));
eT = mean(abs(d.TB(sel) - d.T_true(sel))); eP = mean(abs(d.TB(sel) - Tas));
[log, n_fail] = chk(log, n_fail, sprintf(['4500~6000 rpm 추정 토크가 가정 곡선(차이 %.2f N·m)보다 ' ...
  '정답 곡선(차이 %.2f N·m)에 가까움 = 가정이 아니라 로그에서 나온 값'], eP, eT), eT < eP);

% ---- 조향 ----
s = R.steer; st = R.steer_truth;
[log, n_fail] = chk(log, n_fail, sprintf('언더스티어 기울기 %.2f deg/g (정답 %.2f, ±0.2)', s.K_us, st.K_us), abs(s.K_us - st.K_us) < 0.2);
[log, n_fail] = chk(log, n_fail, sprintf('롤 기울기 %.2f deg/g (정답 %.2f, ±0.3)', s.roll_grad, st.roll_grad_deg_g), abs(s.roll_grad - st.roll_grad_deg_g) < 0.3);
[log, n_fail] = chk(log, n_fail, sprintf('한계 근처 추가 조향 %.2f deg 탐지 (> 0.2)', s.extra_at_max), s.extra_at_max > 0.2);

% ---- 제동 ----
b = R.brake; bt = R.brake_truth.events;
ne = numel(b.events);
[log, n_fail] = chk(log, n_fail, sprintf('제동 이벤트 %d개 = 정답 %d개', ne, numel(bt)), ne == numel(bt));
if ne == numel(bt)
  okL = true; okT = true; okS = true;
  for q = 1:ne
    tl = bt(q).t_lock; lk = ~isnan(tl);
    okL = okL && isequal(lk, b.events(q).locked);
    if any(lk), okT = okT && max(abs(b.lock_t(q, lk) - tl(lk))) < 0.05; end
    if ~any(lk), okS = okS && abs(b.events(q).stop_m / bt(q).stop_m - 1) < 0.05; end
  end
  [log, n_fail] = chk(log, n_fail, '잠긴 바퀴 조합이 모든 이벤트에서 정답과 같음', okL);
  [log, n_fail] = chk(log, n_fail, '잠김 시각 오차 50 ms 이내', okT);
  [log, n_fail] = chk(log, n_fail, '잠김 없는 이벤트 정지거리 오차 5% 이내', okS);
  [log, n_fail] = chk(log, n_fail, '최고 압력 이벤트에서 4륜 잠김 판정 (검차 조건)', b.events(end).all4);
  [log, n_fail] = chk(log, n_fail, sprintf('처음 잠기는 바퀴 = FL (%s)', b.events(end).first), strcmp(b.events(end).first, 'FL'));
end

% ---- 요레이트 선회 보정 (주행 세션의 FL 끊김 구간) ----
A = R.session; T = R.session_truth;
fd = T.faults.fl_dropout;
sel = A.t_wss > fd(1) + 0.3 & A.t_wss < fd(2);
vt = prc_hold(T.t, T.v_kmh, A.t_wss(sel));
e0 = max(abs(A.ref_noyaw(sel) ./ vt - 1)) * 100; e1 = max(abs(A.ref(sel) ./ vt - 1)) * 100;
[log, n_fail] = chk(log, n_fail, sprintf('선회 중 FL 끊김: 기준 차속 오차 보정 전 %.1f%% → 보정 후 %.1f%% (1.5%% 이내)', e0, e1), e1 < 1.5 && e1 < e0);

% ---- 연료맵 가이드 ----
G = R.fuel; [RR, TT] = meshgrid(G.rpm_bp, G.tps_bp);
[log, n_fail] = chk(log, n_fail, sprintf('연료맵 제안이 더미 정답과 최대 %.2f %%p 차이 (1.0 이내)', G.err_max), G.err_max < 1.0);
lean = G.status == 3 & RR >= 7000 & RR <= 9000 & TT >= 60;
[log, n_fail] = chk(log, n_fail, sprintf('고부하 희박 칸 %d개 모두 +6%% 넘게 증량 제안 (정답 +7.1~10.4)', sum(lean(:))), any(lean(:)) && all(G.pct(lean) > 6));
rich = G.status == 3 & RR >= 2000 & RR <= 3500 & TT >= 10 & TT <= 30;
[log, n_fail] = chk(log, n_fail, sprintf('부분부하 농후 칸 %d개 모두 -4%% 넘게 감량 제안 (정답 -5.0~-6.5)', sum(rich(:))), any(rich(:)) && all(G.pct(rich) < -4));
skip = RR <= 3000 & TT >= 80;
[log, n_fail] = chk(log, n_fail, '주행하지 않은 칸(저회전 풀스로틀)은 제안 없음', all(G.status(skip) == 0));
[log, n_fail] = chk(log, n_fail, sprintf('가속 순간 희박 튐이 섞이지 않음 (칸 오차 평균 %.2f %%p, 0.3 이내)', G.err_mean), G.err_mean < 0.3);

% ---- 점화맵 점검 ----
if isfield(R, 'ign') && ~isempty(R.ign)
  I = R.ign; bump = R.fuel_truth.ign_bump;
  [sa, sb] = find(I.spike);
  hit = numel(sa) == 1 && I.rpm_bp(sb) == bump(1) && I.tps_bp(sa) == bump(2);
  [log, n_fail] = chk(log, n_fail, sprintf('점화맵 점검: 넣어 둔 입력 실수 칸(%d rpm, TPS %d%%)만 정확히 찾음 (찾은 칸 %d개)', ...
    bump(1), bump(2), numel(sa)), hit);
  okd = ~I.spike & ~isnan(I.diff);
  [log, n_fail] = chk(log, n_fail, sprintf('나머지 칸은 기준 맵과 %.2f도 이내로 복원', max(abs(I.diff(okd)))), max(abs(I.diff(okd))) < 0.5);
end
% ---- 점화 MBT 스윕 ----
if isfield(R, 'mbt') && ~isempty(R.mbt)
  K = R.mbt;
  [log, n_fail] = chk(log, n_fail, sprintf('MBT 추정 오차 평균 %.2f도, 최대 %.2f도 (최대 0.8 이내)', mean(K.mbt_err), max(K.mbt_err)), max(K.mbt_err) < 0.8);
  [log, n_fail] = chk(log, n_fail, sprintf('노킹 감시 없음 → 진각 후보 %d칸이 있어도 적용 권장 진각 0칸', sum(K.cand > 0)), ...
    sum(K.cand > 0) > 0 && all(K.apply <= 0));
  [log, n_fail] = chk(log, n_fail, '점화 변경은 풀스로틀 행(TPS 80% 이상)에만 반영', ...
    all(all(K.delta_apply(R.P.ign.tps_bp < 80, :) == 0)));
end

log{end+1} = sprintf('합계: %d개 중 실패 %d', numel(log), n_fail);
for i = 1:numel(log), fprintf('%s\n', log{i}); end
end

function [log, n_fail] = chk(log, n_fail, name, ok)
if ok, log{end+1} = ['PASS  ' name];
else, log{end+1} = ['FAIL  ' name]; n_fail = n_fail + 1; end
end
