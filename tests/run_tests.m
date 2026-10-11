function [n_fail, log] = run_tests(P, db, outdir)
% RUN_TESTS  답을 아는 입력으로 각 기능을 확인한다. 실패 개수와 결과 줄을 돌려준다.
%   [n_fail, log] = run_tests(prc27_params(), prc_dbc_load('dbc/PRC27_sim_v0.1.dbc'), 'out')
if nargin < 3, outdir = 'out'; end
log = {}; n_fail = 0;

% ---- 1. 휠속도 공식: 설계 문서 예시 (48펄스, 480 Hz, 둘레 1.5 m → 600 rpm, 54 km/h)
[kmh, rpm] = prc_wss_freq2kmh(480, 48, 1.5 / (2*pi));
[log, n_fail] = chk(log, n_fail, '휠속도 공식 문서 예시 600 rpm / 54 km/h', abs(rpm - 600) < 1e-9 && abs(kmh - 54) < 1e-9);

% ---- 2. 에지 생성 → 속도 복원 (일정 속도 50 km/h)
r = P.tire.r_m; tt = (0:1e-3:2)';
v = 50 / 3.6; rev = v * tt / (2*pi*r);
te = prc_wss_edges(tt, rev, P.wss.teeth);
k = prc_wss_from_edges(te, (0.5:0.01:1.9)', P.wss.teeth, r, P.wss.timeout_s);
[log, n_fail] = chk(log, n_fail, sprintf('에지 기반 휠속도 복원 50 km/h (오차 %.3f km/h)', max(abs(k - 50))), max(abs(k - 50)) < 0.05);
k0 = prc_wss_from_edges(te, 2 + P.wss.timeout_s + 0.05, P.wss.teeth, r, P.wss.timeout_s);
[log, n_fail] = chk(log, n_fail, '에지 끊기면 timeout 뒤 0 km/h', k0 == 0);

% ---- 3. DBC 왕복: 모든 메시지·신호를 범위 안 임의값으로 인코딩→디코딩
rng(7); worst = 0; bad = '';
for i = 1:numel(db.msgs)
  m = db.msgs{i}; n = 50; vals = struct();
  for s = 1:numel(m.sigs)
    sg = m.sigs(s);
    vals.(sg.name) = sg.min + (sg.max - sg.min) * rand(n, 1);
  end
  d = prc_can_encode(m, vals, n);
  ph = prc_can_decode(m, d);
  for s = 1:numel(m.sigs)
    sg = m.sigs(s);
    e = max(abs(ph.(sg.name) - vals.(sg.name))) / abs(sg.factor);
    if ~(e <= 0.5 + 1e-6), worst = max(worst, e); bad = [m.name '.' sg.name]; end
  end
end
[log, n_fail] = chk(log, n_fail, ['DBC 인코딩/디코딩 왕복 (양자화 0.5 LSB 이내) ' bad], isempty(bad));

% ---- 4. 손으로 만든 페이로드 디코딩: DAQ_Temp T1 = 25.0 °C → raw 650 = 0x28A
m = prc_dbc_msg(db, 'DAQ_Temp');
ph = prc_can_decode(m, [hex2dec('8A') hex2dec('02') 0 0 0 0 0 0]);
[log, n_fail] = chk(log, n_fail, '알려진 바이트 디코딩 (0x8A 0x02 → T1 25.0 °C)', abs(ph.T1 - 25.0) < 1e-9);
ph = prc_can_decode(m, [255 15 0 0 0 0 0 0]);
[log, n_fail] = chk(log, n_fail, '무효 코드 0xFFF → NaN', isnan(ph.T1));

% ---- 5. Motorola(빅엔디안) 신호: 16비트, start 7 → 바이트0이 상위
sg = struct('name', 'X', 'start', 7, 'len', 16, 'intel', false, 'signed', false, ...
            'factor', 1, 'offset', 0, 'min', 0, 'max', 65535, 'unit', '');
mm = struct('name', 'MOT', 'sigs', sg);
d = prc_can_encode(mm, struct('X', hex2dec('1234')), 1);
[log, n_fail] = chk(log, n_fail, 'Motorola 인코딩 0x1234 → 바이트 12 34', d(1) == hex2dec('12') && d(2) == hex2dec('34'));

% ---- 6. 기어 추정
ratios = P.drv.primary * P.drv.gears * P.drv.final;
vk = 40; wr = vk / 3.6 / (2*pi*r) * 60;
g = prc_gear_estimate(wr * ratios(:), vk * ones(numel(ratios), 1), P);
[log, n_fail] = chk(log, n_fail, '기어 추정: 각 단 이론 RPM → 해당 단', isequal(g(:)', 1:numel(ratios)));
[log, n_fail] = chk(log, n_fail, '기어 추정: 클러치 분리(아이들 RPM) → 0', prc_gear_estimate(1600, 40, P) == 0);
[log, n_fail] = chk(log, n_fail, '기어 추정: 저속 → 0', prc_gear_estimate(3000, 3, P) == 0);

% ---- 7. 기준 차속·센서 무효 판정
R = prc_vehicle_ref([60 60 66 66; 0 60 66 66; 50 45 50 45], true(3, 4), P);
[log, n_fail] = chk(log, n_fail, '슬립 계산 (전 60 / 후 66 → 0.10)', abs(R.slip(1) - 0.10) < 1e-9);
[log, n_fail] = chk(log, n_fail, 'FL 0 km/h(끊김) → FL 무효, 기준 차속은 FR', ~R.valid(2, 1) && abs(R.v_ref(2) - 60) < 1e-9);
[log, n_fail] = chk(log, n_fail, '코너 안팎 차이(10%)는 정상', all(R.valid(3, :)));

% ---- 8. TCS 로직
st = prc_tcs_init();
for i = 1:30, [st, o] = prc_tcs_step(st, struct('slip', 0.4, 'ok', true, 'tps', 1, 'sw', true, 'can_ok', true), P); end
[log, n_fail] = chk(log, n_fail, sprintf('TCS: 슬립 0.4 지속 → 개입 (감소율 %.2f)', o.cut), o.mode == 2 && o.cut > 0.3);
[st, o] = prc_tcs_step(st, struct('slip', 0.4, 'ok', false, 'tps', 1, 'sw', true, 'can_ok', true), P);
[log, n_fail] = chk(log, n_fail, 'TCS: 휠속도 무효 → FAULT, 사유 2', o.mode == 3 && o.reason == 2);
c0 = o.cut;
[st, o] = prc_tcs_step(st, struct('slip', 0.4, 'ok', false, 'tps', 1, 'sw', true, 'can_ok', true), P);
[log, n_fail] = chk(log, n_fail, 'TCS: 고장 시 감소율은 천천히 복귀 (급복귀 없음)', o.cut < c0 && c0 - o.cut <= P.tcs.rate_down * P.tcs.Ts + 1e-12);
st = prc_tcs_init();
[st, o] = prc_tcs_step(st, struct('slip', 0.4, 'ok', true, 'tps', 1, 'sw', false, 'can_ok', true), P);
[log, n_fail] = chk(log, n_fail, 'TCS: 스위치 꺼짐 → OFF, 개입 없음', o.mode == 0 && o.cut == 0);
[st, o] = prc_tcs_step(prc_tcs_init(), struct('slip', 0.4, 'ok', true, 'tps', 1, 'sw', true, 'can_ok', false), P);
[log, n_fail] = chk(log, n_fail, 'TCS: CAN 끊김 → FAULT, 사유 3', o.mode == 3 && o.reason == 3);
[st, o] = prc_tcs_step(prc_tcs_init(), struct('slip', 0.02, 'ok', true, 'tps', 1, 'sw', true, 'can_ok', true), P);
[log, n_fail] = chk(log, n_fail, 'TCS: 슬립이 목표보다 낮으면 개입 없음', o.cut == 0 && o.mode == 1);

% ---- 9. 종방향 시뮬: TCS 효과 (젖은 노면 가정)
Roff = sim_longitudinal(P, struct('tcs_on', false, 'mu_scale', 0.7, 't_end', 8));
Ron  = sim_longitudinal(P, struct('tcs_on', true,  'mu_scale', 0.7, 't_end', 8));
[log, n_fail] = chk(log, n_fail, sprintf('시뮬: 젖은 노면 TCS ON 최대 슬립 %.2f < OFF %.2f', max(Ron.slip_meas), max(Roff.slip_meas)), ...
    max(Ron.slip_meas) < max(Roff.slip_meas));
[log, n_fail] = chk(log, n_fail, sprintf('시뮬: 젖은 노면 75 m 시간 ON %.2f s ≤ OFF %.2f s', Ron.t75, Roff.t75), ...
    ~isnan(Ron.t75) && (isnan(Roff.t75) || Ron.t75 <= Roff.t75));
Rd = sim_longitudinal(P, struct('tcs_on', false, 'mu_scale', 1.0));
[log, n_fail] = chk(log, n_fail, sprintf('시뮬: 건조 75 m %.2f s, 도달 %.1f km/h (물리적으로 그럴듯한 범위 3.5~7 s)', Rd.t75, Rd.v75_kmh), ...
    Rd.t75 > 3.5 && Rd.t75 < 7 && Rd.v75_kmh < 130);
[log, n_fail] = chk(log, n_fail, '시뮬: NaN 없음', ~any(isnan([Ron.v_kmh; Ron.rpm; Ron.cut_act])));

% ---- 9b. IMU 회전·미분·제동 기준 차속
Rm = prc_rotm([2 -3.5 4]); fb0 = [0.3 -0.2 1.0];
cal = struct('Rm', Rm, 'gyro_bias', [0.1 0.2 0.3]);
[fb1, wb1] = prc_imu_apply(cal, (Rm' * fb0')', (Rm' * [1 2 3]')' + [0.1 0.2 0.3]);
[log, n_fail] = chk(log, n_fail, 'IMU 장착 회전 왕복 (센서 축 → 차체 축)', max(abs([fb1 - fb0, wb1 - [1 2 3]])) < 1e-12);
xx = 3 * (0:0.01:2)';
dd = prc_deriv(xx, 0.01, 10);
[log, n_fail] = chk(log, n_fail, '미분 함수: 기울기 3인 직선 → 3', max(abs(dd(~isnan(dd)) - 3)) < 1e-9);
wmx = [10*ones(10,1); zeros(40,1)]; axx = -9.81 * ones(50, 1);
vb = prc_brake_ref(wmx, axx, 0.01);
[log, n_fail] = chk(log, n_fail, sprintf('제동 기준 차속: 바퀴가 갑자기 0이어도 IMU 감속률로만 감소 (0.2 s 뒤 %.2f m/s)', vb(30)), ...
    abs(vb(30) - (10 - 20*0.01*1.05*9.81)) < 0.2);

% ---- 10. 더미 세션 → CSV → 분석: 넣은 고장을 찾아내는가
csv = fullfile(outdir, 'test_session.csv');
[F, truth] = gen_dummy_session(P, db, struct('seed', 3, 'out_csv', csv));
F2 = prc_log_read_csv(csv);
[log, n_fail] = chk(log, n_fail, 'CSV 저장/읽기 왕복 (프레임 수·바이트 동일)', numel(F2.t_us) == numel(F.t_us) && isequal(F2.data, F.data));
S = prc_log_decode(F2, db);
Q = prc_quality_check(S, db, P);
lost = sum([Q.msg.seq_lost]);
seq_ids = [];
for i = 1:numel(db.msgs)
  if any(cellfun(@(x) numel(x) > 3 && strcmp(x(end-2:end), 'Seq'), {db.msgs{i}.sigs.name}))
    seq_ids(end+1) = db.msgs{i}.id; %#ok<AGROW>
  end
end
expect = sum(ismember(truth.faults.dropped_ids, seq_ids));
[log, n_fail] = chk(log, n_fail, sprintf('순번으로 찾은 손실 %d = 순번 있는 메시지에 주입한 누락 %d (전체 누락 %d)', ...
    lost, expect, numel(truth.faults.dropped_ids)), lost == expect);
sh = Q.msg(strcmp({Q.msg.name}, 'DAQ_Shock'));
[log, n_fail] = chk(log, n_fail, sprintf('쇽 60 ms 연속 누락 탐지 (최대 공백 %.0f ms)', sh.max_gap_ms), sh.max_gap_ms >= 60);
fl = Q.wss_faults(strcmp({Q.wss_faults.wheel}, 'FL'));
okfl = ~isempty(fl) && any([fl.t0] < truth.faults.fl_dropout(2) & [fl.t1] > truth.faults.fl_dropout(1));
[log, n_fail] = chk(log, n_fail, 'FL 휠센서 끊김 구간 탐지', okfl);
[log, n_fail] = chk(log, n_fail, 'NTC T3 단선 탐지 (무효 목록에 T3)', any(~cellfun(@isempty, strfind(Q.invalid, 'DAQ_Temp.T3'))));
[log, n_fail] = chk(log, n_fail, '끊김이 없는 바퀴(FR·RL·RR)는 이상 없음', ~any(strcmp({Q.wss_faults.wheel}, 'FR') | ...
    strcmp({Q.wss_faults.wheel}, 'RL') | strcmp({Q.wss_faults.wheel}, 'RR')));
delete(csv);

% ---- 결과
log{end+1} = sprintf('합계: %d개 중 실패 %d', numel(log), n_fail);
for i = 1:numel(log), fprintf('%s\n', log{i}); end

end

function [log, n_fail] = chk(log, n_fail, name, ok)
if ok, log{end+1} = ['PASS  ' name];
else, log{end+1} = ['FAIL  ' name]; n_fail = n_fail + 1; end
end
