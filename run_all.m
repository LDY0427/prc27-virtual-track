% RUN_ALL  27PRC 가상 검증 환경 전체 실행 (MATLAB / GNU Octave 공용)
%
%   이 폴더에서 run_all 을 실행하면:
%     1) 자체 테스트 (tests/run_tests.m)
%     2) 가상 다이노: 토크·출력, 단별 구동력, 변속점 후보
%     3) 직선 가속 75 m: TCS OFF/ON × 건조/젖은 노면 비교
%     4) 더미 주행 세션 생성 → 원본 CAN 로그 CSV
%     5) 그 로그를 분석기에 넣어 품질 점검·TCS 재계산·그림·보고서
%     6) 로드 다이노·코스트다운  7) 조향 특성(SAS)  8) 제동(BPS) — 시험별 더미 로그 생성 → 분석
%     9) 연료맵 가이드: 람다 로그 → RPM×TPS 칸별 연료 증감 제안 + 3D (더미 맵 오차를 찾는지 확인)
%        9b) 점화맵 점검(로그 점화각 3D, 튀는 칸)  9c) 점화 MBT 스윕(오프셋별 토크 → MBT, 진각은 노킹 감시 후)
%    10) 설계·세팅 비교 (종감속비·변속·발진 rpm·질량·CdA·후륜 배분, 약 1분. RUN_SWEEP로 끔)
%    11) 5~9의 분석이 더미에 심어 둔 정답을 다시 찾는지 확인
%   결과는 out/ 폴더에 쌓인다. 모든 결과는 가상 데이터다.
%
% 작성: CLAUDE | 2026-10-04

clear; close all;
RUN_SWEEP = true;      % 설계·세팅 비교(약 1분)를 건너뛰려면 false
RUN_IGN_SWEEP = true;  % 점화 MBT 스윕(약 40초)을 건너뛰려면 false
RUN_VT = true;         % 가상 시험장(2트랙 차량·운전자·도로, 약 3분)을 건너뛰려면 false
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
cd(here);
addpath(fullfile(here, 'config'), fullfile(here, 'lib'), fullfile(here, 'sim'), ...
        fullfile(here, 'analysis'), fullfile(here, 'tests'));
outdir = fullfile(here, 'out');
if ~exist(outdir, 'dir'), mkdir(outdir); end

P  = prc27_params();
db = prc_dbc_load(fullfile(here, 'dbc', 'PRC27_sim_v0.1.dbc'));
fprintf('== 27PRC 가상 검증 %s ==\n', P.version);

% ---------------- 1. 테스트 ----------------
fprintf('\n[1] 자체 테스트\n');
[n_fail, tlog] = run_tests(P, db, outdir);
prc_write_lines(fullfile(outdir, 'test_result.txt'), tlog);

% ---------------- 2. 가상 다이노 ----------------
fprintf('\n[2] 가상 다이노\n');
D = sim_virtual_dyno(P);
ng = numel(D.ratios);
h = prc_fig(1100, 450);
subplot(1,2,1);
[ax, l1, l2] = plotyy(D.rpm, D.Te, D.rpm, D.kW); grid on;
xlabel('engine speed [rpm]'); ylabel(ax(1), 'torque [N m]'); ylabel(ax(2), 'power [kW]');
title('Engine (assumed WOT curve)');
subplot(1,2,2); hold on; grid on;
for g = 1:ng, plot(D.v_kmh(:, g), D.F_N(:, g)); end
plot(D.v_res_kmh, D.F_res_N, 'k--');
plot([0 max(D.v_res_kmh)], D.F_trac_N * [1 1], 'r:');
legend([arrayfun(@(g) sprintf('gear %d', g), 1:ng, 'UniformOutput', false), {'drag+rolling', 'rear grip limit'}], ...
       'Location', 'northeast');
xlabel('vehicle speed [km/h]'); ylabel('tractive force [N]'); title('Tractive force per gear');
prc_savefig(h, fullfile(outdir, 'dyno.png'));
Ld = {'# 가상 다이노 결과 (가정 토크곡선 기준)', '', ...
      sprintf('- 최대 토크 %.1f N·m @ %d rpm, 최대 출력 %.1f kW (%.1f ps) @ %d rpm', ...
              max(D.Te), D.rpm(find(D.Te == max(D.Te), 1)), max(D.kW), max(D.kW)*1.3596, D.rpm(find(D.kW == max(D.kW), 1))), ...
      sprintf('- 후륜 접지 한계(정상 가속, 다운포스 제외) %.0f N', D.F_trac_N), '', ...
      '| 단 | 전체 감속비 | 리밋 차속 km/h | 최대 구동력 N | 접지 한계 초과 | 다음 단 변속 추천 rpm |', ...
      '|---:|---:|---:|---:|:---:|---:|'};
for g = 1:ng
  if g < ng, sr = sprintf('%.0f', D.shift_rpm(g)); else, sr = '-'; end
  yn = 'N'; if D.traction_limited(g), yn = 'Y'; end
  Ld{end+1} = sprintf('| %d | %.2f | %.1f | %.0f | %s | %s |', g, D.ratios(g), D.v_max_gear_kmh(g), max(D.F_N(:, g)), yn, sr); %#ok<SAGROW>
end
Ld{end+1} = '';
Ld{end+1} = '변속 추천 rpm = 같은 차속에서 다음 단 구동력이 더 커지는 지점(변속 시간 손실 미반영). 리밋에 걸리면 리밋 값.';
Ld{end+1} = '![dyno](dyno.png)';
prc_write_lines(fullfile(outdir, 'dyno_report.md'), Ld);
fprintf('%s\n', Ld{[3 4]});

% ---------------- 3. 직선 가속 비교 ----------------
fprintf('\n[3] 직선 가속 75 m: TCS OFF/ON × 노면\n');
cases = {'건조 mu x1.0', 1.0; '젖음 mu x0.7', 0.7};
h = prc_fig(1100, 800); La = {'# 직선 가속 75 m 시뮬레이션 (가상)', '', ...
  '| 노면 | TCS | 75 m 시간 s | 75 m 차속 km/h | 최대 슬립 | 최대 토크 감소율 |', '|---|---|---:|---:|---:|---:|'};
for c = 1:2
  for on = [false true]
    Rr = sim_longitudinal(P, struct('tcs_on', on, 'mu_scale', cases{c, 2}, 't_end', 8));
    onstr = 'OFF'; if on, onstr = 'ON'; end
    La{end+1} = sprintf('| %s | %s | %.2f | %.1f | %.2f | %.2f |', cases{c, 1}, onstr, Rr.t75, Rr.v75_kmh, ...
                        max(Rr.slip_meas), max(Rr.cut_act)); %#ok<SAGROW>
    subplot(2, 2, (c-1)*2 + 1); hold on; grid on;
    plot(Rr.t, Rr.slip_meas); ylim([-0.1 1.5]); ylabel('slip'); title(sprintf('slip, mu x%.1f', cases{c, 2}));
    subplot(2, 2, (c-1)*2 + 2); hold on; grid on;
    plot(Rr.t, Rr.v_kmh); ylabel('km/h'); title(sprintf('speed, mu x%.1f', cases{c, 2}));
  end
  subplot(2, 2, (c-1)*2 + 1); legend('TCS OFF', 'TCS ON', 'Location', 'northeast');
  subplot(2, 2, (c-1)*2 + 2); legend('TCS OFF', 'TCS ON', 'Location', 'southeast'); xlabel('time [s]');
end
prc_savefig(h, fullfile(outdir, 'accel_tcs.png'));
La{end+1} = '';
La{end+1} = '75 m 시간은 스로틀 시작부터. 변속은 P.drv.shift_rpm에서 고정 시간 토크 차단. NaN = 8 s 안에 75 m 미도달.';
La{end+1} = '![accel](accel_tcs.png)';
prc_write_lines(fullfile(outdir, 'accel_report.md'), La);
fprintf('%s\n', La{5:8});

% ---------------- 4. 더미 세션 ----------------
fprintf('\n[4] 더미 주행 세션 생성\n');
csv = fullfile(outdir, 'dummy_session.csv');
[F, truth] = gen_dummy_session(P, db, struct('seed', 1, 'out_csv', csv));
fprintf('프레임 %d개, %.1f s → %s\n', numel(F.t_us), truth.t(end), csv);

% ---------------- 5. 분석 ----------------
fprintf('\n[5] 로그 분석\n');
A = analyze_session(csv, P, db, outdir);
fprintf('품질·TCS·기어 대조 → %s\n', A.report);
fprintf('TCS 로그 vs 재계산 최대 차이 %.1f %%p, 기어 추정 일치 %.1f %%\n', A.tcs_cut_maxdiff_pct, A.gear_agree_pct);

% ---------------- 6~8. 시험별 더미 로그 → 분석 ----------------
fprintf('\n[6] 로드 다이노·코스트다운 (양방향 경사 0.8%%, 정답 토크 곡선은 가정과 다름)\n');
[~, Td] = gen_dyno_runs(P, db, struct('out_csv', fullfile(outdir, 'dummy_dyno.csv')));
Ad = analyze_road_dyno(fullfile(outdir, 'dummy_dyno.csv'), P, db, outdir, Td);
fprintf('CdA %.3f (정답 %.3f), Crr %.4f (정답 %.4f), 토크 오차 IMU %.2f%% / 휠만 %.2f%%\n', ...
        Ad.coast.CdA(2), Td.CdA_eff, Ad.coast.Crr(2), Td.Crr, Ad.dyno.errB_pct, Ad.dyno.errA_pct);

fprintf('\n[7] 조향 특성 (스키드패드 좌·우, 속도 서서히 증가)\n');
[~, Ts] = gen_skidpad_runs(P, db, struct('out_csv', fullfile(outdir, 'dummy_skidpad.csv')));
As = analyze_steering(fullfile(outdir, 'dummy_skidpad.csv'), P, db, outdir, Ts);
fprintf('언더스티어 기울기 %.2f deg/g (정답 %.2f), 롤 기울기 %.2f deg/g (정답 %.2f)\n', ...
        As.K_us, Ts.K_us, As.roll_grad, Ts.roll_grad_deg_g);

fprintf('\n[8] 제동 (40 km/h, 압력 단계별)\n');
[~, Tb] = gen_brake_runs(P, db, struct('out_csv', fullfile(outdir, 'dummy_brake.csv')));
Ab = analyze_brake(fullfile(outdir, 'dummy_brake.csv'), P, db, outdir, Tb);
for q = 1:numel(Ab.events)
  E = Ab.events(q);
  fprintf('  %2.0f bar: 감속 %.2f g, 정지 %.2f m, 첫 잠김 %s, 4륜 잠김 %d\n', E.pF_max, E.decel_g, E.stop_m, E.first, E.all4);
end

fprintf('\n[9] 연료맵 가이드 (더미: 7000~9000 rpm 고부하 8%% 희박, 2000~3500 rpm 부분부하 6%% 농후)\n');
[~, Tm] = gen_map_runs(P, db, struct('out_csv', fullfile(outdir, 'dummy_map.csv')));
Gm = fuel_map_guide(fullfile(outdir, 'dummy_map.csv'), P, db, outdir, Tm);
fprintf('제안 칸 %d개, 정답 대비 평균 %.2f %%p / 최대 %.2f %%p -> %s\n', sum(Gm.status(:) == 3), ...
        Gm.err_mean, Gm.err_max, fullfile(outdir, 'fuel_guide_report.md'));

fprintf('\n[9b] 점화맵 점검 (더미: 6000 rpm·TPS 30%% 칸에 +5도 입력 실수)\n');
Ig = ign_map_review(fullfile(outdir, 'dummy_map.csv'), P, db, outdir, Tm);
fprintf('튀는 칸 %d개, 부하 역전 %d칸 -> %s\n', sum(Ig.spike(:)), sum(Ig.inverse(:)), fullfile(outdir, 'ign_review_report.md'));

if RUN_IGN_SWEEP
  fprintf('\n[9c] 점화 MBT 스윕 (오프셋 -4~+4도 로그 5개, 약 40초)\n');
  [ign_files, Tk] = gen_ign_sweep_runs(P, db, outdir);
  Kg = ign_mbt_sweep(ign_files, Tk.offsets, P, db, outdir, Tk);
  fprintf('MBT 정답 대비 평균 %.2f도 / 최대 %.2f도, 적용 권장 진각 칸 %d개(노킹 감시 없음) -> %s\n', ...
          mean(Kg.mbt_err), max(Kg.mbt_err), sum(Kg.apply > 0), fullfile(outdir, 'ign_mbt_report.md'));
else
  Kg = []; Tk = [];
end

if RUN_SWEEP
  fprintf('\n[10] 설계·세팅 비교\n');
  Wd = sweep_design(P, outdir);
  fprintf('75 m 최소: 종감속 %.1f / 변속 %d rpm / 발진 %d rpm -> %s\n', Wd.final.x(Wd.final.best), ...
          Wd.shift.x(Wd.shift.best), Wd.launch.x(Wd.launch.best), fullfile(outdir, 'sweep_design_report.md'));
end

if RUN_VT
  fprintf('\n[10b] 가상 시험장: 2트랙 차량 + 운전자 + 도로 (가속·스키드패드·오토크로스, 약 3분)\n');
  Vt = vt_report(P, db, outdir);
  fprintf('75 m %.2f s | 스키드패드 2바퀴 %.2f s (목표 9.7) | 오토크로스 %.0f m 랩 %.2f s (QSS %.2f) -> %s\n', ...
          Vt.accel.lap_time_s, Vt.skid.lap_time_s, Vt.autox.road.length_m, Vt.autox.lap_time_s, Vt.autox.qss_time_s, fullfile(outdir, 'vt_report.md'));
  [n_fail_vt, vlog] = check_vt(Vt, P);
else
  n_fail_vt = 0; vlog = {};
end

fprintf('\n[11] 정답 재현 확인\n');
[n_fail2, clog] = check_test_runs(struct('P', P, 'dyno', Ad, 'dyno_truth', Td, 'steer', As, 'steer_truth', Ts, ...
  'brake', Ab, 'brake_truth', Tb, 'session', A, 'session_truth', truth, 'fuel', Gm, 'fuel_truth', Tm, ...
  'ign', Ig, 'mbt', {Kg}, 'mbt_truth', {Tk}));
prc_write_lines(fullfile(outdir, 'test_result.txt'), [tlog, {''}, clog, {''}, vlog]);
n_fail = n_fail + n_fail2 + n_fail_vt;

fprintf('\n== 완료: 테스트 실패 %d, 결과 폴더 %s ==\n', n_fail, outdir);
