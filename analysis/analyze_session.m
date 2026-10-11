function A = analyze_session(logfile, P, db, outdir)
% ANALYZE_SESSION  원본 CAN 로그(CSV) → 디코딩 → 품질 점검 → TCS·기어 재계산 대조 → 그림·보고서.
%
%   A = analyze_session('out/dummy_session.csv', P, db, 'out')
%
% 실제 로거 로그가 생기면 같은 함수에 그 CSV를 넣으면 된다.
% 출력: outdir/session_*.png, outdir/session_report.md
if nargin < 4, outdir = 'out'; end
F = prc_log_read_csv(logfile);
S = prc_log_decode(F, db);
Q = prc_quality_check(S, db, P);
A.S = S; A.Q = Q; A.n_frames = numel(F.t_us);
A.duration_s = (F.t_us(end) - F.t_us(1)) / 1e6;

% ---------- IMU 장착 보정 → 요레이트 ----------
cal = prc_imu_align(S, P);
A.imu = cal;
Gy = S.DAQ_IMU_Gyro;
[~, wb] = prc_imu_apply(cal, zeros(numel(Gy.t), 3), [Gy.GyroX Gy.GyroY Gy.GyroZ]);

% ---------- 휠속도·TCS 재계산 (로그만으로 판정 재현) ----------
W = S.WSS_Speed; TC = S.WSS_TCS;
w = [W.WssFL W.WssFR W.WssRL W.WssRR];
yawW = prc_hold(Gy.t, wb(:, 3), W.t);
R0 = prc_vehicle_ref(w, true(size(w)), P);              % 선회 보정 없음 (비교용)
R = prc_vehicle_ref(w, true(size(w)), P, yawW);         % 요레이트 선회 보정
A.ref_noyaw = R0.v_ref; A.ref = R.v_ref; A.t_wss = W.t;
tps = prc_hold(S.EMU_SIM_0.t, S.EMU_SIM_0.TPS / 100, W.t);
tps(isnan(tps)) = 0;
Trep = prc_tcs_replay(R.slip, R.ref_ok & R.drv_ok, tps, true, true, P);
% 로그의 TCS 프레임과 같은 시각끼리 비교 (프레임 누락분 제외)
cutLog = prc_hold(TC.t, TC.TcsCutReq, W.t + 0.001);
both = ~isnan(cutLog) & abs(prc_hold(TC.t, TC.t, W.t + 0.001) - W.t) < 0.005;
A.tcs_cut_maxdiff_pct = max(abs(cutLog(both) - 100 * Trep.cut(both)));
A.tcs_active_s = sum(Trep.mode == 2) * P.tcs.Ts;
A.tcs_fault_s = sum(Trep.mode == 3) * P.tcs.Ts;

% ---------- 기어 추정 대조 ----------
rpmW = prc_hold(S.EMU_SIM_0.t, S.EMU_SIM_0.RPM, W.t);
gEmu = prc_hold(S.EMU_SIM_1.t, S.EMU_SIM_1.Gear, W.t);
gEst = prc_gear_estimate(rpmW, R.v_drv, P);
cmp = gEst > 0 & gEmu > 0;
A.gear_agree_pct = 100 * mean(gEst(cmp) == gEmu(cmp));

% ---------- 주요 수치 ----------
A.v_max_kmh = max(R.v_ref);
A.slip_max = max(R.slip(R.ref_ok & R.drv_ok));
A.ax_max_g = max(S.DAQ_IMU_Acc.AccX); A.ax_min_g = min(S.DAQ_IMU_Acc.AccX);
A.ay_max_g = max(abs(S.DAQ_IMU_Acc.AccY));
A.rpm_max = max(S.EMU_SIM_0.RPM);
A.brk_max_bar = [max(S.DAQ_Chassis.BrkPressF) max(S.DAQ_Chassis.BrkPressR)];
A.clt_range = [min(S.EMU_SIM_1.CLT) max(S.EMU_SIM_1.CLT)];

% ---------- 그림 ----------
tw = W.t;
h = prc_fig(1100, 800);
subplot(4,1,1); plot(tw, w(:,1), tw, w(:,2), tw, w(:,3), tw, w(:,4), ...
  S.DAQ_GNSS_Vel.t, S.DAQ_GNSS_Vel.GnssSpeed, 'k--'); grid on;
ylabel('km/h'); legend('FL','FR','RL','RR','GNSS','Location','northeast'); title('Wheel speed / GNSS speed');
subplot(4,1,2); plot(S.EMU_SIM_0.t, S.EMU_SIM_0.RPM); grid on; ylabel('rpm'); title('Engine RPM');
subplot(4,1,3); plot(S.EMU_SIM_0.t, S.EMU_SIM_0.TPS); grid on; ylabel('%'); title('TPS');
subplot(4,1,4); plot(S.EMU_SIM_1.t, S.EMU_SIM_1.Gear, tw, gEst, '--'); grid on;
ylabel('gear'); xlabel('time [s]'); legend('ECU gear','estimated','Location','northeast'); title('Gear');
prc_savefig(h, fullfile(outdir, 'session_overview.png'));

h = prc_fig(1100, 800);
subplot(3,1,1); plot(tw, R0.v_ref, ':', tw, R.v_ref, tw, R.v_drv); grid on; ylabel('km/h');
legend('reference, no yaw corr.','reference (front, yaw corr.)','driven (rear)','Location','northeast');
title('Reference vs driven wheel speed');
subplot(3,1,2); plot(tw, R.slip, [tw(1) tw(end)], P.tcs.target_slip*[1 1], 'r--'); grid on;
ylim([-0.2 1.0]); ylabel('slip'); legend('slip','target','Location','northeast'); title('Driven wheel slip');
subplot(3,1,3); plot(TC.t, TC.TcsCutReq, tw, 100*Trep.cut, '--', TC.t, 10*TC.TcsMode, ':'); grid on;
ylabel('% / mode x10'); xlabel('time [s]'); legend('logged cut','MATLAB replay','mode x10','Location','northeast');
title('TCS torque cut request');
prc_savefig(h, fullfile(outdir, 'session_tcs.png'));

h = prc_fig(1100, 800);
C = S.DAQ_Chassis; Sh = S.DAQ_Shock; I = S.DAQ_IMU_Acc;
subplot(4,1,1); plot(C.t, C.SteerAngle); grid on; ylabel('deg'); title('Steering wheel angle');
subplot(4,1,2); plot(C.t, C.BrkPressF, C.t, C.BrkPressR); grid on; ylabel('bar');
legend('front','rear','Location','northeast'); title('Brake pressure');
subplot(4,1,3); plot(I.t, I.AccX, I.t, I.AccY); grid on; ylabel('g');
legend('ax','ay','Location','northeast'); title('IMU acceleration');
subplot(4,1,4); plot(Sh.t, Sh.ShockFL, '.-', Sh.t, Sh.ShockFR, Sh.t, Sh.ShockRL, Sh.t, Sh.ShockRR); grid on;
ylabel('mm'); xlabel('time [s]'); legend('FL','FR','RL','RR','Location','northeast'); title('Damper position');
prc_savefig(h, fullfile(outdir, 'session_chassis.png'));

% ---------- 보고서 ----------
Lr = {};
Lr{end+1} = '# 더미 세션 분석 보고서 (가상 데이터)';
Lr{end+1} = '';
Lr{end+1} = sprintf('> 생성: analyze_session.m | %s | 입력: `%s`', datestr(now, 'yyyy-mm-dd HH:MM'), logfile);
Lr{end+1} = '> 이 로그는 gen_dummy_session.m이 만든 가상 데이터다. 실차 성능 판단에 쓰지 않는다.';
Lr{end+1} = '';
Lr{end+1} = '## 요약';
Lr{end+1} = '';
Lr{end+1} = sprintf('- 프레임 %d개, 길이 %.1f s', A.n_frames, A.duration_s);
Lr{end+1} = sprintf('- 최고 기준 차속 %.1f km/h, 최고 RPM %.0f', A.v_max_kmh, A.rpm_max);
Lr{end+1} = sprintf('- 종가속 %.2f ~ %.2f g, 횡가속 최대 %.2f g', A.ax_min_g, A.ax_max_g, A.ay_max_g);
Lr{end+1} = sprintf('- 제동압 최대 전 %.1f / 후 %.1f bar', A.brk_max_bar(1), A.brk_max_bar(2));
Lr{end+1} = sprintf('- 냉각수 %.1f ~ %.1f °C', A.clt_range(1), A.clt_range(2));
Lr{end+1} = sprintf('- 구동륜 슬립 최대 %.2f, TCS 개입 %.2f s, TCS 고장 대기 %.2f s', A.slip_max, A.tcs_active_s, A.tcs_fault_s);
Lr{end+1} = sprintf('- 로그 TCS 요청 vs MATLAB 재계산 최대 차이 %.1f %%p', A.tcs_cut_maxdiff_pct);
Lr{end+1} = sprintf('- 기어 추정이 ECU 기어와 일치 %.1f %% (둘 다 판정한 구간)', A.gear_agree_pct);
Lr{end+1} = sprintf('- IMU 장착 추정 롤 %.2f° 피치 %.2f° 요 %.2f°, 자이로 영점 %s deg/s', cal.rpy_deg, mat2str(cal.gyro_bias, 2));
Lr{end+1} = '';
Lr{end+1} = '## 데이터 품질';
Lr{end+1} = '';
Lr = [Lr, Q.lines];
Lr{end+1} = '';
Lr{end+1} = '## 그림';
Lr{end+1} = '';
Lr{end+1} = '![overview](session_overview.png)';
Lr{end+1} = '![tcs](session_tcs.png)';
Lr{end+1} = '![chassis](session_chassis.png)';
prc_write_lines(fullfile(outdir, 'session_report.md'), Lr);
A.report = fullfile(outdir, 'session_report.md');
end
