function P = prc27_params()
% PRC27_PARAMS  27PRC 가상 검증용 차량·센서·제어 파라미터
%
%   P = prc27_params();
%
% 출처 태그 (값 옆 주석):
%   [보고서]   PRC-26 설계보고서(엔트리 23, 2026-06-19 제출본)
%   [공개제원] KTM Duke 390(2016) 공개 제원. 실차 확인 전
%   [유도]     보고서 값으로 계산한 값
%   [문서]     PARA 27PRC 설계 문서의 제안값
%   [가정]     계산이 돌도록 넣은 예시값. 근거 없음 → 실측으로 교체
%
% 27PRC는 26 차량 프레임·엔진을 이어 쓴다는 전제의 출발값이다.
% 27PRC 실제 설계가 정해지면 이 파일만 고치면 된다.

P = struct();
P.version = 'sim-v0.1 (2026-10-04, CLAUDE)';

% ===================== 차량 =====================
P.veh.mass_kg     = 210 + 70;  % [보고서] 공차 210 kg + 운전자 70 kg (표 13). 현가 표 5는 275 kg
P.veh.wheelbase_m = 1.510;     % [보고서] 축거
P.veh.track_f_m   = 1.219;     % [보고서] 윤거 전
P.veh.track_r_m   = 1.200;     % [보고서] 윤거 후
P.veh.cg_h_m      = 0.284;     % [보고서] CG 높이 (표 5; ARB 계산은 280 mm)
P.veh.rear_frac   = 0.48;      % [보고서] 무게 배분 전 52 : 후 48
P.veh.ClA_m2      = 3.91;      % [유도] 80 km/h CFD 다운포스 1183.5 N / 동압 302 Pa
P.veh.CdA_m2      = 2.51;      % [유도] 80 km/h CFD 항력 759.4 N / 동압 302 Pa
P.veh.aero_rear_frac = 0.69;   % [유도] 보고서 윙 단품 CFD: 리어윙 489 N + 사이드 절반 → 뒤 약 69%
P.veh.Izz_kgm2    = 140;       % [가정] 요 관성. m·a·b·0.85 수준. 실측(진자 시험) 필요
P.veh.roll_stiff_dist_f = 0.517;  % [보고서] 전륜 롤강성 분담 51.7% (표5)
P.veh.roll_grad_deg_g = 0.93;     % [보고서] 롤 기울기 계산값 (표6)
P.veh.rho         = 1.225;     % 공기 밀도 kg/m^3
P.veh.Crr         = 0.015;     % [가정] 구름저항계수
P.veh.g           = 9.81;

% ===================== 타이어 =====================
% Hoosier 43167 R20 20.0x7.5-13 [보고서]
P.tire.r_m     = 0.508/2;      % [보고서] 타이어 지름 508 mm. 하중 상태 굴림반경은 실측 교정 필요
P.tire.mu      = 1.5;          % [보고서] 제동 계산에 쓴 노면 마찰계수
P.tire.B = 12; P.tire.C = 1.65; P.tire.E = 0.0;  % [가정] 매직포뮬러 형상(종방향). 피크 슬립 약 0.12
P.tire.J_wheel_kgm2 = 0.35;    % [가정] 휠+타이어+디스크 1개 관성
P.tire.v_floor = 1.0;          % 저속 슬립 계산 분모 하한 m/s (수치 안정용)
% 횡방향 (2트랙 모델용). FSAE TTC 데이터로 교체할 자리 — EV 보고서는 TTC Round 9 Run 28을 썼다.
P.tire.Cy_per_N  = 0.30;       % [가정] 코너링 계수 N/deg per N 수직하중 (타이어 1개 C_alpha = Cy·Fz)
P.tire.mu_y0     = 1.5;        % [보고서] 횡 마찰 피크 (기준 하중에서)
P.tire.Fz_ref_N  = 700;        % 기준 수직하중
P.tire.load_sens = 0.00035;    % [가정] 하중 민감도: mu_y = mu_y0 − load_sens·(Fz − Fz_ref)
P.tire.By = 0; P.tire.Cy = 1.4; % 매직포뮬러 형상 (By는 C_alpha에서 계산)

% ===================== 엔진 (KTM Duke 390 2016) =====================
% 최대 토크 37 N·m @7000, 최대 출력 44.6 ps @9000 [보고서 표 17]
% 곡선 형상은 [가정]: 두 점을 지나도록 그린 예시. 리스트릭터·흡배기 변경 미반영.
% 실제 다이노/Lotus 결과가 생기면 이 표만 바꾼다.
P.eng.rpm_tab = [1500 3000 4000 5000 6000 7000 8000 9000 9500 10000 10500];
P.eng.trq_tab = [  18   25   29   32   35   37 36.2 34.9 33.0  30.0  26.0];
P.eng.idle_rpm   = 1600;       % [가정]
P.eng.rpm_limit  = 10500;      % [보고서] 제원상 Rev limit (배기 설계 절)
P.eng.launch_rpm = 7000;       % [가정] 발진 회전수
P.eng.J_kgm2     = 0.01;       % [가정] 크랭크 기준 회전관성. 1단 환산 시 등가질량 약 125 kg으로 결과에 민감
P.eng.fric_a     = 3.0;        % [가정] 스로틀 닫힘 시 마찰 토크 N·m (a + b*krpm)
P.eng.fric_b     = 0.6;

% ===================== 구동계 =====================
P.drv.primary = 80/30;         % [공개제원] 1차 감속 30:80
P.drv.gears   = [32/12 26/14 27/19 24/21 22/23 21/25];  % [공개제원] 1~6단
P.drv.final   = 4.0;           % [보고서] 종감속 기어비 4:1
P.drv.eta     = 0.90;          % [가정] 구동계 효율
P.drv.shift_time_s = 0.08;     % [가정] 변속 중 토크 차단 시간
P.drv.shift_rpm    = 10000;    % [가정] 시뮬 변속 회전수 (가상 다이노 추천값과 비교)
P.drv.gear_tol     = 0.06;     % 기어 추정 허용 비율 오차
P.drv.gear_min_kmh = 8;        % 이 속도 미만은 기어 추정 보류
% 확인 필요: 보고서 표 27(기어별 차속)은 1차×종감속 ≈ 10.34를 쓴 값이다.
% 위 값(80/30 × 4 = 10.67)과 약 3% 차이 → 1차 감속비 또는 굴림반경 실측으로 정리할 것.

P.drv.diff = 'open';           % [미정] 'open'(좌우 같은 토크) 또는 'locked'(스풀). 실제 차동 방식 확인 필요
P.brk.rear_ratio = 0.6;        % [가정] 후륜 회로 압력 / 전륜 회로 압력 (밸런스바)

% ===================== 가상 운전자 (vt_driver) =====================
P.driver.Ld_min_m   = 2.5;     % 전방 주시 최소 거리
P.driver.Ld_gain_s  = 0.25;    % 주시 거리 = gain × 속도
P.driver.k_elat_deg_m = 5.0;   % 횡편차 피드백 deg(바퀴각)/m
P.driver.k_psi      = 0.8;     % 방위 오차 피드백 (deg/deg)
P.driver.Kus_ff_deg_g = 1.0;   % 운전자가 예상하는 언더스티어 보정 deg/g (피드포워드)
P.driver.sw_rate_dps = 600;    % 핸들 속도 한계 deg/s
P.driver.sw_tau_s   = 0.03;    % 핸들 1차 지연 s (저속 떨림 억제. 0.10이면 슬라럼에서 늦어 이탈)
P.driver.preview_s  = 0.4;     % 목표 속도를 이만큼 앞에서 읽음
P.driver.v_scale    = 0.92;    % 준정상상태 상한 대비 운전자 목표 (완벽한 운전자 = 1.0). 0.97이면 슬라럼 뒤 스핀
P.driver.kp = 0.6; P.driver.ki = 0.15;   % 속도 PI → 스로틀(+)/제동(−)
P.driver.pF_max_bar = 50;      % 제동 명령 최대 압력

% ===================== 휠속도 센서 =====================
P.wss.teeth          = 30;     % [가정] 톤휠 톱니 수(회전당 펄스)
P.wss.timeout_s      = 0.25;   % 이 시간 동안 에지 없으면 0 km/h
P.wss.rate_hz        = 100;    % [문서] 휠속도 기록률 제안 100~200 Hz
P.wss.pair_tol_kmh   = 8;      % 좌우 비교: 이 값과 아래 비율 중 큰 쪽보다 차이 나면 낮은 쪽 무효
P.wss.pair_tol_rel   = 0.30;
P.wss.pair_check_min_kmh = 10; % 이 속도 미만에서는 좌우 비교 안 함

% ===================== TCS =====================
% 휠스피드_TCS_구현계획_20260929_GPT.md 기준. 개입 방식(점화/연료/스로틀)은 미정이라
% '엔진 토크 감소율'로만 모델링한다. 모든 수치 [가정].
P.tcs.Ts          = 0.01;      % 제어 주기 s
P.tcs.target_slip = 0.10;      % 목표 슬립
P.tcs.kp          = 3.0;       % 슬립 오차 → 토크 감소율
P.tcs.ki          = 30.0;
P.tcs.max_cut     = 0.7;       % 최대 토크 감소율
P.tcs.rate_up     = 20;        % 감소율 증가 속도 /s
P.tcs.rate_down   = 4;         % 감소율 복귀 속도 /s (급격한 토크 복귀 방지)
P.tcs.tps_min     = 0.10;      % 스로틀 10% 미만이면 대기
P.tcs.v_floor_kmh = 10;        % 슬립 분모 하한 (발진 직후 과대 슬립 방지)
P.tcs.act_delay_s = 0.02;      % ECU 개입 지연 (CAN + EMU 처리)
P.tcs.torque_tau_s = 0.03;     % 토크 반영 1차 지연

% ===================== 조향 (SAS) =====================
% 애커만 조향 표: PRC-26 설계보고서 Fig.9 (CAD 기구학, point 1). 실측 표가 생기면 교체.
Dd = prc26_design_data();
P.steer.lut = Dd.formula.steer.lut(:, 1:3);          % [보고서] 핸들각 | 외륜각 | 내륜각 (deg)
P.steer.ackermann_pct = Dd.formula.steer.lut(:, 5);
P.steer.ratio_tab = Dd.formula.steer.lut(:, 6);
P.steer.sw_max_deg = P.steer.lut(end, 1);           % 기구 조향 제한 ±126°
% 분석기(analyze_steering)용 평균각 표 — LUT에서 만든다
P.steer.swa_tab   = [-flipud(P.steer.lut(2:end, 1)); P.steer.lut(:, 1)]';
avg_ = (P.steer.lut(:, 2) + P.steer.lut(:, 3)) / 2;
P.steer.wheel_tab = [-flipud(avg_(2:end)); avg_]';

% ===================== 제동 (BPS 전·후 회로 각 1개) =====================
% 바퀴 1개당 제동 토크 = 계수 × 해당 회로 압력. 캘리퍼·패드·디스크 유효반경으로 계산해 교체.
P.brk.k_front_Nm_bar = 7.5;    % [가정] 전륜 바퀴당 N·m/bar
P.brk.k_rear_Nm_bar  = 3.5;    % [가정] 후륜 바퀴당 N·m/bar
P.brk.target_stop_m  = 4.5;    % [보고서] 40 km/h에서 4.5 m 이내 정지 목표
P.brk.target_v0_kmh  = 40;
P.brk.lock_slip      = 0.5;    % 이 슬립 이상이 30 ms 이상이면 잠김으로 판정

% ===================== 차체 거동 (더미 생성용) =====================
P.sim.roll_grad_deg_g  = 1.5;  % [가정] 횡가속 1 g당 롤각 (좌회전 시 오른쪽이 내려감)
P.sim.pitch_grad_deg_g = 0.6;  % [가정] 종가속 1 g당 피치각 (가속 시 앞이 들림)

% ===================== IMU (더미 생성용 장착 오차) =====================
% 실제 IMU는 차체 축과 조금씩 어긋나게 달린다. 더미 데이터에 일부러 넣고 prc_imu_align이 찾는지 본다.
P.imu_sim.mount_deg     = [2.0 -3.5 4.0];   % [가정] 장착 롤·피치·요 오차
P.imu_sim.gyro_bias_dps = [0.4 -0.3 0.6];   % [가정] 자이로 영점 오차
P.imu_sim.acc_noise_g   = 0.02;
P.imu_sim.gyro_noise_dps = 0.3;

% ===================== 연료 매핑 가이드 =====================
% 칸 축은 EMU 연료(VE) 표와 맞춰 바꾼다. 부하 축을 MAP으로 쓰면 tps_bp 대신 MAP 축으로 교체.
P.map.rpm_bp = 1500:500:10500;               % RPM 축
P.map.tps_bp = [0 5 10 20 30 40 60 80 100];  % TPS 축 %
% 목표 람다: 팀이 정할 값. 지금은 [가정] 부분부하 1.00 → 풀스로틀 0.88로 TPS에 따라 선형.
P.map.lambda_target_tps = [0 30 80 100];
P.map.lambda_target_val = [1.00 1.00 0.88 0.88];
P.map.lambda_delay_s = 0.12;   % [가정] 연소→센서 지연 (배기 길이·회전수에 따라 다름. 스텝 응답으로 실측)
P.map.min_samples   = 25;      % 칸당 최소 샘플 (50 Hz 기준 0.5 s)
P.map.max_std       = 0.04;    % 칸 안 람다 표준편차 상한 (넘으면 '흔들림')
P.map.max_step_pct  = 15;      % 한 번에 제안하는 최대 연료 변경 %
P.map.clt_min       = 70;      % 냉각수 이 온도 이상에서만 (웜업 보정 제외)
P.map.dtps_max      = 30;      % 정상 상태 판정: TPS 변화율 %/s
P.map.drpm_max      = 1500;    % 정상 상태 판정: RPM 변화율 rpm/s
P.map.settle_s      = 0.5;     % TPS·RPM이 변한 뒤 이 시간 동안은 버림 (가속 증량·센서 지연 안정)

% 현재 연료(VE) 맵 — [더미] EMU 맵 파일을 받으면 표로 교체. 3D 그림의 '적용 전/후'에만 쓴다.
P.map.ve_base = @(rpm, tps) 40 + 45 * (tps/100).^0.6 .* (0.8 + 0.2 * exp(-((rpm - 7500)/3000).^2));

% ===================== 점화 =====================
% 노킹 계측 보류(09-22) 상태라 진각 제안은 '노킹 감시 후 검토'로만 표시하고 적용 권장에서 뺀다.
P.ign.rpm_bp = P.map.rpm_bp;
P.ign.tps_bp = P.map.tps_bp;
% 현재 점화맵 — [더미] EMU 맵 파일을 받으면 교체. deg BTDC
P.ign.base = @(rpm, tps) 8 + 14 * min(rpm, 6000) / 6000 + 14 * (1 - tps/100) - 1.5 * max(rpm - 9000, 0) / 1500;
P.ign.knock_monitoring = false;  % 노킹 감시(센서·청음·실린더압) 있으면 true → 진각 제안도 적용 권장
P.ign.mbt_margin_deg = 2;        % MBT보다 이만큼 늦게 (노킹·편차 여유)
P.ign.max_step_deg   = 2;        % 한 번에 바꾸는 최대 각도
P.ign.spike_deg      = 2.5;      % 주변 칸 평균과 이만큼 다르면 '튀는 칸'
P.ign.min_samples    = 25;

% ===================== 로그(더미) 주기 =====================
% CAN 프레임 이름 → 기대 주기 s (dbc/PRC27_sim_v0.1.dbc)
P.log.period_s = struct( ...
  'EMU_SIM_0',    0.02, ...
  'EMU_SIM_1',    0.02, ...
  'DAQ_Temp',     0.10, ...
  'DAQ_Shock',    0.01, ...
  'DAQ_Chassis',  0.01, ...
  'DAQ_GNSS_Pos', 0.10, ...
  'DAQ_GNSS_Vel', 0.10, ...
  'DAQ_IMU_Acc',  0.02, ...
  'DAQ_IMU_Gyro', 0.02, ...
  'DAQ_Status',   0.50, ...
  'WSS_Speed',    0.01, ...
  'WSS_TCS',      0.01);
end
