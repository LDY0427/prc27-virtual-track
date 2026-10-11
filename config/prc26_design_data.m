function D = prc26_design_data()
% PRC26_DESIGN_DATA  두 설계보고서에서 옮겨 적은 설계값 (시뮬레이션·설계 반영용 원자료).
%
%   D = prc26_design_data();   D.formula (PRC-26, 내연기관)
%
% 출처:
%   [F] PRC-26 설계보고서, 엔트리 23, 2026-06-19 제출
% 쪽 번호는 보고서 인쇄 쪽수. 표가 그림으로만 들어 있는 값은 '그림에서 읽음'으로 표시.
% 값은 CAD/계산값이며 실차 실측이 아니다. 실측이 생기면 이 파일을 고친다.

% ===================== PRC-26 (포뮬러, KTM Duke 390) =====================
F = struct();
F.src = 'PRC-26 설계보고서 (엔트리 23)';
% --- 차량 제원 [F p.1 표1] ---
F.length_m = 2.897; F.width_m = 1.433; F.height_m = 1.172;
F.wheelbase_m = 1.510; F.track_f_m = 1.219; F.track_r_m = 1.200;
F.mass_curb_kg = 210; F.driver_kg = 70;
F.front_frac = 0.52;                      % 운전자 탑승 목표 배분 52:48
F.cg_h_m = 0.284;                         % [F p.7 표6] (ARB 계산은 280 mm)
% --- 조향 [F p.12~13, Fig.9 표 (그림에서 읽음), point 1] ---
% 열: 핸들각 deg | 외륜각 deg | 내륜각 deg | 회전반경 m | 애커만 % | 조향비
F.steer.lut = [ ...
    0     0      0      NaN    NaN     NaN
    9     2.23   2.28   38.38  99.02   3.99
   18     4.42   4.61   19.19  97.79   3.99
   27     6.58   7.00   12.78  96.53   3.98
   36     8.71   9.46    9.58  95.34   3.97
   45    10.82  12.01    7.65  94.24   3.95
   54    12.91  14.67    6.36  93.30   3.93
   63    14.96  17.45    5.44  92.66   3.91
   72    17.04  20.38    4.74  91.86   3.88
   81    19.10  23.52    4.20  91.49   3.84
   90    21.14  26.91    3.76  91.57   3.80
   99    23.19  30.66    3.40  92.14   3.75
  108    25.23  34.95    3.09  93.64   3.69
  117    27.28  40.14    2.82  96.62   3.61
  121.5  28.31  43.38    2.69  99.23   3.56
  126    29.34  47.51    2.56 103.47   3.49 ];
F.steer.note = 'CAD 기구학 기준 (3D 모델에서 랙 이동 → 외내륜각). 랙 20 mm 하향, 애커만 90~100% 목표. 실측 미확인';
F.steer.ratio_avg = 3.82;                 % 표 평균
F.steer.ackermann_avg_pct = 95.26;
F.steer.torque_static_Nm = 21.659;        % [F p.14] 정지 조향 토크 계산값 (단위 N으로 적혀 있으나 토크)
F.steer.rack_lowered_mm = 20;
% --- 현가 [F p.6~7 표4·5·6] ---
F.susp.camber_gain_f = 0.85; F.susp.camber_gain_r = 0.72;        % deg/deg roll
F.susp.camber_static_f = -1.39; F.susp.camber_static_r = -1.31;  % deg
F.susp.camber_max_f = -2.18; F.susp.camber_max_r = -2.31;
F.susp.corner_sprung_f_kg = 61.7; F.susp.corner_sprung_r_kg = 56.2;
F.susp.spring_f_lbin = 340; F.susp.spring_r_lbin = 235;
F.susp.motion_ratio_f = 0.59; F.susp.motion_ratio_r = 0.70;      % CAD 측정 0.594/0.705
F.susp.wheel_rate_f_Nmm = 20.73; F.susp.wheel_rate_r_Nmm = 20.75;
F.susp.ride_freq_f_hz = 2.67; F.susp.ride_freq_r_hz = 2.80;
F.susp.roll_rate_f_Nmdeg = 382.15; F.susp.roll_rate_r_Nmdeg = 356.70;
F.susp.roll_stiff_dist_f = 0.517;                                 % 전륜 롤강성 분담
F.susp.arb_contrib_f = 0.297; F.susp.arb_contrib_r = 0.269;
F.susp.roll_center_f_mm = 31.98; F.susp.roll_center_r_mm = 29.02;
F.susp.roll_gradient_deg_g = 0.93;                                % 계산값 (PRC-25는 1.66)
F.susp.roll_at_1p56g_deg = 1.44;
% --- 타이어·제동 [F p.19~20] ---
F.tire.name = 'Hoosier 43167 R20 20.0x7.5-13'; F.tire.dia_m = 0.508; F.tire.mu_design = 1.5;
F.brake.disk_dia_mm = 220; F.brake.pad_mu = 0.4; F.brake.piston_area_mm2 = 791.7;
F.brake.mc_area_mm2 = 87.7; F.brake.area_ratio = 18.06; F.brake.pedal_force_meas_N = 588.6;
F.brake.target = '40 km/h → 4.5 m 이내 정지'; F.brake.decel_target_g = 1.42;
F.brake.lever_R_to_CG_mm = 644;
% --- 목표 [F p.3 표2] ---
F.target.skidpad_s = 9.7; F.target.skidpad_R_m = [8.625 9.125 9.625];
F.target.skidpad_v_kmh = [50.88 48.09 45.59]; F.target.ay_g = [1.48 1.56 1.65];
% --- 엔진·구동 [F p.17, 표17·27] ---
F.eng.name = 'KTM Duke 390 (2016) 373.2 cc'; F.eng.torque_Nm = 37; F.eng.torque_rpm = 7000;
F.eng.power_ps = 44.6; F.eng.power_rpm = 9000; F.eng.rev_limit = 10500; F.eng.mass_kg = 37.65;
F.drv.final = 4.0;
F.drv.table27_overall_ratio_g1 = 10.34;  % 표27 기어별 차속에서 역산한 1차×종감속 (공개 80/30×4 = 10.67과 3% 차이)
% --- 공력 [F p.74~76] ---
F.aero.v_ref_kmh = 80; F.aero.downforce_N = 1183.477; F.aero.drag_N = 759.447;
F.aero.ClA_m2 = 3.91; F.aero.CdA_m2 = 2.51;                    % 유도값
F.aero.front_wing_N = 245.15; F.aero.side_N = 241.519; F.aero.rear_wing_N = 488.731;   % @80 km/h
F.aero.drag_front_N = 86.667; F.aero.drag_side_N = 169.089; F.aero.drag_rear_N = 182.234;
% 전/후 다운포스 배분 추정: 프런트윙은 앞축, 리어윙은 뒷축, 사이드는 반반 → 앞 약 31%, 뒤 약 69% (가정)
F.aero.down_front_frac_est = (245.15 + 241.519/2) / 1183.477;

D.formula = F;   % 공개본: 같은 동아리 EV팀 설계값 블록은 뺐다(코드에서 쓰지 않음)
end
