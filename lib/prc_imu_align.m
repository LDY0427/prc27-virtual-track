function cal = prc_imu_align(S, P)
% PRC_IMU_ALIGN  로그만으로 IMU 장착 각도와 자이로 영점을 추정한다.
%
%   cal = prc_imu_align(S, P)       S = prc_log_decode 결과
%   cal.rpy_deg  : 장착 [롤 피치 요] deg  (f_body = cal.Rm * f_sensor)
%   cal.gyro_bias: 자이로 영점 deg/s (센서 축)
%
% 1) 정지 구간(휠속도 0, 자이로 작음): 중력 방향 → 롤·피치, 자이로 평균 → 영점
% 2) 직진 가감속 구간(핸들 거의 0, 요레이트 작음, |가속| > 0.15 g):
%    수평으로 편 가속도의 좌우 성분이 0이 되도록 요 각도
% 주의: 정지 구간 바닥이 기울어 있으면 그 기울기가 롤·피치에 섞인다.
%       평평한 바닥에서, 또는 차를 180° 돌려 두 번 재서 평균한다.
A = S.DAQ_IMU_Acc; G = S.DAQ_IMU_Gyro;
ti = A.t;
acc = [A.AccX A.AccY A.AccZ];                         % g
gyr = [prc_hold(G.t, G.GyroX, ti) prc_hold(G.t, G.GyroY, ti) prc_hold(G.t, G.GyroZ, ti)];
W = S.WSS_Speed;
wmax = prc_hold(W.t, max([W.WssFL W.WssFR W.WssRL W.WssRR], [], 2), ti);
static = wmax < 0.5 & sqrt(sum(gyr.^2, 2)) < 5 & abs(sqrt(sum(acc.^2, 2)) - 1) < 0.05 ...
         & all(~isnan([acc gyr]), 2);
% 출발 직후·정지 직전에는 휠속도가 아직 0으로 읽히는데 차는 가감속 중이다.
% 1 s 이상 이어진 정지만 쓰고 앞뒤 0.5 s를 버린다.
rs = prc_runs(static, ti, 1.0);
static(:) = false;
for k = 1:size(rs, 1)
  sel = ti >= ti(rs(k,1)) + 0.5 & ti <= ti(rs(k,2)) - 0.5;
  static(sel) = true;
end
cal.n_static = sum(static);
if cal.n_static < 50
  warning('prc_imu_align:static', '정지 구간이 부족해 장착 보정을 하지 않음 (%d 샘플)', cal.n_static);
  cal.rpy_deg = [0 0 0]; cal.Rm = eye(3); cal.gyro_bias = [0 0 0]; cal.n_straight = 0; cal.ok = false;
  return;
end
gm = mean(acc(static, :), 1);
roll = atan2(gm(2), gm(3));
pitch = atan2(-gm(1), sqrt(gm(2)^2 + gm(3)^2));
cal.gyro_bias = mean(gyr(static, :), 1);
Rlev = prc_rotm([roll pitch 0] * 180/pi);              % Ry*Rx
fl = (Rlev * acc')';
gz = gyr(:, 3) - cal.gyro_bias(3);
swa = prc_hold(S.DAQ_Chassis.t, S.DAQ_Chassis.SteerAngle, ti);
straight = abs(swa) < 5 & abs(gz) < 3 & wmax > 5 & abs(fl(:, 1)) > 0.15 & ~isnan(fl(:, 1));
cal.n_straight = sum(straight);
if cal.n_straight >= 20
  yaw = atan2(-sum(fl(straight, 2) .* fl(straight, 1)), sum(fl(straight, 1).^2));
else
  warning('prc_imu_align:straight', '직진 가감속 구간이 부족해 요 장착각은 0으로 둠');
  yaw = 0;
end
cal.rpy_deg = [roll pitch yaw] * 180/pi;
cal.Rm = prc_rotm(cal.rpy_deg);
cal.ok = true;
end
