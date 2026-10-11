function v = prc_brake_ref(w_max, ax, dt)
% PRC_BRAKE_REF  제동 중 차속 추정 (바퀴가 잠겨도 유지).
%   w_max : 네 바퀴 중 가장 빠른 바퀴 속도 m/s,  ax : 차체 종가속도 m/s^2 (IMU, 정렬 후)
%
%   제동 중 바퀴는 차보다 빠를 수 없으므로 가장 빠른 바퀴가 하한이다.
%   그리고 차속은 IMU가 잰 감속보다 빨리 줄 수 없다(여유 0.05 g).
%     v(k) = max( w_max(k),  v(k-1) + (ax(k) − 0.05 g)·dt )
%   바퀴가 잠기기 시작해 급히 느려져도 기준 차속은 IMU 감속률로만 내려간다.
%   차가 실제로 감속 중(ax < −0.5 m/s^2)이 아니면 바퀴 값을 그대로 쓴다(정지 후 0으로 복귀).
n = numel(w_max); v = zeros(n, 1);
v(1) = w_max(1);
for k = 2:n
  if ax(k) < -0.5 && ~isnan(ax(k))
    v(k) = max(w_max(k), v(k-1) + (ax(k) - 0.05*9.81) * dt);
  else
    v(k) = w_max(k);
  end
  v(k) = max(v(k), 0);
end
end
