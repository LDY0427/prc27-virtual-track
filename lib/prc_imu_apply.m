function [f_b, w_b] = prc_imu_apply(cal, acc, gyr)
% PRC_IMU_APPLY  센서 축 측정값(N×3)을 차체 축으로 돌린다. 자이로는 영점을 먼저 뺀다.
%   차체 축: x 앞, y 왼쪽, z 위.  가속도계 값은 '중력 포함 비력'이라 경사·롤 성분이 섞여 있다.
f_b = (cal.Rm * acc')';
if nargin >= 3 && ~isempty(gyr)
  w_b = (cal.Rm * (gyr - repmat(cal.gyro_bias, size(gyr, 1), 1))')';
else
  w_b = [];
end
end
