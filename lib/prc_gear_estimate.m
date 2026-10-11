function [gear, ratio_meas] = prc_gear_estimate(rpm, v_drive_kmh, P)
% PRC_GEAR_ESTIMATE  엔진 RPM ÷ 구동륜 회전수 → 단 추정. 판정 불가면 0.
%   클러치 분리·변속 중·헛돌기·저속에서는 비율이 어느 단과도 맞지 않아 0이 된다.
%   추정 기어로 다시 차속을 만드는 순환 계산에는 쓰지 않는다.
ratios = P.drv.primary * P.drv.gears * P.drv.final;
w_rpm = v_drive_kmh / 3.6 / (2*pi*P.tire.r_m) * 60;
ratio_meas = rpm ./ w_rpm;
gear = zeros(size(rpm));
for g = 1:numel(ratios)
  hit = abs(ratio_meas / ratios(g) - 1) < P.drv.gear_tol & gear == 0;
  gear(hit) = g;
end
gear(~(v_drive_kmh >= P.drv.gear_min_kmh)) = 0;   % NaN 포함
end
