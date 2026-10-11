function sw = prc_steer_inv(P, dAvg_deg)
% PRC_STEER_INV  원하는 평균 바퀴 조향각(deg) → 핸들각(deg). prc_steer_lut의 역함수(평균각 기준).
T = P.steer.lut;
avg = (T(:, 2) + T(:, 3)) / 2;
a = min(abs(dAvg_deg(:)), avg(end));
sw = sign(dAvg_deg(:)) .* interp1(avg, T(:, 1), a);
sw(dAvg_deg(:) == 0) = 0;
sw = reshape(sw, size(dAvg_deg));
end
