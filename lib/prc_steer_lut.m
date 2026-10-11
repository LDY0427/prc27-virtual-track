function [dL, dR, dAvg] = prc_steer_lut(P, sw_deg)
% PRC_STEER_LUT  핸들각(deg, 좌회전 +) → 좌·우 바퀴 조향각(deg, 좌회전 +)과 평균각.
%   애커만 표(P.steer.lut: 핸들각 | 외륜각 | 내륜각)를 선형 보간한다. 좌회전이면 왼쪽이 내륜.
%   표 범위를 넘으면 끝값으로 고정(기계적 조향 제한).
T = P.steer.lut;
a = min(abs(sw_deg(:)), T(end, 1));
outer = interp1(T(:, 1), T(:, 2), a);
inner = interp1(T(:, 1), T(:, 3), a);
s = sign(sw_deg(:)); s(s == 0) = 1;
left = inner; right = outer;                 % 좌회전 기준
left(s < 0) = outer(s < 0); right(s < 0) = inner(s < 0);
dL = s .* left; dR = s .* right;
dAvg = s .* (inner + outer) / 2;
dL = reshape(dL, size(sw_deg)); dR = reshape(dR, size(sw_deg)); dAvg = reshape(dAvg, size(sw_deg));
end
