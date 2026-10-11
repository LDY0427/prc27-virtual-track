function [gear, rpm] = seg_gear_rpm(v, P, rpm_max)
% SEG_GEAR_RPM  차속에 맞는 단·회전수 (rpm_max를 넘지 않는 가장 낮은 단). 저속은 클러치 분리(아이들).
if nargin < 3, rpm_max = 9000; end
ratios = P.drv.primary * P.drv.gears * P.drv.final;
gear = zeros(size(v)); rpm = P.eng.idle_rpm * ones(size(v));
for i = 1:numel(v)
  rr = v(i) / P.tire.r_m * ratios * 60 / (2*pi);
  k = find(rr <= rpm_max, 1);
  if isempty(k), k = numel(ratios); end
  if v(i) > 3/3.6 && rr(k) >= P.eng.idle_rpm
    gear(i) = k; rpm(i) = rr(k);
  elseif v(i) > 3/3.6
    gear(i) = 1;
  end
end
end
