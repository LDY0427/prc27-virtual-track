function [kmh, rpm] = prc_wss_freq2kmh(f_hz, teeth, r_m)
% PRC_WSS_FREQ2KMH  휠속도 센서 펄스 주파수 → 휠 RPM, 주변속도 km/h.
%   휠 RPM = 60 × f / 회전당 펄스 수
%   km/h   = 휠 RPM × 굴림둘레(m) × 60 / 1000
% (01_설계/휠속도_기어비_차속계획_GPT.md 식 1. 기어비는 필요 없다.)
rpm = 60 * f_hz ./ teeth;
kmh = rpm * (2*pi*r_m) * 60 / 1000;
end
