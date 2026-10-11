function R = prc_vehicle_ref(w, valid_in, P, yaw_dps)
% PRC_VEHICLE_REF  4륜 휠속도 → 기준 차속(비구동 전륜), 구동륜 속도, 슬립, 유효성.
%
%   R = prc_vehicle_ref(w, valid_in, P)
%   R = prc_vehicle_ref(w, valid_in, P, yaw_dps)   % 요레이트(deg/s, 좌회전 +)로 선회 보정
%   w        : N×4 km/h [FL FR RL RR]
%   valid_in : N×4 센서 자체 유효 플래그 (모듈 자가진단)
%   R.valid  : 좌우 비교까지 거친 최종 유효
%   R.v_ref, R.v_drv (km/h, 쓸 바퀴가 없으면 NaN), R.slip, R.ref_ok, R.drv_ok
%
% 후륜구동 가정. 같은 축 좌우가 크게 다르면 '낮게 읽히는 쪽'을 무효로 본다
% (단선·톤휠 누락은 0에 가깝게 읽히기 때문). 코너 안팎 차이(약 10%)는 허용폭 안이다.
% 구동륜이 함께 헛도는 것은 좌우 비교로 잡히지 않으므로 고장으로 오인하지 않는다.
%
% 선회 보정: 좌회전하면 왼쪽 바퀴가 요레이트×윤거/2 만큼 느리고 오른쪽이 빠르다.
% 각 바퀴 속도를 차체 중심선 속도로 환산한 뒤 평균하므로, 한쪽 바퀴가 무효가 되어도
% 기준 차속이 튀지 않는다. (전륜 조향에 따른 경로 차이는 무시. 보정값 R.w_corr)
if nargin < 2 || isempty(valid_in), valid_in = true(size(w)); end
if nargin >= 4 && ~isempty(yaw_dps)
  yr = yaw_dps(:) * pi/180 * 3.6;                      % km/h per m
  yr(isnan(yr)) = 0;
  w = w + [yr*P.veh.track_f_m/2, -yr*P.veh.track_f_m/2, yr*P.veh.track_r_m/2, -yr*P.veh.track_r_m/2];
end
R.w_corr = w;
valid = logical(valid_in) & ~isnan(w);
W = P.wss;
pairs = [1 2; 3 4];
for p = 1:2
  a = pairs(p, 1); b = pairs(p, 2);
  hi = max(w(:, a), w(:, b));
  d = abs(w(:, a) - w(:, b));
  bad = d > max(W.pair_tol_kmh, W.pair_tol_rel * hi) & hi > W.pair_check_min_kmh;
  valid(bad & w(:, a) < w(:, b), a) = false;
  valid(bad & w(:, b) < w(:, a), b) = false;
end
R.valid = valid;
R.v_ref = masked_mean(w(:, 1:2), valid(:, 1:2));
R.v_drv = masked_mean(w(:, 3:4), valid(:, 3:4));
R.ref_ok = ~isnan(R.v_ref);
R.drv_ok = ~isnan(R.v_drv);
R.slip = (R.v_drv - R.v_ref) ./ max(R.v_ref, P.tcs.v_floor_kmh);
end

function m = masked_mean(x, ok)
x(~ok) = 0;
n = sum(ok, 2);
m = sum(x, 2) ./ n;
m(n == 0) = NaN;
end
