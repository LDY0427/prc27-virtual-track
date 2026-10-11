function [u, dstate] = vt_driver(dstate, st, R, Q, P, dt)
% VT_DRIVER  경로 추종 + 속도 추종 운전자 (CarMaker IPGDriver의 역할, 단순화).
%
%   조향: 순수 추종(pure pursuit). 전방 주시 거리 Ld = max(Ld_min, k·v)에서 목표점을 잡고
%         필요한 곡률 → 평균 바퀴각 → 애커만 LUT 역함수로 핸들각. 핸들 속도 제한.
%   속도: laptime_qss가 만든 목표 속도 v_ref(s)를 조금 앞(0.3 s)에서 읽어 PI 제어 → 스로틀/제동.
%   dstate: 운전자 내부 상태 (s 위치, 적분항, 이전 핸들각). vt_driver_init으로 만든다.
Dp = P.driver;
% ---------- 경로 상 위치 ----------
[dstate.s, idx, e_lat] = nearest_s(R, st.x, st.y, dstate.idx);
dstate.idx = idx;
v = max(st.vx, 1);
Ld = max(Dp.Ld_min_m, Dp.Ld_gain_s * v);
s_t = dstate.s + Ld;
if R.closed, s_t = mod(s_t, R.length_m); else, s_t = min(s_t, R.length_m); end
xt = interp1(R.s, R.x, s_t); yt = interp1(R.s, R.y, s_t);
dx = xt - st.x; dy = yt - st.y;
% 차체 좌표로
lx = cos(st.psi)*dx + sin(st.psi)*dy; ly = -sin(st.psi)*dx + cos(st.psi)*dy;
curv = 2 * ly / max(lx^2 + ly^2, 1e-6);
% 저속·고속 보정: 운동학 곡률 → 바퀴각, 언더스티어 보정은 K_us 가정으로 (단순)
% 횡편차(왼쪽 +)·방위 오차 피드백을 더한다. 왼쪽으로 벗어났으면(e_lat>0) 오른쪽으로.
psi_err = wrap_pi(R.psi(idx) - st.psi);
dAvg = atan(P.veh.wheelbase_m * curv) * 180/pi + Dp.Kus_ff_deg_g * (v^2 * curv / P.veh.g) ...
       - Dp.k_elat_deg_m * e_lat + Dp.k_psi * psi_err * 180/pi;
sw_cmd = prc_steer_inv(P, dAvg);
sw_cmd = min(max(sw_cmd, -P.steer.sw_max_deg), P.steer.sw_max_deg);
% 1차 지연(팔 반응) + 핸들 속도 제한
dsw = (sw_cmd - dstate.sw) * min(dt / Dp.sw_tau_s, 1);
dsw = min(max(dsw, -Dp.sw_rate_dps * dt), Dp.sw_rate_dps * dt);
dstate.sw = dstate.sw + dsw;
% ---------- 속도 ----------
s_v = dstate.s + Dp.preview_s * v;
if R.closed, s_v = mod(s_v, R.length_m); else, s_v = min(s_v, R.length_m); end
v_ref = interp1(Q.s(1:numel(R.s)), Q.v_ms(1:numel(R.s)), s_v) * Dp.v_scale;
e = v_ref - st.vx;
dstate.ie = min(max(dstate.ie + e * dt, -3), 3);
cmd = Dp.kp * e + Dp.ki * dstate.ie;
tps = min(max(cmd, 0), 1);
pF = min(max(-cmd, 0), 1) * Dp.pF_max_bar;
if st.vx < 0.5 && v_ref < 0.5, tps = 0; pF = 5; end
u = struct('sw_deg', dstate.sw, 'tps', tps, 'pF_bar', pF, 'tcs_cut', 0);
dstate.e_lat = e_lat; dstate.v_ref = v_ref;
end

function a = wrap_pi(a)
a = mod(a + pi, 2*pi) - pi;
end

function [s, idx, e_lat] = nearest_s(R, x, y, idx0)
n = numel(R.s);
win = max(idx0 - 20, 1):min(idx0 + 200, n);
if R.closed && idx0 + 200 > n, win = [win, 1:min(idx0 + 200 - n, n)]; end
d2 = (R.x(win) - x).^2 + (R.y(win) - y).^2;
[~, k] = min(d2); idx = win(k);
s = R.s(idx);
% 좌측 +인 횡편차
e_lat = -sin(R.psi(idx)) * (x - R.x(idx)) + cos(R.psi(idx)) * (y - R.y(idx));
end
