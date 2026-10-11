function st = vt_vehicle_init(P, v0, x0, y0, psi0)
% VT_VEHICLE_INIT  2트랙 차량 모델 초기 상태 (속도 v0 m/s, 위치·방위).
if nargin < 2, v0 = 0; end
if nargin < 3, x0 = 0; y0 = 0; psi0 = 0; end
st.x = x0; st.y = y0; st.psi = psi0;
st.vx = v0; st.vy = 0; st.r = 0; st.ax = 0; st.ay = 0;
st.om = v0 / P.tire.r_m * ones(1, 4);
st.rev = zeros(1, 4);
ratios = P.drv.primary * P.drv.gears * P.drv.final;
rr = v0 / P.tire.r_m * ratios * 60/(2*pi);
g = find(rr <= 9000, 1); if isempty(g), g = numel(ratios); end
st.gear = max(g, 1);
st.rpm = max(rr(st.gear), P.eng.idle_rpm);
st.locked = v0 > 1;
st.shifting = false; st.t_shift = 0;
end
