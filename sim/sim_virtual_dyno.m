function D = sim_virtual_dyno(P)
% SIM_VIRTUAL_DYNO  정상상태 가상 다이노: 엔진 토크·출력표, 단별 구동력-차속 선도, 변속점 후보.
%
%   D = sim_virtual_dyno(P)
%   D.rpm, D.Te, D.kW                 : 엔진 표 (P.eng 토크 곡선 기준)
%   D.v_kmh(:,g), D.F_N(:,g)          : g단 차속과 바퀴 구동력
%   D.F_trac_N                        : 후륜 접지 한계(정상 가속, 하중 이동 포함, 다운포스 제외)
%   D.F_res_N(v)                      : 공기저항+구름저항 (D.v_res_kmh 축)
%   D.shift_rpm(g)                    : g→g+1 변속 추천 회전수 (같은 차속에서 다음 단 구동력이 더 커지는 점)
%   D.v_max_gear_kmh(g)               : g단 리밋 회전수 차속
%
% 변속 시간 손실·과도 응답은 반영하지 않은 정상상태 비교다. 실측 토크곡선으로 바꾸면 그대로 다시 쓴다.
E = P.eng; Dr = P.drv; V = P.veh; T = P.tire;
ratios = Dr.primary * Dr.gears * Dr.final;
ng = numel(ratios);
rpm = (E.rpm_tab(1):50:E.rpm_limit)';
Te = prc_engine_torque(P, rpm - 1e-9, ones(size(rpm)));   % 리밋 직전 값
kW = Te .* rpm * 2*pi/60 / 1000;
D.rpm = rpm; D.Te = Te; D.kW = kW;
D.ratios = ratios;
D.v_kmh = zeros(numel(rpm), ng); D.F_N = zeros(numel(rpm), ng);
for g = 1:ng
  D.v_kmh(:, g) = rpm / ratios(g) * 2*pi*T.r_m / 60 * 3.6;
  D.F_N(:, g) = Te * ratios(g) * Dr.eta / T.r_m;
end
D.v_max_gear_kmh = D.v_kmh(end, :);
% 접지 한계: Fx = mu*(Fz_s + m a h/L), a = Fx/m  →  Fx = mu*Fz_s / (1 - mu*h/L)
Fz_s = V.mass_kg * V.g * V.rear_frac;
D.F_trac_N = T.mu * Fz_s / (1 - T.mu * V.cg_h_m / V.wheelbase_m);
D.v_res_kmh = (0:1:max(D.v_max_gear_kmh))';
vv = D.v_res_kmh / 3.6;
D.F_res_N = 0.5 * V.rho * V.CdA_m2 * vv.^2 + V.Crr * V.mass_kg * V.g;
D.F_down_N = 0.5 * V.rho * V.ClA_m2 * vv.^2;
% 변속점: g단 곡선 위에서, 같은 차속의 g+1단 구동력이 g단보다 커지는 첫 회전수
D.shift_rpm = nan(1, ng - 1);
for g = 1:ng-1
  Fn = interp1(D.v_kmh(:, g+1), D.F_N(:, g+1), D.v_kmh(:, g));   % 범위 밖 NaN
  k = find(~isnan(Fn) & Fn > D.F_N(:, g), 1);
  if isempty(k), D.shift_rpm(g) = E.rpm_limit; else, D.shift_rpm(g) = rpm(k); end
end
% 단별 최고 구동력이 접지 한계를 넘는지
D.traction_limited = max(D.F_N, [], 1) > D.F_trac_N;
end
