function Te = prc_engine_torque(P, rpm, tps)
% PRC_ENGINE_TORQUE  엔진 토크 N·m. WOT 곡선 × 스로틀, 닫힘 쪽은 마찰 토크.
%   스로틀 비례는 단순 가정이다(부분부하 맵 아님). 리밋 이상은 연료/점화 차단.
E = P.eng;
rr = min(max(rpm, E.rpm_tab(1)), E.rpm_tab(end));
Twot = interp1(E.rpm_tab, E.trq_tab, rr);
Tfric = E.fric_a + E.fric_b * rpm / 1000;
Te = tps .* Twot - (1 - tps) .* Tfric;
over = rpm >= E.rpm_limit;
Te(over) = -Tfric(over);
end
