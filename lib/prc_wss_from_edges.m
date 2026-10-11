function kmh = prc_wss_from_edges(te, tq, teeth, r_m, timeout_s)
% PRC_WSS_FROM_EDGES  에지 시각 → 각 질의 시각의 휠 주변속도 (펌웨어 계산 방식의 기준 모델).
%
%   1) 최근 두 에지 간격으로 주파수 계산 (입력캡처 주기 측정)
%   2) 마지막 에지 이후 경과시간이 그 간격보다 길면 경과시간으로 상한 → 감속을 빨리 반영
%   3) timeout_s 동안 에지가 없으면 0 km/h
% 펌웨어 구현을 이 함수 결과와 같은 입력으로 비교하면 된다.
te = sort(te(:)); tq = tq(:);
kmh = zeros(size(tq));
if numel(te) < 2, return; end
j = prc_count_le(te, tq);
ok = j >= 2;
jj = j(ok);
per = te(jj) - te(jj - 1);
age = tq(ok) - te(jj);
per = max(per, age);
v = prc_wss_freq2kmh(1 ./ per, teeth, r_m);
v(age > timeout_s) = 0;
kmh(ok) = v;
end
