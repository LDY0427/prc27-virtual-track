function te = prc_wss_edges(t, rev, teeth)
% PRC_WSS_EDGES  누적 바퀴 회전수(rev) 시계열 → 톤휠 에지 시각 (가상 센서 입력).
%   한 시간 간격에 에지가 여러 개 지나가도 선형 보간으로 모두 만든다.
%   회전은 정방향(단조 증가)만 가정한다.
t = t(:); lv = rev(:) * teeth;
k = floor(lv);
dk = diff(k);
idx = find(dk > 0);
if isempty(idx), te = zeros(0, 1); return; end
n = dk(idx);
irep = repelem(idx, n);
offs = (1:sum(n))' - repelem(cumsum(n) - n, n);    % 각 구간 안에서 1..n
lev = repelem(k(idx), n) + offs;                    % 넘은 정수 레벨
l0 = lv(irep); l1 = lv(irep + 1);
te = t(irep) + (lev - l0) ./ (l1 - l0) .* (t(irep + 1) - t(irep));
end
