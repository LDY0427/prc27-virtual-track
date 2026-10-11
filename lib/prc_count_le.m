function j = prc_count_le(te, tq)
% PRC_COUNT_LE  정렬된 te에서 각 tq 이하인 원소 개수 (= 마지막 이전 이벤트 인덱스, 없으면 0).
te = te(:); tq = tq(:);
ne = numel(te); nq = numel(tq);
[~, ord] = sort([te; tq]);               % 안정 정렬: 같은 시각이면 te가 먼저
isE = [true(ne, 1); false(nq, 1)];
c = cumsum(isE(ord));
pos = zeros(ne + nq, 1);
pos(ord) = 1:(ne + nq);
j = c(pos(ne+1:end));
end
