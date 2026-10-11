function R = prc_runs(mask, t, min_dur)
% PRC_RUNS  논리 마스크에서 연속 참 구간 [시작 인덱스, 끝 인덱스] 목록 (min_dur 초 이상만).
mask = logical(mask(:));
e = diff([0; mask; 0]);
s0 = find(e == 1); s1 = find(e == -1) - 1;
if nargin >= 3 && ~isempty(s0)
  keep = (t(s1) - t(s0)) >= min_dur;
  s0 = s0(keep); s1 = s1(keep);
end
R = [s0 s1];
end
