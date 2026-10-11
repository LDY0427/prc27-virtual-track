function B = prc_cell_bin(x, y, v, xbp, ybp, mask)
% PRC_CELL_BIN  (x, y) 샘플을 가장 가까운 표 칸(xbp × ybp)으로 모아 칸별 개수·평균·표준편차.
%   B.n, B.mean, B.std 는 numel(ybp) × numel(xbp) (행 = y 축, 열 = x 축. ECU 표와 같은 배치)
if nargin < 6, mask = true(size(x)); end
x = x(:); y = y(:); v = v(:); mask = mask(:) & ~isnan(x + y + v);
[~, ix] = min(abs(repmat(x, 1, numel(xbp)) - repmat(xbp(:)', numel(x), 1)), [], 2);
[~, iy] = min(abs(repmat(y, 1, numel(ybp)) - repmat(ybp(:)', numel(y), 1)), [], 2);
B.n = zeros(numel(ybp), numel(xbp)); B.mean = nan(size(B.n)); B.std = nan(size(B.n));
for a = 1:numel(ybp)
  for b = 1:numel(xbp)
    s = mask & iy == a & ix == b;
    B.n(a, b) = sum(s);
    if B.n(a, b) > 0, B.mean(a, b) = mean(v(s)); B.std(a, b) = std(v(s)); end
  end
end
end
