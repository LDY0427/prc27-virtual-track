function data = prc_can_encode(msg, vals, N)
% PRC_CAN_ENCODE  신호 물리값 → N×8 페이로드.
%
%   data = prc_can_encode(msg, vals, N)
%   vals.(신호이름) : N×1 또는 스칼라. 없는 신호는 0.
%   NaN은 무효 코드로 보낸다: 부호 없음 = 전부 1 (예 0xFFF), 부호 있음 = 최솟값 (예 0x8000).
%   범위를 넘는 값은 원시값 범위로 자른다.
bits = zeros(N, 64);
for s = 1:numel(msg.sigs)
  sg = msg.sigs(s);
  if isfield(vals, sg.name), v = double(vals.(sg.name)(:)); else, v = 0; end
  if numel(v) == 1, v = repmat(v, N, 1); end
  if numel(v) ~= N
    error('prc_can_encode:size', '%s.%s: 길이 %d, 기대 %d', msg.name, sg.name, numel(v), N);
  end
  nanm = isnan(v);
  r = round((v - sg.offset) / sg.factor);
  if sg.signed
    lo = -2^(sg.len-1); hi = 2^(sg.len-1) - 1;
  else
    lo = 0; hi = 2^sg.len - 1;
  end
  r = min(max(r, lo), hi);
  if sg.signed, r(nanm) = lo; else, r(nanm) = hi; end
  r(r < 0) = r(r < 0) + 2^sg.len;          % 2의 보수
  pos = prc_can_bitpos(sg);
  for i = 1:sg.len
    bits(:, pos(i) + 1) = mod(floor(r / 2^(i-1)), 2);
  end
end
data = zeros(N, 8);
for b = 0:7
  data(:, b+1) = bits(:, b*8 + (1:8)) * (2.^(0:7))';
end
end
