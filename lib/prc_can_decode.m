function [phys, raw] = prc_can_decode(msg, data)
% PRC_CAN_DECODE  N×8 페이로드 → 신호별 물리값 (N×1).
%
%   [phys, raw] = prc_can_decode(msg, data)
%   phys.(신호이름) : 물리값. DBC [min|max] 범위 밖(무효 코드 포함)은 NaN
%   raw.(신호이름)  : 원시 정수값
data = double(data);
N = size(data, 1);
if size(data, 2) < 8, data(:, end+1:8) = 0; end
bits = zeros(N, 64);
for b = 0:7
  for k = 0:7
    bits(:, b*8 + k + 1) = mod(floor(data(:, b+1) / 2^k), 2);
  end
end
phys = struct(); raw = struct();
for s = 1:numel(msg.sigs)
  sg = msg.sigs(s);
  pos = prc_can_bitpos(sg);
  r = bits(:, pos + 1) * (2.^(0:sg.len-1))';
  if sg.signed
    neg = r >= 2^(sg.len-1);
    r(neg) = r(neg) - 2^sg.len;
  end
  v = r * sg.factor + sg.offset;
  if sg.max > sg.min
    tol = abs(sg.factor) * 1e-6;
    v(v < sg.min - tol | v > sg.max + tol) = NaN;
  end
  phys.(sg.name) = v;
  raw.(sg.name) = r;
end
end
