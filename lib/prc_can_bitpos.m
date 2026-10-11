function bitpos = prc_can_bitpos(sig)
% PRC_CAN_BITPOS  신호가 차지하는 비트 위치(0-based, 바이트*8+비트), LSB→MSB 순.
%   Intel(@1): start = LSB 위치.  Motorola(@0): start = MSB 위치(DBC 톱니 번호).
if sig.intel
  bitpos = sig.start + (0:sig.len-1);
else
  p = zeros(1, sig.len);
  p(1) = sig.start;
  for k = 2:sig.len
    if mod(p(k-1), 8) == 0
      p(k) = p(k-1) + 15;   % 다음 바이트의 MSB로
    else
      p(k) = p(k-1) - 1;
    end
  end
  bitpos = fliplr(p);
end
if any(bitpos < 0 | bitpos > 63)
  error('prc_can_bitpos:range', '%s: 비트 위치가 0..63 범위를 벗어남', sig.name);
end
end
