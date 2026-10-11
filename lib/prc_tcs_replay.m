function T = prc_tcs_replay(slip, ok, tps, sw, can_ok, P)
% PRC_TCS_REPLAY  기록된 시계열에 TCS 로직을 다시 돌린다 (개입 없는 판정 검증용).
%   모든 입력은 같은 길이의 열벡터(스칼라면 전 구간 동일). 샘플 간격은 P.tcs.Ts로 본다.
N = numel(slip);
ex = @(x) repmat(x, N / numel(x), 1);
if numel(ok) == 1, ok = ex(ok); end
if numel(tps) == 1, tps = ex(tps); end
if numel(sw) == 1, sw = ex(sw); end
if numel(can_ok) == 1, can_ok = ex(can_ok); end
T.cut = zeros(N, 1); T.mode = zeros(N, 1); T.reason = zeros(N, 1);
st = prc_tcs_init();
for i = 1:N
  in = struct('slip', slip(i), 'ok', ok(i), 'tps', tps(i), 'sw', sw(i), 'can_ok', can_ok(i));
  [st, o] = prc_tcs_step(st, in, P);
  T.cut(i) = o.cut; T.mode(i) = o.mode; T.reason(i) = o.reason;
end
end
