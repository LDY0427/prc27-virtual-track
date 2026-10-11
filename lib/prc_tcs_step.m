function [st, out] = prc_tcs_step(st, in, P)
% PRC_TCS_STEP  휠스피드 TCS 한 주기 계산 (P.tcs.Ts 마다 호출).
%
%   in.slip   : 구동륜 슬립 (prc_vehicle_ref)
%   in.ok     : 휠속도 유효 (기준·구동 모두)
%   in.tps    : 스로틀 0..1
%   in.sw     : 운전자 TCS 스위치 (true = 켜짐)
%   in.can_ok : ECU/휠속도 CAN 수신 정상
%   out.cut   : 엔진 토크 감소율 요청 0..max_cut
%   out.mode  : 0 OFF, 1 STANDBY, 2 ACTIVE, 3 FAULT
%   out.reason: 0 없음, 1 스위치 꺼짐, 2 휠속도 무효, 3 CAN 끊김, 4 스로틀 낮음
%
% PI 제어 + 조건부 적분(포화 시 적분 정지) + 증가/복귀 속도 제한.
% 금지 조건이 생기면 적분을 비우고 감소율을 rate_down 속도로 0까지 되돌린다
% (갑작스러운 토크 복귀 방지). 실제 ECU 개입 방식은 미정이다.
C = P.tcs;
reason = 0;
if ~in.sw
  reason = 1;
elseif ~in.can_ok
  reason = 3;
elseif ~in.ok || isnan(in.slip)
  reason = 2;
elseif in.tps < C.tps_min
  reason = 4;
end
if reason == 0
  e = in.slip - C.target_slip;
  I_try = st.I + C.ki * e * C.Ts;
  u = C.kp * e + I_try;
  if (u >= C.max_cut && e > 0) || (u <= 0 && e < 0)
    I_try = st.I;                         % 포화 방향이면 적분 정지
  end
  st.I = min(max(I_try, 0), C.max_cut);
  target = min(max(C.kp * e + st.I, 0), C.max_cut);
else
  st.I = 0;
  target = 0;
end
step_up = C.rate_up * C.Ts; step_dn = C.rate_down * C.Ts;
st.cut = st.cut + min(max(target - st.cut, -step_dn), step_up);
switch reason
  case 0
    if st.cut > 1e-3, st.mode = 2; else, st.mode = 1; end
  case 1
    st.mode = 0;
  case {2, 3}
    st.mode = 3;
  otherwise
    st.mode = 1;
end
out.cut = st.cut; out.mode = st.mode; out.reason = reason;
end
