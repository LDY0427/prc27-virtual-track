function st = prc_tcs_init()
% PRC_TCS_INIT  TCS 제어기 초기 상태.
st.I = 0;      % 적분항
st.cut = 0;    % 토크 감소율 명령 0..1
st.mode = 1;   % 0 OFF, 1 STANDBY, 2 ACTIVE, 3 FAULT
end
