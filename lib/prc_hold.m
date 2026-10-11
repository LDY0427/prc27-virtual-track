function y = prc_hold(t_src, x_src, t_q)
% PRC_HOLD  샘플-홀드: 각 t_q에서 그 시각 이전 마지막 x_src 값. 첫 샘플 전은 NaN.
%   느린 신호를 보간하지 않고 직전값을 쓰는 로그 변환 원칙(DAQ 설계 15.2)과 같다.
j = prc_count_le(t_src, t_q);
y = nan(numel(t_q), 1);
ok = j > 0;
y(ok) = x_src(j(ok));
end
