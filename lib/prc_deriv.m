function d = prc_deriv(x, dt, k)
% PRC_DERIV  등간격 신호의 미분 (앞뒤 k개 점에 직선 맞춤 = Savitzky-Golay 1차).
%   창 길이 (2k+1)·dt. 휠속도 양자화 잡음을 줄이려면 0.2 s 안팎이 적당하다. 양 끝 k개는 NaN.
x = x(:); n = numel(x);
d = zeros(n, 1);
for j = -k:k
  idx = min(max((1:n)' + j, 1), n);
  d = d + j * x(idx);
end
d = d / (dt * sum((-k:k).^2));
d([1:min(k, n), max(n-k+1, 1):n]) = NaN;
end
