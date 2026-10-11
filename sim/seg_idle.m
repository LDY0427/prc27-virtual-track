function X = seg_idle(T, P)
% SEG_IDLE  정차 아이들 T 초 (dt 1e-3).
n = round(T / 1e-3);
X = seg_blank(n);
X.rpm = P.eng.idle_rpm + 100 * filter(0.05, [1 -0.95], randn(n, 1));
end
