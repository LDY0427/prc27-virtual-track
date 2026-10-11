function X = seg_kin(v0, v1, a_abs, P, o)
% SEG_KIN  v0 → v1 (m/s)을 일정 가감속 a_abs(m/s^2)로 가는 운동학 구간. 직진, 슬립 없음.
%   o.tps (가속 시 기본 0.6), o.pF (감속 시 기본 a에 비례), o.rpm_max (변속 기준 9000)
if nargin < 5, o = struct(); end
o = prc_opts(o, struct('tps', 0.6, 'pF', [], 'rpm_max', 9000));
dt = 1e-3;
sgn = sign(v1 - v0);
n = max(1, ceil(abs(v1 - v0) / a_abs / dt));
v = v0 + (v1 - v0) * (1:n)' / n;
X = seg_blank(n);
X.v = v; X.w = repmat(v, 1, 4); X.ax(:) = sgn * a_abs;
[X.gear, X.rpm] = seg_gear_rpm(v, P, o.rpm_max);
if sgn > 0
  X.tps(:) = o.tps;
else
  if isempty(o.pF), o.pF = 30 * a_abs / P.veh.g; end
  X.pF(:) = o.pF; X.pR(:) = 0.6 * o.pF;
end
end
