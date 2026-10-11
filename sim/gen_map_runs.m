function [F, truth] = gen_map_runs(P, db, opt)
% GEN_MAP_RUNS  연료 매핑 가이드 검증용 더미 로그.
%
%   [F, truth] = gen_map_runs(P, db, struct('out_csv', 'out/dummy_map.csv'))
%
% 상황 가정: 부하 다이노(또는 긴 직선·오르막)에서 RPM과 TPS를 여러 조합으로 1.5 s씩 유지한다.
% 현재 연료맵에 '정답 오차'를 일부러 넣었다:
%   7000~9000 rpm, TPS 60% 이상  : 연료 8% 부족 (희박 → 위험 구간)
%   2000~3500 rpm, TPS 10~30%    : 연료 6% 과다 (농후)
%   나머지                       : ±1.5% 이내 잔오차
% 람다 센서 = 실제 람다 → 0.12 s 지연 + 0.05 s 1차 지연 + 잡음.
% 가속 순간에는 가속 증량 부족으로 짧은 희박 튐을 넣었다(정상 상태 필터가 걸러내야 함).
% 일부 칸은 일부러 주행하지 않았다(데이터 부족 표시 확인).
if nargin < 3, opt = struct(); end
opt = prc_opts(opt, struct('seed', 6, 'out_csv', '', 'hold_s', 1.5, 'gear', 3, 'ign_bump', [6000 30 5]));
rng(opt.seed);
dt = 1e-3; r = P.tire.r_m;
ratios = P.drv.primary * P.drv.gears * P.drv.final; ig = ratios(opt.gear);
rpm_list = 2000:500:10000;
tps_list = [5 10 20 30 40 60 80 100];
% 정답 연료 오차 (공급/필요 − 1)
% 구간 경계는 칸과 칸 사이에 둔다(칸 중심에 경계가 걸리면 정답 자체가 모호해진다).
err = @(rpm, tps) -0.08 * (rpm > 6750 & rpm < 9250 & tps > 50) ...
                  + 0.06 * (rpm > 1750 & rpm < 3750 & tps > 7.5 & tps < 35) ...
                  + 0.015 * sin(rpm/900) .* cos(tps/17);
seq = [];
for rr = rpm_list
  for tt = tps_list
    if rr <= 3000 && tt >= 80, continue; end          % 저회전 풀스로틀은 일부러 생략 (데이터 없음 표시 확인)
    if rr >= 9500 && tt <= 10, continue; end
    if rr == 5500 && tt == 40, continue; end          % 중간에 빈 칸 하나
    seq(end+1, :) = [rr tt]; %#ok<AGROW>
  end
end
seq = seq(randperm(size(seq, 1)), :);                 % 순서는 섞는다
parts = {seg_idle(5, P)};
prev = [P.eng.idle_rpm 0];
for k = 1:size(seq, 1)
  nT = round(0.3 / dt); nH = round(opt.hold_s / dt);
  rpm = [linspace(prev(1), seq(k,1), nT)'; seq(k,1) * ones(nH, 1)];
  tps = [linspace(prev(2), seq(k,2), nT)'; seq(k,2) * ones(nH, 1)];
  S = seg_blank(nT + nH);
  S.rpm = rpm + 15 * randn(size(rpm)); S.tps = tps / 100; S.gear(:) = opt.gear;
  S.v = rpm / ig / 60 * 2*pi*r; S.w = repmat(S.v, 1, 4); S.ax = [0; diff(S.v)] / dt;
  parts{end+1} = S; %#ok<AGROW>
  prev = seq(k, :);
end
parts{end+1} = seg_idle(3, P);
X = seg_cat(parts{:});
% 실제 람다 → 센서 람다
lam_t = prc_lambda_target(P, X.tps * 100);
e = err(X.rpm, X.tps * 100);
lam_true = lam_t ./ (1 + e);
dtps = [0; diff(X.tps * 100)] / dt;
tip = filter(1, [1 -exp(-dt/0.15)], max(dtps, 0) * dt) * 0.002;     % 가속 순간 희박 튐 (TPS 50%p 올리면 람다 +0.1)
lam_true = lam_true + tip;
lam_true(X.tps < 0.02) = 1.0;                                       % 아이들
nd = round(P.map.lambda_delay_s / dt);
lam_d = [lam_true(1) * ones(nd, 1); lam_true(1:end-nd)];
lam_s = filter(1 - exp(-dt/0.05), [1 -exp(-dt/0.05)], lam_d, lam_d(1) * exp(-dt/0.05));
X.lambda = lam_s;
% 점화각: 현재 맵 + 일부러 넣은 '입력 실수' 한 칸 (점화맵 점검이 찾는지 확인)
b = opt.ign_bump;
X.ign = P.ign.base(X.rpm, X.tps * 100) + b(3) * (abs(X.rpm - b(1)) < 250 & abs(X.tps*100 - b(2)) < 4);
[F, aux] = prc_make_frames(P, db, X, struct('drop_rate', 0.001, 'clt0', 85)); %#ok<ASGLU>
truth.err_fun = err;
truth.rpm_list = rpm_list; truth.tps_list = tps_list; truth.seq = seq;
truth.X = X; truth.ign_bump = opt.ign_bump;
if ~isempty(opt.out_csv)
  prc_log_write_csv(opt.out_csv, F, sprintf('dummy fuel-map test seed=%d (SIMULATED DATA)', opt.seed));
end
end
