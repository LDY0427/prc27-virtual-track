function [files, truth] = gen_ign_sweep_runs(P, db, outdir, opt)
% GEN_IGN_SWEEP_RUNS  점화 오프셋 스윕 시험 더미 로그 (오프셋마다 로그 1개).
%
%   [files, truth] = gen_ign_sweep_runs(P, db, 'out')
%
% 실제 시험 가정: EMU에서 풀스로틀 영역 점화를 전체 −4/−2/0/+2/+4°로 바꿔 가며
% 같은 길 양방향 3단 풀스로틀 당기기 + 코스트다운 (로드 다이노와 같은 절차).
% 정답: 현재 맵 대비 MBT 오프셋 = 3500 rpm +3°, 5000 +2°, 7000 +1°, 9000 0°, 10500 −1.5°
%        토크 = 현재 맵 토크 × (1 − k·(오프셋 − MBT)²) / (1 − k·MBT²),  k = 0.0025 /deg²
%        (MBT에서 2° 벗어나면 약 1%, 4°면 약 4% 토크 감소)
% 노킹은 이 더미에 없다. 실제로는 MBT 전에 노킹이 먼저 올 수 있다.
if nargin < 4, opt = struct(); end
opt = prc_opts(opt, struct('offsets', [-4 -2 0 2 4], 'k', 0.0025, ...
                           'mbt_rpm', [3500 5000 7000 9000 10500], 'mbt_deg', [3 2 1 0 -1.5]));
base = [17 24 30 30.5 33 36.5 36 34.2 32.5 29.5 25];      % gen_dyno_runs 기본 정답 토크
rt = P.eng.rpm_tab;
mb = interp1(opt.mbt_rpm, opt.mbt_deg, min(max(rt, opt.mbt_rpm(1)), opt.mbt_rpm(end)));
files = cell(1, numel(opt.offsets));
for i = 1:numel(opt.offsets)
  o = opt.offsets(i);
  trq = base .* (1 - opt.k * (o - mb).^2) ./ (1 - opt.k * mb.^2);
  files{i} = fullfile(outdir, sprintf('dummy_ign_off%+d.csv', o));
  gen_dyno_runs(P, db, struct('seed', 20 + i, 'n_rep', 1, 'trq_tab', trq, 'out_csv', files{i}));
end
truth.offsets = opt.offsets;
truth.mbt = @(rpm) interp1(opt.mbt_rpm, opt.mbt_deg, min(max(rpm, opt.mbt_rpm(1)), opt.mbt_rpm(end)));
truth.k = opt.k;
end
