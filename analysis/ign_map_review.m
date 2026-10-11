function I = ign_map_review(logfile, P, db, outdir, truth)
% IGN_MAP_REVIEW  로그에 찍힌 실제 점화각으로 점화맵을 3D로 복원하고 이상한 칸을 찾는다 (진각 제안 아님).
%
%   I = ign_map_review('로그.csv', P, db, 'out')
%
% 하는 일:
%   1) 정상 상태 구간의 ECU 점화각(IgnAngle)을 RPM × TPS 칸별로 평균 → '실제로 돌아간 점화맵'
%   2) 기준 맵(P.ign.base, 지금은 더미)과 차이
%   3) 튀는 칸: 양옆·위아래 칸으로 직선 보간한 값과 P.ign.spike_deg 이상 다르면 표시 (입력 실수 의심)
%   4) 부하 역전: 같은 RPM에서 TPS가 높은데 점화가 1° 넘게 더 진각된 칸 표시 (보통 부하↑ → 진각↓)
% 점화 시기를 '얼마나 당겨라'는 여기서 정하지 않는다. 그건 ign_mbt_sweep(토크 비교 시험)으로 한다.
if nargin < 4, outdir = 'out'; end
if nargin < 5, truth = []; end
C = P.ign; M = P.map;
F = prc_log_read_csv(logfile);
S = prc_log_decode(F, db);
fs = 50; dt = 1/fs;
[tu, Y] = prc_grid(S, fs, {'EMU_SIM_0','RPM'; 'EMU_SIM_0','TPS'; 'EMU_SIM_1','IgnAngle'; 'EMU_SIM_1','CLT'});
dT = abs(prc_deriv(Y.TPS, dt, 5)); dR = abs(prc_deriv(Y.RPM, dt, 5));
unsteady = ~(dT < M.dtps_max & dR < M.drpm_max);
tun = tu(unsteady); last_un = prc_hold(tun, tun, tu);
ok = ~unsteady & (isnan(last_un) | (tu - last_un) >= 0.3) & Y.TPS >= 2 & Y.CLT >= M.clt_min;
B = prc_cell_bin(Y.RPM, Y.TPS, Y.IgnAngle, C.rpm_bp, C.tps_bp, ok);
[RR, TT] = meshgrid(C.rpm_bp, C.tps_bp);
I.rpm_bp = C.rpm_bp; I.tps_bp = C.tps_bp;
I.map = B.mean; I.map(B.n < C.min_samples) = NaN; I.n = B.n;
I.ref = C.base(RR, TT);
I.diff = I.map - I.ref;
% 튀는 칸: 각 축으로 양옆 값의 직선 보간과 비교
[nt, nr] = size(I.map);
I.spike = false(nt, nr); I.pred = nan(nt, nr);
for a = 1:nt
  for b = 1:nr
    if isnan(I.map(a, b)), continue; end
    p = [];
    if b > 1 && b < nr && ~isnan(I.map(a, b-1)) && ~isnan(I.map(a, b+1))
      w = (C.rpm_bp(b) - C.rpm_bp(b-1)) / (C.rpm_bp(b+1) - C.rpm_bp(b-1));
      p(end+1) = I.map(a, b-1) + w * (I.map(a, b+1) - I.map(a, b-1)); %#ok<AGROW>
    end
    if a > 1 && a < nt && ~isnan(I.map(a-1, b)) && ~isnan(I.map(a+1, b))
      w = (C.tps_bp(a) - C.tps_bp(a-1)) / (C.tps_bp(a+1) - C.tps_bp(a-1));
      p(end+1) = I.map(a-1, b) + w * (I.map(a+1, b) - I.map(a-1, b)); %#ok<AGROW>
    end
    if ~isempty(p)
      I.pred(a, b) = mean(p);
      I.spike(a, b) = abs(I.map(a, b) - I.pred(a, b)) > C.spike_deg;
    end
  end
end
% 튀는 칸 바로 옆 칸은 그 칸 때문에 예측이 끌려가므로, 가장 크게 튀는 칸만 남긴다
sp = find(I.spike);
for q = sp'
  [a, b] = ind2sub([nt nr], q);
  nb = [a-1 b; a+1 b; a b-1; a b+1];
  for z = 1:4
    if nb(z,1) >= 1 && nb(z,1) <= nt && nb(z,2) >= 1 && nb(z,2) <= nr && I.spike(nb(z,1), nb(z,2))
      if abs(I.map(nb(z,1), nb(z,2)) - I.pred(nb(z,1), nb(z,2))) < abs(I.map(a, b) - I.pred(a, b))
        I.spike(nb(z,1), nb(z,2)) = false;
      end
    end
  end
end
% 부하 역전
I.inverse = false(nt, nr);
for b = 1:nr
  for a = 2:nt
    if ~isnan(I.map(a, b)) && ~isnan(I.map(a-1, b)) && I.map(a, b) - I.map(a-1, b) > 1.0
      I.inverse(a, b) = true;
    end
  end
end

% ---------- 그림 ----------
prc_plot3d_maps(C.rpm_bp, C.tps_bp, {I.map, I.ref, I.diff}, ...
  {'Ignition map as logged (ECU report)', 'Reference map (DUMMY until EMU map file)', 'Logged - reference [deg]'}, ...
  {'advance [deg BTDC]', 'advance [deg BTDC]', 'difference [deg]'}, fullfile(outdir, 'ign_review_3d.png'));
h = prc_fig(1100, 520);
V = I.map; V(isnan(V)) = min(V(:));
imagesc(1:nr, 1:nt, V); axis xy; colormap(jet); cb = colorbar; ylabel(cb, 'advance [deg BTDC]');
xl = arrayfun(@(x) sprintf('%d', x), C.rpm_bp, 'UniformOutput', false); xl(mod(C.rpm_bp, 1000) ~= 0) = {''};
set(gca, 'XTick', 1:nr, 'XTickLabel', xl, 'YTick', 1:nt, 'YTickLabel', arrayfun(@(x) sprintf('%d', x), C.tps_bp, 'UniformOutput', false));
xlabel('RPM'); ylabel('TPS [%]'); title('Logged ignition map  (box = spike vs neighbours, triangle = more advance at higher load)');
hold on;
for a = 1:nt
  for b = 1:nr
    if isnan(I.map(a, b))
      patch(b + [-0.5 0.5 0.5 -0.5], a + [-0.5 -0.5 0.5 0.5], [0.85 0.85 0.85], 'EdgeColor', 'none');
    else
      text(b, a, sprintf('%.1f', I.map(a, b)), 'HorizontalAlignment', 'center', 'FontSize', 7, 'Color', 'w');
    end
    if I.spike(a, b), rectangle('Position', [b-0.48 a-0.48 0.96 0.96], 'EdgeColor', 'k', 'LineWidth', 2.5); end
    if I.inverse(a, b), plot(b + 0.3, a + 0.3, 'k^', 'MarkerFaceColor', 'y'); end
  end
end
prc_savefig(h, fullfile(outdir, 'ign_review.png'));

Lr = {'# 점화맵 점검 (진각 제안 아님)', '', sprintf('> 입력: `%s`', logfile), '', ...
  '로그의 ECU 점화각을 정상 상태 구간에서 칸별로 평균해 실제로 돌아간 점화맵을 복원했다. 기준 맵은 EMU 맵 파일을 받기 전까지 더미다.', ''};
[sa, sb] = find(I.spike);
if isempty(sa), Lr{end+1} = '튀는 칸: 없음';
else
  Lr{end+1} = sprintf('**튀는 칸 %d개** (주변 칸 보간값과 %.1f° 넘게 차이 — 입력 실수 의심):', numel(sa), C.spike_deg);
  for q = 1:numel(sa)
    Lr{end+1} = sprintf('- %d rpm, TPS %d%%: %.1f° (주변 기준 %.1f°, 차이 %+.1f°)', C.rpm_bp(sb(q)), C.tps_bp(sa(q)), ...
                        I.map(sa(q), sb(q)), I.pred(sa(q), sb(q)), I.map(sa(q), sb(q)) - I.pred(sa(q), sb(q))); %#ok<AGROW>
  end
end
[ia, ib] = find(I.inverse);
if isempty(ia), Lr{end+1} = '부하 역전 칸: 없음';
else
  Lr{end+1} = sprintf('**부하 역전 %d칸** (TPS가 높은데 1° 넘게 더 진각):', numel(ia));
  for q = 1:numel(ia)
    Lr{end+1} = sprintf('- %d rpm: TPS %d%% %.1f° > TPS %d%% %.1f°', C.rpm_bp(ib(q)), C.tps_bp(ia(q)), I.map(ia(q), ib(q)), ...
                        C.tps_bp(ia(q)-1), I.map(ia(q)-1, ib(q))); %#ok<AGROW>
  end
end
if ~isempty(truth) && isfield(truth, 'ign_bump')
  Lr{end+1} = sprintf('더미 정답: %d rpm, TPS %d%% 칸에 %+.0f° 입력 실수를 넣어 두었다.', truth.ign_bump);
end
Lr = [Lr, {'', '튀는 칸은 바로 고치기 전에 의도한 값인지(예: 특정 회전수 공진 회피) 맵 작성자에게 확인한다.', ...
  '진각을 늘리는 판단은 이 점검으로 하지 않는다. `ign_mbt_sweep`의 토크 비교 시험과 노킹 감시가 필요하다.', '', ...
  '![ign review](ign_review.png)', '![ign review 3d](ign_review_3d.png)'}];
prc_write_lines(fullfile(outdir, 'ign_review_report.md'), Lr);
end
