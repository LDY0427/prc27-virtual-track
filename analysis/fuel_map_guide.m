function G = fuel_map_guide(logfile, P, db, outdir, truth)
% FUEL_MAP_GUIDE  주행 로그의 람다로 연료맵(RPM × TPS) 칸별 '연료 몇 % 늘리기/줄이기'를 제안한다.
%
%   G = fuel_map_guide('로그.csv', P, db, 'out')
%
% 원리: 칸마다  제안 % = (측정 람다 ÷ 목표 람다 − 1) × 100
%   측정 람다가 목표보다 크면(희박) 연료를 그만큼 늘리고, 작으면(농후) 줄인다.
%   예) 목표 0.88인데 0.95가 나오면 +8.0% → EMU 연료(VE) 표 그 칸에 ×1.080
%
% 믿을 수 있는 데이터만 쓴다:
%   - 람다 센서 지연(P.map.lambda_delay_s)만큼 람다를 앞당겨 운전 조건과 맞춘다
%   - 정상 상태만: TPS·RPM 변화가 작고, 변한 뒤 P.map.settle_s 지난 구간
%   - 냉각수 P.map.clt_min 이상, TPS 2% 이상(아이들·감속 연료 차단 제외), 람다 0.6~1.4
%   - 칸당 샘플 P.map.min_samples 이상이고 흔들림(표준편차) P.map.max_std 이하일 때만 '제안'
%   - 한 번에 ±P.map.max_step_pct % 이내로 자른다. 적용 후 다시 주행해 반복한다.
%
% 하지 않는 것: ECU에 직접 쓰지 않는다. 점화 시기는 제안하지 않는다(노킹 계측 없음).
% 출력: fuel_guide.png, fuel_guide_pct.csv (행 TPS, 열 RPM), fuel_guide_report.md
if nargin < 4, outdir = 'out'; end
if nargin < 5, truth = []; end
M = P.map;
F = prc_log_read_csv(logfile);
S = prc_log_decode(F, db);
fs = 50; dt = 1/fs;
[tu, Y] = prc_grid(S, fs, {'EMU_SIM_0','RPM'; 'EMU_SIM_0','TPS'; 'EMU_SIM_1','Lambda'; 'EMU_SIM_1','CLT'});
% 람다를 센서 지연만큼 앞당긴다
lam = interp1(tu, Y.Lambda, tu + M.lambda_delay_s, 'linear', NaN);
% 정상 상태 판정
dT = abs(prc_deriv(Y.TPS, dt, 5)); dR = abs(prc_deriv(Y.RPM, dt, 5));
unsteady = ~(dT < M.dtps_max & dR < M.drpm_max);
tun = tu(unsteady);
last_un = prc_hold(tun, tun, tu);                    % 직전 불안정 시각
settled = isnan(last_un) | (tu - last_un) >= M.settle_s;
ok = ~unsteady & settled & Y.CLT >= M.clt_min & Y.TPS >= 2 & lam > 0.6 & lam < 1.4 & ~isnan(lam + Y.RPM + Y.TPS);
% 가장 가까운 칸
[~, ir] = min(abs(repmat(Y.RPM, 1, numel(M.rpm_bp)) - repmat(M.rpm_bp, numel(tu), 1)), [], 2);
[~, it] = min(abs(repmat(Y.TPS, 1, numel(M.tps_bp)) - repmat(M.tps_bp, numel(tu), 1)), [], 2);
nr = numel(M.rpm_bp); nt = numel(M.tps_bp);
G.rpm_bp = M.rpm_bp; G.tps_bp = M.tps_bp;
G.n = zeros(nt, nr); G.lam = nan(nt, nr); G.std = nan(nt, nr); G.target = nan(nt, nr);
G.pct_raw = nan(nt, nr); G.pct = nan(nt, nr); G.status = zeros(nt, nr);   % 0 없음 1 부족 2 흔들림 3 제안
for a = 1:nt
  lt = prc_lambda_target(P, M.tps_bp(a));
  G.target(a, :) = lt;
  for b = 1:nr
    s = ok & it == a & ir == b;
    G.n(a, b) = sum(s);
    if G.n(a, b) == 0, continue; end
    G.lam(a, b) = mean(lam(s)); G.std(a, b) = std(lam(s));
    G.pct_raw(a, b) = (G.lam(a, b) / lt - 1) * 100;
    if G.n(a, b) < M.min_samples
      G.status(a, b) = 1;
    elseif G.std(a, b) > M.max_std
      G.status(a, b) = 2;
    else
      G.status(a, b) = 3;
      G.pct(a, b) = min(max(G.pct_raw(a, b), -M.max_step_pct), M.max_step_pct);
    end
  end
end
G.n_used = sum(ok); G.n_total = numel(tu);
if ~isempty(truth)
  [RR, TT] = meshgrid(M.rpm_bp, M.tps_bp);
  G.pct_true = (1 ./ (1 + truth.err_fun(RR, TT)) - 1) * 100;
  m = G.status == 3;
  G.err_max = max(abs(G.pct(m) - G.pct_true(m)));
  G.err_mean = mean(abs(G.pct(m) - G.pct_true(m)));
end

% ---------- 그림 ----------
h = prc_fig(1150, 560);
V = G.pct; V(isnan(V)) = 0;
imagesc(1:nr, 1:nt, V); axis xy; caxis([-12 12]);
cm = [linspace(0.75,1,32)' linspace(0.2,1,32)' linspace(0.2,1,32)'; ...
      linspace(1,0.2,32)' linspace(1,0.45,32)' linspace(1,0.85,32)'];
colormap(flipud(cm)); cb = colorbar; ylabel(cb, 'fuel change [%]  (+ = add fuel)');
xl = arrayfun(@(x) sprintf('%d', x), M.rpm_bp, 'UniformOutput', false);
xl(mod(M.rpm_bp, 1000) ~= 0) = {''};                % 1000 rpm마다만 표시 (겹침 방지)
set(gca, 'XTick', 1:nr, 'XTickLabel', xl, ...
         'YTick', 1:nt, 'YTickLabel', arrayfun(@(x) sprintf('%d', x), M.tps_bp, 'UniformOutput', false));
xlabel('RPM'); ylabel('TPS [%]');
title('Fuel map guide: suggested change per cell (grey = no/insufficient data, ~ = unstable)');
hold on;
for a = 1:nt
  for b = 1:nr
    switch G.status(a, b)
      case 3, text(b, a, sprintf('%+.1f', G.pct(a, b)), 'HorizontalAlignment', 'center', 'FontSize', 8);
      case 2, text(b, a, '~', 'HorizontalAlignment', 'center', 'FontSize', 10);
      otherwise, patch(b + [-0.5 0.5 0.5 -0.5], a + [-0.5 -0.5 0.5 0.5], [0.85 0.85 0.85], 'EdgeColor', 'none');
        if G.status(a, b) == 1, text(b, a, sprintf('n%d', G.n(a, b)), 'HorizontalAlignment', 'center', 'FontSize', 7, 'Color', [0.4 0.4 0.4]); end
    end
  end
end
prc_savefig(h, fullfile(outdir, 'fuel_guide.png'));

% ---------- 3D: 현재 맵 / 제안 반영 후 / 증감량 ----------
[RR, TT] = meshgrid(M.rpm_bp, M.tps_bp);
dz = G.pct; dz0 = dz; dz0(isnan(dz0)) = 0;
if isfield(M, 've_base')
  G.ve_before = M.ve_base(RR, TT);
  G.ve_after = G.ve_before .* (1 + dz0 / 100);          % 제안 없는 칸은 그대로
  prc_plot3d_maps(M.rpm_bp, M.tps_bp, {G.ve_before, G.ve_after, dz}, ...
    {'Current fuel map (DUMMY until EMU map file)', 'After applying suggestion', 'Suggested change [%]'}, ...
    {'VE [%]', 'VE [%]', 'fuel change [%]'}, fullfile(outdir, 'fuel_guide_3d.png'));
  fid = fopen(fullfile(outdir, 'fuel_map_after.csv'), 'w', 'n', 'UTF-8');
  fprintf(fid, 'TPS\\RPM'); fprintf(fid, ',%d', M.rpm_bp); fprintf(fid, '\n');
  for a = nt:-1:1
    fprintf(fid, '%d', M.tps_bp(a)); fprintf(fid, ',%.1f', G.ve_after(a, :)); fprintf(fid, '\n');
  end
  fclose(fid);
else
  prc_plot3d_maps(M.rpm_bp, M.tps_bp, {dz}, {'Suggested change [%]'}, {'fuel change [%]'}, ...
    fullfile(outdir, 'fuel_guide_3d.png'), 600);
end

% ---------- 표(CSV): 행 TPS, 열 RPM, 빈칸 = 제안 없음 ----------
fid = fopen(fullfile(outdir, 'fuel_guide_pct.csv'), 'w', 'n', 'UTF-8');
fprintf(fid, 'TPS\\RPM'); fprintf(fid, ',%d', M.rpm_bp); fprintf(fid, '\n');
for a = nt:-1:1
  fprintf(fid, '%d', M.tps_bp(a));
  for b = 1:nr
    if G.status(a, b) == 3, fprintf(fid, ',%.1f', G.pct(a, b)); else, fprintf(fid, ','); end
  end
  fprintf(fid, '\n');
end
fclose(fid);

% ---------- 보고서 ----------
[sv, si] = sort(abs(G.pct(:)), 'descend');
Lr = {'# 연료맵 가이드', '', sprintf('> 입력: `%s` | 사용 샘플 %d / %d (정상 상태·웜업·람다 유효)', logfile, G.n_used, G.n_total), '', ...
  '**제안일 뿐 ECU에 자동으로 쓰지 않는다.** 아래 % 만큼 EMU 연료(VE) 표의 해당 칸에 곱하고, 다시 주행해 람다를 확인한다.', ...
  sprintf('+는 연료를 늘리라는 뜻(희박), −는 줄이라는 뜻(농후). 한 번에 최대 ±%d%%로 잘랐다.', M.max_step_pct), ''};
Lr{end+1} = '## 크게 바꿀 칸 (상위 10)';
Lr{end+1} = '';
Lr{end+1} = '| RPM | TPS % | 목표 λ | 측정 λ | 샘플 | 제안 | 메모 |';
Lr{end+1} = '|---:|---:|---:|---:|---:|---:|---|';
cnt = 0;
for q = 1:numel(si)
  if isnan(sv(q)) || cnt >= 10, continue; end
  [a, b] = ind2sub([nt nr], si(q));
  memo = '';
  if G.pct(a, b) > 3 && M.tps_bp(a) >= 60, memo = '고부하 희박 — 먼저 고친다'; end
  if abs(G.pct_raw(a, b)) > M.max_step_pct, memo = [memo ' 한도로 자름, 재측정 필요']; end
  Lr{end+1} = sprintf('| %d | %d | %.2f | %.3f | %d | **%+.1f%%** | %s |', M.rpm_bp(b), M.tps_bp(a), G.target(a, b), ...
                      G.lam(a, b), G.n(a, b), G.pct(a, b), memo); %#ok<AGROW>
  cnt = cnt + 1;
end
Lr{end+1} = '';
Lr{end+1} = sprintf('제안 칸 %d개, 데이터 부족 %d개, 흔들림 %d개, 주행 안 한 칸 %d개.', sum(G.status(:) == 3), ...
                    sum(G.status(:) == 1), sum(G.status(:) == 2), sum(G.status(:) == 0));
if ~isempty(truth)
  Lr{end+1} = sprintf('더미 정답 대비 오차: 평균 %.2f %%p, 최대 %.2f %%p', G.err_mean, G.err_max);
end
Lr = [Lr, {'', '## 적용 순서', '', ...
  '1. 고부하(TPS 60% 이상) 희박 칸부터 고친다. 엔진 손상 위험이 가장 크다.', ...
  '2. 제안값을 그대로 넣거나, 처음에는 50~70%만 넣어 다시 주행한다.', ...
  '3. 주변 칸과 크게 튀면 매끄럽게 이어 준다(ECU가 칸 사이를 보간하므로).', ...
  '4. 데이터 없는 칸은 건드리지 않는다. 그 조건을 일부러 주행해 채운다.', ...
  '5. 냉간 시동·가속 증량·감속 연료 차단은 이 표로 고치지 않는다(별도 보정 표).', '', ...
  '## 주의', '', ...
  sprintf('- 목표 람다(지금 부분부하 %.2f → 풀스로틀 %.2f)는 팀이 정할 값이다. P.map에서 바꾼다.', M.lambda_target_val(1), M.lambda_target_val(end)), ...
  sprintf('- 람다 센서 지연 %.2f s는 가정값이다. 스로틀 스텝을 주고 람다 반응 시간을 재서 바꾼다.', M.lambda_delay_s), ...
  '- 칸 판정은 가장 가까운 칸 기준이다. ECU 보간 가중치는 반영하지 않았다.', ...
  '- 점화 시기는 제안하지 않는다(노킹 계측 보류). 연료만 다룬다.', ...
  '- 람다 센서 자체 오차·배기 누설이 있으면 모든 칸이 같은 방향으로 틀어진다. 전 칸이 한쪽으로 쏠리면 센서부터 확인한다.', ...
  '- 3D 그림의 「현재 맵」은 EMU 맵 파일을 받기 전까지 더미(P.map.ve_base)다. `fuel_map_after.csv`도 그 더미 기준이다.', ...
  '', '![fuel guide](fuel_guide.png)', '![fuel guide 3d](fuel_guide_3d.png)'}];
prc_write_lines(fullfile(outdir, 'fuel_guide_report.md'), Lr);
end
