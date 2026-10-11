function K = ign_mbt_sweep(files, offsets, P, db, outdir, truth)
% IGN_MBT_SWEEP  점화 오프셋별 로드 다이노 로그로 회전수별 MBT(토크 최대 점화)를 찾고 점화 변경을 제안한다.
%
%   K = ign_mbt_sweep({'off-4.csv', ..., 'off+4.csv'}, [-4 -2 0 2 4], P, db, 'out')
%
% 1) 로그마다 analyze_road_dyno로 RPM별 엔진 토크 추정
% 2) RPM 칸마다 토크 = a·오프셋² + b·오프셋 + c 로 맞춤 → MBT = −b / 2a (시험 범위 안일 때만)
% 3) 제안 = MBT − P.ign.mbt_margin_deg, 한 번에 ±P.ign.max_step_deg 이내
% 4) 안전 규칙: P.ign.knock_monitoring = false 이면
%      늦추는 제안(−)만 '적용 권장', 당기는 제안(+)은 '노킹 감시 후 검토'로만 표시
%    MBT가 시험 범위 끝(더 당겨도 토크가 계속 오름)이면 MBT를 정하지 않는다.
% 풀스로틀 영역(TPS ≥ 80%) 칸에만 반영해 3D 맵으로 보여 준다. 부분부하 점화는 이 시험으로 정하지 않는다.
if nargin < 5, outdir = 'out'; end
if nargin < 6, truth = []; end
C = P.ign;
tmp = fullfile(outdir, 'ign_sweep_tmp');
if ~exist(tmp, 'dir'), mkdir(tmp); end
no = numel(offsets);
for i = 1:no
  A = analyze_road_dyno(files{i}, P, db, tmp);
  if i == 1, rpm = A.dyno.rpm; T = nan(numel(rpm), no); end
  T(:, i) = A.dyno.TB;
end
K.rpm = rpm; K.offsets = offsets(:)'; K.T = T;
nb = numel(rpm);
K.mbt = nan(nb, 1); K.status = cell(nb, 1); K.gain_pct = nan(nb, 1);
K.cand = nan(nb, 1); K.apply = nan(nb, 1);
o = offsets(:);
for j = 1:nb
  y = T(j, :)';
  if any(isnan(y)), K.status{j} = '데이터 부족'; continue; end
  p = [o.^2 o ones(no, 1)] \ y;
  if p(1) >= 0
    K.status{j} = '곡선이 볼록하지 않음(잡음) — 판단 보류'; continue;
  end
  m = -p(2) / (2*p(1));
  if m > max(o)
    K.status{j} = '더 당겨도 토크 증가 — 시험 범위 밖, MBT 미확정'; K.cand(j) = C.max_step_deg; continue;
  elseif m < min(o)
    K.status{j} = '더 늦춰도 토크 증가 — 시험 범위 밖, MBT 미확정'; K.cand(j) = -C.max_step_deg; continue;
  end
  K.mbt(j) = m;
  K.gain_pct(j) = (polyval(p, m) / polyval(p, 0) - 1) * 100;
  K.status{j} = 'MBT 확인';
end
% 칸마다 토크 차이가 1~4%라 잡음이 크다 → ±500 rpm 안의 MBT를 평균해 매끄럽게
K.mbt_raw = K.mbt;
v = find(~isnan(K.mbt_raw));
for j = v'
  w = v(abs(rpm(v) - rpm(j)) <= 500);
  K.mbt(j) = mean(K.mbt_raw(w));
end
okm = ~isnan(K.mbt);
K.cand(okm) = min(max(K.mbt(okm) - C.mbt_margin_deg, -C.max_step_deg), C.max_step_deg);
% 안전 규칙
K.apply = K.cand;
K.apply(isnan(K.apply)) = 0;
if ~C.knock_monitoring
  K.apply = min(K.apply, 0);
end
K.apply(~strcmp(K.status, 'MBT 확인') & K.apply > 0) = 0;
% 표에 반영 (TPS ≥ 80 행, RPM 축은 보간, 범위 밖 0)
[RR, TT] = meshgrid(C.rpm_bp, C.tps_bp);
okb = ~isnan(K.cand);
dRow = interp1(rpm(okb), K.apply(okb), C.rpm_bp, 'linear', 0);
cRow = interp1(rpm(okb), K.cand(okb), C.rpm_bp, 'linear', NaN);
wot = C.tps_bp >= 80;
K.map_before = C.base(RR, TT);
K.delta_apply = zeros(size(RR)); K.delta_apply(wot, :) = repmat(dRow, sum(wot), 1);
K.delta_cand = nan(size(RR)); K.delta_cand(wot, :) = repmat(cRow, sum(wot), 1);
K.map_after = K.map_before + K.delta_apply;
if ~isempty(truth)
  mt = truth.mbt(rpm);
  sel = ~isnan(K.mbt);
  K.mbt_err = abs(K.mbt(sel) - mt(sel));
  K.mbt_true = mt;
end

% ---------- 그림 ----------
h = prc_fig(1150, 480);
subplot(1, 2, 1); hold on; grid on;
pick = round(linspace(find(~isnan(K.mbt), 1), find(~isnan(K.mbt), 1, 'last'), 5));
cols = lines(numel(pick)); oo = linspace(min(o), max(o), 40);
hl = zeros(1, numel(pick));
for q = 1:numel(pick)
  j = pick(q); y = T(j, :);
  plot(o, (y / y(o == 0) - 1) * 100, 'o', 'Color', cols(q, :));
  p = [o.^2 o ones(no, 1)] \ y(:);
  hl(q) = plot(oo, (polyval(p, oo) / y(o == 0) - 1) * 100, '-', 'Color', cols(q, :));
end
xlabel('ignition offset from current map [deg]'); ylabel('torque change vs offset 0 [%]');
title('Torque vs ignition offset (markers = data, lines = fit)');
legend(hl, arrayfun(@(j) sprintf('%d rpm', round(rpm(j))), pick, 'UniformOutput', false), 'Location', 'south');
subplot(1, 2, 2); hold on; grid on;
plot(rpm, K.mbt, 'bo-', rpm, K.cand, 'm--', rpm, K.apply, 'r-', 'LineWidth', 1.5);
lg = {'MBT (estimated)', 'candidate = MBT - margin', 'apply (no knock monitoring: retard only)'};
if ~isempty(truth), plot(rpm, K.mbt_true, 'g:', 'LineWidth', 2); lg{end+1} = 'MBT truth (dummy)'; end
plot([min(rpm) max(rpm)], [0 0], 'k:');
legend(lg, 'Location', 'northeast'); xlabel('RPM'); ylabel('offset from current map [deg]');
title('MBT and suggested ignition change (WOT)');
prc_savefig(h, fullfile(outdir, 'ign_mbt.png'));
prc_plot3d_maps(C.rpm_bp, C.tps_bp, {K.map_before, K.map_after, K.delta_cand}, ...
  {'Current ignition map (DUMMY until EMU map file)', 'After applying (retard only)', 'Candidate change, WOT rows [deg]'}, ...
  {'advance [deg BTDC]', 'advance [deg BTDC]', 'change [deg]'}, fullfile(outdir, 'ign_mbt_3d.png'));

% ---------- 보고서 ----------
kn = '없음 → 늦추는 제안만 적용 권장'; if C.knock_monitoring, kn = '있음 → 당기는 제안도 적용 권장'; end
Lr = {'# 점화 MBT 스윕 분석', '', sprintf('> 오프셋 %s° 로그 %d개, 노킹 감시: %s', mat2str(offsets), no, kn), '', ...
  '**ECU에 자동으로 쓰지 않는다.** 노킹을 못 보는 상태에서 진각은 엔진 손상 위험이 있다.', '', ...
  '| RPM | MBT 오프셋 ° | MBT에서 토크 이득 | 후보 변경 ° (MBT − 여유) | 적용 권장 ° | 상태 |', '|---:|---:|---:|---:|---:|---|'};
for j = 1:nb
  f = @(x) iif_nan(x, '%+.1f');
  Lr{end+1} = sprintf('| %d | %s | %s | %s | %s | %s |', round(rpm(j)), f(K.mbt(j)), iif_nan(K.gain_pct(j), '%+.1f%%'), ...
                      f(K.cand(j)), f(K.apply(j)), K.status{j}); %#ok<AGROW>
end
if ~isempty(truth)
  Lr{end+1} = '';
  Lr{end+1} = sprintf('더미 정답 대비 MBT 오차: 평균 %.2f°, 최대 %.2f° (%d칸)', mean(K.mbt_err), max(K.mbt_err), numel(K.mbt_err));
end
Lr = [Lr, {'', '## 시험·적용 규칙', '', ...
  '1. 오프셋은 늦추는 쪽(−)부터 시험한다. 당기는 쪽은 노킹 감시(노크 센서, 청음, 실린더압 중 하나)를 갖춘 뒤에만.', ...
  '2. 오프셋마다 같은 길·같은 방향·같은 기온에서 2회 이상. 토크 차이가 1~4%라 조건 차이가 더 클 수 있다.', ...
  '3. 노킹 소리·출력 저하·배기온 급상승이 있으면 즉시 중단하고 그 오프셋은 쓰지 않는다.', ...
  '4. MBT는 토크가 더 안 오르는 지점이다. 실제 맵은 MBT보다 여유(지금 2°)만큼 늦게 둔다.', ...
  '5. 부분부하 점화는 이 시험으로 정하지 않는다. 부하 다이노가 필요하다.', '', ...
  '![mbt](ign_mbt.png)', '![mbt 3d](ign_mbt_3d.png)'}];
prc_write_lines(fullfile(outdir, 'ign_mbt_report.md'), Lr);
end

function s = iif_nan(x, fmt)
if isnan(x), s = '-'; else, s = sprintf(fmt, x); end
end
