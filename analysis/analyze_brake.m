function A = analyze_brake(logfile, P, db, outdir, truth)
% ANALYZE_BRAKE  BPS(전·후 압력)·휠속도·IMU로 제동 이벤트별 감속·정지거리·바퀴 잠김을 분석한다.
%
%   A = analyze_brake('out/dummy_brake.csv', P, db, 'out')
%
% 이벤트: 전륜 압력 > 3 bar가 0.3 s 이상.
% 차속 기준: prc_brake_ref (가장 빠른 바퀴, 네 바퀴가 다 미끄러지면 IMU 적분)
% 잠김: 바퀴 슬립 > P.brk.lock_slip 이 30 ms 이상 (차속 5 km/h 이상에서만)
% 출력: 시작 속도, 최대 압력(전/후), 평균 감속(IMU), bar당 감속, 정지거리,
%       처음 잠긴 바퀴·시각·그때 압력, 네 바퀴 모두 잠김 여부(검차 제동 시험 조건)
if nargin < 4, outdir = 'out'; end
if nargin < 5, truth = []; end
g = P.veh.g;
F = prc_log_read_csv(logfile);
S = prc_log_decode(F, db);
cal = prc_imu_align(S, P);
fs = 100; dt = 1/fs;
[tu, Y] = prc_grid(S, fs, {'WSS_Speed','WssFL'; 'WSS_Speed','WssFR'; 'WSS_Speed','WssRL'; 'WSS_Speed','WssRR'; ...
  'DAQ_Chassis','BrkPressF'; 'DAQ_Chassis','BrkPressR'; 'DAQ_IMU_Acc','AccX'; 'DAQ_IMU_Acc','AccY'; 'DAQ_IMU_Acc','AccZ'});
fb = prc_imu_apply(cal, [Y.AccX Y.AccY Y.AccZ]);
ax = fb(:, 1) * g; ax(isnan(ax)) = 0;
w = [Y.WssFL Y.WssFR Y.WssRL Y.WssRR] / 3.6;
w(isnan(w)) = 0;
vref = prc_brake_ref(max(w, [], 2), ax, dt);
slip = (repmat(vref, 1, 4) - w) ./ repmat(max(vref, 0.1), 1, 4);
pF = Y.BrkPressF; pR = Y.BrkPressR; pF(isnan(pF)) = 0; pR(isnan(pR)) = 0;
evr = prc_runs(pF > 3, tu, 0.3);
wn = {'FL', 'FR', 'RL', 'RR'};
A.events = struct('t0', {}, 'v0_kmh', {}, 'pF_max', {}, 'pR_max', {}, 'decel_g', {}, 'decel_per_bar', {}, ...
                  'stop_m', {}, 'locked', {}, 'first', {}, 't_first', {}, 'p_first', {}, 'all4', {});
for e = 1:size(evr, 1)
  i0 = evr(e, 1); i1 = evr(e, 2);
  v0 = vref(max(i0-1, 1)) * 3.6;
  pFm = max(pF(i0:i1)); pRm = max(pR(i0:i1));
  ist = find(vref(i0:i1) < 0.3, 1); if isempty(ist), ist = i1 - i0 + 1; end
  ie = i0 + ist - 1;
  stop_m = sum(vref(i0:ie)) * dt;
  win = (i0:ie)'; win = win(pF(win) >= 0.9 * pFm & vref(win) > 5/3.6);
  dec = -mean(ax(win)) / g;
  locked = false(1, 4); tf = nan(1, 4); pf = nan(1, 4);
  for k = 1:4
    lk = slip(i0:ie, k) > P.brk.lock_slip & vref(i0:ie) > 5/3.6;
    rr = prc_runs(lk, tu(i0:ie), 0.03);
    if ~isempty(rr)
      locked(k) = true; tf(k) = tu(i0 + rr(1,1) - 1) - tu(i0);
      if k <= 2, pf(k) = pF(i0 + rr(1,1) - 1); else, pf(k) = pR(i0 + rr(1,1) - 1); end
    end
  end
  if any(locked)
    [t_first, kk] = min(tf); first = wn{kk}; p_first = pf(kk);
  else
    t_first = NaN; first = '-'; p_first = NaN;
  end
  A.events(e) = struct('t0', tu(i0), 'v0_kmh', v0, 'pF_max', pFm, 'pR_max', pRm, 'decel_g', dec, ...
    'decel_per_bar', dec / mean(pF(win)), 'stop_m', stop_m, 'locked', locked, 'first', first, ...
    't_first', t_first, 'p_first', p_first, 'all4', all(locked));
  A.lock_t(e, :) = tf; A.lock_p(e, :) = pf;
end
A.imu = cal;

% ---------- 그림 ----------
ne = numel(A.events);
h = prc_fig(1100, 700);
for e = 1:min(ne, 6)
  i0 = evr(e, 1); s = max(i0 - 30, 1):min(i0 + 200, numel(tu));
  subplot(2, 3, e); hold on; grid on;
  plot(tu(s) - tu(i0), w(s, :) * 3.6); plot(tu(s) - tu(i0), vref(s) * 3.6, 'k--', 'LineWidth', 1.5);
  plot(tu(s) - tu(i0), pF(s), 'm:');
  title(sprintf('event %d: pF %.0f bar', e, A.events(e).pF_max)); xlabel('t from brake on [s]');
  if e == 1, legend('FL','FR','RL','RR','v ref','pF [bar]', 'Location', 'northeast'); ylabel('km/h'); end
end
prc_savefig(h, fullfile(outdir, 'brake_events.png'));

Lr = {'# 제동 분석 (가상 데이터)', '', sprintf('> 입력: `%s`', logfile), '', ...
  sprintf('목표(설계보고서): %d km/h에서 %.1f m 이내 정지. 검차 제동 시험은 네 바퀴가 모두 잠겨야 한다.', ...
          P.brk.target_v0_kmh, P.brk.target_stop_m), '', ...
  '| # | 시작 km/h | 압력 전/후 bar | 평균 감속 g | g/bar | 정지거리 m | 잠김 | 첫 잠김 (s, bar) | 4륜 |', ...
  '|---:|---:|---|---:|---:|---:|---|---|:---:|'};
for e = 1:ne
  E = A.events(e);
  lk = strjoin(wn(E.locked), ','); if isempty(lk), lk = '-'; end
  y = 'N'; if E.all4, y = 'Y'; end
  fst = '-'; if ~isnan(E.t_first), fst = sprintf('%s %.2f s @ %.0f', E.first, E.t_first, E.p_first); end
  Lr{end+1} = sprintf('| %d | %.1f | %.0f / %.0f | %.2f | %.4f | %.2f | %s | %s | %s |', e, E.v0_kmh, E.pF_max, E.pR_max, ...
                      E.decel_g, E.decel_per_bar, E.stop_m, lk, fst, y); %#ok<AGROW>
end
if ~isempty(truth)
  Lr{end+1} = '';
  Lr{end+1} = '정답(더미) 잠김 시각 s [FL FR RL RR]:';
  for e = 1:numel(truth.events)
    Lr{end+1} = sprintf('- %d bar: %s, 정지 %.2f m', truth.events(e).p, mat2str(truth.events(e).t_lock, 3), truth.events(e).stop_m); %#ok<AGROW>
  end
end
Lr = [Lr, {'', '정지거리는 압력이 3 bar를 넘은 순간부터. 페달을 밟기 시작한 지연(운전자 반응)은 빠져 있다.', ...
  '잠김 판정은 기준 차속이 맞아야 한다. 네 바퀴가 한꺼번에 잠기면 IMU 적분으로 차속을 이어 가므로 IMU 장착 보정이 중요하다.', ...
  '', '![brake](brake_events.png)'}];
prc_write_lines(fullfile(outdir, 'brake_report.md'), Lr);
end
