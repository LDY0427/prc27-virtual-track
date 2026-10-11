function Q = prc_quality_check(S, db, P)
% PRC_QUALITY_CHECK  디코딩된 로그의 데이터 품질 점검.
%
%   - 메시지별 수신 개수·평균 주기·기대 주기 대비 끊김(2.5배 이상)·최대 공백
%   - 순번(…Seq) 건너뜀 → 프레임 손실 추정 (순번이 없는 메시지는 손실을 셀 수 없다)
%   - 신호별 무효(NaN) 비율
%   - 휠속도 좌우 비교로 찾은 센서 이상 구간
%   Q.lines : 보고서용 텍스트 줄
Q = struct('msg', struct('name', {}, 'n', {}, 'rate_hz', {}, 'exp_hz', {}, ...
           'gaps', {}, 'max_gap_ms', {}, 'seq_lost', {}), ...
           'invalid', {{}}, 'wss_faults', struct('wheel', {}, 't0', {}, 't1', {}), ...
           'unknown_ids', S.unknown_ids, 'missing', {{}}, 'lines', {{}});
names = fieldnames(P.log.period_s);
for i = 1:numel(names)
  nm = names{i};
  per = P.log.period_s.(nm);
  if ~isfield(S, nm), Q.missing{end+1} = nm; continue; end
  M = S.(nm);
  t = M.t; n = numel(t);
  dt = diff(t);
  r.name = nm; r.n = n;
  r.rate_hz = (n - 1) / max(t(end) - t(1), eps);
  r.exp_hz = 1 / per;
  r.gaps = sum(dt > 2.5 * per);
  r.max_gap_ms = 1000 * max([dt; 0]);
  r.seq_lost = 0;
  m = prc_dbc_msg(db, nm);
  for s = 1:numel(m.sigs)
    sg = m.sigs(s);
    if numel(sg.name) > 3 && strcmp(sg.name(end-2:end), 'Seq')
      d = mod(diff(M.raw.(sg.name)), 2^sg.len);
      % 한 프레임에 순번이 여러 개면(예 WSS_TCS) 같은 손실을 중복 집계하지 않도록 최댓값
      r.seq_lost = max(r.seq_lost, sum(d(d > 1) - 1));
    elseif any(isnan(M.(sg.name)))
      k = sum(isnan(M.(sg.name)));
      Q.invalid{end+1} = sprintf('%s.%s: %d/%d (%.1f%%)', nm, sg.name, k, n, 100*k/n);
    end
  end
  Q.msg(end+1) = r;
end
% 휠속도 이상 구간
if isfield(S, 'WSS_Speed')
  W = S.WSS_Speed;
  w = [W.WssFL W.WssFR W.WssRL W.WssRR];
  vin = true(size(w));
  if isfield(S, 'WSS_TCS') && numel(S.WSS_TCS.t) == numel(W.t)
    vb = S.WSS_TCS.WssValid;
    for k = 1:4, vin(:, k) = mod(floor(vb / 2^(k-1)), 2) == 1; end
  end
  R = prc_vehicle_ref(w, vin, P);
  wn = {'FL', 'FR', 'RL', 'RR'};
  for k = 1:4
    bad = ~R.valid(:, k);
    e = diff([0; bad; 0]);
    s0 = find(e == 1); s1 = find(e == -1) - 1;
    for q = 1:numel(s0)
      if W.t(s1(q)) - W.t(s0(q)) >= 0.05
        Q.wss_faults(end+1) = struct('wheel', wn{k}, 't0', W.t(s0(q)), 't1', W.t(s1(q)));
      end
    end
  end
end
% 텍스트
L = {};
L{end+1} = '| 메시지 | 수신 | 평균 Hz | 기대 Hz | 끊김(>2.5주기) | 최대 공백 ms | 순번 손실 |';
L{end+1} = '|---|---:|---:|---:|---:|---:|---:|';
for i = 1:numel(Q.msg)
  r = Q.msg(i);
  L{end+1} = sprintf('| %s | %d | %.1f | %.1f | %d | %.0f | %d |', r.name, r.n, ...
                     r.rate_hz, r.exp_hz, r.gaps, r.max_gap_ms, r.seq_lost); %#ok<AGROW>
end
if ~isempty(Q.missing), L{end+1} = ['수신 없음: ' strjoin(Q.missing, ', ')]; end
if ~isempty(Q.unknown_ids)
  L{end+1} = ['DBC에 없는 ID: ' strjoin(arrayfun(@(x) sprintf('0x%X', x), Q.unknown_ids, 'UniformOutput', false), ', ')];
end
L{end+1} = '';
if isempty(Q.invalid), L{end+1} = '무효 신호: 없음';
else, L{end+1} = '무효 신호(NaN):'; L = [L, strcat({'- '}, Q.invalid)]; end
L{end+1} = '';
if isempty(Q.wss_faults), L{end+1} = '휠속도 센서 이상 구간: 없음';
else
  L{end+1} = '휠속도 센서 이상 구간(좌우 비교):';
  for q = 1:numel(Q.wss_faults)
    f = Q.wss_faults(q);
    L{end+1} = sprintf('- %s: %.2f ~ %.2f s', f.wheel, f.t0, f.t1); %#ok<AGROW>
  end
end
Q.lines = L;
end
