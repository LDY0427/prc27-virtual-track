function [tu, Y] = prc_grid(S, fs, list)
% PRC_GRID  여러 메시지 신호를 하나의 등간격 시간축으로 맞춘다 (선형 보간, 범위 밖 NaN).
%   [tu, Y] = prc_grid(S, 100, {'WSS_Speed','WssFL'; 'EMU_SIM_0','RPM'; ...})
%   Y.(신호이름) : 열벡터. 같은 이름이 겹치면 '메시지_신호'로 저장.
%   기어·모드처럼 계단 신호는 '…:hold'를 붙이면 샘플-홀드한다. 예 {'EMU_SIM_1','Gear:hold'}
t0 = inf; t1 = -inf;
for i = 1:size(list, 1)
  t0 = min(t0, S.(list{i, 1}).t(1)); t1 = max(t1, S.(list{i, 1}).t(end));
end
tu = (ceil(t0 * fs) / fs : 1/fs : t1)';
Y = struct();
for i = 1:size(list, 1)
  M = S.(list{i, 1}); nm = list{i, 2}; hold_ = false;
  if numel(nm) > 5 && strcmp(nm(end-4:end), ':hold'), nm = nm(1:end-5); hold_ = true; end
  key = nm; if isfield(Y, key), key = [list{i, 1} '_' nm]; end
  x = M.(nm); tt = M.t;
  ok = ~isnan(x);
  if hold_
    Y.(key) = prc_hold(tt(ok), x(ok), tu);
  else
    Y.(key) = interp1(tt(ok), x(ok), tu, 'linear', NaN);
  end
end
end
