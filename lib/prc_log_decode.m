function S = prc_log_decode(F, db)
% PRC_LOG_DECODE  원본 프레임 → 메시지별 신호 구조체.
%   S.(메시지).t        : 수신 시각 s
%   S.(메시지).(신호)    : 물리값 (무효 = NaN)
%   S.(메시지).raw.(신호): 원시값
%   S.unknown_ids       : DBC에 없는 ID
S = struct();
S.unknown_ids = [];
ids = unique(F.id);
for i = 1:numel(ids)
  k = find(db.ids == ids(i), 1);
  if isempty(k), S.unknown_ids(end+1) = ids(i); continue; end
  m = db.msgs{k};
  sel = F.id == ids(i);
  [phys, raw] = prc_can_decode(m, F.data(sel, :));
  phys.t = F.t_us(sel) / 1e6;
  phys.raw = raw;
  S.(m.name) = phys;
end
end
