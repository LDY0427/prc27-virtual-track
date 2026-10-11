function m = prc_dbc_msg(db, key)
% PRC_DBC_MSG  이름(문자열) 또는 ID(숫자)로 메시지 정의를 찾는다.
if ischar(key)
  k = find(cellfun(@(x) strcmp(x.name, key), db.msgs), 1);
else
  k = find(db.ids == key, 1);
end
if isempty(k)
  if ischar(key), error('prc_dbc_msg:none', 'DBC에 없는 메시지: %s', key);
  else, error('prc_dbc_msg:none', 'DBC에 없는 메시지 ID: 0x%X', key); end
end
m = db.msgs{k};
end
