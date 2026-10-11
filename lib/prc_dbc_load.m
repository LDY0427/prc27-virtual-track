function db = prc_dbc_load(fname)
% PRC_DBC_LOAD  DBC 파일에서 메시지(BO_)·신호(SG_)·값 이름(VAL_)을 읽는다.
%
%   db = prc_dbc_load('dbc/PRC27_sim_v0.1.dbc');
%   db.msgs{k}   : id, name, dlc, tx, sigs(구조체 배열)
%   db.ids       : 메시지 ID 배열 (db.msgs와 같은 순서)
%   db.enums     : VAL_ 정의 (id, sig, codes, names)
%
% Classic CAN, 일반 신호만 지원한다. 멀티플렉스 신호는 경고 후 건너뛴다.

txt = fileread(fname);
lines = regexp(txt, '\r?\n', 'split');
msgs = {};
enums = struct('id', {}, 'sig', {}, 'codes', {}, 'names', {});
sgpat = ['^SG_\s+(\w+)([^:]*):\s*(\d+)\|(\d+)@([01])([+-])\s*' ...
         '\(\s*([^,]+),\s*([^)]+)\)\s*\[\s*([^|]*)\|([^\]]*)\]\s*"([^"]*)"'];
for i = 1:numel(lines)
  L = strtrim(lines{i});
  if strncmp(L, 'BO_ ', 4)
    t = regexp(L, '^BO_\s+(\d+)\s+(\w+)\s*:\s*(\d+)\s+(\w+)', 'tokens', 'once');
    if isempty(t), continue; end
    m = struct();
    m.id = str2double(t{1});
    m.name = t{2};
    m.dlc = str2double(t{3});
    m.tx = t{4};
    m.sigs = empty_sigs();
    msgs{end+1} = m; %#ok<AGROW>
  elseif strncmp(L, 'SG_ ', 4) && ~isempty(msgs)
    t = regexp(L, sgpat, 'tokens', 'once');
    if isempty(t)
      warning('prc_dbc_load:sg', '신호 줄을 해석하지 못함: %s', L);
      continue;
    end
    if numel(t) == 11 && ~isempty(strtrim(t{2}))
      warning('prc_dbc_load:mux', '멀티플렉스 신호는 지원하지 않아 건너뜀: %s', t{1});
      continue;
    end
    if numel(t) == 10, t = [t(1) {''} t(2:end)]; end   % 빈 그룹이 빠지는 경우 대비
    s = struct('name', t{1}, 'start', str2double(t{3}), 'len', str2double(t{4}), ...
               'intel', strcmp(t{5}, '1'), 'signed', strcmp(t{6}, '-'), ...
               'factor', str2double(t{7}), 'offset', str2double(t{8}), ...
               'min', str2double(t{9}), 'max', str2double(t{10}), 'unit', t{11});
    msgs{end}.sigs(end+1) = s;
  elseif strncmp(L, 'VAL_ ', 5)
    t = regexp(L, '^VAL_\s+(\d+)\s+(\w+)\s+(.*);', 'tokens', 'once');
    if isempty(t), continue; end
    pr = regexp(t{3}, '(-?\d+)\s+"([^"]*)"', 'tokens');
    e = struct('id', str2double(t{1}), 'sig', t{2}, ...
               'codes', cellfun(@str2double, cellfun(@(c) c{1}, pr, 'UniformOutput', false)), ...
               'names', {cellfun(@(c) c{2}, pr, 'UniformOutput', false)});
    enums(end+1) = e; %#ok<AGROW>
  end
end
db.file = fname;
db.msgs = msgs;
db.ids = cellfun(@(m) m.id, msgs);
db.enums = enums;
end

function s = empty_sigs()
s = struct('name', {}, 'start', {}, 'len', {}, 'intel', {}, 'signed', {}, ...
           'factor', {}, 'offset', {}, 'min', {}, 'max', {}, 'unit', {});
end
