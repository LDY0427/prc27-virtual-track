function o = prc_opts(o, d)
% PRC_OPTS  옵션 구조체 o에 없는 필드를 기본값 d로 채운다.
if isempty(o), o = struct(); end
f = fieldnames(d);
for i = 1:numel(f)
  if ~isfield(o, f{i}), o.(f{i}) = d.(f{i}); end
end
end
