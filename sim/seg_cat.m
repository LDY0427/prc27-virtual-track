function X = seg_cat(varargin)
% SEG_CAT  구간들을 이어 붙인다. 마지막에 X.t를 0부터 dt 간격으로 만든다(dt = 1e-3 고정).
X = seg_blank(0);
f = fieldnames(X);
for k = 1:nargin
  for i = 1:numel(f), X.(f{i}) = [X.(f{i}); varargin{k}.(f{i})]; end
end
X.t = (0:numel(X.v)-1)' * 1e-3;
end
