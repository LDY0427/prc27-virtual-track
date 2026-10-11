function FR = prc_frames_add(FR, db, name, tq, vals)
% PRC_FRAMES_ADD  한 메시지의 시각 열과 신호값을 인코딩해 프레임 목록에 붙인다.
%   …Seq 신호를 주지 않으면 0부터 1씩 늘어나는 순번을 자동으로 채운다.
m = prc_dbc_msg(db, name);
n = numel(tq);
for s = 1:numel(m.sigs)
  sg = m.sigs(s);
  if numel(sg.name) > 3 && strcmp(sg.name(end-2:end), 'Seq') && ~isfield(vals, sg.name)
    vals.(sg.name) = mod((0:n-1)', 2^sg.len);
  end
end
FR(end+1) = struct('t', tq(:), 'id', m.id * ones(n, 1), 'data', prc_can_encode(m, vals, n));
end
