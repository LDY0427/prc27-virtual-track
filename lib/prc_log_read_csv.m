function F = prc_log_read_csv(fname)
% PRC_LOG_READ_CSV  PRCLOG-CSV v0 읽기 → F.t_us, bus, id, dlc, data(N×8). 시각순 정렬.
M = dlmread(fname, ',', 2, 0);
M = M(any(M, 2), :);                       % 끝 빈 줄 제거
[~, o] = sort(M(:, 1));
M = M(o, :);
F.t_us = M(:, 1); F.bus = M(:, 2); F.id = M(:, 3); F.dlc = M(:, 4);
F.data = M(:, 5:12);
end
