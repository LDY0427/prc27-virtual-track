function prc_log_write_csv(fname, F, note)
% PRC_LOG_WRITE_CSV  원본 CAN 프레임을 CSV로 저장 (PRCLOG-CSV v0).
%
%   F.t_us (N×1, 정수 µs), F.bus, F.id, F.dlc, F.data (N×8)
%   형식: 첫 줄 '# ...' 설명, 둘째 줄 헤더, 이후 t_us,bus,id,dlc,d0..d7 (10진수)
%   DAQ 설계의 PRCLOG v1(바이너리) 변환기가 만들 CSV의 임시 형태다.
if nargin < 3, note = ''; end
fid = fopen(fname, 'w', 'n', 'UTF-8');
if fid < 0, error('prc_log_write_csv:open', '파일을 열 수 없음: %s', fname); end
fprintf(fid, '# PRCLOG-CSV v0 %s\n', note);
fprintf(fid, 't_us,bus,id,dlc,d0,d1,d2,d3,d4,d5,d6,d7\n');
M = [F.t_us(:) F.bus(:) F.id(:) F.dlc(:) F.data];
fprintf(fid, '%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n', M');
fclose(fid);
end
