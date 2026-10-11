function prc_write_lines(fname, lines)
% PRC_WRITE_LINES  셀 배열의 각 줄을 UTF-8 텍스트로 저장.
fid = fopen(fname, 'w', 'n', 'UTF-8');
if fid < 0, error('prc_write_lines:open', '파일을 열 수 없음: %s', fname); end
for i = 1:numel(lines), fprintf(fid, '%s\n', lines{i}); end
fclose(fid);
end
