function prc_savefig(h, fname)
% PRC_SAVEFIG  그림을 PNG로 저장하고 닫는다.
%   OneDrive 동기화 등으로 파일이 잠깐 잠겨 있으면 1 s 간격으로 3번까지 다시 시도한다.
set(h, 'PaperPositionMode', 'auto');
for k = 1:4
  try
    print(h, fname, '-dpng', '-r100');
    break;
  catch err
    if k == 4, close(h); rethrow(err); end
    pause(1);
  end
end
close(h);
end
