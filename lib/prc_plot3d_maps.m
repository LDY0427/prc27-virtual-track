function prc_plot3d_maps(xbp, ybp, Zs, titles, zlab, fname, w)
% PRC_PLOT3D_MAPS  ECU 표 여러 개를 3D 곡면으로 나란히 그려 PNG로 저장한다.
%   Zs, titles, zlab : 셀 배열 (표마다 하나). 표는 numel(ybp) × numel(xbp). NaN 칸은 비워 둔다.
if nargin < 7, w = 1250; end
n = numel(Zs);
h = prc_fig(w, 470);
[XX, YY] = meshgrid(xbp, ybp);
for k = 1:n
  subplot(1, n, k);
  surf(XX, YY, Zs{k}); shading faceted; colormap(jet); colorbar;
  view(-130, 28); grid on;
  xlabel('RPM'); ylabel('TPS [%]'); zlabel(zlab{k});
  title(titles{k});
  xlim([min(xbp) max(xbp)]); ylim([min(ybp) max(ybp)]);
end
prc_savefig(h, fname);
end
