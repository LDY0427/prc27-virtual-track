function h = prc_fig(w, hgt)
% PRC_FIG  화면에 띄우지 않는 그림 창 (저장용).
if nargin < 1, w = 1000; end
if nargin < 2, hgt = 700; end
% Octave를 창 없이 실행하면 fltk는 저장이 안 된다 → gnuplot으로 전환
if exist('OCTAVE_VERSION', 'builtin') && strcmp(graphics_toolkit(), 'fltk') ...
   && any(strcmp(available_graphics_toolkits(), 'gnuplot'))
  graphics_toolkit('gnuplot');
end
h = figure('Visible', 'off', 'Position', [50 50 w hgt], 'Color', 'w');
end
