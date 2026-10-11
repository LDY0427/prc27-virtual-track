function export_dash_json(P, outfile)
% EXPORT_DASH_JSON  가상 시험장 주행 3종 + 엔진 곡선을 실시간 대시보드(HTML)용 JSON으로 내보낸다.
%
%   export_dash_json(prc27_params(), 'out/dash_data.json')
%   runs[]: name, dt, t, v_kmh, rpm, gear, tps, pF, sw_deg, ax_g, ay_g, yaw_dps, slip4, Fz4, tcs_cut, x, y, s,
%           torque_Nm(엔진), power_kW, road{x,y,width}, lap_time_s, qss_time_s
%   dyno: rpm, torque, kW
% 20 Hz로 줄여 저장(파일 크기). 모두 가상 데이터.
D = sim_virtual_dyno(P);
runs = {};
specs = {'accel', struct(); 'skidpad', struct(); 'autocross', struct('laps', 1)};
for k = 1:size(specs, 1)
  S = vt_run(P, specs{k, 1}, specs{k, 2});
  step = 5;                                   % 100 Hz → 20 Hz
  idx = 1:step:numel(S.t);
  Te = prc_engine_torque(P, S.rpm(idx), S.tps(idx)) .* (1 - S.tcs_cut(idx));
  r.name = specs{k, 1}; r.dt = (S.t(2) - S.t(1)) * step;
  r.t = S.t(idx) - S.t(1); r.v_kmh = S.v_kmh(idx); r.rpm = S.rpm(idx); r.gear = S.gear(idx);
  r.tps = S.tps(idx); r.pF = S.pF(idx); r.sw_deg = S.sw_deg(idx);
  r.ax_g = S.ax_g(idx); r.ay_g = S.ay_g(idx); r.yaw_dps = S.yaw_dps(idx);
  r.slip4 = S.slip4(idx, :); r.Fz4 = S.Fz4(idx, :); r.tcs_cut = S.tcs_cut(idx);
  r.x = S.x(idx); r.y = S.y(idx); r.s = S.s(idx);
  r.torque_Nm = Te; r.power_kW = Te .* S.rpm(idx) * 2*pi/60 / 1000;
  rd = S.road; ri = 1:4:numel(rd.x);
  r.road = struct('x', rd.x(ri), 'y', rd.y(ri), 'psi', rd.psi(ri), 'width', rd.width_m, 'closed', rd.closed);
  r.lap_time_s = S.lap_time_s; r.qss_time_s = S.qss_time_s;
  runs{end+1} = r; %#ok<AGROW>
end
out.generated = datestr(now, 'yyyy-mm-dd HH:MM');
out.note = 'SIMULATED DATA - PRC-26 report parameters + assumptions. Not measured.';
out.dyno = struct('rpm', D.rpm, 'torque', D.Te, 'kW', D.kW, 'rpm_limit', P.eng.rpm_limit, 'shift_rpm', P.drv.shift_rpm);
out.params = struct('mass_kg', P.veh.mass_kg, 'final', P.drv.final, 'gears', P.drv.primary * P.drv.gears * P.drv.final, ...
                    'tire_r_m', P.tire.r_m, 'sw_max_deg', P.steer.sw_max_deg);
out.runs = runs;
fid = fopen(outfile, 'w', 'n', 'UTF-8');
fprintf(fid, '%s', tojson(out));
fclose(fid);
end

function s = tojson(v)
% 최소 JSON 직렬화 (MATLAB jsonencode 없이도 Octave에서 동작)
if isstruct(v)
  f = fieldnames(v); parts = cell(1, numel(f));
  for i = 1:numel(f), parts{i} = sprintf('"%s":%s', f{i}, tojson(v.(f{i}))); end
  s = ['{' strjoin(parts, ',') '}'];
elseif iscell(v)
  parts = cellfun(@tojson, v, 'UniformOutput', false);
  s = ['[' strjoin(parts, ',') ']'];
elseif ischar(v)
  s = ['"' strrep(strrep(v, '\', '\\'), '"', '\"') '"'];
elseif islogical(v)
  if v, s = 'true'; else, s = 'false'; end
elseif isnumeric(v)
  if isempty(v), s = '[]';
  elseif isscalar(v), s = num1(v);
  elseif isvector(v), s = ['[' strjoin(arrayfun(@num1, v(:)', 'UniformOutput', false), ',') ']'];
  else
    rows = cell(1, size(v, 1));
    for i = 1:size(v, 1), rows{i} = tojson(v(i, :)); end
    s = ['[' strjoin(rows, ',') ']'];
  end
else
  s = 'null';
end
end

function s = num1(x)
if isnan(x) || isinf(x), s = 'null'; else, s = sprintf('%.5g', x); end
end
