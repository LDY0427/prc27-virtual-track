function E = export_prc26_dyno(P, outdir)
% EXPORT_PRC26_DYNO  PRC-26 설계보고서 제원 기준 가상 다이노 결과를 CSV로 내보낸다 (PM 보고 자료용).
%
%   E = export_prc26_dyno(prc27_params(), 'out/pm')
%   engine.csv   rpm, 토크 N·m, 출력 kW, 출력 ps
%   tractive.csv 단별 차속 km/h, 구동력 N (열: v1,F1,v2,F2,...)
%   resist.csv   차속, 주행저항 N
%   accel.csv    75 m 가속 시계열 (건조, TCS 없음): t, x, v_kmh, rpm, gear
%   summary.csv  항목, 값
if nargin < 2, outdir = fullfile('out', 'pm'); end
if ~exist(outdir, 'dir'), mkdir(outdir); end
D = sim_virtual_dyno(P);
ng = numel(D.ratios);
dlmwrite(fullfile(outdir, 'engine.csv'), [D.rpm D.Te D.kW D.kW*1.35962], 'precision', '%.4f');
M = [];
for g = 1:ng, M = [M D.v_kmh(:, g) D.F_N(:, g)]; end %#ok<AGROW>
dlmwrite(fullfile(outdir, 'tractive.csv'), M, 'precision', '%.4f');
dlmwrite(fullfile(outdir, 'resist.csv'), [D.v_res_kmh D.F_res_N], 'precision', '%.4f');
% 단별 최고속(구동력 = 주행저항)과 차량 최고속
vtop = nan(1, ng);
for g = 1:ng
  Fr = interp1(D.v_res_kmh, D.F_res_N, D.v_kmh(:, g));
  k = find(D.F_N(:, g) < Fr, 1);
  if isempty(k), vtop(g) = D.v_max_gear_kmh(g); else, vtop(g) = D.v_kmh(k, g); end
end
R = sim_longitudinal(P, struct('tcs_on', false, 'mu_scale', 1.0, 't_end', 8));
sel = 1:10:numel(R.t);
dlmwrite(fullfile(outdir, 'accel.csv'), [R.t(sel) R.x(sel) R.v_kmh(sel) R.rpm(sel) R.gear(sel)], 'precision', '%.4f');
Rw = sim_longitudinal(P, struct('tcs_on', false, 'mu_scale', 0.7, 't_end', 8));
Rwt = sim_longitudinal(P, struct('tcs_on', true, 'mu_scale', 0.7, 't_end', 8));
E.D = D; E.vtop = vtop; E.t75 = R.t75; E.v75 = R.v75_kmh;
E.t75_wet_off = Rw.t75; E.t75_wet_on = Rwt.t75;
E.vmax = max(vtop);
[~, kT] = max(D.Te); [~, kP] = max(D.kW);
fid = fopen(fullfile(outdir, 'summary.csv'), 'w', 'n', 'UTF-8');
fprintf(fid, 'peak_torque_Nm,%.2f\npeak_torque_rpm,%d\npeak_power_kW,%.2f\npeak_power_rpm,%d\n', D.Te(kT), D.rpm(kT), D.kW(kP), D.rpm(kP));
fprintf(fid, 'grip_limit_N,%.1f\nt75_dry_s,%.3f\nv75_dry_kmh,%.2f\nt75_wet_off_s,%.3f\nt75_wet_on_s,%.3f\nvmax_kmh,%.1f\n', ...
        D.F_trac_N, R.t75, R.v75_kmh, Rw.t75, Rwt.t75, E.vmax);
for g = 1:ng
  sr = NaN; if g < ng, sr = D.shift_rpm(g); end
  fprintf(fid, 'gear%d,%.4f,%.2f,%.1f,%d,%.1f\n', g, D.ratios(g), D.v_max_gear_kmh(g), max(D.F_N(:, g)), D.traction_limited(g), sr);
  fprintf(fid, 'vtop%d,%.1f\n', g, vtop(g));
end
fclose(fid);
end
