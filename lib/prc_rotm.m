function Rm = prc_rotm(rpy_deg)
% PRC_ROTM  장착 각도 [롤 피치 요](deg) → 회전행렬 Rm = Rz(요)·Ry(피치)·Rx(롤).
%   센서 측정값 f_s와 차체 기준값 f_b의 관계:  f_s = Rm' * f_b,  f_b = Rm * f_s
a = rpy_deg(:)' * pi / 180;
cr = cos(a(1)); sr = sin(a(1)); cp = cos(a(2)); sp = sin(a(2)); cy = cos(a(3)); sy = sin(a(3));
Rx = [1 0 0; 0 cr -sr; 0 sr cr];
Ry = [cp 0 sp; 0 1 0; -sp 0 cp];
Rz = [cy -sy 0; sy cy 0; 0 0 1];
Rm = Rz * Ry * Rx;
end
