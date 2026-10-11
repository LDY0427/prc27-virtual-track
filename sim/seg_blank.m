function X = seg_blank(n)
% SEG_BLANK  빈 차량 상태 구간 (더미 생성용). 필드는 prc_make_frames 입력과 같다.
z = zeros(n, 1);
X = struct('v', z, 'w', zeros(n, 4), 'rpm', z, 'tps', z, 'gear', z, 'cut', z, 'ax', z, 'ay', z, ...
           'yaw', z, 'steer', z, 'pF', z, 'pR', z, 'grade', z);
end
