function R = vt_road(name, opt)
% VT_ROAD  가상 시험장 도로 (CarMaker Road의 역할). 구간 목록 → 중심선·곡률·폭.
%
%   R = vt_road('skidpad')      FSK 스키드패드: 진입 직선 → 반경 9.125 m 원 2바퀴(좌) → 탈출
%   R = vt_road('accel')        75 m 직선 + 감속 구간
%   R = vt_road('autocross')    직선·슬라럼·헤어핀·고속 코너를 섞은 약 480 m 폐곡선 (실제 코스 아님)
%   R = vt_road('segments', struct('seg', {{'s', 30; 'a', [10 90]; ...}}))   직접 정의
%     's' 길이 m / 'a' [반경 m, 각도 deg](+ 좌회전, − 우회전)
%
%   R.s, R.x, R.y, R.psi, R.kappa (1/m, 좌회전 +), R.width_m, R.closed, R.name
%   R.marks: 이름 붙은 지점(s) 예) 75 m 선, 스키드패드 계측 구간
if nargin < 2, opt = struct(); end
opt = prc_opts(opt, struct('ds', 0.25, 'width_m', 3.5, 'seg', {{}}, 'laps', 3));   % 폭 3.5 m = FSAE 오토크로스 최소 폭
ds = opt.ds; marks = struct('name', {}, 's', {});
closed = false;
switch lower(name)
  case 'skidpad'
    Rk = 9.125;                              % [보고서] 주행선 반경
    nl = max(opt.laps, 3);
    seg = {'s', 20; 'a', [Rk 360*nl]; 's', 20};
    marks(end+1) = struct('name', 'timing_start', 's', 20 + 2*pi*Rk*(nl-2));   % 마지막 2바퀴 계측 (= 보고서 목표 9.7 s와 비교)
    marks(end+1) = struct('name', 'timing_end', 's', 20 + 2*pi*Rk*nl);
  case 'accel'
    seg = {'s', 75; 's', 100};
    marks(end+1) = struct('name', 'finish_75m', 's', 75);
  case 'autocross'
    % 닫힌 코스: 긴 변 두 개(각각 슬라럼 포함) + 반경 12 m 180° 끝 두 개. 같은 슬라럼을 양쪽 긴 변에
    % 넣어 횡 어긋남이 상쇄되므로 정확히 닫힌다. 길이 약 250 m. 실제 대회 코스가 아니다.
    slalom = {'a', [8 45]; 'a', [-8 90]; 'a', [8 90]; 'a', [-8 45]};
    seg = [{'s', 20}; slalom; {'s', 20; 'a', [12 90]; 's', 10; 'a', [12 90]; 's', 20}; slalom; ...
           {'s', 20; 'a', [12 90]; 's', 10; 'a', [12 90]}];
    closed = true;
  case 'segments'
    seg = opt.seg;
  otherwise
    error('vt_road:name', '모르는 도로: %s', name);
end
x = 0; y = 0; psi = 0; s = 0;
X = x; Y = y; PSI = psi; S = s; K = 0;
for i = 1:size(seg, 1)
  if seg{i, 1} == 's'
    L = seg{i, 2}; n = max(1, round(L / ds)); d = L / n;
    for k = 1:n
      x = x + d*cos(psi); y = y + d*sin(psi); s = s + d;
      X(end+1) = x; Y(end+1) = y; PSI(end+1) = psi; S(end+1) = s; K(end+1) = 0; %#ok<AGROW>
    end
  else
    Rr = seg{i, 2}(1); ang = seg{i, 2}(2) * pi/180; sg = sign(Rr); Rr = abs(Rr);
    L = Rr * ang; n = max(2, round(L / ds)); d = L / n; dpsi = sg * d / Rr;
    for k = 1:n
      psi_m = psi + dpsi/2;
      x = x + d*cos(psi_m); y = y + d*sin(psi_m); psi = psi + dpsi; s = s + d;
      X(end+1) = x; Y(end+1) = y; PSI(end+1) = psi; S(end+1) = s; K(end+1) = sg / Rr; %#ok<AGROW>
    end
  end
end
R.name = name; R.s = S(:); R.x = X(:); R.y = Y(:); R.psi = PSI(:); R.kappa = K(:);
R.width_m = opt.width_m; R.closed = closed; R.length_m = S(end); R.marks = marks;
if closed
  R.closure_gap_m = hypot(X(end) - X(1), Y(end) - Y(1));
end
end
