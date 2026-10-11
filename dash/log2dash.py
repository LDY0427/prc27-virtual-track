# CAN 로그 CSV(PRCLOG-CSV v0) → 대시보드 JSON. Octave 없이도 실행 가능 (Python 3만 필요).
# 실제 로거 로그에도 그대로 쓴다: python log2dash.py 로그.csv [이름] ...  (DBC는 ../dbc/PRC27_sim_v0.1.dbc)
import csv, json, math, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
DBC = os.path.join(HERE, '..', 'dbc', 'PRC27_sim_v0.1.dbc')

# ---- 차량 파라미터 (prc27_params.m과 같은 값; 토크 곡선·기어비) ----
RPM_TAB = [1500, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 9500, 10000, 10500]
TRQ_TAB = [18, 25, 29, 32, 35, 37, 36.2, 34.9, 33.0, 30.0, 26.0]
RPM_LIMIT, SHIFT_RPM, IDLE = 10500, 10000, 1600
FRIC_A, FRIC_B = 3.0, 0.6
GEARS = [80/30 * g * 4.0 for g in (32/12, 26/14, 27/19, 24/21, 22/23, 21/25)]
TIRE_R, MASS = 0.254, 280


def interp(xs, ys, x):
    if x <= xs[0]: return ys[0]
    if x >= xs[-1]: return ys[-1]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            t = (x - xs[i-1]) / (xs[i] - xs[i-1]); return ys[i-1] + t * (ys[i] - ys[i-1])
    return ys[-1]


def engine_torque(rpm, tps):
    twot = interp(RPM_TAB, TRQ_TAB, min(max(rpm, RPM_TAB[0]), RPM_TAB[-1]))
    fric = FRIC_A + FRIC_B * rpm / 1000
    if rpm >= RPM_LIMIT: return -fric
    return tps * twot - (1 - tps) * fric


# ---- 최소 DBC 파서 (Intel 신호) ----
def load_dbc(path):
    msgs = {}
    cur = None
    for line in open(path, encoding='utf-8'):
        s = line.strip()
        m = re.match(r'^BO_\s+(\d+)\s+(\w+)\s*:', s)
        if m:
            cur = {'name': m.group(2), 'sigs': []}; msgs[int(m.group(1))] = cur; continue
        m = re.match(r'^SG_\s+(\w+)\s*:\s*(\d+)\|(\d+)@([01])([+-])\s*\(([^,]+),([^)]+)\)\s*\[([^|]*)\|([^\]]*)\]', s)
        if m and cur:
            cur['sigs'].append(dict(name=m.group(1), start=int(m.group(2)), ln=int(m.group(3)), intel=m.group(4) == '1',
                                    signed=m.group(5) == '-', f=float(m.group(6)), o=float(m.group(7)),
                                    mn=float(m.group(8)), mx=float(m.group(9))))
    return msgs


def decode(sig, data):
    raw = 0
    if sig['intel']:
        v = int.from_bytes(bytes(data), 'little'); raw = (v >> sig['start']) & ((1 << sig['ln']) - 1)
    else:
        raise NotImplementedError('Motorola')
    if sig['signed'] and raw >= 1 << (sig['ln'] - 1): raw -= 1 << sig['ln']
    val = raw * sig['f'] + sig['o']
    if sig['mx'] > sig['mn'] and (val < sig['mn'] - 1e-9 or val > sig['mx'] + 1e-9): return None
    return val


def read_log(path, msgs):
    S = {}
    with open(path, encoding='utf-8') as f:
        for row in csv.reader(f):
            if not row or row[0].startswith('#') or row[0] == 't_us': continue
            t = int(row[0]) / 1e6; mid = int(row[2]); data = [int(x) for x in row[4:12]]
            if mid not in msgs: continue
            m = msgs[mid]; d = S.setdefault(m['name'], {'t': []})
            d['t'].append(t)
            for sg in m['sigs']: d.setdefault(sg['name'], []).append(decode(sg, data))
    return S


def hold(ts, xs, tq):
    out, j, last = [], 0, None
    for t in tq:
        while j < len(ts) and ts[j] <= t:
            if xs[j] is not None: last = xs[j]
            j += 1
        out.append(last)
    return out


def road(name):
    seg = {'skidpad': [('s', 20), ('a', (9.125, 360 * 8)), ('s', 20)],
           'autocross': [('s', 20)] + SLALOM + [('s', 20), ('a', (12, 90)), ('s', 10), ('a', (12, 90)), ('s', 20)] + SLALOM + [('s', 20), ('a', (12, 90)), ('s', 10), ('a', (12, 90))]}[name]
    x = y = psi = 0.0; X, Y, PSI = [0.0], [0.0], [0.0]; ds = 1.0
    for kind, v in seg:
        if kind == 's':
            n = max(1, round(v / ds)); d = v / n
            for _ in range(n): x += d * math.cos(psi); y += d * math.sin(psi); X.append(x); Y.append(y); PSI.append(psi)
        else:
            R, ang = v; sg = 1 if R > 0 else -1; R = abs(R); L = R * math.radians(ang); n = max(2, round(L / ds)); d = L / n; dpsi = sg * d / R
            for _ in range(n):
                pm = psi + dpsi / 2; x += d * math.cos(pm); y += d * math.sin(pm); psi += dpsi; X.append(x); Y.append(y); PSI.append(psi)
    return {'x': [round(v, 2) for v in X], 'y': [round(v, 2) for v in Y], 'psi': [round(v, 3) for v in PSI], 'width': 3.5, 'closed': name == 'autocross'}

SLALOM = [('a', (8, 45)), ('a', (-8, 90)), ('a', (8, 90)), ('a', (-8, 45))]


def convert(path, name, dt=0.05):
    msgs = load_dbc(DBC); S = read_log(path, msgs)
    E0, E1, W, TC, CH, IA, IG, GP = (S.get(k, {'t': []}) for k in ('EMU_SIM_0', 'EMU_SIM_1', 'WSS_Speed', 'WSS_TCS', 'DAQ_Chassis', 'DAQ_IMU_Acc', 'DAQ_IMU_Gyro', 'DAQ_GNSS_Pos'))
    t0 = W['t'][0]; t1 = W['t'][-1]
    tq = [t0 + i * dt for i in range(int((t1 - t0) / dt))]
    g = lambda M, k: hold(M['t'], M[k], tq) if k in M else [None] * len(tq)
    rpm = g(E0, 'RPM'); tps = [v / 100 if v is not None else 0 for v in g(E0, 'TPS')]; gear = g(E1, 'Gear')
    cut = [v / 100 if v is not None else 0 for v in g(E1, 'TcCutPct')]
    w = [g(W, k) for k in ('WssFL', 'WssFR', 'WssRL', 'WssRR')]
    vref = g(TC, 'VehSpeedRef')
    sw = g(CH, 'SteerAngle'); pF = g(CH, 'BrkPressF')
    ax = g(IA, 'AccX'); ay = g(IA, 'AccY'); yaw = g(IG, 'GyroZ')
    lat = g(GP, 'Latitude'); lon = g(GP, 'Longitude')
    lat0, lon0 = 35.23, 129.08
    x = [((lo - lon0) * 111320 * math.cos(math.radians(lat0))) if lo is not None else 0 for lo in lon]
    y = [((la - lat0) * 111320) if la is not None else 0 for la in lat]
    n = len(tq); nz = lambda a, d=0: [d if v is None else v for v in a]
    rpm = nz(rpm, IDLE); vref = nz(vref); sw = nz(sw); pF = nz(pF); ax = nz(ax); ay = nz(ay); yaw = nz(yaw); gear = nz(gear)
    w = [nz(a) for a in w]
    tq_e = [engine_torque(rpm[i], tps[i]) * (1 - cut[i]) for i in range(n)]
    v_kmh = [max(w[0][i], w[1][i]) if vref[i] == 0 else vref[i] for i in range(n)]   # 기준 차속(전륜) 없으면 전륜 최대
    slip4 = [[(w[k][i] - v_kmh[i]) / max(v_kmh[i], 10) for k in range(4)] for i in range(n)]
    # 거리 s: 속도 적분
    s, acc = [], 0.0
    for i in range(n): acc += v_kmh[i] / 3.6 * dt; s.append(acc)
    r = lambda a, d=3: [round(v, d) for v in a]
    return {'name': name, 'dt': dt, 't': r([t - t0 for t in tq], 2), 'v_kmh': r(v_kmh, 1), 'rpm': r(rpm, 0), 'gear': r(gear, 0), 'tps': r(tps),
            'pF': r(pF, 1), 'sw_deg': r(sw, 1), 'ax_g': r(ax), 'ay_g': r(ay), 'yaw_dps': r(yaw, 1),
            'slip4': [r(v) for v in slip4], 'Fz4': [[None] * 4] * n, 'tcs_cut': r(cut), 'x': r(x, 2), 'y': r(y, 2), 's': r(s, 1),
            'torque_Nm': r(tq_e, 1), 'power_kW': r([tq_e[i] * rpm[i] * 2 * math.pi / 60 / 1000 for i in range(n)], 1),
            'road': road(name), 'lap_time_s': None, 'qss_time_s': None,
            'source': os.path.basename(path), 'note': 'CAN 로그에서 변환: 위치는 GNSS, 토크는 rpm·스로틀로 추정, 수직하중 없음'}


if __name__ == '__main__':
    out_dir = os.path.join(HERE, '..', 'out')
    jobs = [(os.path.join(out_dir, 'vt_skidpad.csv'), 'skidpad'), (os.path.join(out_dir, 'vt_autocross.csv'), 'autocross')]
    if len(sys.argv) > 1:
        jobs = [(sys.argv[i], sys.argv[i + 1] if i + 1 < len(sys.argv) else os.path.basename(sys.argv[i])) for i in range(1, len(sys.argv), 2)]
    runs = [convert(p, nm) for p, nm in jobs if os.path.exists(p)]
    rpm_grid = list(range(1500, 10501, 50))
    out = {'generated': 'log2dash.py', 'note': 'SIMULATED DATA converted from CAN logs. Not measured.',
           'dyno': {'rpm': rpm_grid, 'torque': [round(engine_torque(r, 1.0), 2) for r in rpm_grid],
                    'kW': [round(engine_torque(r, 1.0) * r * 2 * math.pi / 60 / 1000, 2) for r in rpm_grid], 'rpm_limit': RPM_LIMIT, 'shift_rpm': SHIFT_RPM},
           'params': {'mass_kg': MASS, 'final': 4.0, 'gears': GEARS, 'tire_r_m': TIRE_R, 'sw_max_deg': 126}, 'runs': runs}
    dst = os.path.join(out_dir, 'dash_data.json')
    json.dump(out, open(dst, 'w', encoding='utf-8'), ensure_ascii=False, separators=(',', ':'))
    print(dst, os.path.getsize(dst), [(r['name'], len(r['t'])) for r in runs])
