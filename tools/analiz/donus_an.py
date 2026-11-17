"""Vincle kaldirma testi: kameranin huniye gore egimi (roll/pitch) ve yaw'u + bu acilarda
kucuk tag / duvar tag tahminlerinin tutarliligi. Kayit BITTIKTEN sonra calistir.
Kullanim: python3 -s donus_an.py <bag_dizini>

Huni ekseni (asagi) kamera frame'inde: kucuk tag'in z'si (tag'in icine = asagi) ya da
duvar tag'inde R @ (0, .708, .706). Egim = eksenin kamera z'siyle acisi;
roll ~ atan2(a_x, a_z), pitch ~ atan2(a_y, a_z) (kamera optik frame'i, x sag, y asagi).
Yaw = kucuk tag x ekseninin goruntu duzlemindeki acisi; duvar tag'lerinde ID basina sabit
fark ayni karelerden ogrenilir (90 derece adimlar + montaj).
"""
import sys, glob, sqlite3, collections, numpy as np
from rclpy.serialization import deserialize_message
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
NODE = 0.26
META = {0: ((0, .3004, -.0959), .26), 20: ((0, .3004, -.0959), .26), 24: ((0, .3004, -.0959), .26), 34: ((0, .3004, -.0959), .26),
        10: ((+.045, +.0754, 0), .065), 12: ((-.045, +.0754, 0), .065), 14: ((+.045, -.0646, 0), .065), 16: ((-.045, -.0646, 0), .065)}
WALL, SMALL = (0, 20, 24, 34), (10, 12, 14, 16)
AXW = np.array([0, .708, .706])
def Rq(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
def wrap(a): return (a + 180) % 360 - 180
def pct(v, p): return np.percentile(v, p) if len(v) else float('nan')

db = sqlite3.connect(glob.glob(sys.argv[1] + '/*.db3')[0])
T = {r[1]: r[0] for r in db.execute('select id, name from topics')}
F = []   # kare: (cam, t, {id: (pv, R, est)})
for cam, topic in (('i1', '/tag_detections'), ('i2', '/tag_detections_infra2')):
    if topic not in T: continue
    for (data,) in db.execute('select data from messages where topic_id=? order by timestamp', (T[topic],)):
        m = deserialize_message(data, AprilTagDetectionArray)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        D = {}
        for x in m.detections:
            if x.id not in META: continue
            p = x.pose.pose.pose.position; q = x.pose.pose.pose.orientation
            off, size = META[x.id]
            pv = np.array([p.x, p.y, p.z]) * (size / NODE); R = Rq((q.x, q.y, q.z, q.w))
            if np.linalg.norm(pv) > 4: continue
            D[x.id] = (pv, R, pv + R @ np.array(off))
        F.append((cam, t, D))
print(f'kare: {len(F)}  (bos olmayan {sum(1 for f in F if f[2])})')

# --- 1) duvar ID'si basina yaw farki (kucuk tag'le ayni karelerden) ---
def yaw_small(D):
    ys = [np.degrees(np.arctan2(D[i][1][1, 0], D[i][1][0, 0])) for i in SMALL if i in D]
    return np.degrees(np.angle(np.mean(np.exp(1j * np.radians(ys))))) if ys else None
def yaw_wall_raw(R): return np.degrees(np.arctan2(R[1, 0], R[0, 0]))
dw = collections.defaultdict(list)
for cam, t, D in F:
    ys = yaw_small(D)
    if ys is None: continue
    for i in WALL:
        if i in D: dw[i].append(wrap(yaw_wall_raw(D[i][1]) - ys))
woff = {i: np.degrees(np.angle(np.mean(np.exp(1j * np.radians(v))))) for i, v in dw.items() if len(v) >= 5}
print('duvar yaw farki (kucuk tag yaw\'una gore, ayni kare):', {i: f'{v:+.1f} (n={len(dw[i])}, std {np.std(dw[i]):.1f})' for i, v in woff.items()})

# --- 2) kare basina tutum + kucuk/duvar farki ---
rows = []   # cam, t, egim, roll, pitch, yaw, mesafe, eksen farki mm, yanal fark mm, nw, ns
for cam, t, D in F:
    ax = [D[i][1][:, 2] for i in SMALL if i in D] + [D[i][1] @ AXW for i in WALL if i in D]
    if not ax: continue
    a = np.mean(ax, axis=0); a /= np.linalg.norm(a)
    tilt = np.degrees(np.arccos(np.clip(a[2], -1, 1)))
    roll, pitch = np.degrees(np.arctan2(a[0], a[2])), np.degrees(np.arctan2(a[1], a[2]))
    yaw = yaw_small(D)
    if yaw is None:
        yw = [wrap(yaw_wall_raw(D[i][1]) - woff[i]) for i in WALL if i in D and i in woff]
        yaw = np.degrees(np.angle(np.mean(np.exp(1j * np.radians(yw))))) if yw else np.nan
    est = [D[i][2] for i in D]
    dist = np.linalg.norm(np.mean(est, axis=0))
    s = [D[i][2] for i in SMALL if i in D]; w = [D[i][2] for i in WALL if i in D]
    da = dl = np.nan
    if s and w:
        d = np.mean(s, axis=0) - np.mean(w, axis=0)
        if np.linalg.norm(d) < 0.30:
            da = d @ a * 1000; dl = np.linalg.norm(d - a * (d @ a)) * 1000
    rows.append((cam, t, tilt, roll, pitch, yaw, dist, da, dl, len(w), len(s)))
A = np.array([r[1:] for r in rows], dtype=float); cams = np.array([r[0] for r in rows])
t, tilt, roll, pitch, yaw, dist, da, dl, nw, ns = A.T
print(f'\nTUTUM (huni eksenine gore, {len(A)} kare):')
print(f'  egim  medyan {np.median(tilt):.1f}  p90 {pct(tilt, 90):.1f}  max {tilt.max():.1f} derece')
print(f'  roll  {pct(roll, 5):+.1f}..{pct(roll, 95):+.1f}   pitch {pct(pitch, 5):+.1f}..{pct(pitch, 95):+.1f}  (p5..p95)')
yv = yaw[~np.isnan(yaw)]
if len(yv): print(f'  yaw   {pct(yv, 5):+.0f}..{pct(yv, 95):+.0f} (p5..p95, -180..180 sarar)')
print(f'  mesafe {pct(dist, 5):.2f}..{pct(dist, 95):.2f} m')

# yaw hizi (infra1, ardisik kareler)
m1 = (cams == 'i1') & ~np.isnan(yaw)
if m1.sum() > 2:
    tt, yy = t[m1], yaw[m1]; dt = np.diff(tt); ok = (dt > 0) & (dt < 0.5)
    rate = np.abs(wrap(np.diff(yy))[ok] / dt[ok])
    print(f'  yaw hizi medyan {np.median(rate):.0f}  p90 {pct(rate, 90):.0f}  max {rate.max():.0f} derece/s')

print('\nKUCUK - DUVAR tahmini (ayni kare): egime gore')
ok = ~np.isnan(da)
for lo, hi in ((0, 5), (5, 10), (10, 20), (20, 90)):
    k = ok & (tilt >= lo) & (tilt < hi)
    if k.sum() < 5: print(f'  egim {lo:2d}-{hi:2d}: az veri ({k.sum()})'); continue
    print(f'  egim {lo:2d}-{hi:2d}: n={k.sum():5d} | yanal medyan {np.median(dl[k]):5.1f} p90 {pct(dl[k], 90):5.1f} mm | eksen {np.median(da[k]):+6.1f} mm')
print('KUCUK - DUVAR: mesafeye gore')
for lo, hi in ((0, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 4)):
    k = ok & (dist >= lo) & (dist < hi)
    if k.sum() < 5: continue
    print(f'  {lo:.1f}-{hi:.1f} m: n={k.sum():5d} | yanal medyan {np.median(dl[k]):5.1f} p90 {pct(dl[k], 90):5.1f} mm | eksen {np.median(da[k]):+6.1f} mm')

# tespit orani: yaw hizina gore (bulaniklik)
print('\nTESPIT: duvar/kucuk tag sayisi (infra1)')
for lo, hi in ((0, 10), (10, 30), (30, 90), (90, 1e9)):
    if m1.sum() <= 2: break
    k = np.zeros(len(t), bool); idx = np.where(m1)[0][1:][ok_ := ((np.diff(t[m1]) > 0) & (np.diff(t[m1]) < 0.5))]
    r = np.abs(wrap(np.diff(yaw[m1]))[ok_] / np.diff(t[m1])[ok_])
    sel = idx[(r >= lo) & (r < hi)]
    if len(sel) < 5: continue
    print(f'  yaw hizi {lo:3.0f}-{hi if hi < 1e8 else 999:3.0f} d/s: n={len(sel):5d} | duvar ort {nw[sel].mean():.2f} | kucuk ort {ns[sel].mean():.2f}')
