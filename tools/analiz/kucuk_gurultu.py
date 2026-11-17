"""Kucuk tag gurultusu mesafeye gore (infra1 + infra2). Kare basina:
  W = duvar tag tahminlerinin ortalamasi, S = kucuk tag ortalamasi,
  F = node'daki gibi TUM tag'ler (median kapisi 0.4 m + ortalama), F0 = sadece duvar.
Olculenler (kamera optik frame'i, otopilota giden eksenler; x/y yanal, z derinlik):
  - S - W: sapma (medyan) ve sacilma (IQR/1.35)
  - F - F0: kucuk tag'lerin fuzyon ciktisini ne kadar kaydirdigi
  - kareden kareye oynama: F ve F0'in ardisik kare farklarinin std'si / sqrt2
Kullanim: python3 -s kucuk_gurultu.py <bag>
"""
import sys, glob, sqlite3, collections, numpy as np
from rclpy.serialization import deserialize_message
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
NODE = 0.26
META = {0: ((0, .3004, -.0959), .26), 20: ((0, .3004, -.0959), .26), 24: ((0, .3004, -.0959), .26), 34: ((0, .3004, -.0959), .26),
        10: ((+.045, +.0754, 0), .065), 12: ((-.045, +.0754, 0), .065), 14: ((+.045, -.0646, 0), .065), 16: ((-.045, -.0646, 0), .065)}
WALL, SMALL = (0, 20, 24, 34), (10, 12, 14, 16)
def Rq(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
def fuse(E):
    if not E: return None
    E = np.array(E); med = np.median(E, axis=0)
    k = np.linalg.norm(E - med, axis=1) <= 0.4
    return E[k].mean(axis=0)
def rs(v): return (np.percentile(v, 75) - np.percentile(v, 25)) / 1.349 if len(v) > 3 else np.nan

db = sqlite3.connect(glob.glob(sys.argv[1] + '/*.db3')[0])
T = {r[1]: r[0] for r in db.execute('select id, name from topics')}
rows = collections.defaultdict(list)   # cam -> (t, dist, S-W, F-F0, F, F0, ns)
for cam, topic in (('i1', '/tag_detections'), ('i2', '/tag_detections_infra2')):
    for (data,) in db.execute('select data from messages where topic_id=? order by timestamp', (T[topic],)):
        m = deserialize_message(data, AprilTagDetectionArray)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        D = {}
        for x in m.detections:
            if x.id not in META: continue
            p = x.pose.pose.pose.position; q = x.pose.pose.pose.orientation
            off, size = META[x.id]
            pv = np.array([p.x, p.y, p.z]) * (size / NODE)
            D[x.id] = pv + Rq((q.x, q.y, q.z, q.w)) @ np.array(off)
        w = [D[i] for i in WALL if i in D]; s = [D[i] for i in SMALL if i in D]
        if len(w) < 2 or not s: continue   # duvar referansi saglam olsun
        W, S = np.mean(w, axis=0), np.mean(s, axis=0)
        F, F0 = fuse(list(D.values())), fuse(w)
        if np.linalg.norm(S - W) > 0.5: continue
        rows[cam].append((t, np.linalg.norm(F0), S - W, F - F0, F, F0, len(s)))

BINS = ((1.0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 3.0), (3.0, 4.0))
for cam in ('i1', 'i2'):
    R = rows[cam]
    if not R: continue
    t = np.array([r[0] for r in R]); d = np.array([r[1] for r in R])
    SW = np.array([r[2] for r in R]) * 1000; FF = np.array([r[3] for r in R]) * 1000
    F = np.array([r[4] for r in R]) * 1000; F0 = np.array([r[5] for r in R]) * 1000
    ns = np.array([r[6] for r in R])
    dt = np.diff(t); cons = (dt > 0) & (dt < 0.2)          # ardisik kareler
    dF, dF0 = np.diff(F, axis=0), np.diff(F0, axis=0)
    print(f'\n=== {cam} ({len(R)} kare, duvar>=2 + kucuk ayni karede) — mm ===')
    print('mesafe   | n    kucuk | KUCUK-DUVAR sapma (x,y,z) | sacilma (x,y,z) | FUZYON kaymasi (x,y,z) | kare-kare oynama z: tum / sadece duvar | xy: tum / duvar')
    for lo, hi in BINS:
        k = (d >= lo) & (d < hi)
        if k.sum() < 20: continue
        kk = cons & k[1:] & k[:-1]
        jz = np.std(dF[kk, 2]) / np.sqrt(2); jz0 = np.std(dF0[kk, 2]) / np.sqrt(2)
        jxy = np.std(np.linalg.norm(dF[kk, :2], axis=1)) / np.sqrt(2); jxy0 = np.std(np.linalg.norm(dF0[kk, :2], axis=1)) / np.sqrt(2)
        md = np.median(SW[k], axis=0); sc = [rs(SW[k, j]) for j in range(3)]; mf = np.median(FF[k], axis=0)
        print(f'{lo:.1f}-{hi:.1f} m | {k.sum():5d} {ns[k].mean():.1f} | {md[0]:+5.0f} {md[1]:+5.0f} {md[2]:+5.0f} | {sc[0]:4.0f} {sc[1]:4.0f} {sc[2]:4.0f} | '
              f'{mf[0]:+5.1f} {mf[1]:+5.1f} {mf[2]:+6.1f} | {jz:5.1f} / {jz0:5.1f} | {jxy:4.1f} / {jxy0:4.1f}')
