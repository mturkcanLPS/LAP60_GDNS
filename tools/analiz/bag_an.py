"""Canli kalibrasyon bag'i analizi (canli node'un TAM cozunurluk tespitleri).
1) Cift-fit: ayni karede iki duvar tag'i -> ortak ofset; kamera x mesafe bandi; yaricap, koni acisi.
2) Stereo olcek: ayni cekim anindaki infra1/infra2 ayni-ID tespiti -> (p1-p2).x = (1+eps)*baseline.
Kullanim: python3 -s bag_an.py <bag_dizini> [cikti.json]
"""
import sys, glob, sqlite3, itertools, collections, json, numpy as np
from rclpy.serialization import deserialize_message
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo
db = sqlite3.connect(glob.glob(sys.argv[1] + '/*.db3')[0])
T = {r[1]: r[0] for r in db.execute('select id, name from topics')}
def Rq(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
ci = deserialize_message(db.execute('select data from messages where topic_id=? limit 1', (T['/camera/camera/infra2/camera_info'],)).fetchone()[0], CameraInfo)
print('infra2 K', np.round(ci.k, 2).tolist(), 'P[3]', round(ci.p[3], 3))
F = {'i1': [], 'i2': []}; RAW = {'i1': [], 'i2': []}
for cam, topic in (('i1', '/tag_detections'), ('i2', '/tag_detections_infra2')):
    for (data,) in db.execute('select data from messages where topic_id=? order by timestamp', (T[topic],)):
        m = deserialize_message(data, AprilTagDetectionArray); st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        d = {}; raw = []
        for x in m.detections:
            p = x.pose.pose.pose.position; q = x.pose.pose.pose.orientation
            raw.append({'id': int(x.id), 'p': [p.x, p.y, p.z], 'q': [q.x, q.y, q.z, q.w]})
            if x.id in (0, 20, 24, 34): d[x.id] = (np.array([p.x, p.y, p.z]), Rq((q.x, q.y, q.z, q.w)))
        F[cam].append((st, d)); RAW[cam].append((st, raw))
if len(sys.argv) > 2: json.dump(RAW, open(sys.argv[2], 'w'))
for c in F: print(f"{c}: {len(F[c])} kare, duvar tagli {sum(1 for _, d in F[c] if d)}, >=2 {sum(1 for _, d in F[c] if len(d) >= 2)}, "
                  f"id {dict(sorted(collections.Counter(i for _, d in F[c] for i in d).items()))}")
oF = np.array([0, .3013, -.1092]); oC = np.array([0, .2948, -.1027])
def fit(rows):
    A = np.vstack([r[1] - r[3] for r in rows]); b = np.concatenate([-(r[0] - r[2]) for r in rows])
    U, s, Vt = np.linalg.svd(A, full_matrices=False); a = Vt[-1] * np.sign(Vt[-1][1])
    c = U.T @ (b - A @ oF); o = oF + sum((c[i] / s[i]) * Vt[i] for i in range(2))
    r = lambda o: np.median([np.linalg.norm((x[0] + x[1] @ o) - (x[2] + x[3] @ o)) for x in rows]) * 1000
    return o, np.linalg.norm(o - a * (o @ a)) * 1000, np.degrees(np.arctan2(a[2], a[1])), r(o), r(oF), r(oC)
bands = [(0.3, 0.85), (0.85, 0.95), (0.95, 1.05), (1.05, 1.2), (1.2, 9)]
G = collections.defaultdict(list); ang = collections.defaultdict(list)
for cam in F:
    for _, d in F[cam]:
        for (ia, (pa, Ra)), (ib, (pb, Rb)) in itertools.combinations(sorted(d.items()), 2):
            dist = (np.linalg.norm(pa) + np.linalg.norm(pb)) / 2
            if dist > 3: continue
            row = (pa, Ra, pb, Rb)
            for lo, hi in bands:
                if lo <= dist < hi: G[(cam, f'{lo:.2f}-{hi:.2f}')].append(row); G[('**', f'{lo:.2f}-{hi:.2f}')].append(row)
            G[(cam, 'hepsi')].append(row); G[('**', 'hepsi')].append(row)
            c = (pa + Ra @ oC + pb + Rb @ oC) / 2
            ang[(ia, ib)].append(((np.degrees(np.arctan2(pb[1]-c[1], pb[0]-c[0]) - np.arctan2(pa[1]-c[1], pa[0]-c[0])) + 180) % 360) - 180)
print('yon (b-a, saat yonu +):', {k: round(float(np.median(v)), 1) for k, v in sorted(ang.items())})
print(f"\n1) CIFT-FIT {'grup':22s} {'n':>5s}  {'fit ofset':26s} yaricap  koni   cift farki fit / Fable / mevcut (mm)")
for g in sorted(G):
    if len(G[g]) < 20: continue
    o, rad, ha, rf, rF, rC = fit(G[g])
    print(f"   {str(g):30s} {len(G[g]):5d}  {str(np.round(o, 4)):26s} {rad:6.1f}  {ha:5.2f}   {rf:5.1f} / {rF:5.1f} / {rC:5.1f}")
base = -ci.p[3] / ci.p[0]
print(f'\n2) STEREO OLCEK (baseline {base*1000:.2f} mm): eps>0 = pozlar gercekten UZAK')
i2 = {round(st, 3): d for st, d in F['i2']}; out = collections.defaultdict(list)
for st, d in F['i1']:
    d2 = i2.get(round(st, 3))
    if not d2: continue
    for tid in set(d) & set(d2):
        p1, _ = d[tid]; p2, _ = d2[tid]; dist = np.linalg.norm(p1)
        if dist > 3: continue
        for lo, hi in bands + [(0, 3)]:
            if lo <= dist < hi: out[(lo, hi)].append(((p1 - p2)[0] / base - 1, (p1 - p2)[1] * 1000, (p1 - p2)[2] * 1000))
for k in bands + [(0, 3)]:
    if len(out[k]) < 10: continue
    v = np.array(out[k])
    print(f'   {k[0]:.2f}-{k[1]:.2f} m n={len(v):5d}  eps medyan {np.median(v[:,0])*100:+.2f}% (IQR {np.percentile(v[:,0],25)*100:+.1f}..{np.percentile(v[:,0],75)*100:+.1f})  dy {np.median(v[:,1]):+.1f} mm  dz {np.median(v[:,2]):+.1f} mm')
