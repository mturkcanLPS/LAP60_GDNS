"""Kucuk tag (10/12/14/16) dogrulamasi: canli bag'deki tespitlerden, AYNI karede duvar tag'leri de
gorulduyse duvar tahmini (kesin kalibre) referans alinip kucuk tag'lerin:
  - koddaki ofsetle tahmini ile duvar tahmini arasindaki fark (eksen boyu = z hizasi, yanal = isaret/ofset hatasi)
  - veriden olculen gercek ofseti  o_i = R_i^T (c_duvar - p_i)
Kullanim: python3 -s kucuk_an.py <bag_dizini>
"""
import sys, glob, sqlite3, collections, numpy as np
from rclpy.serialization import deserialize_message
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
NODE = 0.26
MINW = int(sys.argv[2]) if len(sys.argv) > 2 else 1   # en az bu kadar duvar tag'i olan kareler
META = {0: ((0, .3004, -.0959), .26), 20: ((0, .3004, -.0959), .26), 24: ((0, .3004, -.0959), .26), 34: ((0, .3004, -.0959), .26),
        10: ((+.045, +.0754, 0), .065), 12: ((-.045, +.0754, 0), .065), 14: ((+.045, -.0646, 0), .065), 16: ((-.045, -.0646, 0), .065)}
WALL, SMALL = (0, 20, 24, 34), (10, 12, 14, 16)
def Rq(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
db = sqlite3.connect(glob.glob(sys.argv[1] + '/*.db3')[0])
T = {r[1]: r[0] for r in db.execute('select id, name from topics')}
cnt = collections.Counter(); res = collections.defaultdict(list); oe = collections.defaultdict(list); dist = collections.defaultdict(list)
for cam, topic in (('i1', '/tag_detections'), ('i2', '/tag_detections_infra2')):
    for (data,) in db.execute('select data from messages where topic_id=?', (T[topic],)):
        m = deserialize_message(data, AprilTagDetectionArray)
        D = {}
        for x in m.detections:
            if x.id not in META: continue
            p = x.pose.pose.pose.position; q = x.pose.pose.pose.orientation
            off, size = META[x.id]
            pv = np.array([p.x, p.y, p.z]) * (size / NODE); R = Rq((q.x, q.y, q.z, q.w))
            if np.linalg.norm(pv) > 3: continue
            D[x.id] = (pv, R, pv + R @ np.array(off))
        for i in D: cnt[i] += 1
        w = [D[i][2] for i in WALL if i in D]
        if len(w) < MINW: continue
        cw = np.mean(w, axis=0)
        for i in SMALL:
            if i not in D: continue
            pv, R, est = D[i]
            ax = R[:, 2]                                    # tag z (tag'in icine) = huni ekseni asagi
            d = est - cw
            if np.linalg.norm(d) > 0.30: cnt[f'{i}_uzak'] += 1; continue   # sahte/yanlis eslesme
            res[i].append((d @ ax * 1000, np.linalg.norm(d - ax * (d @ ax)) * 1000, len(w)))
            oe[i].append(R.T @ (cw - pv)); dist[i].append(np.linalg.norm(pv))
print('tespit sayilari (X_uzak = duvar tahmininden >30 cm, analiz disi):', dict(sorted(cnt.items(), key=str)))
print('\nkucuk tag tahmini - duvar tahmini (ayni kare):  eksen boyu + = kucuk daha DERIN   | yanal | n | mesafe')
for i in SMALL:
    if not res[i]: print(f'  {i}: duvarla birlikte gorulmedi'); continue
    v = np.array(res[i])
    print(f'  {i}: eksen {np.median(v[:,0]):+6.1f} mm (IQR {np.percentile(v[:,0],25):+.0f}..{np.percentile(v[:,0],75):+.0f}) | yanal {np.median(v[:,1]):5.1f} mm | n={len(v)} | {np.median(dist[i]):.2f} m')
print('\nveriden olculen kucuk tag ofsetleri (tag frame, m) vs koddaki:')
for i in SMALL:
    if not oe[i]: continue
    o = np.median(np.array(oe[i]), axis=0)
    print(f'  {i}: olculen {np.round(o, 4)}   kod {np.array(META[i][0])}')
allz = [r[0] for i in SMALL for r in res[i]]
if allz: print(f'\nTUM kucuk tag: eksen farki medyan {np.median(allz):+.1f} mm -> kucuk tag z ofseti onerisi {-np.median(allz)/1000:+.4f} m '
               f'(referans huni dibi cemberi kalirsa)')
