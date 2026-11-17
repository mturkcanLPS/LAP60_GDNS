"""Canli: LED blob alani vs yukseklik. infra1 goruntusu (~3 Hz) + en son tag tespitinden
kamera-huni mesafesi. Yuvarlak blob'lar (doluluk >= 0.6, en/boy <= 1.5) icinden dik ucgen
(1.36 m bacak) aranir; bulunursa 3 LED'in alani + optik Z + tag Z CSV'ye.
Kullanim: python3 -s led_kayit.py cikti.csv
"""
import sys, time, itertools, csv, numpy as np, cv2, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
FX, LEG, TH = 647.5, 1.36, 240
E = sorted([LEG, LEG, np.hypot(LEG, LEG)])
WALL_OFF = np.array([0, .3004, -.0959]); SMALL_OFF = {10: (.045, .0754, 0), 12: (-.045, .0754, 0), 14: (.045, -.0646, 0), 16: (-.045, -.0646, 0)}
def Rq(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
rclpy.init(); n = rclpy.create_node('led_kayit'); st = {'tagz': np.nan, 'tagt': 0, 'last': 0}
f = open(sys.argv[1], 'w', newline=''); W = csv.writer(f)
W.writerow(['t', 'tag_z', 'opt_z', 'tri_err', 'a1', 'a2', 'a3', 'fill_min', 'n_round', 'n_all'])
def tagcb(m):
    est = []
    for d in m.detections:
        p = d.pose.pose.pose.position; q = d.pose.pose.pose.orientation
        pv = np.array([p.x, p.y, p.z]); R = Rq((q.x, q.y, q.z, q.w))
        if d.id in (0, 20, 24, 34): est.append(pv + R @ WALL_OFF)
        elif d.id in SMALL_OFF: est.append(pv * .25 + R @ np.array(SMALL_OFF[d.id]))
    if est: st['tagz'] = float(np.median([e[2] for e in est])); st['tagt'] = time.time()
def imgcb(m):
    now = time.time()
    if now - st['last'] < 0.33: return
    st['last'] = now
    g = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
    _, b = cv2.threshold(g, TH, 255, cv2.THRESH_BINARY)
    k, L, S, C = cv2.connectedComponentsWithStats(b, connectivity=8)
    rnd = []
    for i in range(1, k):
        w, h, a = S[i, 2], S[i, 3], S[i, 4]
        fill, asp = a / (w * h), max(w, h) / min(w, h)
        if a >= 2 and fill >= 0.6 and asp <= 1.5: rnd.append((a, C[i][0], C[i][1], fill))
    rnd.sort(reverse=True); cand = rnd[:12]
    tz = st['tagz'] if now - st['tagt'] < 0.5 else np.nan
    best = None
    for tri in itertools.combinations(cand, 3):
        P = [np.array(t[1:3]) for t in tri]
        s = sorted(np.linalg.norm(P[i] - P[j]) for i, j in itertools.combinations(range(3), 2))
        if sum(s) < 3: continue
        kk = sum(s) / sum(E); err = np.mean([abs(s[q] - kk * E[q]) / (kk * E[q]) for q in range(3)])
        if not np.isnan(tz) and abs(FX / kk - tz) > 0.25 * tz: continue   # tag yuksekligiyle tutarsiz olcek
        if best is None or err < best[0]: best = (err, kk, tri)
    if best and best[0] < 0.12:
        err, kk, tri = best; a = sorted(t[0] for t in tri)
        W.writerow([f'{now:.2f}', f'{tz:.3f}', f'{FX / kk:.3f}', f'{err:.3f}', *a, f'{min(t[3] for t in tri):.2f}', len(rnd), k - 1])
    else:
        W.writerow([f'{now:.2f}', f'{tz:.3f}', '', '', '', '', '', '', len(rnd), k - 1])
    f.flush()
n.create_subscription(AprilTagDetectionArray, '/tag_detections', tagcb, qos_profile_sensor_data)
n.create_subscription(Image, '/camera/camera/infra1/image_rect_raw', imgcb, qos_profile_sensor_data)
rclpy.spin(n)
