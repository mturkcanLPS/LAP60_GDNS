"""Kutu merkezi -> huni merkezi ofseti (metre, kamera ekseni), overlay'li eski kayitlardan.
EO videosundaki 'px=(u,v) bx= by=' yazisi (gonderilen bearing) + infra1 videosundaki tag 'pos x y z'.
Ayni andaki (|dt| <= 0.2 s) ciftlerden: ofset = (tag_yon - bearing) * z.
Kullanim: python3 -s ofset_olc.py <oturum no, orn 0175>
"""
import sys, glob, csv, io, cv2, numpy as np
sys.path.insert(0, '/home/lapis/huni_kalib')
import hud_oku as h
F = cv2.FONT_HERSHEY_SIMPLEX
def w55(t): return cv2.getTextSize(t, F, 0.55, 1)[0][0]
_c = {}
def r55(t):
    if t not in _c:
        if len(_c) > 3000: _c.clear()
        im = np.zeros((26, 640), np.uint8); cv2.putText(im, t, (10, 18), F, 0.55, 255, 1, cv2.LINE_AA); _c[t] = im.astype(np.float32)
    return _c[t]
def readseq(band, pre, cands, stop, maxn=9):
    out = ''
    for _ in range(maxn):
        base = 10 + w55(pre); best = None
        for c in cands:
            adv = w55(pre + c) - w55(pre); x0, x1 = base - 1, base + adv + 1
            patch = band[:, x0:x1]
            if c == ' ': s = 0.45 if band[3:24, base + 1:base + adv - 1].max() < 150 else -1
            else:
                t = r55(pre + c)[:, x0:x1] - r55(pre)[:, x0:x1]; tm, pm = t - t.mean(), patch - patch.mean()
                s = float((tm * pm).sum() / (np.linalg.norm(tm) * np.linalg.norm(pm) + 1e-6))
            if best is None or s > best[0]: best = (s, c)
        if best[1] in stop: return out, pre + best[1]
        out += best[1]; pre += best[1]
    return out, pre
def eo_read(img):
    g = img[:46].max(axis=2).astype(np.float32); band = g[20:46]
    u, pre = readseq(band, 'px=(', list('0123456789,'), ',')
    v, pre = readseq(band, pre, list('0123456789)'), ')')
    pre += '  bx='
    bx, pre = readseq(band, pre, list('0123456789-. '), ' ')
    pre += 'by='
    by, pre = readseq(band, pre, list('0123456789-. '), ' ')
    try: return float(u), float(v), float(bx), float(by)
    except ValueError: return None
s = sys.argv[1]; D = '/home/lapis/lps_records/'
txt = open(glob.glob(D + f'ir_s{s}_*.csv')[0], 'rb').read().replace(b'\x00', b'').decode('utf-8', 'ignore')
R = [r for r in csv.DictReader(io.StringIO(txt)) if r.get('t_ms') and r.get('z_radar') not in (None, '')]
T = float(R[-1]['t_ms']) / 1000
tc = np.array([float(r['t_ms']) for r in R]) / 1000; zc = np.array([float(r['z_radar']) for r in R])
def nframes(f):
    cap = cv2.VideoCapture(f); n = 0
    while cap.grab(): n += 1
    return n
fe, fi = glob.glob(D + f'eo_s{s}_*.avi')[0], glob.glob(D + f'lps_infra1_s{s}_*.avi')[0]
NE, NI = nframes(fe), nframes(fi)
eo = []; cap = cv2.VideoCapture(fe); i = 0
while cap.grab():
    if i % 3 == 0:
        _, img = cap.retrieve(); g = img[50:].astype(int)
        if ((g[..., 1] > 200) & (g[..., 2] < 90) & (g[..., 0] < 90)).sum() > 150:
            body = img[60:, :480].astype(int)
            ir = np.median(np.abs(body[..., 2] - body[..., 1]) + np.abs(body[..., 1] - body[..., 0])) < 2
            r = eo_read(img)
            if r and abs(r[2]) < 1.5 and abs(r[3]) < 1.5: eo.append((i / NE * T, 1 if ir else 0) + r)
    i += 1
tg = []; cap = cv2.VideoCapture(fi); i = 0
while cap.grab():
    if i % 2 == 0:
        _, img = cap.retrieve(); roi = img[6:20, 6:110].astype(int)
        if ((roi > 190).all(axis=2)).sum() > ((roi[..., 2] > 170) & (roi[..., 1] < 90) & (roi[..., 0] < 90)).sum():
            r = h.read_frame(img)
            if r[1] == 1 and r[4] > 0.5 and not np.isnan(r[2] + r[3]): tg.append((i / NI * T, r[2] + 0.032962, r[3] + 0.045, r[4]))
    i += 1
eo, tg = np.array(eo), np.array(tg)
print(f's{s}: sure {T:.0f} s | EO kilitli okunan {len(eo)} | tag okunan {len(tg)}')
rows = []
if len(eo) and len(tg):
    for e in eo:
        j = np.argmin(np.abs(tg[:, 0] - e[0]))
        if abs(tg[j, 0] - e[0]) > 0.2: continue
        _, x, y, z = tg[j]
        rows.append((e[0], e[1], z, e[4], e[5], x / z, y / z, (x / z - e[4]) * z, (y / z - e[5]) * z,
                     zc[np.searchsorted(tc, e[0]).clip(0, len(zc) - 1)]))
R = np.array(rows); np.save(f'/home/lapis/huni_kalib/ofset{s}.npy', R)
print('eslesen cift', len(R))
if len(R):
    ox, oy = R[:, 7], R[:, 8]
    print(f'  tag z {R[:,2].min():.1f}-{R[:,2].max():.1f} m | kanal IR %{100*R[:,1].mean():.0f}')
    print(f'  OFSET x medyan {np.median(ox):+.2f} (p10 {np.percentile(ox,10):+.2f} p90 {np.percentile(ox,90):+.2f}) | y medyan {np.median(oy):+.2f} (p10 {np.percentile(oy,10):+.2f} p90 {np.percentile(oy,90):+.2f}) m')
    for lo, hi in ((2, 4), (4, 6), (6, 8), (8, 10), (10, 14)):
        k = (R[:, 2] >= lo) & (R[:, 2] < hi)
        if k.sum() >= 3: print(f'    tag z {lo}-{hi} m: n={k.sum():3d} | x {np.median(ox[k]):+.2f} y {np.median(oy[k]):+.2f} | sacilma x {np.std(ox[k]):.2f} y {np.std(oy[k]):.2f} | EKF irtifa medyan {np.median(R[k,9]):.1f}')
