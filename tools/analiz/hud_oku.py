"""lps_infra1/infra2 videolarinin ust yazisini (HUD) kare kare okur -> npy.
Yazi bicimi debug_recorder.hpp'den biliniyor (Hershey 0.42, {6,17} ve {6,36}); karakterler
sirayla, bilinen x konumunda aday sablonlarla eslestirilir (OCR degil, sablon eslestirme).
Kullanim: python3 -s hud_oku.py video.avi cikti.npy [adim]
Sutunlar: kare, t_s, tag(0 yok,1 ok,2 stale), px, py, pz, dx, dy, Q, consist, tex, guven
"""
import sys, cv2, numpy as np
F, SC = cv2.FONT_HERSHEY_SIMPLEX, 0.42
H, W = 24, 640
_cache = {}
def render(txt, y):
    k = (txt, y)
    if k not in _cache:
        if len(_cache) > 3000: _cache.clear()
        im = np.zeros((H, W), np.uint8); cv2.putText(im, txt, (6, y), F, SC, 255, 1, cv2.LINE_AA)
        _cache[k] = im.astype(np.float32)
    return _cache[k]
def width(txt): return cv2.getTextSize(txt, F, SC, 1)[0][0]
NUM = list('0123456789+-. ')
def decode(band, y, spec):
    """band: float32 HxW (0-255). spec: [('lit', s) | ('num', n) | ('cho', [..])] -> (alanlar, guven)"""
    pre, out, conf = '', [], []
    for kind, arg in spec:
        if kind == 'lit': pre += arg; continue
        if kind == 'cho':
            best = None
            for c in arg:
                x0, x1 = 6 + width(pre), 6 + width(pre + max(arg, key=len)) + 2
                t = render(pre + c, y)[:, x0:x1] - render(pre, y)[:, x0:x1]
                s = float((t * band[:, x0:x1]).sum() / (np.linalg.norm(t) * np.linalg.norm(band[:, x0:x1]) + 1e-6))
                if best is None or s > best[0]: best = (s, c)
            out.append(best[1]); conf.append(best[0]); pre += best[1]; continue
        s_field = ''
        for _ in range(arg):
            base = 6 + width(pre)
            best = None
            for c in NUM:
                adv = width(pre + c) - width(pre)
                x0, x1 = base - 1, base + adv + 1
                patch = band[:, x0:x1]
                if c == ' ':
                    s = 0.5 if band[4:22, base + 1:base + adv - 1].max() < 140 else -1.0
                else:
                    t = render(pre + c, y)[:, x0:x1] - render(pre, y)[:, x0:x1]
                    tm, pm = t - t.mean(), patch - patch.mean()
                    s = float((tm * pm).sum() / (np.linalg.norm(tm) * np.linalg.norm(pm) + 1e-6))
                if best is None or s > best[0]: best = (s, c)
            s_field += best[1]; pre += best[1]; conf.append(best[0])
        out.append(s_field)
    return out, float(np.min(conf)) if conf else 0.0
L1 = [('lit', 't='), ('num', 8), ('lit', 's  TAG:'), ('cho', ['OK   ', 'YOK  ', 'STALE']), ('lit', ' pos '),
      ('num', 6), ('lit', ' '), ('num', 6), ('lit', ' '), ('num', 6)]
L2 = [('lit', 'FLOW dx='), ('num', 6), ('lit', ' dy='), ('num', 6), ('lit', '  Q='), ('num', 3),
      ('lit', '  consist='), ('num', 4), ('lit', ' tex='), ('num', 4)]
def fl(s):
    try: return float(s.replace(' ', ''))
    except ValueError: return np.nan
def read_frame(img):
    g = img[:44].max(axis=2).astype(np.float32)
    a, c1 = decode(g[0:24], 17, L1)
    b, c2 = decode(g[19:43], 17, L2)      # 2. satir: taban y=36 -> bant 19'dan baslar, yerel y=17
    tag = {'OK   ': 1, 'YOK  ': 0, 'STALE': 2}[a[1]]
    return [fl(a[0]), tag, fl(a[2]), fl(a[3]), fl(a[4]), fl(b[0]), fl(b[1]), fl(b[2]), fl(b[3]), fl(b[4]), min(c1, c2)]
if __name__ == '__main__':
    step = int(sys.argv[3]) if len(sys.argv) > 3 else 1
    cap = cv2.VideoCapture(sys.argv[1]); rows = []; i = 0
    while cap.grab():
        if i % step == 0:
            ok, img = cap.retrieve()
            if ok: rows.append([i] + read_frame(img))
        i += 1
    np.save(sys.argv[2], np.array(rows, float)); print(sys.argv[1], 'kare', i, 'okunan', len(rows))
