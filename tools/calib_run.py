#!/usr/bin/env python3
"""Q → gercek akis hatasi kalibrasyonu (offline, video HUD'undan).

Kullanim: python3 calib_run.py <ucus_videosu.avi>

Referans: yesil tag cercevesi merkezinin kare-arasi hareketi. Akis EMA'li
oldugu icin referansa da esdeger EMA uygulanir (alpha_eff = 1-0.75^n).
Video yari cozunurluk → tag kaymasi x2. Cikti: Q kovasi basina medyan/p90
hata (px/frame) + hareket bandina gore ayrim.
"""
import sys
import os
import cv2
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hud_parse import parse_frame

path = sys.argv[1]
cap = cv2.VideoCapture(path)

rows = []
i = 0
while True:
    ok, fr = cap.read()
    if not ok:
        break
    vals, center = parse_frame(fr)
    if vals is not None:
        rows.append((i, vals["t"], vals["dx"], vals["dy"], vals["q"], center))
    i += 1
    if i % 1000 == 0:
        print(f"  {i} kare islendi...", flush=True)
cap.release()
print(f"{path}: {i} kare, {len(rows)} cozuldu")

CAM_DT = 1.0 / 30.0
pairs = []
ref_ema = None
prev = None
for r in rows:
    idx, t, dx, dy, q, c = r
    if prev is not None and c is not None and prev[5] is not None:
        dt = t - prev[1]
        if 0.03 < dt < 0.25 and idx - prev[0] <= 4:
            n = max(1, round(dt / CAM_DT))
            ref = np.array([(c[0] - prev[5][0]) * 2.0 / n,
                            (c[1] - prev[5][1]) * 2.0 / n])
            a = 1.0 - 0.75 ** n
            ref_ema = ref if ref_ema is None else (1 - a) * ref_ema + a * ref
            flow = np.array([dx, dy])
            err = float(np.linalg.norm(flow - ref_ema))
            pairs.append((q, err, float(np.linalg.norm(ref_ema)),
                          float(np.linalg.norm(flow))))
        else:
            ref_ema = None
    if c is None:
        ref_ema = None
    prev = r

pairs = np.array(pairs)
print(f"eslesen cift: {len(pairs)}")
if len(pairs) < 20:
    sys.exit(0)

print(f"\n{'Q kovasi':>10} {'n':>6} {'medyan hata':>12} {'p90 hata':>10} "
      f"{'medyan |ref|':>13}")
for lo in range(0, 100, 20):
    hi = lo + 20
    sel = pairs[(pairs[:, 0] >= lo) & (pairs[:, 0] < hi)]
    if len(sel) < 5:
        print(f"{lo:>3}-{hi:<3}    {len(sel):>6}          (az ornek)")
        continue
    print(f"{lo:>3}-{hi:<3}    {len(sel):>6} {np.median(sel[:,1]):>10.2f}px "
          f"{np.percentile(sel[:,1],90):>8.2f}px {np.median(sel[:,2]):>11.2f}px")

print("\nhareket bandina gore (|ref| px/frame):")
for name, lo, hi in [("hover  <1", 0, 1), ("yavas 1-4", 1, 4),
                     ("hizli  >4", 4, 1e9)]:
    sel = pairs[(pairs[:, 2] >= lo) & (pairs[:, 2] < hi)]
    if len(sel) < 5:
        continue
    print(f"  {name}: n={len(sel):5d}  medyan hata={np.median(sel[:,1]):.2f}px  "
          f"medyan Q={np.median(sel[:,0]):.0f}")

out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   os.path.basename(path).replace('.avi', '') + '_pairs.npy')
np.save(out, pairs)
