#!/usr/bin/env python3
"""Kayit videosu HUD cozucu — debug_recorder'in kendi fontuyla (Hershey
Simplex 0.42/1px) sablon eslestirme. Cikti: her kare icin t, dx, dy, Q ve
yesil tag cercevesinin merkezi. (lps_serial_sender ucus kayitlari icin)"""
import cv2
import numpy as np

FONT, SCALE, THICK = cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1
CHARS = "0123456789.+-=/:sQdxytexconsitfFLOWTAGYKOSMEPRparlgm"

def make_templates():
    tmpl = {}
    for ch in set(CHARS):
        canvas = np.zeros((24, 20), np.uint8)
        cv2.putText(canvas, ch, (2, 18), FONT, SCALE, 255, THICK, cv2.LINE_AA)
        ys, xs = np.nonzero(canvas > 100)
        if len(xs) == 0:
            continue
        tmpl[ch] = canvas[ys.min():ys.max()+1, xs.min():xs.max()+1]
    return tmpl

TMPL = make_templates()

def classify(seg):
    h, w = seg.shape
    if h <= 3 and w <= 3:
        return "."
    if h <= 6 and w >= 4:
        rows = np.nonzero(seg.sum(axis=1))[0]
        gaps = np.diff(rows)
        return "=" if (gaps > 1).any() else "-"
    best, bch = -1.0, "?"
    segf = seg.astype(np.float32)
    for ch, t in TMPL.items():
        th, tw = t.shape
        if abs(th - h) > 2 or abs(tw - w) > 2:
            continue
        H, W = max(th, h), max(tw, w)
        a = np.zeros((H, W), np.float32); a[:h, :w] = segf
        b = np.zeros((H, W), np.float32); b[:th, :tw] = t
        denom = np.sqrt((a * a).sum() * (b * b).sum())
        if denom == 0:
            continue
        s = (a * b).sum() / denom
        if s > best:
            best, bch = s, ch
    return bch

def parse_line(img_line, thresh=110):
    g = img_line if img_line.ndim == 2 else cv2.cvtColor(img_line, cv2.COLOR_BGR2GRAY)
    b = (g > thresh).astype(np.uint8)
    colsum = b.sum(axis=0)
    out, x = [], 0
    W = b.shape[1]
    while x < W:
        if colsum[x] == 0:
            x += 1
            continue
        x0 = x
        while x < W and colsum[x] > 0:
            x += 1
        seg = b[:, x0:x]
        ys = np.nonzero(seg.sum(axis=1))[0]
        seg = (seg[ys.min():ys.max()+1, :] * 255).astype(np.uint8)
        if out and x0 - out[-1][1] > 5:
            out.append((" ", x0, x0))
        out.append((classify(seg), x0, x))
    return "".join(c for c, _, _ in out)

def parse_frame(fr):
    """dondurur: dict(t, dx, dy, q) veya None; ayrica tag merkezi (veya None)"""
    import re
    l1 = parse_line(fr[6:20, :]).replace(" ", "")
    l2 = parse_line(fr[25:40, :]).replace(" ", "")
    def num(s):
        s = (s.replace("O", "0").replace("o", "0").replace("i", "1")
              .replace("l", "1").replace("t", "1").replace("I", "1"))
        try:
            return float(s)
        except ValueError:
            return None
    vals = None
    m1 = re.search(r"t=(.+?)s", l1)
    m2 = re.search(r"dx=([+-].+?)dy=([+-].+?)Q=(.+?)c", l2)
    if m1 and m2:
        t  = num(m1.group(1))
        dx = num(m2.group(1))
        dy = num(m2.group(2))
        q  = num(m2.group(3))
        if None not in (t, dx, dy, q) and 0 <= q <= 100:
            vals = dict(t=t, dx=dx, dy=dy, q=int(q))
    body = fr[44:, :]
    b, gch, r = body[:, :, 0].astype(int), body[:, :, 1].astype(int), body[:, :, 2].astype(int)
    mask = (gch > 150) & (gch - b > 60) & (gch - r > 60)
    center = None
    if mask.sum() > 40:
        ys, xs = np.nonzero(mask)
        center = (float(xs.mean()), float(ys.mean()) + 44)
    return vals, center

if __name__ == "__main__":
    import sys
    fr = cv2.imread(sys.argv[1])
    vals, center = parse_frame(fr)
    print("parsed:", vals, "tag_center:", center)
