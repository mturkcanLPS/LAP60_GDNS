"""Otopilota giden tag konumu (GDNS_ApAvgPos, kamera ekseni) vs RTK hedef + tutum. Kullanim: rtk_tag.py <mat>"""
import sys, numpy as np, scipy.io as sio
m = sio.loadmat(sys.argv[1], squeeze_me=True); f = lambda k: m[k].astype(float)
t = (m['timeusec'] - m['timeusec'][0]) / 1e6
X, Y, Z, V, ts = f('GDNS_ApAvgPosX')/100, f('GDNS_ApAvgPosY')/100, f('GDNS_ApAvgPosZ')/100, f('GDNS_ApAvgValidity'), f('GDNS_ApTimestamp')
N, E, D = f('YRTK_HedefN'), f('YRTK_HedefE'), f('YRTK_HedefD')
ro, pi, ya = np.radians(f('NavRollFilt')), np.radians(f('NavPitchFilt')), np.radians(f('NavYawFilt'))
idx = np.where(np.r_[False, np.diff(ts) != 0] & (V > 0) & (Z > 1.5) & (f('YRTK_Durum') > 0) & (D > 2))[0]
cr, sr, cp, sp, cy, sy = [g(a[idx]) for a, g in ((ro, np.cos), (ro, np.sin), (pi, np.cos), (pi, np.sin), (ya, np.cos), (ya, np.sin))]
n, e, d = N[idx], E[idx], D[idx]
x1 = cy*n + sy*e; y1 = -sy*n + cy*e; x2 = cp*x1 - sp*d; z2 = sp*x1 + cp*d; y3 = cr*y1 + sr*z2; z3 = -sr*y1 + cr*z2
px, py = y3 / z3, -x2 / z3          # cam x = sag, cam y = -ileri
tx, ty = X[idx] / Z[idx], Y[idx] / Z[idx]
rx, ry = px - tx, py - ty
print(f'{sys.argv[1].split("/")[-1]}: {len(idx)} tag olcumu, tag z {np.min(Z[idx]):.1f}-{np.max(Z[idx]):.1f} m, RTK D/tag Z medyan {np.median(d/Z[idx]):.2f}')
for lo, hi in ((1.5, 4), (4, 7), (7, 12)):
    k = (Z[idx] >= lo) & (Z[idx] < hi)
    if k.sum() >= 20:
        print(f'  tag z {lo}-{hi} m n={k.sum():4d}: aci yanlilik x {np.degrees(np.median(rx[k])):+.2f} y {np.degrees(np.median(ry[k])):+.2f} der | yerde {np.median(np.hypot(rx[k],ry[k])*Z[idx][k]):.2f} m')
