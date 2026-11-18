"""Otopilot .mat (RTK hedef + tutum) ile gonderilen car bearing karsilastirmasi.
Kullanim: python3 -s -W ignore rtk_bearing.py <mat dosyasi>"""
import sys, itertools, numpy as np, scipy.io as sio
m = sio.loadmat(sys.argv[1], squeeze_me=True)
t = (m['timeusec'] - m['timeusec'][0]) / 1e6
f = lambda k: m[k].astype(float)
bx, by, cts = f('GDNS_CarBearingX'), f('GDNS_CarBearingY'), f('GDNS_CarTimestamp')
N, E, D = f('YRTK_HedefN'), f('YRTK_HedefE'), f('YRTK_HedefD')
ro, pi, ya = np.radians(f('NavRollFilt')), np.radians(f('NavPitchFilt')), np.radians(f('NavYawFilt'))
# yeni car verisi gelen satirlar (zaman damgasi degisti) + gecerli (0 degil) + RTK iyi
new = np.r_[False, np.diff(cts) != 0] & ((bx != 0) | (by != 0)) & (f('YRTK_Durum') > 0) & (D > 2.5)
idx = np.where(new)[0]
def pred(i, shift=0.0):
    j = np.clip(np.searchsorted(t, t[i] - shift), 0, len(t) - 1)   # tutum/RTK 'shift' s once
    cr, sr, cp, sp, cy, sy = np.cos(ro[j]), np.sin(ro[j]), np.cos(pi[j]), np.sin(pi[j]), np.cos(ya[j]), np.sin(ya[j])
    n, e, d = N[j], E[j], D[j]
    # NED -> govde (ZYX): once yaw, sonra pitch, sonra roll
    x1 = cy*n + sy*e; y1 = -sy*n + cy*e; z1 = d
    x2 = cp*x1 - sp*z1; z2 = sp*x1 + cp*z1; y2 = y1
    y3 = cr*y2 + sr*z2; z3 = -sr*y2 + cr*z2; x3 = x2
    return x3, y3, z3   # govde: ileri, sag, asagi
fw, rt, dn = pred(idx)
# kamera ekseni eslemesi: 8 aday (cam x = +-sag/ileri, cam y = +-ileri/sag)
best = None
for (ax, sx), (ay, sy_) in itertools.product([('rt', 1), ('rt', -1), ('fw', 1), ('fw', -1)], repeat=2):
    if ax == ay: continue
    v = {'rt': rt, 'fw': fw}
    px, py = sx * v[ax] / dn, sy_ * v[ay] / dn
    e = np.median(np.hypot(px - bx[idx], py - by[idx]) * D[idx])
    if best is None or e < best[0]: best = (e, ax, sx, ay, sy_)
_, ax, sx, ay, sy_ = best
print(f'{sys.argv[1].split("/")[-1]}: {len(idx)} car olcumu | kamera esleme: cam x = {"+" if sx>0 else "-"}{ax}, cam y = {"+" if sy_>0 else "-"}{ay}')
def residual(shift=0.0, sel=slice(None)):
    fw, rt, dn = pred(idx[sel], shift); v = {'rt': rt, 'fw': fw}
    return sx * v[ax] / dn - bx[idx][sel], sy_ * v[ay] / dn - by[idx][sel]
# gecikme taramasi
print('gecikme taramasi (tutum+RTK kaydirma) -> yerde hata medyan:')
for sh in (0, 0.05, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5):
    rx, ry = residual(sh); print(f'   {sh*1000:4.0f} ms: {np.median(np.hypot(rx, ry) * D[idx]):.2f} m')
rx, ry = residual(0.0); h = D[idx]
print('\nirtifaya gore (gecikme 0): hata = beklenen(RTK+tutum) - gonderilen')
for lo, hi in ((2.5, 8), (8, 14), (14, 20), (20, 27)):
    k = (h >= lo) & (h < hi)
    if k.sum() < 20: continue
    print(f'  HedefD {lo:4.1f}-{hi:2.0f} m n={k.sum():4d}: yerde hata medyan {np.median(np.hypot(rx[k], ry[k]) * h[k]):.2f} m | aci yanlilik x {np.degrees(np.median(rx[k])):+.2f} y {np.degrees(np.median(ry[k])):+.2f} der (metre: {np.median(rx[k]*h[k]):+.2f} / {np.median(ry[k]*h[k]):+.2f})')
np.save('/tmp/claude-1000/-home-lapis-ros2-ws/ae892145-43b4-4335-89c7-55834ec12cf8/scratchpad/rtkb_' + sys.argv[1].split('_')[-2] + '.npy',
        np.c_[t[idx], h, rx, ry, bx[idx], by[idx], ro[idx], pi[idx], ya[idx], N[idx], E[idx]])
