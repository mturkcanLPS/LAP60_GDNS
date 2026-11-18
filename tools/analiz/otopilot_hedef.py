"""Otopilotun car bearing'den hesapladigi hedef konumu (Gud_DLat/DLon) vs RTK hedef (YRTK_HedefN/E).
Kullanim: python3 -s -W ignore otopilot_hedef.py <mat>"""
import sys, numpy as np, scipy.io as sio
m = sio.loadmat(sys.argv[1], squeeze_me=True); F = lambda k: m[k].astype(float)
t = (m['timeusec'] - m['timeusec'][0]) / 1e6
bx, by, cts = F('GDNS_CarBearingX'), F('GDNS_CarBearingY'), F('GDNS_CarTimestamp')
N, E, D = F('YRTK_HedefN'), F('YRTK_HedefE'), F('YRTK_HedefD')
gla, glo = F('Gud_DLat'), F('Gud_DLon')
ro, pi_, ya = np.radians(F('NavRollFilt')), np.radians(F('NavPitchFilt')), np.radians(F('NavYawFilt'))
mode = F('GDNS_HedefModu'); las, ekz = F('LaserAltitude'), F('ekfOutPosZ')
i = np.where(np.r_[False, np.diff(cts) != 0] & ((bx != 0) | (by != 0)) & (F('YRTK_Durum') > 0) & (D > 2.5) & (mode == 1))[0]
h = D[i]; cy, sy = np.cos(ya[i]), np.sin(ya[i])
# RTK gercegi govdede (ileri, sag)
tf = cy * N[i] + sy * E[i]; tr = -sy * N[i] + cy * E[i]
# otopilot hesabi: Gud_DLat/DLon -> once N/E varsay, govdeye cevir
for nm, (gn, ge) in (('Gud (Lat=N, Lon=E)', (gla[i], glo[i])),):
    gf = cy * gn + sy * ge; gr = -sy * gn + cy * ge
# ideal: bizim yon + tam egim duzeltmesi (kamera x = govde sag, kamera y = -ileri), yukseklik RTK D
def ideal(hh):
    v = np.c_[-by[i], bx[i], np.ones(len(i))]   # govde: ileri, sag, asagi (kamera ekseninde ray)
    cr, sr, cp, sp = np.cos(ro[i]), np.sin(ro[i]), np.cos(pi_[i]), np.sin(pi_[i])
    # govde -> yatay (roll, pitch; yaw haric)
    x, y, z = v[:, 0], v[:, 1], v[:, 2]
    y2 = cr * y - sr * z; z2 = sr * y + cr * z
    x3 = cp * x + sp * z2; z3 = -sp * x + cp * z2
    return x3 / z3 * hh, y2 / z3 * hh
idf, idr = ideal(h)
ham_f, ham_r = -by[i] * h, bx[i] * h          # egim duzeltmesiz
print(f'{sys.argv[1].split("/")[-1]}: hedef modunda {len(i)} car olcumu')
print('  RTK D    |   n  | otopilot (Gud) hatasi med/p90 | ideal (bizim yon+tam egim) | egim duzeltmesiz (yon x h)')
for lo, hi in ((2.5, 10), (10, 20), (20, 30), (30, 45), (45, 70)):
    k = (h >= lo) & (h < hi)
    if k.sum() < 15: continue
    eg = np.hypot(gf[k] - tf[k], gr[k] - tr[k]); ei = np.hypot(idf[k] - tf[k], idr[k] - tr[k]); eh = np.hypot(ham_f[k] - tf[k], ham_r[k] - tr[k])
    print(f'  {lo:4.1f}-{hi:2.0f} | {k.sum():4d} | {np.median(eg):5.2f} / {np.percentile(eg, 90):5.2f} m           | {np.median(ei):5.2f} / {np.percentile(ei, 90):5.2f} m      | {np.median(eh):5.2f} / {np.percentile(eh, 90):5.2f} m')
# otopilot formulunu tahmin: Gud ~ a*(yon*h) + b*(tan(egim)*h)
A = np.c_[bx[i] * h, np.tan(ro[i]) * h]; cr_ = np.linalg.lstsq(A, gr, rcond=None)[0]
B = np.c_[-by[i] * h, np.tan(pi_[i]) * h]; cf_ = np.linalg.lstsq(B, gf, rcond=None)[0]
print(f'  regresyon Gud_sag  = {cr_[0]:+.2f}*(bx*h) {cr_[1]:+.2f}*(tan(roll)*h)   [ideal: +1, -1]')
print(f'  regresyon Gud_ileri= {cf_[0]:+.2f}*(-by*h) {cf_[1]:+.2f}*(tan(pitch)*h) [ideal: +1, +1 ya da -1]')
print(f'  roll medyan {np.degrees(np.median(ro[i])):+.1f} der (p10 {np.degrees(np.percentile(ro[i],10)):+.1f} p90 {np.degrees(np.percentile(ro[i],90)):+.1f}), pitch medyan {np.degrees(np.median(pi_[i])):+.1f}')
k = h > 40
if k.sum():
    print(f'  >40 m: lazer - RTK D medyan {np.median(las[i][k] - h[k]):+.1f} m, ekfZ - RTK D {np.median(ekz[i][k] - h[k]):+.1f} m, lazer=0 orani %{100*np.mean(las[i][k] <= 0.1):.0f}')
