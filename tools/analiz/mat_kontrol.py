"""OKAD otopilot .mat'i ile Jetson kayitlarinin kapsamli karsilastirmasi (18_11).
Kullanim: python3 -s -W ignore mat_kontrol.py <OKAD_...mat> <oturum, orn 0209> [--bore X Y]
  1) Optik akis: gonderilen akis vs INS hizindan beklenen akis; cevrimdisi girdi karsilastirmasi
     (all/raw, gyro telafili/telafisiz) -> hangisi gercek hizi en iyi izliyor.
  2) Arac yonu vs RTK hedef (+ tutum): irtifa bantlari, aci/metre ayrisimi (huni ofseti),
     montaj duzeltmesi simulasyonu, kirpik kutu (kenara degen) ayrimi.
  3) 0x4A: bize gelen EKF z ve basliklar mat ile tutarli mi.
Zaman eslemesi: mat GDNS_OpTimestamp (16 bit ms) = bizim msSinceStart (flow CSV t_ms) mod 65536."""
import sys, glob, csv, io, numpy as np, scipy.io as sio
D = '/home/lapis/lps_records/'
mat, ses = sys.argv[1], sys.argv[2]
BORE = (-2.2, -1.1)
if '--bore' in sys.argv: i = sys.argv.index('--bore'); BORE = (float(sys.argv[i + 1]), float(sys.argv[i + 2]))
m = sio.loadmat(mat, squeeze_me=True); F = lambda k: m[k].astype(float)
T = (m['timeusec'] - m['timeusec'][0]) / 1e6
def rd(p):
    return list(csv.DictReader(io.StringIO(open(glob.glob(D + p)[0], 'rb').read().replace(b'\x00', b'').decode('utf-8', 'ignore'))))
def fl(x):
    try: return float(x)
    except Exception: return np.nan

# ---------------- zaman eslemesi: mat t <-> serial ms ----------------
ots = F('GDNS_OpTimestamp'); ch = np.where(np.r_[True, np.diff(ots) != 0])[0]
# 16 bit basa donme: mat zamanindan beklenen artisa en yakin 65536 kati (log bosluklarinda da dogru)
tc_ = T[ch]; u = np.empty(len(ch)); u[0] = ots[ch[0]]
for q in range(1, len(ch)):
    beklenen = u[q - 1] + (tc_[q] - tc_[q - 1]) * 1000.0
    u[q] = ots[ch[q]] + np.round((beklenen - ots[ch[q]]) / 65536.0) * 65536.0
FL = [r for r in rd(f'flow_s{ses}_*.csv') if r.get('durum') not in (None, '')]
ft = np.array([fl(r['t_ms']) for r in FL])
best = None
for k in range(-2, 40):
    uu = u + k * 65536; j = np.searchsorted(ft, uu); j = np.clip(j, 0, len(ft) - 1)
    hit = (np.abs(ft[j] - uu) < 0.5).sum()
    if best is None or hit > best[0]: best = (hit, k)
u += best[1] * 65536
ms2t = lambda ms: np.interp(ms, u, T[ch])       # serial ms -> mat t
print(f'== {mat.split("/")[-1]} <-> s{ses} | akis paketi eslesen {best[0]}/{len(u)}')

# ---------------- 1) OPTIK AKIS ----------------
ox, oy = F('GDNS_OpmotionX'), F('GDNS_OpmotionY')
fmap = {int(r['t_ms']): r for r in FL}
pairs = [(fl(fmap[int(x)]['comp_dx_kare']), ox[ch[i]]) for i, x in enumerate(u) if int(x) in fmap and fmap[int(x)]['durum'] == '0']
pairs = np.array([p for p in pairs if np.isfinite(p[0]) and p[0] != 0])
sc = np.median(pairs[:, 1] / pairs[:, 0]) if len(pairs) else np.nan
print(f'\n[1] OPTIK AKIS: mat OpmotionX / CSV comp_dx olcegi {sc:.1f} (n={len(pairs)}) -> paket = CSV degeri')
# IMU ile cevrimdisi telafi (OKAD sabitleri)
I = rd(f'imu_s{ses}_*.csv')
it = np.array([fl(r['stamp_s']) for r in I]); itm = np.array([fl(r['t_ms']) for r in I])
gx = np.array([fl(r['gx']) for r in I]); gy = np.array([fl(r['gy']) for r in I])
kt = np.array([fl(r['kam_t']) for r in FL]); dur = np.array([r['durum'] for r in FL])
cols = {k: np.array([fl(r[k]) for r in FL]) for k in ('all_dx', 'all_dy', 'raw_dx', 'raw_dy', 'comp_dx_kare', 'comp_dy_kare')}
tf = ms2t(ft)
# yukseklik EKF'den: lazer 40 m ustunde olcmuyor (18_11)
hgt = np.interp(tf, T, F('ekfOutPosZ')); vn = np.interp(tf, T, F('Nav_North_Speed')); ve = np.interp(tf, T, F('Nav_East_Speed'))
psi = np.radians(np.interp(tf, T, np.unwrap(np.radians(F('NavYawFilt'))) * 180 / np.pi))
vf = np.cos(psi) * vn + np.sin(psi) * ve; vr = -np.sin(psi) * vn + np.cos(psi) * ve
# gyro donmesi (kesintisiz bolgelerde; saat adimlarini atla)
thx = np.full(len(FL), np.nan); thy = np.full(len(FL), np.nan)
cut = np.where(np.abs(np.diff(it)) > 1.0)[0]; segs = np.split(np.arange(len(it)), cut + 1)
for sg in segs:
    if len(sg) < 400: continue
    o = sg[np.argsort(it[sg])]; t_ = it[o]
    ax = np.r_[0, np.cumsum(0.5 * (gx[o][1:] + gx[o][:-1]) * np.diff(t_))]; ay = np.r_[0, np.cumsum(0.5 * (gy[o][1:] + gy[o][:-1]) * np.diff(t_))]
    sel = np.where((kt > t_[0] + 0.2) & (kt < t_[-1] - 0.2))[0]; sel = sel[sel > 0]
    t1, t0 = kt[sel] - 0.010, kt[sel - 1] - 0.010
    thx[sel] = np.interp(t1, t_, ax) - np.interp(t0, t_, ax); thy[sel] = np.interp(t1, t_, ay) - np.interp(t0, t_, ay)
dt = np.r_[np.nan, np.diff(kt)]; nrm = (1 / 30) / dt
ok = (dur == '0') & (hgt > 4) & np.isfinite(thx) & (dt > 0.015) & (dt < 0.1)
cand = {
    'gonderilen (paket)': (cols['comp_dx_kare'], cols['comp_dy_kare']),
    'all + gyro (simdiki varsayilan)': ((cols['all_dx'] + 845 * thy) * nrm, (cols['all_dy'] - 740 * thx) * nrm),
    'raw + gyro': ((cols['raw_dx'] + 845 * thy) * nrm, (cols['raw_dy'] - 740 * thx) * nrm),
    'all, gyro YOK': (cols['all_dx'] * nrm, cols['all_dy'] * nrm),
}
print(f'   havada (EKF z > 4 m) {ok.sum()} kare | beklenen akis = hiz / yukseklik (INS N/E + yaw -> govde)')
print('   girdi                              | dx~sag hiz: kazanc  R2  | dy~ileri hiz: kazanc  R2  | hiz hatasi std (m/s) sag/ileri')
for nm, (cx, cy) in cand.items():
    k = ok & np.isfinite(cx) & np.isfinite(cy)
    px, py = vr[k] / hgt[k], vf[k] / hgt[k]
    ax_ = np.polyfit(px, cx[k], 1); ay_ = np.polyfit(py, cy[k], 1)
    r2x = np.corrcoef(px, cx[k])[0, 1] ** 2; r2y = np.corrcoef(py, cy[k])[0, 1] ** 2
    ex = (cx[k] - ax_[1]) / ax_[0] * hgt[k] - vr[k]; ey = (cy[k] - ay_[1]) / ay_[0] * hgt[k] - vf[k]
    print(f'   {nm:34s} | {ax_[0]:7.1f}  {r2x:.2f}         | {ay_[0]:7.1f}  {r2y:.2f}          | {np.std(ex):.2f} / {np.std(ey):.2f}')
print(f'   (kazanc = px/kare / (1/s); beklenen ~ -/+ f/30 ~ 21-25. Isaret eksen yonunu gosterir.)')
# irtifa bantlari: SABIT kazancla (s0209'dan: dx -21.6, dy +22.3) hiz tahmini -> INS'e gore hata
cx, cy = cand['gonderilen (paket)']
print('   gonderilen akis, sabit kazancla hiz hatasi (INS hizina gore):  EKF z | n | hata std sag/ileri (m/s) | hiz std sag/ileri (INS)')
for lo, hi in ((4, 10), (10, 20), (20, 30), (30, 45), (45, 70)):
    k = ok & np.isfinite(cx) & (hgt >= lo) & (hgt < hi)
    if k.sum() < 50: continue
    er = cx[k] / -21.6 * hgt[k] - vr[k]; ef = cy[k] / 22.3 * hgt[k] - vf[k]
    print(f'      {lo:2d}-{hi:2d} m | {k.sum():5d} | {np.std(er):.2f} / {np.std(ef):.2f} | {np.std(vr[k]):.2f} / {np.std(vf[k]):.2f}')

# ---------------- 2) ARAC YONU vs RTK ----------------
bx, by, cts = F('GDNS_CarBearingX'), F('GDNS_CarBearingY'), F('GDNS_CarTimestamp')
N, E, Dd = F('YRTK_HedefN'), F('YRTK_HedefE'), F('YRTK_HedefD')
ro, pi_, ya = np.radians(F('NavRollFilt')), np.radians(F('NavPitchFilt')), np.radians(F('NavYawFilt'))
idx = np.where(np.r_[False, np.diff(cts) != 0] & ((bx != 0) | (by != 0)) & (F('YRTK_Durum') > 0) & (Dd > 2.5))[0]
cr, sr, cp, sp, cy_, sy_ = np.cos(ro[idx]), np.sin(ro[idx]), np.cos(pi_[idx]), np.sin(pi_[idx]), np.cos(ya[idx]), np.sin(ya[idx])
n_, e_, d_ = N[idx], E[idx], Dd[idx]
x1 = cy_ * n_ + sy_ * e_; y1 = -sy_ * n_ + cy_ * e_; x2 = cp * x1 - sp * d_; z2 = sp * x1 + cp * d_
y3 = cr * y1 + sr * z2; z3 = -sr * y1 + cr * z2
px, py = y3 / z3, -x2 / z3; sx, sy = bx[idx], by[idx]; h = Dd[idx]
def bore(bx_, by_, xd, yd):
    a, b = np.radians(xd), np.radians(yd)
    X = bx_ * np.cos(a) + np.sin(a); Z = -bx_ * np.sin(a) + np.cos(a); Y = by_ * np.cos(b) + Z * np.sin(b); Z2 = -by_ * np.sin(b) + Z * np.cos(b)
    return X / Z2, Y / Z2
cx_, cy2 = bore(sx, sy, *BORE)
print(f'\n[2] ARAC YONU vs RTK: {len(idx)} olcum | montaj duzeltmesi simulasyonu x {BORE[0]:+.1f} y {BORE[1]:+.1f} der')
print('   RTK D (m)  |  n   | hata medyan (m): gonderilen -> duzeltmeli | p90 gonderilen -> duzeltmeli')
for lo, hi in ((2.5, 8), (8, 14), (14, 20), (20, 30), (30, 45), (45, 70)):
    k = (h >= lo) & (h < hi)
    if k.sum() < 15: continue
    e0 = np.hypot(px[k] - sx[k], py[k] - sy[k]) * h[k]; e1 = np.hypot(px[k] - cx_[k], py[k] - cy2[k]) * h[k]
    print(f'   {lo:4.1f}-{hi:4.1f} | {k.sum():4d} | {np.median(e0):.2f} -> {np.median(e1):.2f}            | {np.percentile(e0, 90):.2f} -> {np.percentile(e1, 90):.2f}')
# aci + metre ayrisimi: hata_m = aci * h + ofset_m (kamera ekseni)
for nm, ex in (('x (goruntu sag)', (px - sx) * h), ('y (goruntu asagi = arac arkasi)', (py - sy) * h)):
    A = np.c_[h, np.ones_like(h)]; coef = np.linalg.lstsq(A, ex, rcond=None)[0]
    print(f'   {nm:32s}: sabit aci {np.degrees(coef[0]):+.2f} der + sabit metre {coef[1]:+.2f} m  (metre kismi ~ huni ofseti hatasi)')
# kirpik kutu ayrimi (EO CSV, EKF ile hizalanmis)
try:
    C = rd(f'eo_s{ses}_*.csv'); RX = [r for r in rd(f'rx_s{ses}_*.csv') if r['tip'] == 'E' and r['ekf_z']]
    te = np.array([fl(r['t_ms']) for r in RX]); ze = np.array([fl(r['ekf_z']) for r in RX])
    tc = np.array([fl(r['t_ms']) for r in C]); ac = np.array([fl(r['irtifa_m']) for r in C])
    kk = np.isfinite(ac) & (ac > 1); bs = min(((np.nanmean(np.abs(np.interp(tc[kk] + s, te, ze) - ac[kk]))), s) for s in np.arange(-30, 30, 0.1))[1]
    tcm = ms2t(tc + bs * 1000)   # EO satiri -> mat t
    kes = np.zeros(len(C), bool)
    for i, r in enumerate(C):
        for b in (r.get('kutular(id:sol:ust:gen:yuk:conf:kilitli:elendi:boy_m|...)') or '').split('|'):
            p = b.split(':')
            if len(p) >= 9 and p[6] == '1':
                l, t_, w, hh = map(float, p[1:5]); kes[i] = l <= 4 or t_ <= 4 or l + w >= 1276 or t_ + hh >= 716
    j = np.clip(np.searchsorted(tcm, T[idx]), 0, len(tcm) - 1); near = np.abs(tcm[j] - T[idx]) < 0.1
    for nm, mk in (('kenara degmeyen', near & ~kes[j]), ('KIRPIK (kenara degen)', near & kes[j])):
        if mk.sum() >= 10:
            e1 = np.hypot(px[mk] - cx_[mk], py[mk] - cy2[mk]) * h[mk]
            print(f'   {nm:22s}: n={mk.sum():4d} RTK D medyan {np.median(h[mk]):4.1f} m | duzeltmeli hata medyan {np.median(e1):.2f} p90 {np.percentile(e1, 90):.2f} m')
except Exception as ex:
    print('   kirpik ayrimi yapilamadi:', ex)

# ---------------- 3) 0x4A ----------------
RX = [r for r in rd(f'rx_s{ses}_*.csv') if r['tip'] == 'E' and r['ekf_z']]
tr = ms2t(np.array([fl(r['t_ms']) for r in RX]))
z = np.array([fl(r['ekf_z']) for r in RX]); dh = np.array([fl(r['dron_baslik']) for r in RX]); yh = np.array([fl(r['yer_baslik']) for r in RX])
ok3 = (tr > T[0] + 1) & (tr < T[-1] - 1)
zm = np.interp(tr[ok3], T, F('ekfOutPosZ')); lz = np.interp(tr[ok3], T, F('LaserAltitude'))
yawm = np.interp(tr[ok3], T, np.degrees(np.unwrap(np.radians(F('NavYawFilt'))))) % 360
vnh = np.interp(tr[ok3], T, np.degrees(np.unwrap(F('YRTK_VN_Heading')))) % 360
wrap = lambda a: (a + 180) % 360 - 180
print(f'\n[3] 0x4A: {ok3.sum()} mesaj | EKF z - mat ekfOutPosZ medyan {np.median(z[ok3] - zm):+.2f} m (|fark| p90 {np.percentile(np.abs(z[ok3] - zm), 90):.2f}) | - lazer medyan {np.median(z[ok3] - lz):+.2f}')
print(f'   dron basligi - NavYawFilt medyan {np.median(wrap(dh[ok3] - yawm)):+.2f} der | yer basligi - YRTK_VN_Heading medyan {np.median(wrap(yh[ok3] - vnh)):+.2f} der')
