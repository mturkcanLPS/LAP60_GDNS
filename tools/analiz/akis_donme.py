"""Optik akis: IMU ile donme temizleme denemesi (kayitlardan, kod degistirmeden).
Gercek referans: tag'lerin gorundugu anlarda kameranin gercek oteleme hizi
    v = -(dp/dt) - w x p      (p: huni merkezi kamera cercevesinde, w: gyro)
Akistan hiz: v_akis = -akis * Z / (f * dt)   (Z = tag z, gercek derinlik)
Karsilastirilan: EMA (gonderilen), RANSAC ham, dokulu ortalama, dokulu + donme temizlenmis.
Kullanim: python3 -s akis_donme.py 0192 [0193 ...]
"""
import sys, csv, io, glob, numpy as np
F_PX = 647.5
OFF = np.array([0.032962, 0.045])   # HUD pos platform merkezine gore -> infra1 kamera cercevesine geri
def rd(f):
    txt = open(f, 'rb').read().replace(b'\x00', b'').decode('utf-8', 'ignore')
    return list(csv.DictReader(io.StringIO(txt)))
def smooth(t, x, w):
    """t'ye gore w saniyelik merkezli kayan ortalama (NaN'lari atlar)"""
    out = np.full_like(x, np.nan)
    ok = np.isfinite(x)
    cs = np.r_[0, np.cumsum(np.where(ok, x, 0))]; cn = np.r_[0, np.cumsum(ok)]
    lo = np.searchsorted(t, t - w / 2); hi = np.searchsorted(t, t + w / 2)
    n = cn[hi] - cn[lo]; out[n > 0] = (cs[hi] - cs[lo])[n > 0] / n[n > 0]
    return out
for s in sys.argv[1:]:
    I = rd(glob.glob(f'/home/lapis/lps_records/imu_s{s}_*.csv')[0])
    Fl = rd(glob.glob(f'/home/lapis/lps_records/flow_s{s}_*.csv')[0])
    H = np.load(f'/home/lapis/huni_kalib/hud_s{s}.npy')
    it = np.array([float(r['stamp_s']) for r in I]); itm = np.array([float(r['t_ms']) for r in I]) / 1000
    g = np.array([[float(r['gx']), float(r['gy']), float(r['gz'])] for r in I])
    kt = np.array([float(r['kam_t']) for r in Fl]); ft = np.array([float(r['t_ms']) for r in Fl]) / 1000
    col = lambda k: np.array([float(r[k]) for r in Fl])
    tex = np.c_[col('tex_dx'), col('tex_dy')]; raw = np.c_[col('raw_dx'), col('raw_dy')]; ema = np.c_[col('ema_dx'), col('ema_dy')]
    # gyro'yu integre et; kare araligindaki donme acisi (kamera saati)
    cg = np.vstack([np.zeros(3), np.cumsum((g[1:] + g[:-1]) / 2 * np.diff(it)[:, None], axis=0)])
    ang = np.array([np.interp(kt, it, cg[:, k]) for k in range(3)]).T
    dang = np.vstack([np.zeros(3), np.diff(ang, axis=0)]); dt = np.r_[np.nan, np.diff(kt)]
    # teori: aci dth kadar donme -> merkezde du = -f*dth_y, dv = +f*dth_x
    rot = np.c_[-F_PX * dang[:, 1], F_PX * dang[:, 0]]
    der = tex - rot
    # tag: HUD t (serial saati) -> kamera saati: flow CSV'deki t_ms ile kam_t arasindaki fark
    kofs = np.median(kt - ft)
    good = np.isfinite(H[:, 1]) & (H[:, 2] == 1) & np.isfinite(H[:, 3:6]).all(1) & (H[:, 5] > 1) & (H[:, 5] < 20) \
           & (np.abs(H[:, 3]) < 5) & (np.abs(H[:, 4]) < 5)
    ht = H[good, 1] + kofs; P = np.c_[H[good, 3] + OFF[0], H[good, 4] + OFF[1], H[good, 5]]
    o = np.argsort(ht); ht, P = ht[o], P[o]
    # tag hizi: 0.6 s pencereli dogrusal egim (1 cm cozunurlukte HUD -> gurultu icin)
    v_tag = np.full((len(ht), 3), np.nan)
    for i in range(len(ht)):
        m = (ht > ht[i] - 0.3) & (ht < ht[i] + 0.3)
        if m.sum() < 4 or ht[m].ptp() < 0.35: continue
        dp = np.array([np.polyfit(ht[m] - ht[i], P[m, k], 1)[0] for k in range(3)])
        w = np.array([np.interp(ht[i], it, g[:, k]) for k in range(3)])
        v_tag[i] = -dp - np.cross(w, P[i])
    # akistan hiz (kamera xy), tag anlarina
    Zt = np.interp(kt, ht, P[:, 2], left=np.nan, right=np.nan)
    # tag'in olmadigi uzun bosluklarda interp yapma
    gap = np.interp(kt, ht, np.r_[0, np.diff(ht)].cumsum(), left=np.nan, right=np.nan)
    near = np.array([np.min(np.abs(ht - x)) if len(ht) else 9 for x in kt]) < 0.15
    Zt[~near] = np.nan
    res = {}
    for name, d in (('EMA (gonderilen)', ema), ('RANSAC ham', raw), ('dokulu ort.', tex), ('dokulu + IMU donme temiz', der)):
        v = -d * Zt[:, None] / (F_PX * dt[:, None])
        vs = np.c_[smooth(kt, v[:, 0], 0.6), smooth(kt, v[:, 1], 0.6)]
        vt = np.c_[np.interp(kt, ht, v_tag[:, 0]), np.interp(kt, ht, v_tag[:, 1])]
        ok = near & np.isfinite(vs).all(1) & np.isfinite(vt).all(1) & (dt < 0.05)
        e = vs[ok] - vt[ok]
        res[name] = (ok.sum(), np.sqrt(np.mean(e ** 2, axis=0)), np.median(np.abs(e), axis=0),
                     [np.corrcoef(vs[ok, k], vt[ok, k])[0, 1] for k in range(2)], np.sqrt(np.mean(vt[ok] ** 2, axis=0)))
    n0 = next(iter(res.values()))[0]
    print(f'\n=== s{s}: tag referansli {n0} kare (Z {np.nanmin(Zt):.1f}-{np.nanmax(Zt):.1f} m), gercek hiz RMS x {res[next(iter(res))][4][0]:.2f} y {res[next(iter(res))][4][1]:.2f} m/s')
    print('   yontem                       | hata RMS x / y (m/s) | hata medyan x / y | korelasyon x / y')
    for name, (n, rms, med, cor, _) in res.items():
        print(f'   {name:28s} | {rms[0]:5.2f} / {rms[1]:5.2f}        | {med[0]:5.2f} / {med[1]:5.2f}     | {cor[0]:5.2f} / {cor[1]:5.2f}')
