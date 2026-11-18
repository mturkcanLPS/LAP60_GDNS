"""Kamera montaj acisi, YERDE hareketsizken: D435i ivme (kameranin yercekimine gore egimi) - otopilot
roll/pitch (govdenin egimi). RTK gerektirmez. Kullanim: montaj_yer.py <mat> <oturum> [<mat> <oturum> ...]
Kamera ekseni: x = govde sag, y = govde geri, z = asagi (car bearing analiziyle ayni)."""
import sys, glob, csv, io, numpy as np, scipy.io as sio
D = '/home/lapis/lps_records/'
def rd(p): return list(csv.DictReader(io.StringIO(open(glob.glob(D + p)[0], 'rb').read().replace(b'\x00', b'').decode('utf-8', 'ignore'))))
a = sys.argv[1:]
for mat, ses in zip(a[::2], a[1::2]):
    m = sio.loadmat(mat, squeeze_me=True); F = lambda k: m[k].astype(float)
    T = (m['timeusec'] - m['timeusec'][0]) / 1e6
    ots = F('GDNS_OpTimestamp'); ch = np.where(np.r_[True, np.diff(ots) != 0])[0]; tc = T[ch]; u = np.empty(len(ch)); u[0] = ots[ch[0]]
    for q in range(1, len(ch)):
        b = u[q - 1] + (tc[q] - tc[q - 1]) * 1000; u[q] = ots[ch[q]] + np.round((b - ots[ch[q]]) / 65536) * 65536
    FL = [r for r in rd(f'flow_s{ses}_*.csv') if r.get('t_ms')]; ft = np.array([float(r['t_ms']) for r in FL])
    k = max(range(-2, 40), key=lambda k: (np.abs(ft[np.clip(np.searchsorted(ft, u + k * 65536), 0, len(ft) - 1)] - (u + k * 65536)) < 0.5).sum())
    u += k * 65536
    I = rd(f'imu_s{ses}_*.csv'); ti = np.array([float(r['t_ms']) for r in I]); A = np.array([[float(r[c]) for c in ('ax', 'ay', 'az')] for r in I])
    tm = np.interp(ti, u, tc)
    ro, pi_ = np.interp(tm, T, F('NavRollFilt')), np.interp(tm, T, F('NavPitchFilt'))
    hz = np.interp(tm, T, F('ekfOutPosZ')); mot = np.interp(tm, T, F('motor1'))
    ok = (tm > tc[0] + 1) & (tm < tc[-1] - 1) & (hz < 1.0)
    # 1 s'lik hareketsiz dilimler
    rows = []
    for s in np.arange(ti[ok].min(), ti[ok].max(), 1000):
        w = ok & (ti >= s) & (ti < s + 1000)
        if w.sum() < 150: continue
        aw = A[w]; n = np.linalg.norm(aw, axis=1)
        if np.std(n) > 0.05: continue                      # hareket / motor titresimi
        g = -aw.mean(0); g /= np.linalg.norm(g)            # kamera ekseninde yercekimi (asagi)
        kx = np.degrees(np.arctan2(g[0], g[2])); ky = np.degrees(np.arctan2(g[1], g[2]))
        # govdeden beklenen (montaj 0): g_govde = [-sin p, sin r cos p, cos r cos p] (ileri, sag, asagi)
        r_, p_ = np.radians(ro[w].mean()), np.radians(pi_[w].mean())
        gb = np.array([-np.sin(p_), np.sin(r_) * np.cos(p_), np.cos(r_) * np.cos(p_)])
        bx_ = np.degrees(np.arctan2(gb[1], gb[2])); by_ = np.degrees(np.arctan2(-gb[0], gb[2]))
        rows.append((kx, ky, bx_, by_, mot[w].mean()))
    R = np.array(rows)
    if not len(R): print(mat.split('/')[-1], ses, 'hareketsiz yer dilimi yok'); continue
    dx, dy = R[:, 0] - R[:, 2], R[:, 1] - R[:, 3]
    print(f'{mat.split("/")[-1]:28s} s{ses}: {len(R):4d} s yerde hareketsiz | kamera egimi x {np.median(R[:,0]):+.2f} y {np.median(R[:,1]):+.2f} | govde (otopilot) x {np.median(R[:,2]):+.2f} y {np.median(R[:,3]):+.2f} | MONTAJ FARKI x {np.median(dx):+.2f} y {np.median(dy):+.2f} der (std {np.std(dx):.2f}/{np.std(dy):.2f})')
