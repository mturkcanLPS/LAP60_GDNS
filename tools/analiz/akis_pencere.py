import sys, numpy as np
for S in ('0192', '0193'):
    sys.argv = ['x', S]
    exec(open('akis_donme.py').read().split("    res = {}")[0])
    def tagp(t0, w=0.25):
        m = np.abs(ht - t0) < w
        return np.median(P[m], axis=0) if m.sum() >= 2 else None
    print(f'\n=== s{S}')
    for W in (1.5, 3.0):
        out = {k: [] for k in ('EMA (gonderilen)', 'RANSAC ham', 'dokulu ort.', 'dokulu + IMU donme')}
        ref = []
        for t0 in np.arange(kt[0] + 1, kt[-1] - W - 1, W / 2):
            p0, p1 = tagp(t0), tagp(t0 + W)
            if p0 is None or p1 is None: continue
            m = (kt > t0) & (kt <= t0 + W) & np.isfinite(dt) & (dt < 0.05)
            if m.sum() < 0.8 * W * 25: continue
            Z = 0.5 * (p0[2] + p1[2])
            # gercek oteleme (kamera cercevesi, xy): -dp - sum(dtheta x p)
            th = ang[np.searchsorted(kt, t0 + W)] - ang[np.searchsorted(kt, t0)]
            tr = -(p1 - p0) - np.cross(th, 0.5 * (p0 + p1))
            ref.append(tr[:2])
            for name, d in (('EMA (gonderilen)', ema), ('RANSAC ham', raw), ('dokulu ort.', tex), ('dokulu + IMU donme', der)):
                out[name].append(-d[m].sum(0) * Z / F_PX)
        ref = np.array(ref)
        print(f'  pencere {W} s: {len(ref)} pencere | gercek yer degistirme RMS x {np.sqrt(np.mean(ref[:,0]**2)):.2f} y {np.sqrt(np.mean(ref[:,1]**2)):.2f} m')
        for name, v in out.items():
            v = np.array(v); e = v - ref
            c = [np.corrcoef(v[:, k], ref[:, k])[0, 1] for k in range(2)]
            sl = [np.polyfit(ref[:, k], v[:, k], 1)[0] for k in range(2)]
            print(f'    {name:20s} hata RMS x {np.sqrt(np.mean(e[:,0]**2)):.2f} y {np.sqrt(np.mean(e[:,1]**2)):.2f} m | korelasyon x {c[0]:.2f} y {c[1]:.2f} | olcek x {sl[0]:.2f} y {sl[1]:.2f}')
