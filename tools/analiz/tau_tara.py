"""Gyroflow kare-gyro zaman kaymasi (tau) taramasi. Kullanim: python3 -s tau_tara.py <oturum> [<oturum> ...]
Havadaki kareler (flow CSV all_dx/dy + IMU), sabit OKAD kazanci; kalan robust std en kucuk tau en iyisi."""
import sys, glob, csv, io, numpy as np
D = '/home/lapis/lps_records/'
def rd(p): return list(csv.DictReader(io.StringIO(open(glob.glob(D + p)[0], 'rb').read().replace(b'\x00', b'').decode('utf-8', 'ignore'))))
mad = lambda e: 1.4826 * np.median(np.abs(e - np.median(e)))
for s in sys.argv[1:]:
    FL = [r for r in rd(f'flow_s{s}_*.csv') if r.get('durum') == '0']; f = lambda k: np.array([float(r[k]) for r in FL])
    kt, adx, ady, q = f('kam_t'), f('all_dx'), f('all_dy'), f('raw_q')
    I = rd(f'imu_s{s}_*.csv'); it = np.array([float(r['stamp_s']) for r in I]); gx = np.array([float(r['gx']) for r in I]); gy = np.array([float(r['gy']) for r in I])
    o = np.argsort(it); it, gx, gy = it[o], gx[o], gy[o]
    ax = np.r_[0, np.cumsum(0.5 * (gx[1:] + gx[:-1]) * np.diff(it))]; ay = np.r_[0, np.cumsum(0.5 * (gy[1:] + gy[:-1]) * np.diff(it))]
    dt = np.r_[np.nan, np.diff(kt)]; idx = np.where((dt > 0.015) & (dt < 0.1) & (q > 40))[0]   # havada (kaliteli doku)
    idx = idx[idx > 0]
    sat = []
    for tau in np.arange(-0.015, 0.0201, 0.0025):
        t1, t0 = kt[idx] - tau, kt[idx - 1] - tau
        thx = np.interp(t1, it, ax) - np.interp(t0, it, ax); thy = np.interp(t1, it, ay) - np.interp(t0, it, ay)
        sat.append((tau, mad(ady[idx] - 740 * thx), mad(adx[idx] + 845 * thy)))
    S = np.array(sat); b = S[np.argmin(S[:, 1] + S[:, 2])]
    print(f's{s} (n={len(idx)}, gyro RMS x {np.degrees(np.std(gx)):.1f} der/s): en iyi tau {b[0]*1000:+.1f} ms | kalan dy/dx: tau=10ms {S[np.argmin(abs(S[:,0]-0.01)),1]:.2f}/{S[np.argmin(abs(S[:,0]-0.01)),2]:.2f} -> en iyi {b[1]:.2f}/{b[2]:.2f} px')
    print('    tau(ms):', ' '.join(f'{t*1000:+.1f}:{a+c:.2f}' for t, a, c in S))
