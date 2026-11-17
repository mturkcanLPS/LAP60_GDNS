import sys, numpy as np
sys.argv = ['x', '0193']
exec(open('akis_donme.py').read().split("    res = {}")[0])
v_der = -der * Zt[:, None] / (F_PX * dt[:, None])
vd = np.c_[smooth(kt, v_der[:, 0], 0.6), smooth(kt, v_der[:, 1], 0.6)]
print('tag hiz ornek sayisi', np.isfinite(v_tag[:, 0]).sum(), '| tag pos gurultusu (ardisik fark std, m):', np.round(np.nanstd(np.diff(P, axis=0), axis=0), 3))
for lag in (-0.4, -0.2, 0.0, 0.2, 0.4):
    vt = np.c_[np.interp(kt + lag, ht, v_tag[:, 0]), np.interp(kt + lag, ht, v_tag[:, 1])]
    ok = near & np.isfinite(vd).all(1) & np.isfinite(vt).all(1) & (dt < 0.05)
    C = np.array([[np.corrcoef(vd[ok, a], vt[ok, b])[0, 1] for b in range(2)] for a in range(2)])
    print(f'gecikme {lag:+.1f} s: korelasyon akis(x,y) x tag(x,y) = {np.round(C, 2).tolist()}  n={ok.sum()}')
# tag pozisyonunun kendisi ile akisin integrali (birikmis yer degistirme) karsilastirmasi
