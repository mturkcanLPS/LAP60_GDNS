"""Huni ofseti dogrulamasi (overlay'siz EO + eo_sNNNN.csv). Kullanim: python3 -s ofset_dogrula.py 0209
Tag: infra1 HUD (hud_oku). Gonderilen yon: eo CSV bx,by (ofsetli). Ofsetsiz yon geri hesaplanir.
EO CSV zamani RX CSV'ye EKF irtifasiyla hizalanir. Hata = (tag_yon - yon) * tag_z [m]."""
import sys, glob, csv, io, cv2, numpy as np
sys.path.insert(0, '/home/lapis/huni_kalib'); import hud_oku as h
s = sys.argv[1]; D = '/home/lapis/lps_records/'
def rd(f): return list(csv.DictReader(io.StringIO(open(f,'rb').read().replace(b'\x00',b'').decode('utf-8','ignore'))))
def fl(x):
    try: return float(x)
    except: return np.nan
IR = [r for r in rd(glob.glob(D+f'ir_s{s}_*.csv')[0]) if r.get('t_ms')]; T = fl(IR[-1]['t_ms'])/1000
E = [r for r in rd(glob.glob(D+f'rx_s{s}_*.csv')[0]) if r['tip']=='E' and r['ekf_z']]
te = np.array([fl(r['t_ms']) for r in E])/1000; ze = np.array([fl(r['ekf_z']) for r in E])
C = [r for r in rd(glob.glob(D+f'eo_s{s}_*.csv')[0]) if r.get('t_ms')]
tc = np.array([fl(r['t_ms']) for r in C])/1000; ac = np.array([fl(r['irtifa_m']) for r in C])
# EO zamanini EKF ile hizala
best = None
for sh in np.arange(-30, 30, 0.1):
    k = np.isfinite(ac) & (ac > 1)
    zz = np.interp(tc[k]+sh, te, ze); e = np.nanmean(np.abs(zz-ac[k]))
    if best is None or e < best[0]: best = (e, sh)
print(f's{s}: EO->RX kayma {best[1]:+.1f} s (irtifa farki {best[0]:.2f} m)')
tc = tc + best[1]
# tag serisi
fi = glob.glob(D+f'lps_infra1_s{s}_*.avi')[0]; cap = cv2.VideoCapture(fi); NI = 0
while cap.grab(): NI += 1
tg = []; cap = cv2.VideoCapture(fi); i = 0
while cap.grab():
    if i % 2 == 0:
        _, img = cap.retrieve(); roi = img[6:20, 6:110].astype(int)
        if ((roi > 190).all(axis=2)).sum() > ((roi[..., 2] > 170) & (roi[..., 1] < 90) & (roi[..., 0] < 90)).sum():
            r = h.read_frame(img)
            if r[1] == 1 and r[4] > 0.5 and not np.isnan(r[2]+r[3]): tg.append((i/NI*T, r[2]+0.032962, r[3]+0.045, r[4]))
    i += 1
tg = np.array(tg); print(f'  tag okunan {len(tg)} (z {tg[:,3].min():.1f}-{tg[:,3].max():.1f} m)' if len(tg) else '  tag yok')
rows = []
for r, t in zip(C, tc):
    if r['gecerli'] != '1' or not len(tg): continue
    j = np.argmin(np.abs(tg[:,0]-t))
    if abs(tg[j,0]-t) > 0.15: continue
    _, x, y, z = tg[j]; bx, by, alt = fl(r['bx']), fl(r['by']), fl(r['irtifa_m']); dps = fl(r['baslik_farki'])
    k = np.clip((alt-3)/3, 0, 1) if alt > 0.5 else 0.0
    ox, oy = 0.0, 1.45
    if np.isfinite(dps):
        a = -np.radians(dps); ox, oy = np.cos(a)*0-np.sin(a)*1.45, np.sin(a)*0+np.cos(a)*1.45
    b0x, b0y = bx - k*ox/alt, by - k*oy/alt
    rows.append((t, z, alt, dps, r['kanal']=='IR', (x/z-bx)*z, (y/z-by)*z, (x/z-b0x)*z, (y/z-b0y)*z, k))
R = np.array(rows, float); print('  eslesen cift', len(R))
if len(R):
    e1 = np.hypot(R[:,5], R[:,6]); e0 = np.hypot(R[:,7], R[:,8])
    print(f'  kanal IR %{100*R[:,4].mean():.0f} | baslik farki {np.nanmin(R[:,3]):+.1f}..{np.nanmax(R[:,3]):+.1f} der (NaN %{100*np.isnan(R[:,3]).mean():.0f}) | z/EKF orani medyan {np.median(R[:,1]/R[:,2]):.2f}')
    print(f'  HUNIYE UZAKLIK medyan: ofsetsiz {np.median(e0):.2f} m -> ofsetli {np.median(e1):.2f} m | kalan x {np.median(R[:,5]):+.2f} y {np.median(R[:,6]):+.2f} m')
    for lo, hi in ((0,3),(3,6),(6,8),(8,10),(10,14)):
        k = (R[:,1] >= lo) & (R[:,1] < hi)
        if k.sum() >= 5: print(f'    tag z {lo:2d}-{hi:2d} m n={k.sum():4d}: ofsetsiz {np.median(e0[k]):.2f} -> ofsetli {np.median(e1[k]):.2f} m | kalan x {np.median(R[k,5]):+.2f} y {np.median(R[k,6]):+.2f} | agirlik {np.median(R[k,9]):.2f}')
    np.save(f'/home/lapis/huni_kalib/dogrula{s}.npy', R)
