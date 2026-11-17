"""Sahte tespit filtre secenekleri: gercek (bag, duvar ID, <2.5 m) vs sahte (bag diger + ucus74 hepsi)."""
import numpy as np, collections
def load(f):
    try: return np.loadtxt(f, ndmin=2)   # kare id ham margin mesafe kenar egim
    except Exception: return np.zeros((0, 7))
B = np.vstack([load('ham_infra1.txt'), load('ham_infra2.txt')]); U = load('ham_ucus74.txt')
wall = np.isin(B[:, 1], [0, 20, 24, 34])
T = B[wall & (B[:, 4] < 2.5)]; Fb = B[~(wall & (B[:, 4] < 2.5))]; F = np.vstack([Fb, U])
print(f'gercek {len(T)} | sahte: ic mekan {len(Fb)}, ucus74 {len(U)} (ucus ID: {dict(collections.Counter(U[:,1].astype(int)).most_common(8))})')
def d(x, c, nm): 
    if len(x): print(f'  {nm:10s} hamming 0/1/2 = {np.mean(x[:,2]==0)*100:4.0f}/{np.mean(x[:,2]==1)*100:4.0f}/{np.mean(x[:,2]==2)*100:4.0f}%  '
                     f'margin p5/p50/p95 {np.percentile(x[:,3],5):5.1f}/{np.median(x[:,3]):5.1f}/{np.percentile(x[:,3],95):5.1f}  '
                     f'kenar px p5/p50 {np.percentile(x[:,5],5):5.1f}/{np.median(x[:,5]):5.1f}  egim p5/p50/p95 {np.percentile(x[:,6],5):3.0f}/{np.median(x[:,6]):3.0f}/{np.percentile(x[:,6],95):3.0f}')
d(T, 0, 'GERCEK'); d(Fb, 0, 'sahte ic'); d(U, 0, 'sahte ucus')
print('\nfiltre                         gercek kalan   sahte ic kalan   sahte ucus kalan   (ucusta duvar ID kalan)')
def show(nm, f):
    uw = np.isin(U[:, 1], [0, 20, 24, 34])
    print(f'  {nm:30s} {np.mean(f(T))*100:6.1f}%  {np.sum(f(Fb)):6d}/{len(Fb):<6d}  {np.sum(f(U)):6d}/{len(U):<6d}  {np.sum(f(U) & uw):5d}')
show('yok (simdiki)', lambda x: np.ones(len(x), bool))
for h in (1, 0): show(f'hamming <= {h}', lambda x, h=h: x[:, 2] <= h)
for m in (20, 30, 40, 50, 60): show(f'margin >= {m}', lambda x, m=m: x[:, 3] >= m)
for k in (14, 18, 22): show(f'kenar >= {k} px', lambda x, k=k: x[:, 5] >= k)
show('hamming<=1 & margin>=30', lambda x: (x[:, 2] <= 1) & (x[:, 3] >= 30))
show('hamming<=1 & margin>=40', lambda x: (x[:, 2] <= 1) & (x[:, 3] >= 40))
show('hamming0 | (h1 & margin>=40)', lambda x: (x[:, 2] == 0) | ((x[:, 2] == 1) & (x[:, 3] >= 40)))
show('hamming<=1 & m>=30 & kenar>=14', lambda x: (x[:, 2] <= 1) & (x[:, 3] >= 30) & (x[:, 5] >= 14))
