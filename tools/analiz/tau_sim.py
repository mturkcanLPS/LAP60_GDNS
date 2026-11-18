import sys, glob, csv, io, numpy as np
D='/home/lapis/lps_records/'
def rd(p): return list(csv.DictReader(io.StringIO(open(glob.glob(D+p)[0],'rb').read().replace(b'\x00',b'').decode('utf-8','ignore'))))
mad=lambda e: 1.4826*np.median(np.abs(e-np.median(e)))
import os
TAUS=np.arange(-0.005,0.0151,0.0025); A=1/float(os.environ.get('N','300')); CAP=25.0; HYS=0.95; INIT=0.010
for s in sys.argv[1:]:
    FL=[r for r in rd(f'flow_s{s}_*.csv') if r.get('durum')=='0']; f=lambda k: np.array([float(r[k]) for r in FL])
    kt,adx,ady,q=f('kam_t'),f('all_dx'),f('all_dy'),f('raw_q')
    I=rd(f'imu_s{s}_*.csv'); it=np.array([float(r['stamp_s']) for r in I]); gx=np.array([float(r['gx']) for r in I]); gy=np.array([float(r['gy']) for r in I])
    o=np.argsort(it); it,gx,gy=it[o],gx[o],gy[o]
    ax=np.r_[0,np.cumsum(0.5*(gx[1:]+gx[:-1])*np.diff(it))]; ay=np.r_[0,np.cumsum(0.5*(gy[1:]+gy[:-1])*np.diff(it))]
    dt=np.r_[np.nan,np.diff(kt)]; idx=np.where((dt>0.015)&(dt<0.1))[0]; idx=idx[idx>0]
    C=np.zeros((len(idx),len(TAUS),2))
    for j,tau in enumerate(TAUS):
        t1,t0=kt[idx]-tau,kt[idx-1]-tau; nrm=(1/30)/dt[idx]
        C[:,j,0]=(adx[idx]+845*(np.interp(t1,it,ay)-np.interp(t0,it,ay)))*nrm
        C[:,j,1]=(ady[idx]-740*(np.interp(t1,it,ax)-np.interp(t0,it,ax)))*nrm
    ema=np.zeros(len(TAUS)); n=0; sel=int(np.argmin(abs(TAUS-INIT))); out=np.zeros((len(idx),2)); chosen=np.zeros(len(idx)); sw=0
    for k in range(len(idx)):
        out[k]=C[k,sel]; chosen[k]=TAUS[sel]
        if q[idx[k]]>=30:
            e=np.minimum((C[k]**2).sum(1),CAP); ema=e if n==0 else ema+A*(e-ema); n+=1
            if n>=150 and k%30==0:
                b=int(np.argmin(ema))
                if b!=sel and ema[b]<HYS*ema[sel]: sel=b; sw+=1
    hv=q[idx]>=40
    # ilk kalkis: kalite >=40 olan ilk 5 s'lik kesintisiz bolge
    hvs=np.convolve(hv,np.ones(150),'same')>=140; t_up=tt0=None
    fix=C[:,np.argmin(abs(TAUS-0.010))]; best=[mad(C[hv,j,0])+mad(C[hv,j,1]) for j in range(len(TAUS))]
    tt=kt[idx]-kt[idx][0]
    up=tt[np.argmax(hvs)]; bt=TAUS[int(np.argmin([mad(C[hv,j,0])+mad(C[hv,j,1]) for j in range(len(TAUS))]))]
    near=np.abs(chosen-bt)<=0.0026; ia=np.where(near&(tt>=up))[0]
    print(f'   s{s}: kalkis ~{up:.0f} s, en iyi {bt*1000:+.1f} ms; +-2.5 ms icine kalkistan {tt[ia[0]]-up if len(ia) else -1:.0f} s sonra, havada yakin kalma %{100*np.mean(near[hv]):.0f}')
    print(f's{s}: secilen tau (ms) 30 s/2 dk/10 dk/son: {chosen[np.searchsorted(tt,30)]*1000:+.1f}/{chosen[np.searchsorted(tt,120)]*1000:+.1f}/{chosen[min(np.searchsorted(tt,600),len(tt)-1)]*1000:+.1f}/{chosen[-1]*1000:+.1f} | gecis {sw} | en iyi sabit {TAUS[int(np.argmin(best))]*1000:+.1f} | kalan dx+dy (havada): sabit10 {mad(fix[hv,0])+mad(fix[hv,1]):.2f} -> oto {mad(out[hv,0])+mad(out[hv,1]):.2f} (en iyi sabit {min(best):.2f}) px')
