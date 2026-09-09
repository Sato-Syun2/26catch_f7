"""Compare measured, amplitude-normalized DOB responses; no fitted time shift."""
import sys
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal

paths = [Path(x) for x in sys.argv[1:]]
fig, axes = plt.subplots(len(paths), 2, figsize=(14, 3.6*len(paths)), squeeze=False)
summary = []
for row, p in enumerate(paths):
    meta = json.loads((p/'metadata.json').read_text())
    stats = json.loads((p/'analysis.json').read_text())
    d = np.genfromtxt(p/'feedback.csv', delimiter=',', names=True, dtype=None, encoding='utf8')
    d = d[np.isin(d['stage'], ['pre', 'chirp', 'post'])]
    t = d['time']-d['time'][0]
    u = d['target']
    v = d['velocity']
    ref = np.zeros(len(t))
    ref[0] = v[0]
    for i in range(1, len(t)):
        a = np.exp(-meta['alpha']*(t[i]-t[i-1]))
        ref[i] = a*ref[i-1]+(1-a)*u[i-1]
    amp = meta['amplitude']
    ax = axes[row,0]
    ax.plot(t, v/amp, lw=.6, label='Measured / command amplitude')
    ax.plot(t, ref/amp, lw=.9, label='Reference / command amplitude')
    ax.set_ylabel('Normalized velocity')
    ax.set_title(f'{amp:g} mm/s command; RMSE {stats["velocity_rmse_mm_s"]:.2f} mm/s')
    ax.legend(fontsize=8)
    ax = axes[row,1]
    ax.plot(t, v, lw=.8, label='Measured')
    ax.plot(t, u, lw=.7, label='Command')
    ax.plot(t, ref, lw=.9, label='Reference')
    ax.set_xlim(2,5)
    ax.set_ylabel('Velocity [mm/s]')
    ax.set_title('Start of chirp (1.5 Hz region)')
    ax.legend(fontsize=8)
    summary.append(dict(path=str(p.resolve()), metadata=meta, analysis=stats))
for ax in axes.flat:
    ax.set_xlabel('Time from pre-test [s]')
    ax.grid(alpha=.3)
fig.suptitle('ID4 DOB vs 10/(s+10): 1.5–10 Hz, 60 s, current limit 4 A')
fig.tight_layout()
out = paths[-1].parent / ('dob_comparison_'+paths[-1].name.removeprefix('dob_chirp_'))
out.mkdir(exist_ok=True)
fig.savefig(out/'comparison.png', dpi=160)
(out/'summary.json').write_text(json.dumps(summary,indent=2))
print(out.resolve())

# ホスト受信時刻で等間隔化した近似FRF。時刻合わせの最適化は行わない。
fig, axes = plt.subplots(3, 1, figsize=(10,10), sharex=True)
for p in paths:
    meta = json.loads((p/'metadata.json').read_text())
    d = np.genfromtxt(p/'feedback.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
    d = d[d['stage']=='chirp']
    t = d['time']
    grid = np.arange(t[0], t[-1], .01)
    u = d['target'][np.searchsorted(t,grid,side='right')-1]
    v = np.interp(grid,t,d['velocity'])
    opts = dict(fs=100,nperseg=1024,noverlap=512)
    f, pu = signal.welch(u,**opts)
    _, pv = signal.welch(v,**opts)
    _, cross = signal.csd(u,v,**opts)
    h = cross/np.maximum(pu,1e-20)
    coh = abs(cross)**2/np.maximum(pu*pv,1e-20)
    b = (f>=1.5)&(f<=10)
    label = f'{meta["amplitude"]:g} mm/s'
    gain = np.where(coh>=.8,20*np.log10(np.maximum(abs(h),1e-20)),np.nan)
    phase = np.where(coh>=.8,np.angle(h,deg=True),np.nan)
    axes[0].plot(f[b],gain[b],label=label)
    axes[1].plot(f[b],phase[b],label=label)
    axes[2].plot(f[b],coh[b],label=label)
f = np.geomspace(1.5,10,200)
h = 10/(10+2j*np.pi*f)
axes[0].plot(f,20*np.log10(abs(h)),'k--',label='Reference 10/(s+10)')
axes[1].plot(f,np.angle(h,deg=True),'k--')
for ax in axes:
    ax.set_xscale('log')
    ax.grid(True,which='both',alpha=.3)
axes[0].set_ylabel('Gain [dB]');axes[0].legend()
axes[1].set_ylabel('Phase [deg]')
axes[2].set_ylabel('Coherence');axes[2].set_ylim(0,1.05)
axes[2].set_xlabel('Frequency [Hz]')
fig.suptitle('Approximate chirp FRF (Welch); gain/phase hidden where coherence < 0.8\nIncludes ROS/CAN timing; no fitted delay compensation')
fig.tight_layout()
fig.savefig(out/'frequency_response.png',dpi=160)
