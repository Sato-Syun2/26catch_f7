"""周波数軸で速度振幅を正規化して比較する。異なる掃引を時間伸縮しない。"""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('directories',nargs='+',type=Path);p.add_argument('--output',type=Path,required=True)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
fig,axes=plt.subplots(3,2,figsize=(13,11));runs=[]
for path in a.directories:
    meta=json.loads((path/'metadata.json').read_text());s=json.loads((path/'velocity_sweep.json').read_text())
    rows=s['windows'];f=np.array([r['frequency_hz'] for r in rows]);amp=meta['velocity_amplitude_deg_s']
    gain=np.array([r['position_input_gain'] for r in rows]);phase=np.array([r['position_input_phase_deg'] for r in rows])
    label=f'{amp:g} deg/s ({meta["frequency_hz"]:g}-{meta["frequency_end_hz"]:g} Hz)'
    axes[0,0].semilogx(f,60*gain,'o-',label=label)
    axes[0,1].semilogx(f,phase,'o-',label=label)
    axes[1,0].semilogx(f,[r['position_model_gain'] for r in rows],'o-',label=label)
    axes[1,1].semilogx(f,[r['filtered_model_gain'] for r in rows],'o-',label=label)
    axes[2,0].semilogx(f,[100*r['current_saturation_fraction'] for r in rows],'o-',label=label)
    axes[2,1].semilogx(f,[r['position_peak_to_peak_deg'] for r in rows],'o-',label=label)
    runs.append(dict(source=str(path),amplitude=amp,result=meta['result'],frequency=f.tolist(),gain=gain.tolist(),phase=phase.tolist(),
                     raw_gain=[r['raw_input_gain'] for r in rows],raw_phase=[r['raw_input_phase_deg'] for r in rows],
                     current_peak_A=s['current_peak_A'],position_min_deg=s['position_min_deg'],position_max_deg=s['position_max_deg']))
fr=np.geomspace(.5,10,200);h=1/(1+1j*2*np.pi*fr*.1)
axes[0,0].semilogx(fr,60*abs(h),'k--',label='Ideal tau=0.1s')
axes[0,1].semilogx(fr,np.angle(h,deg=True),'k--',label='Ideal tau=0.1s')
axes[1,0].axhline(1,color='k',ls='--');axes[1,1].axhline(1,color='k',ls='--')
axes[2,1].axhline(5,color='k',ls=':',label='5deg p-p (backlash caution below)')
labels=['Velocity scaled to 60deg/s input [deg/s]','Velocity / input phase [deg]',
        'Position-derived velocity / model gain','Filtered Type2 velocity / model gain',
        'Current saturation samples [%]','Fitted motion [deg p-p]']
for ax,label in zip(axes.flat,labels):
    ax.set_ylabel(label);ax.set_xlabel('Frequency [Hz]');ax.grid(True);ax.legend(fontsize=7)
fig.suptitle('Elbow amplitude consistency: same-frequency comparison, not time scaling')
fig.tight_layout();fig.savefig(a.output/'amplitude_comparison.png',dpi=160)
lo=max(2.,max(min(r['frequency']) for r in runs));hi=min(max(r['frequency']) for r in runs)
grid=np.geomspace(lo,hi,30) if hi>lo else np.array([])
responses=[]
for r in runs:
    h=np.array(r['gain'])*np.exp(1j*np.deg2rad(r['phase']))
    responses.append(np.interp(grid,r['frequency'],h.real)+1j*np.interp(grid,r['frequency'],h.imag))
pairs=[]
for i in range(1,len(runs)):
    if not len(grid): continue
    ratio=responses[i]/responses[0]
    pairs.append(dict(reference_amplitude=runs[0]['amplitude'],amplitude=runs[i]['amplitude'],
        gain_ratio_median=float(np.median(abs(ratio))),gain_ratio_min=float(min(abs(ratio))),gain_ratio_max=float(max(abs(ratio))),
        relative_phase_abs_median_deg=float(np.median(abs(np.angle(ratio,deg=True)))),
        relative_complex_difference_median=float(np.median(abs(ratio-1)))))
report=dict(runs=runs,common_frequency_range_hz=[lo,hi],pairs=pairs,
    note='Interpolated local complex frequency responses. Saturation, posture drift, backlash, and different sweep rates can break apparent consistency; amplitude scaling does not correct these effects.')
rawfig,rawaxes=plt.subplots(2,1,figsize=(10,8),sharex=True)
rawresponses=[]
for r in runs:
    phase=np.deg2rad(r['raw_phase']);hraw=np.array(r['raw_gain'])*np.exp(1j*phase)
    rawresponses.append(np.interp(grid,r['frequency'],hraw.real)+1j*np.interp(grid,r['frequency'],hraw.imag))
    rawaxes[0].semilogx(r['frequency'],60*np.array(r['raw_gain']),'o-',label=f'{r["amplitude"]:g}deg/s input')
    rawaxes[1].semilogx(r['frequency'],r['raw_phase'],'o-')
rawpairs=[]
for i in range(1,len(runs)):
    if not len(grid): continue
    ratio=rawresponses[i]/rawresponses[0]
    rawpairs.append(dict(reference_amplitude=runs[0]['amplitude'],amplitude=runs[i]['amplitude'],
        gain_ratio_median=float(np.median(abs(ratio))),gain_ratio_min=float(min(abs(ratio))),gain_ratio_max=float(max(abs(ratio))),
        relative_phase_abs_median_deg=float(np.median(abs(np.angle(ratio,deg=True)))),
        relative_complex_difference_median=float(np.median(abs(ratio-1)))))
report['raw_type2_pairs']=rawpairs
rawaxes[0].semilogx(fr,60/np.sqrt(1+(2*np.pi*fr*.1)**2),'k--',label='Ideal tau=0.1s')
rawaxes[1].semilogx(fr,-np.rad2deg(np.arctan(2*np.pi*fr*.1)),'k--')
rawaxes[0].set_ylabel('Raw Type2 velocity scaled to 60deg/s input [deg/s]')
rawaxes[1].set_ylabel('Raw Type2 velocity / input phase [deg]');rawaxes[1].set_xlabel('Frequency [Hz]')
rawaxes[0].legend()
for ax in rawaxes: ax.grid(True)
rawfig.tight_layout();rawfig.savefig(a.output/'raw_velocity_comparison.png',dpi=160)
(a.output/'amplitude_comparison.json').write_text(json.dumps(report,indent=2))
print(json.dumps(dict(common_frequency_range_hz=[lo,hi],pairs=pairs,raw_type2_pairs=rawpairs),indent=2))
