"""ID1/2の既存チャープのモデル追従と通信レートを比較する。"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser()
p.add_argument('directories', nargs='+', type=Path)
p.add_argument('--output', required=True, type=Path)
a=p.parse_args(); a.output.mkdir(parents=True, exist_ok=True)
fig, axes=plt.subplots(3,1,figsize=(12,10),sharex=True)
summary=[]
for out in a.directories:
    m=json.loads((out/'metadata.json').read_text()); mid=m['velocity_motor']
    report=json.loads((out/'velocity_sweep.json').read_text()); rows=report['windows']
    f=np.array([r['frequency_hz'] for r in rows])
    gain=np.array([r['raw_model_gain'] for r in rows])
    phase=np.array([r['position_model_phase_deg']+r['raw_input_phase_deg']-r['position_input_phase_deg'] for r in rows])
    error=abs(gain*np.exp(1j*np.deg2rad(phase))-1)
    for ax, y in zip(axes, (gain,phase,[r.get('raw_velocity_residual_rms_deg_s',np.nan) for r in rows])):
        ax.semilogx(f,y,'o-',label=m['tag'])
    active=[r for r in csv.DictReader((out/'feedback.csv').open())
            if r['stage']=='velocity_chirp' and r['type']=='1' and int(r['id'])==mid]
    start=min(float(r['f7_time']) for r in active); end=max(float(r['f7_time']) for r in active)
    stop=[r for r in csv.DictReader((out/'feedback.csv').open())
          if r['type']=='1' and int(r['id'])==mid and end+1<float(r['f7_time'])<end+4.8]
    stop_position=np.array([float(r['position']) for r in stop])
    stop_velocity=np.array([float(r['velocity']) for r in stop])
    rate=[r for r in csv.DictReader((out/'rate/rate.csv').open()) if start+1<=float(r['tick'])/1000<=end]
    summary.append(dict(directory=str(out),motor=mid,tag=m['tag'],
        error_below3hz=float(error[f<3].mean()),error_all=float(error.mean()),
        command_peak_A=report['current_peak_A'],position_min=report['position_min_deg'],position_max=report['position_max_deg'],
        stop_position_pp_deg=float(np.ptp(stop_position)) if len(stop) else None,
        stop_raw_velocity_rms_deg_s=float(np.sqrt(np.mean(stop_velocity**2))) if len(stop) else None,
        complete_rate_windows=len(rate),
        rates={k:[min(int(r[k]) for r in rate),max(int(r[k]) for r in rate)] for k in (f'target{mid}',f'type2_{mid}') } if rate else {},
        errors={k:max(int(r[k]) for r in rate) for k in ('ring_overrun','priority_full','tx_errors','can_errors')} if rate else {}))
axes[0].axhline(1,color='k',ls='--'); axes[1].axhline(0,color='k',ls='--')
for ax,title in zip(axes,('Raw velocity / reference gain','Raw velocity / reference phase [deg]','Raw velocity fit residual RMS [deg/s]')):
    ax.set_ylabel(title); ax.grid(); ax.legend(fontsize=8)
axes[-1].set_xlabel('Frequency [Hz]')
fig.suptitle('Local chirp fits: high-frequency small motion / noise caveat')
fig.tight_layout(); fig.savefig(a.output/'comparison.png',dpi=150)
(a.output/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
