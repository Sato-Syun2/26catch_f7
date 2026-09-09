"""同条件の肘チャープを未LPF速度のモデル追従・停止振動で比較する。"""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('directories',nargs='+',type=Path);p.add_argument('--output',required=True,type=Path)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
fig,ax=plt.subplots(2,2,figsize=(12,9));summary=[]
for out in a.directories:
    meta=json.loads((out/'metadata.json').read_text());report=json.loads((out/'velocity_sweep.json').read_text());w=report['windows'];label=meta['tag']
    f=np.array([r['frequency_hz'] for r in w]);gain=np.array([r['raw_model_gain'] for r in w])
    phase=np.array([r['position_model_phase_deg']+r['raw_input_phase_deg']-r['position_input_phase_deg'] for r in w])
    rel=gain*np.exp(1j*np.deg2rad(phase));error=abs(rel-1)
    ax[0,0].semilogx(f,gain,'o-',label=label);ax[0,1].semilogx(f,phase,'o-',label=label)
    fb=list(csv.DictReader((out/'feedback.csv').open()));active=[r for r in fb if r['stage']=='velocity_chirp' and r['type']=='1' and r['id']=='1']
    start=min(float(r['f7_time']) for r in active);end=max(float(r['f7_time']) for r in active)
    dob=list(csv.DictReader((out/'dob.csv').open()));stop=[r for r in dob if end+1<float(r['tick'])/1000<end+4.8]
    t=np.array([float(r['tick'])/1000-end for r in stop]);pos=np.array([float(r['position']) for r in stop]);cur=np.array([float(r['current']) for r in stop])
    if len(t):
        ax[1,0].plot(t,pos-pos.mean(),label=label);ax[1,1].plot(t,cur,label=label,alpha=.7)
    rate=list(csv.DictReader((out/'rate/rate.csv').open()));rate=[r for r in rate if start+1<=float(r['tick'])/1000<=end]
    summary.append(dict(directory=str(out),tag=label,raw_complex_relative_error_mean=float(error.mean()),raw_complex_relative_error_below3hz=float(error[f<3].mean()),raw_gain_at_last=float(gain[-1]),raw_phase_at_last=float(phase[-1]),stop_position_pp=float(np.ptp(pos)) if len(pos) else None,stop_command_rms=float(np.sqrt(np.mean(cur*cur))) if len(cur) else None,current_peak=report['current_peak_A'],whole_rate_windows=len(rate),target1_range=[min(int(r['target1']) for r in rate),max(int(r['target1']) for r in rate)] if rate else [],type2_1_range=[min(int(r['type2_1']) for r in rate),max(int(r['type2_1']) for r in rate)] if rate else [],errors={k:max(int(r[k]) for r in rate) for k in ['ring_overrun','priority_full','tx_errors','can_errors']} if rate else {}))
ax[0,0].axhline(1,color='k',ls='--');ax[0,1].axhline(0,color='k',ls='--')
for x in ax[0]:x.set_xlabel('Frequency [Hz]')
for x in ax[1]:x.set_xlabel('Seconds after chirp end')
ax[0,0].set_ylabel('Raw Type2 velocity / reference model gain');ax[0,1].set_ylabel('Raw Type2 velocity / model phase [deg]')
ax[1,0].set_ylabel('Stop position minus mean [deg]');ax[1,1].set_ylabel('Stop current command [A]')
for x in ax.flat:x.grid();x.legend(fontsize=7)
fig.suptitle('Elbow tuning: 60 deg/s, 0.5–10 Hz, 30 s; high-frequency small-motion caveat')
fig.tight_layout();fig.savefig(a.output/'comparison.png',dpi=150)
(a.output/'comparison.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
