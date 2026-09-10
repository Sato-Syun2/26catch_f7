"""内蔵PP保持とDOB速度0保持の位置変動を比較。電流ソースは別表示。"""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('directories',nargs='+',type=Path);p.add_argument('--output',required=True,type=Path);p.add_argument('--seconds',type=float,default=30.);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
fig,ax=plt.subplots(3,1,figsize=(12,10),sharex=True);summary=[]
for out in a.directories:
 meta=json.loads((out/'metadata.json').read_text());stage='dob_zero_hold' if meta.get('hold_only') else ('velocity_zero' if meta.get('dob') else 'position_hold')
 rows=[r for r in csv.DictReader((out/'feedback.csv').open()) if r['type']=='1' and r['id']=='1' and r['stage']==stage]
 if not rows:raise ValueError('missing hold '+str(out))
 start=float(rows[0]['f7_time']);rows=[r for r in rows if float(r['f7_time'])-start<a.seconds]
 t=np.array([float(r['f7_time']) for r in rows]);t-=t[0]
 pos=np.array([float(r['position']) for r in rows]);cur=np.array([float(r['current']) for r in rows]);vel=np.array([float(r['velocity']) for r in rows]);label=meta['tag']
 detrend=pos-np.polyval(np.polyfit(t,pos,1),t)
 metrics=dict(directory=str(out),tag=label,samples=len(t),duration=float(t[-1]),position_start=float(pos[0]),position_end=float(pos[-1]),position_min=float(pos.min()),position_max=float(pos.max()),position_peak_to_peak=float(np.ptp(pos)),net_drift=float(pos[-1]-pos[0]),detrended_position_rms=float(np.sqrt(np.mean(detrend*detrend))),current_source=meta['current_feedback_source'],current_rms=float(np.sqrt(np.mean(cur*cur))),velocity_rms=float(np.sqrt(np.mean(vel*vel))),max_fb_gap=float(np.diff(t).max()),windows=[])
 for lo in range(0,30,5):
  selected=(t>=lo)&(t<lo+5)
  if selected.any():metrics['windows'].append(dict(start=lo,position_peak_to_peak=float(np.ptp(pos[selected])),position_mean=float(pos[selected].mean())))
 summary.append(metrics)
 ax[0].plot(t,pos,label=label,lw=.8)
 ax[1].plot(t,pos-pos[0],label=label,lw=.8)
 ax[2].plot(t,cur,label=label+' : '+meta['current_feedback_source'],lw=.7)
ax[0].set_ylabel('Position [deg]');ax[1].set_ylabel('Position - initial [deg]');ax[2].set_ylabel('Current [different reference units]');ax[2].set_xlabel('Hold elapsed [s]')
for x in ax:x.grid();x.legend(fontsize=7)
fig.suptitle('PP position hold vs DOB zero velocity; different control objectives / feedback quantization')
fig.tight_layout();fig.savefig(a.output/'hold_comparison.png',dpi=150)
(a.output/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
