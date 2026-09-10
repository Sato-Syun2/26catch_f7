"""Mode5位置ステップの実測とMPC速度指令をまとめる。"""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args();out=a.directory
m=json.loads((out/'metadata.json').read_text());mid=m['position_motor']
fb=[r for r in csv.DictReader((out/'feedback.csv').open()) if r['type']=='1' and int(r['id'])==mid]
active=[r for r in fb if r['stage']==f'center_{mid}']
t0=min(float(r['f7_time']) for r in active);end=max(float(r['f7_time']) for r in active)
t=np.array([float(r['f7_time'])-t0 for r in active]);q=np.array([float(r['position']) for r in active]);target=m['position_target']
error=q-target;tail=t>t[-1]-1
outside=np.flatnonzero(abs(error)>.5)
settled=float(t[outside[-1]+1]) if len(outside) and outside[-1]+1<len(t) else (0. if not len(outside) else None)
rows=list(csv.DictReader((out/'dob.csv').open()));rt=np.array([float(r['tick'])/1000-t0 for r in rows])
y={k:np.array([float(r[k]) for r in rows]) for k in ('target','model','velocity','current','reference_current')}
step_mask=(rt>=0)&(rt<=end-t0)
rate=[r for r in csv.DictReader((out/'rate/rate.csv').open()) if t0+1<=float(r['tick'])/1000<=end]
summary=dict(motor=mid,target=target,result=m['result'],last_second_mean_error_deg=float(error[tail].mean()),
    last_second_max_abs_error_deg=float(abs(error[tail]).max()),settle_within_half_deg_s=settled,
    position_min=float(q.min()),position_max=float(q.max()),command_peak_A=float(abs(y['current'][step_mask]).max()),
    mpc_velocity_peak_deg_s=float(abs(y['target'][step_mask]).max()),
    filtered_velocity_peak_deg_s=float(abs(y['velocity'][step_mask]).max()),
    raw_velocity_peak_deg_s=max(abs(float(r['velocity'])) for r in active),
    whole_rate_windows=len(rate),rates={k:[min(int(r[k]) for r in rate),max(int(r[k]) for r in rate)]
      for k in (f'target{mid}',f'type2_{mid}','loop_gap_ms')} if rate else {},
    errors={k:max(int(r[k]) for r in rate) for k in ('ring_overrun','priority_full','tx_errors','can_errors')} if rate else {})
fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
axes[0].plot(t,q,label='Measured position');axes[0].axhline(target,color='C1',label='Position target');axes[0].set_ylabel('Position [deg]')
for k,label in [('target','MPC velocity command'),('model','DOB reference model'),('velocity','40ms LPF feedback')]:axes[1].plot(rt,y[k],label=label,lw=.8)
axes[1].set_ylabel('Velocity [deg/s]')
axes[2].plot(rt,y['current'],label='F7 current command [A]')
axes[2].plot(rt,y['reference_current'],label='Type2 torque/Kt [reference]',alpha=.6)
axes[2].set_ylabel('Current (different reference units)');axes[2].set_xlabel('F7 time from position step [s]')
for ax in axes:ax.grid();ax.legend(fontsize=8)
axes[0].set_xlim(-.5,t[-1]);fig.suptitle(f'ID{mid} Mode5 position step: {m["tag"]}')
fig.tight_layout();fig.savefig(out/'mpc_tracking.png',dpi=150)
(out/'mpc_summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
