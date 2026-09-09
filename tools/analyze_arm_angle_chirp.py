"""角度振幅・飽和・位置由来の速度周波数応答を周期ごとに評価する。"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('directory',type=Path);p.add_argument('--current-limit',type=float,required=True)
a=p.parse_args();out=a.directory;m=json.loads((out/'metadata.json').read_text());mid=m['velocity_motor']
rows=[r for r in csv.DictReader((out/'feedback.csv').open())
      if r['type']=='1' and int(r['id'])==mid and r['stage']=='angle_chirp' and r['chirp_phase']]
if len(rows)<10: raise SystemExit('no usable angle chirp rows')
v={k:np.array([float(r[k]) for r in rows]) for k in
   ('time','f7_time','position','velocity','current','desired_position','chirp_phase','target')}
t=v['time']-v['time'][0];wave=m['angle_wave'];cycles=np.floor(v['chirp_phase']/(2*np.pi)).astype(int)
ram=list(csv.DictReader((out/'dob.csv').open()))
rt=np.array([float(r['tick'])*.001 for r in ram]);rc=np.array([float(r['current']) for r in ram])
rm=np.array([float(r['model']) for r in ram]);rv=np.array([float(r['velocity']) for r in ram])
summary=[]
for cycle in sorted(set(cycles)):
    selected=(cycles==cycle)&(t>=wave['ramp'])&(t<=wave['ramp']+wave['seconds'])
    ix=np.where(selected)[0]
    if len(ix)<20: continue
    phase=v['chirp_phase'][ix]
    if np.ptp(phase)<5.8: continue
    times=v['time'][ix];centered=times-times.mean()
    omega=(phase[-1]-phase[0])/(times[-1]-times[0])
    design=np.column_stack((np.sin(phase),np.cos(phase),np.ones(len(ix)),centered))
    fit=np.linalg.lstsq(design,v['position'][ix],rcond=None)[0]
    # q=a*sin(phi)+b*cos(phi)より、速度のcos基準複素振幅はomega*(a+j*b)。
    physical_velocity=omega*complex(fit[0],fit[1])
    model_samples=np.interp(v['f7_time'][ix],rt,rm)
    model_fit=np.linalg.lstsq(design,model_samples,rcond=None)[0]
    model_velocity=complex(model_fit[1],-model_fit[0])
    ratio=physical_velocity/model_velocity if abs(model_velocity)>1e-6 else complex(np.nan,np.nan)
    ri=(rt>=v['f7_time'][ix[0]])&(rt<=v['f7_time'][ix[-1]])
    saturation=float(np.mean(np.abs(rc[ri])>=a.current_limit-.01)) if ri.any() else 1.
    span=2*float(np.hypot(fit[0],fit[1]))
    summary.append(dict(cycle=int(cycle),frequency_hz=omega/(2*np.pi),
        fitted_travel_deg=span,raw_travel_deg=float(np.ptp(v['position'][ix])),
        position_fit_residual_deg=float(np.sqrt(np.mean((design@fit-v['position'][ix])**2))),
        saturation_fraction=saturation,valid=bool(span>=5 and saturation<=.05),
        physical_velocity_model_gain=float(abs(ratio)),physical_velocity_model_phase_deg=float(np.angle(ratio,deg=True))))
if summary:
    with (out/'angle_cycles.csv').open('w') as f:
        w=csv.DictWriter(f,list(summary[0]));w.writeheader();w.writerows(summary)
valid=[r for r in summary if r['valid']]
report=dict(result=m['result'],motor=mid,cycles=len(summary),valid_cycles=len(valid),
            actual_position_min_deg=float(v['position'].min()),actual_position_max_deg=float(v['position'].max()),
            peak_current_command_A=float(np.abs(rc).max()),
            current_command_saturation_fraction=float(np.mean(np.abs(rc)>=a.current_limit-.01)),
            cycles_detail=summary)
(out/'angle_summary.json').write_text(json.dumps(report,indent=2))
fig,axes=plt.subplots(4,1,figsize=(12,11))
axes[0].plot(t,v['desired_position'],label='Designed angle')
axes[0].plot(t,v['position'],label='Measured angle');axes[0].set_ylabel('Position [deg]');axes[0].legend()
offset=v['f7_time'][0]
axes[1].plot(rt-offset,rm,label='F7 ideal model')
axes[1].plot(rt-offset,rv,label='F7 filtered velocity');axes[1].set_ylabel('Velocity [deg/s]');axes[1].legend()
axes[2].plot(rt-offset,rc,label='F7 current command [A]')
axes[2].plot(t,v['current'],label='Torque/Kt [Arms equivalent]',alpha=.7)
axes[2].axhline(a.current_limit,color='r',ls='--');axes[2].axhline(-a.current_limit,color='r',ls='--')
axes[2].legend();axes[2].set_xlabel('Elapsed time [s]')
if summary:
    freq=np.array([r['frequency_hz'] for r in summary]);span=np.array([r['fitted_travel_deg'] for r in summary])
    axes[3].plot(freq,span,'o-',label='Measured fitted peak-to-peak angle')
    axes[3].axhline(10,color='g',ls='--',label='Designed 10deg travel')
    axes[3].axhline(5,color='r',ls='--',label='Minimum accepted 5deg travel')
axes[3].set_xlabel('Chirp frequency [Hz]');axes[3].set_ylabel('Travel [deg]');axes[3].legend()
for ax in axes:ax.grid(True)
fig.suptitle(f'ID{mid}: constant-angle chirp, tau={wave["tau"]}s, {a.current_limit:g}A limit')
fig.tight_layout();fig.savefig(out/'angle_chirp.png',dpi=150);plt.close(fig)
print(json.dumps({k:v for k,v in report.items() if k!='cycles_detail'},indent=2))
if valid:
    print('valid frequency range',min(r['frequency_hz'] for r in valid),max(r['frequency_hz'] for r in valid))
