"""速度チャープのRAMログから局所周波数応答を推定。実機への書き込みなし。"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser()
p.add_argument('directory',type=Path)
p.add_argument('--current-limit',type=float,required=True)
a=p.parse_args();out=a.directory
meta=json.loads((out/'metadata.json').read_text())
mid=meta['velocity_motor']
fb=list(csv.DictReader((out/'feedback.csv').open()))
active=[r for r in fb if r['stage']=='velocity_chirp' and r['type']=='1' and r['id']==str(mid)]
if not active: raise ValueError('no velocity chirp feedback')
t0=min(float(r['f7_time']) for r in active)
rows=list(csv.DictReader((out/'dob.csv').open()))
t=np.array([float(r['tick'])/1000-t0 for r in rows])
data={k:np.array([float(r[k]) for r in rows]) for k in ('target','model','velocity','position','current','reference_current')}
body=meta.get('velocity_wave',{}).get('seconds',meta['duration'])
ramp=meta.get('velocity_wave',{}).get('ramp',0.)
k=np.log(meta['frequency_end_hz']/meta['frequency_hz'])/body
phase=2*np.pi*meta['frequency_hz']*(ramp+np.expm1(k*(t-ramp))/k)
freq=meta['frequency_hz']*np.exp(k*(t-ramp))
result=[]
for center in np.geomspace(meta['frequency_hz']*1.15,meta['frequency_end_hz']*.95,24):
    tc=ramp+np.log(center/meta['frequency_hz'])/k
    # 約4周期の窓。位相の共通誤差はactual/model比で相殺する。
    selected=(abs(t-tc)<2/center)&(t>ramp+.15)&(t<ramp+body-.1)
    if selected.sum()<16: continue
    ts=t[selected]; ph=phase[selected]
    if ts[0]>tc-2/center+.05 or ts[-1]<tc+2/center-.05:
        continue  # 中止直前の不完全な周期窓を周波数応答に混ぜない。
    X=np.column_stack((np.cos(ph),np.sin(ph),np.ones(len(ts)),ts-ts.mean()))
    fit={};residual={}
    for name,y in data.items():
        beta=np.linalg.lstsq(X,y[selected],rcond=None)[0]
        fit[name]=complex(beta[0],-beta[1])
        residual[name]=float(np.sqrt(np.mean((y[selected]-X@beta)**2)))
    if abs(fit['model'])<.02 or abs(fit['target'])<.02: continue
    # 位置基本波を微分した速度。窓内周波数変化を中心周波数で近似。
    position_velocity=1j*2*np.pi*center*fit['position']
    relative=position_velocity/fit['model']
    filtered=data['velocity'][selected]
    # ROS上の未LPF Type2速度も同じF7時刻基準の位相で評価。
    # RAM波形の線形補間は高周波振幅を減らすため、共通基底の係数同士を比較する。
    rt=np.array([float(r['f7_time'])-t0 for r in active])
    rv=np.array([float(r['velocity']) for r in active])
    mask=(abs(rt-tc)<2/center)&(rt>ramp+.15)&(rt<ramp+body-.1)
    rp=2*np.pi*meta['frequency_hz']*(ramp+np.expm1(k*(rt[mask]-ramp))/k)
    RX=np.column_stack((np.cos(rp),np.sin(rp),np.ones(mask.sum()),rt[mask]-tc))
    rb=np.linalg.lstsq(RX,rv[mask],rcond=None)[0]
    rf=dict(velocity=complex(rb[0],-rb[1]),model=fit['model'],target=fit['target'])
    result.append(dict(frequency_hz=float(center),samples=int(selected.sum()),
        target_velocity_amplitude=abs(fit['target']),model_velocity_amplitude=abs(fit['model']),
        filtered_velocity_amplitude=abs(fit['velocity']),position_velocity_amplitude=abs(position_velocity),
        position_model_gain=abs(relative),position_model_phase_deg=float(np.angle(relative,deg=True)),
        position_input_gain=abs(position_velocity/fit['target']),
        position_input_phase_deg=float(np.angle(position_velocity/fit['target'],deg=True)),
        filtered_input_gain=abs(fit['velocity']/fit['target']),
        raw_velocity_amplitude=abs(rf['velocity']),raw_model_gain=abs(rf['velocity']/rf['model']),
        raw_input_gain=abs(rf['velocity']/rf['target']),
        raw_input_phase_deg=float(np.angle(rf['velocity']/rf['target'],deg=True)),
        filtered_model_gain=abs(fit['velocity']/fit['model']),
        position_peak_to_peak_deg=2*abs(fit['position']),position_residual_rms_deg=residual['position'],
        small_motion_below_5deg=2*abs(fit['position'])<5,
        current_command_peak_A=float(np.max(abs(data['current'][selected]))),
        current_saturation_fraction=float(np.mean(abs(data['current'][selected])>=a.current_limit*.98))))
fig,axes=plt.subplots(3,2,figsize=(13,11))
for name in ('target','model','velocity'):
    axes[0,0].plot(t,data[name],label=name,lw=.7)
axes[0,0].set_ylabel('Velocity [deg/s]');axes[0,0].legend()
axes[0,1].plot(t,data['position']);axes[0,1].set_ylabel('Position [deg]')
axes[1,0].plot(t,data['current'],label='command')
axes[1,0].plot(t,data['reference_current'],label='Type2 torque/Kt',alpha=.6)
axes[1,0].set_ylabel('Current [A, different reference units]');axes[1,0].legend()
f=np.array([r['frequency_hz'] for r in result])
for name in ('target','model','filtered','position'):
    axes[1,1].semilogx(f,[r[name+'_velocity_amplitude'] for r in result],'o-',label=name)
axes[1,1].set_ylabel('Fundamental velocity amplitude [deg/s]');axes[1,1].legend()
axes[2,0].semilogx(f,[r['position_model_gain'] for r in result],'o-',label='position-derived / model')
axes[2,0].semilogx(f,[r['filtered_model_gain'] for r in result],'o-',label='filtered Type2 / model')
axes[2,0].axhline(1,color='k',ls='--');axes[2,0].set_ylabel('Gain');axes[2,0].legend()
axes[2,1].semilogx(f,[r['position_model_phase_deg'] for r in result],'o-')
axes[2,1].set_ylabel('Position-derived velocity / model phase [deg]')
for i,j in ((0,0),(0,1),(1,0)): axes[i,j].set_xlabel('Time from chirp [s]')
for i,j in ((1,1),(2,0),(2,1)): axes[i,j].set_xlabel('Frequency [Hz]')
for ax in axes.flat: ax.grid(True)
fig.suptitle(f'ID{mid}: raw velocity chirp; high-frequency small motion includes backlash/noise')
fig.tight_layout();fig.savefig(out/'velocity_sweep.png',dpi=150)
summary=dict(motor=mid,result=meta['result'],source=str(out),
    interpretation='Local four-cycle fits, 50Hz RAM samples. Small motion/backlash and the configured velocity LPF affect high frequencies. See firmware snapshot for LPF. Not a large-amplitude bandwidth certification.',
    current_peak_A=float(np.max(abs(data['current']))),
    position_min_deg=float(data['position'].min()),position_max_deg=float(data['position'].max()),windows=result)
(out/'velocity_sweep.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k!='windows'},indent=2))
print(json.dumps(result[-4:],indent=2))
