"""初期アーム試験の位置由来速度と生FBを比較（非因果平滑化は解析専用）。"""
import csv
import json
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

out=Path('debug_logs/arm_initial_20260909');out.mkdir(exist_ok=True)
results=[]
for directory in sorted(Path('debug_logs').glob('arm_step_20260909_*')):
    if not (directory/'metadata.json').exists(): continue
    meta=json.loads((directory/'metadata.json').read_text())
    motor=meta.get('velocity_motor')
    if motor is None or meta.get('result')!='complete' or meta.get('chirp',False): continue
    allrows=list(csv.DictReader((directory/'feedback.csv').open()))
    rows=[r for r in allrows if r['type']=='1' and int(r['id'])==motor and r['stage']=='velocity_sine']
    if len(rows)<30:continue
    t=np.array([float(r['time']) for r in rows]);t-=t[0]
    x=np.array([float(r['position']) for r in rows])
    u=np.array([float(r['target']) for r in rows])
    v=np.array([float(r['velocity']) for r in rows])
    current=np.array([float(r['current']) for r in rows])
    # ホスト到着揺らぎに左右されない均等時刻へ補間してから微分。
    grid=np.linspace(t[0],t[-1],len(t));dt=grid[1]-grid[0]
    derivative=savgol_filter(np.interp(grid,t,x),21,3,deriv=1,delta=dt)
    design=np.c_[np.sin(np.pi*grid),np.cos(np.pi*grid),np.ones(len(grid))]
    cv=np.linalg.lstsq(design,derivative,rcond=None)[0]
    cu=np.linalg.lstsq(design,np.interp(grid,t,u),rcond=None)[0]
    H=complex(cv[0],cv[1])/complex(cu[0],cu[1])
    row=dict(directory=str(directory),motor=motor,dob=meta.get('dob',False),
             amplitude=meta.get('velocity_amplitude_deg_s',3),frequency_hz=.5,
             position_min=float(x.min()),position_max=float(x.max()),
             current_peak=float(abs(current).max()),gain=float(abs(H)),phase_deg=float(np.angle(H,deg=True)),
             ros_fb_rate_hz=float((len(t)-1)/(t[-1]-t[0])),max_host_gap_ms=float(np.diff(t).max()*1000))
    results.append(row)
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
    axes[0].plot(t,x-x[0]);axes[0].set_ylabel('Position change [deg]')
    axes[1].plot(t,u,label='Velocity command');axes[1].plot(t,v,alpha=.3,label='Raw velocity FB')
    axes[1].plot(grid,derivative,label='Position derivative (offline smoothed)')
    if row['dob']:
        model=np.zeros(len(grid))
        for i in range(1,len(grid)):
            beta=np.exp(-dt/.5);model[i]=beta*model[i-1]+(1-beta)*np.interp(grid[i],t,u)
        axes[1].plot(grid,model,label='Nominal tau=0.5s')
    axes[1].legend(fontsize=8);axes[1].set_ylabel('Velocity [deg/s]')
    axes[2].plot(t,current);axes[2].set_ylabel('Measured current [A]');axes[2].set_xlabel('Time [s]')
    for ax in axes:ax.grid(True)
    fig.suptitle(f"ID{motor} {'DOB' if row['dob'] else 'Internal velocity'}: {row['amplitude']} deg/s, 0.5 Hz")
    fig.tight_layout();fig.savefig(out/(directory.name+'.png'),dpi=140);plt.close(fig)
(out/'summary.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results,indent=2))
