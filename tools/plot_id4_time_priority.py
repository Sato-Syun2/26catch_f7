"""同距離移動の二次評価MPCと時間優先試験を比較する。"""
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1]/'debug_logs'
fig,ax=plt.subplots(3,1,figsize=(10,9),sharex=True)
for name,label in [('id4_mpc_20260909_162816','Baseline: quadratic MPC'),
                   ('id4_mpc_20260909_170100','Trial: arrival-time priority')]:
 with (root/name/'feedback.csv').open() as f:r=[q for q in csv.DictReader(f) if q['stage']=='step']
 t=np.array([float(q['time']) for q in r]);t-=t[0]
 x=np.array([float(q['position']) for q in r]);v=np.array([float(q['velocity']) for q in r]);i=np.array([float(q['current']) for q in r])
 ax[0].plot(t,x-400,label=label);ax[1].plot(t,v,label=label);ax[2].plot(t,i,label=label)
ax[0].axhspan(-.5,.5,color='green',alpha=.2);ax[0].set_ylim(-30,15)
ax[0].set_ylabel('Position - target [mm]');ax[0].legend()
ax[1].set_ylabel('Measured velocity [mm/s]');ax[2].set_ylabel('Measured current [A]')
ax[2].set_xlabel('Time from command stage [s]')
for q in ax:q.set_xlim(0,1.1);q.grid(alpha=.3)
fig.suptitle('ID4: 100 -> 400 mm, 825 mm/s reference limit, 10 A current limit\nActual ROS data; no fitted time shifts')
fig.tight_layout();out=root/'id4_time_priority_20260909'/'comparison.png'
fig.savefig(out,dpi=160);print(out)
