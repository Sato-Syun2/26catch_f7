"""中央部と機構端付近の同距離移動を、指令送信時刻基準で比較する。"""
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
root=Path(__file__).resolve().parents[1]/'debug_logs'
out=root/'id4_mpc_tuning_20260909';out.mkdir(exist_ok=True)
fig,axes=plt.subplots(3,1,figsize=(10,9),sharex=True)
for name,label,target in [
 ('id4_mpc_20260909_162144','Center: 100 -> 400 mm',400),
 ('id4_mpc_20260909_162241','Near end: 220 -> 520 mm',520),
 ('id4_mpc_20260909_162311','Return: 520 -> 220 mm',220)]:
 with (root/name/'feedback.csv').open() as f:r=[x for x in csv.DictReader(f) if x['stage']=='step']
 t=np.array([float(q['time']) for q in r]);t-=t[0]
 x=np.array([float(q['position']) for q in r]);v=np.array([float(q['velocity']) for q in r]);current=np.array([float(q['current']) for q in r])
 sign=1 if target>x[0] else -1
 axes[0].plot(t,sign*(target-x),label=label)
 axes[1].plot(t,sign*v,label=label)
 axes[2].plot(t,current,label=label)
axes[0].axhspan(-.5,.5,color='green',alpha=.2,label='Position tolerance +/-0.5 mm')
axes[0].set_ylabel('Remaining distance [mm]');axes[0].set_ylim(-5,310);axes[0].legend()
axes[1].set_ylabel('Velocity toward target [mm/s]')
axes[2].set_ylabel('Measured current [A]');axes[2].set_xlabel('Time from command stage [s]')
for ax in axes:ax.grid();ax.set_xlim(0,1.4)
fig.suptitle('ID4 Mode 5: MPC + DOB, 10 A limit / 800 mm/s reference limit\nRaw ROS feedback; no fitted time shifts')
fig.tight_layout();fig.savefig(out/'center_vs_endpoint.png',dpi=160)
print(out/'center_vs_endpoint.png')
