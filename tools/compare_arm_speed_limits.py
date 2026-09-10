"""両軸・速度上限別の位置MPC試験を比較する。"""
import argparse,csv,json,re
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('directories',nargs='+',type=Path);p.add_argument('--output',required=True,type=Path);p.add_argument('--current-comparison',action='store_true');a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=True)
fig,axes=plt.subplots(3,2,figsize=(14,10),sharex=True)
summary=[]
for directory in a.directories:
    m=json.loads((directory/'metadata.json').read_text());mid=m['position_motor']
    cap=float(re.search(r'#define ARM_MPC_SPEED ([0-9.]+)',m['mpc_header']).group(1))
    fb=[r for r in csv.DictReader((directory/'feedback.csv').open()) if r['type']=='1' and int(r['id'])==mid and r['stage']==f'center_{mid}']
    t0=float(fb[0]['f7_time']);end=float(fb[-1]['f7_time'])
    rows=[r for r in csv.DictReader((directory/'dob.csv').open()) if t0<=float(r['tick'])/1000<=end]
    t=np.array([float(r['tick'])/1000-t0 for r in rows]);label=f'Limit {cap:g} deg/s'
    color='C0' if cap==180 else 'C1'
    if a.current_comparison:
        label=m['tag']
        color='C1' if 'current5' in label else 'C0'
    axes[0,mid-1].plot([float(r['f7_time'])-t0 for r in fb],[float(r['position']) for r in fb],label=label,color=color)
    axes[1,mid-1].plot(t,[float(r['target']) for r in rows],ls='--',color=color,label=label+' command')
    axes[1,mid-1].plot(t,[float(r['velocity']) for r in rows],color=color,label=label+' FB (40ms)')
    axes[2,mid-1].plot(t,[float(r['current']) for r in rows],color=color,label=label)
    s=json.loads((directory/'mpc_summary.json').read_text());s.update(directory=str(directory),limit_deg_s=cap)
    summary.append(s)
for col,name in enumerate(('ID1 elbow','ID2 root')):
    axes[0,col].set_title(name);axes[0,col].axhline(90,color='k',ls=':',lw=.7)
    for row,label in enumerate(('Position [deg]','Velocity [deg/s]','F7 current command [A]')):
        axes[row,col].set_ylabel(label);axes[row,col].grid(True);axes[row,col].legend(fontsize=8);axes[row,col].set_xlim(0,4)
    axes[2,col].set_xlabel('Time from position step [s]')
fig.suptitle('Individual 0 to 90 degree tests: current limits at 400 deg/s' if a.current_comparison else 'Individual 0 to 90 degree tests: 180 vs 400 deg/s limits')
fig.tight_layout();fig.savefig(a.output/'comparison.png',dpi=150)
(a.output/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
