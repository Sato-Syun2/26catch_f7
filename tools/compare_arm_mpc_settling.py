"""位置ステップの目標偏差とMPC速度指令を同じ時間基準で比較する。"""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('directories',nargs='+',type=Path);p.add_argument('--output',type=Path,required=True)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
fig,axes=plt.subplots(2,1,figsize=(12,8),sharex=True);summary=[]
for out in a.directories:
    m=json.loads((out/'metadata.json').read_text());mid=m['position_motor']
    rows=[r for r in csv.DictReader((out/'feedback.csv').open()) if r['type']=='1' and int(r['id'])==mid and r['stage']==f'center_{mid}']
    t0=float(rows[0]['f7_time']);t=np.array([float(r['f7_time'])-t0 for r in rows]);q=np.array([float(r['position'])-m['position_target'] for r in rows])
    axes[0].plot(t,q,label=m['tag'],lw=.9)
    ram=list(csv.DictReader((out/'dob.csv').open()));rt=np.array([float(r['tick'])/1000-t0 for r in ram])
    axes[1].plot(rt,[float(r['target']) for r in ram],label=m['tag'],lw=.7)
    summary.append(dict(directory=str(out),**json.loads((out/'mpc_summary.json').read_text())))
axes[0].axhspan(-.5,.5,color='green',alpha=.1);axes[0].set_ylabel('Position error [deg]')
axes[1].set_ylabel('MPC velocity command [deg/s]');axes[1].set_xlabel('F7 time from step [s]')
for ax in axes:ax.grid();ax.legend(fontsize=7);ax.set_xlim(0,8)
fig.suptitle('Settling comparison — different step directions / initial postures')
fig.tight_layout();fig.savefig(a.output/'settling_comparison.png',dpi=150)
(a.output/'summary.json').write_text(json.dumps(summary,indent=2))
