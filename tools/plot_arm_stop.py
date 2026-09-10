"""チャープ終了後の位置と電流指令を比較（既存50Hz RAMログ）。"""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);out=p.parse_args().directory
fb=list(csv.DictReader((out/'feedback.csv').open()))
end=max(float(r['f7_time']) for r in fb if r['stage']=='velocity_chirp' and r['type']=='1' and r['id']=='1')
rows=[r for r in csv.DictReader((out/'dob.csv').open()) if float(r['tick'])/1000>end]
t=np.array([float(r['tick'])/1000-end for r in rows]);pos=np.array([float(r['position']) for r in rows])
fig,ax=plt.subplots(2,1,figsize=(11,6),sharex=True)
ax[0].plot(t,pos);ax[0].set_ylabel('Elbow position [deg]')
for key in ['current','reference_current']:ax[1].plot(t,[float(r[key]) for r in rows],label=key)
ax[1].set_ylabel('Command A / torque-Kt reference');ax[1].set_xlabel('Seconds after chirp end');ax[1].legend()
for x in ax:x.grid()
fig.tight_layout();fig.savefig(out/'stop_overview.png',dpi=150)
summary=dict(duration=float(t[-1]-t[0]),position_min=float(pos.min()),position_max=float(pos.max()),position_peak_to_peak=float(np.ptp(pos)))
(out/'stop_summary.json').write_text(json.dumps(summary,indent=2));print(summary)
