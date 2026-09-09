"""速度チャープと一次遅れ目標モデルを同一時刻で比較する。"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p = argparse.ArgumentParser()
p.add_argument('directory', type=Path)
p.add_argument('--tau', type=float, required=True)
a = p.parse_args()
m = json.loads((a.directory/'metadata.json').read_text())
mid = m['velocity_motor']
rows = [r for r in csv.DictReader((a.directory/'feedback.csv').open())
        if r['type']=='1' and int(r['id'])==mid and r['stage'] in
        ('hold','velocity_chirp','velocity_sine','velocity_zero')]
t = np.array([float(r['time']) for r in rows]); t -= t[0]
u, v, x, current = [np.array([float(r[k]) for r in rows])
                    for k in ('target','velocity','position','current')]
model = np.zeros(len(t))
filtered = np.zeros(len(t))
for i in range(1,len(t)):
    beta = np.exp(-(t[i]-t[i-1])/a.tau)
    model[i] = beta*model[i-1]+(1-beta)*u[i-1]
    beta_v = np.exp(-(t[i]-t[i-1])/.04)
    filtered[i] = beta_v*filtered[i-1]+(1-beta_v)*v[i]
selected = np.array([r['stage'].startswith('velocity_') and r['stage']!='velocity_zero' for r in rows])
error = v[selected]-model[selected]
metrics = dict(motor=mid,tau_s=a.tau,result=m['result'],
               velocity_rmse_deg_s=float(np.sqrt(np.mean(error**2))),
               model_rms_deg_s=float(np.sqrt(np.mean(model[selected]**2))),
               reference_current_peak_Arms=float(np.max(np.abs(current))),
               position_min_deg=float(x.min()),position_max_deg=float(x.max()),
               max_sample_gap_s=float(np.diff(t).max()))
metrics['normalized_rmse'] = metrics['velocity_rmse_deg_s']/max(metrics['model_rms_deg_s'],1e-9)
metrics['offline_40ms_velocity_rmse_deg_s'] = float(np.sqrt(np.mean((filtered[selected]-model[selected])**2)))
# ROSの100Hz記録による近似。F7内500Hzフィルタ状態そのものではない。
metrics['filter_note'] = 'Offline causal 40ms LPF from 100Hz ROS; not the F7 internal state'
(a.directory/'dob_metrics.json').write_text(json.dumps(metrics,indent=2))
fig,axes = plt.subplots(3,1,figsize=(12,9),sharex=True)
axes[0].plot(t,u,label='Command',alpha=.5)
axes[0].plot(t,model,label=f'Ideal: tau={a.tau:g}s',lw=2)
axes[0].plot(t,v,label='Type2 velocity',alpha=.65,lw=.7)
axes[0].plot(t,filtered,label='Offline 40ms LPF',lw=1)
axes[0].set_ylabel('Velocity [deg/s]');axes[0].legend()
axes[1].plot(t,x);axes[1].set_ylabel('Position [deg]')
axes[2].plot(t,current);axes[2].set_ylabel('Torque/Kt [Arms equiv.]')
axes[2].set_xlabel('Host elapsed time [s]')
for ax in axes: ax.grid(True)
fig.suptitle(f'Robstride ID{mid}: DOB model comparison (not measured iqf)')
fig.tight_layout();fig.savefig(a.directory/'dob_comparison.png',dpi=150)
print(json.dumps(metrics,indent=2))
