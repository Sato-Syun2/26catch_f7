"""同じ90度ステップの移動中振動を比較。準備移動・Disable後は除外する。"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p = argparse.ArgumentParser()
p.add_argument('directories', nargs='+', type=Path)
p.add_argument('--output', required=True, type=Path)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
summaries = []
for directory in a.directories:
    m = json.loads((directory/'metadata.json').read_text())
    mid = m['position_motor']
    fb = [r for r in csv.DictReader((directory/'feedback.csv').open())
          if r['type']=='1' and int(r['id'])==mid and r['stage']==f'center_{mid}']
    t0 = float(fb[0]['f7_time'])
    rows = list(csv.DictReader((directory/'dob.csv').open()))
    t = np.array([float(r['tick'])/1000-t0 for r in rows])
    v = np.array([float(r['velocity']) for r in rows])
    current = np.array([float(r['current']) for r in rows])
    steady = (t>1)&(t<2.7)
    vv = v[steady]
    dt = np.median(np.diff(t))
    amplitude = abs(np.fft.rfft((vv-vv.mean())*np.hanning(len(vv))))
    freq = np.fft.rfftfreq(len(vv), dt)
    metrics = json.loads((directory/'mpc_summary.json').read_text())
    hold = (t>float(fb[-1]['f7_time'])-t0-1)&(t<=float(fb[-1]['f7_time'])-t0)
    metrics.update(directory=str(directory), tag=m['tag'],
                   ripple_window_s=[1, 2.7],
                   filtered_velocity_std_deg_s=float(vv.std()),
                   filtered_velocity_min_deg_s=float(vv.min()),
                   filtered_velocity_max_deg_s=float(vv.max()),
                   current_std_A=float(current[steady].std()),
                   last_second_velocity_std_deg_s=float(v[hold].std()),
                   last_second_current_std_A=float(current[hold].std()),
                   dominant_velocity_frequency_Hz=float(freq[1+np.argmax(amplitude[1:])]))
    summaries.append(metrics)
    label=m['tag']
    axes[0].plot([float(r['f7_time'])-t0 for r in fb],
                 [float(r['position']) for r in fb], label=label)
    axes[1].plot(t,v,label=label,lw=.9)
    axes[2].plot(t,current,label=label,lw=.9)
axes[0].axhline(90,color='k',ls='--',lw=.7)
axes[1].axhline(30,color='k',ls='--',lw=.7)
for ax,label in zip(axes,['Position [deg]','40ms filtered velocity [deg/s]', 'F7 current command [A]']):
    ax.set_ylabel(label)
    ax.grid(True)
    ax.legend(fontsize=8)
axes[-1].set_xlabel('Time from 90-degree step [s]')
axes[-1].set_xlim(0,6)
fig.suptitle('Arm 0 to 90 degrees: vibration comparison (no time shift)')
fig.tight_layout()
fig.savefig(a.output/'comparison.png',dpi=150)
(a.output/'summary.json').write_text(json.dumps(summaries,indent=2))
print(json.dumps(summaries,indent=2))
