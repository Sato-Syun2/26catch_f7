"""既存チャープを同一F7時刻で表示。未フィルタFBと制御用LPFを区別する。"""
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
p.add_argument('--lpf-ms', type=float, default=20.)
a = p.parse_args()
out = a.directory
meta = json.loads((out/'metadata.json').read_text())
fb = [r for r in csv.DictReader((out/'feedback.csv').open())
      if r['type']=='1' and int(r['id'])==meta['velocity_motor']]
start = min(float(r['f7_time']) for r in fb if r['stage']=='velocity_chirp')
rt = np.array([float(r['f7_time'])-start for r in fb])
raw = np.array([float(r['velocity']) for r in fb])
rows = list(csv.DictReader((out/'dob.csv').open()))
t = np.array([float(r['tick'])/1000-start for r in rows])
y = {k: np.array([float(r[k]) for r in rows])
     for k in ('target','model','velocity','position','current','reference_current')}
fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
axes[0].plot(rt, raw, color='0.6', alpha=.4, lw=.5, label='Raw Type2 velocity (100Hz ROS)')
axes[0].plot(t, y['target'], color='C0', lw=.6, alpha=.65, label='Velocity command')
axes[0].plot(t, y['velocity'], color='C2', lw=.8, label=f'{a.lpf_ms:g}ms LPF feedback (50Hz RAM)')
axes[0].plot(t, y['model'], color='C1', lw=1.1, label=f'Reference model tau={a.tau:g}s')
axes[0].set_ylabel('Velocity [deg/s]')
axes[1].plot(t, y['position'], label='Type2 position')
axes[1].set_ylabel('Position [deg]')
axes[2].plot(t, y['current'], label='F7 current command [A]', lw=.7)
axes[2].plot(t, y['reference_current'], label='Type2 torque/Kt [Arms equivalent]', lw=.7)
axes[2].set_ylabel('Current (different reference units)')
axes[2].set_xlabel('F7 time from chirp start [s]')
for ax in axes:
    ax.grid(); ax.legend(loc='upper left', fontsize=8)
    ax.axvline(30, color='k', ls=':', alpha=.5)
axes[0].set_xlim(-1, 35)
fig.suptitle(f"ID{meta['velocity_motor']}: 60deg/s, 0.5–10Hz / 30s; tau={a.tau:g}s; {meta['tag']}")
fig.tight_layout(); fig.savefig(out/'tracking_overview.png', dpi=150)
fig, axes = plt.subplots(3, 1, figsize=(13, 9))
k = np.log(meta['frequency_end_hz']/meta['frequency_hz'])/meta['duration']
for ax, freq in zip(axes, (1.,3.,8.)):
    center = np.log(freq/meta['frequency_hz'])/k
    ax.plot(rt, raw, color='0.6', lw=.6, alpha=.7, label='Raw Type2')
    ax.plot(t, y['velocity'], color='C2', lw=1, label=f'{a.lpf_ms:g}ms LPF feedback')
    ax.plot(t, y['model'], color='C1', lw=1.5, label='Reference model')
    ax.set_xlim(center-1/freq, center+1/freq)
    ax.set_ylabel('Velocity [deg/s]'); ax.set_title(f'Near {freq:g}Hz')
    ax.grid(); ax.legend(fontsize=8)
axes[-1].set_xlabel('F7 time from chirp start [s] — no artificial time shift')
fig.suptitle('Tracking detail: high-frequency motion may be below backlash scale')
fig.tight_layout(); fig.savefig(out/'tracking_detail.png', dpi=150)
