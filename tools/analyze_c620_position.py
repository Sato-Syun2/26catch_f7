"""Mode5位置試験の到達時間・オーバーシュート・停止後位置を集計する。"""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def analyze(directory):
    meta = json.loads((directory/'metadata.json').read_text())
    data = np.genfromtxt(directory/'feedback.csv', delimiter=',', names=True,
                         dtype=None, encoding='utf-8')
    drive_start = meta.get('drive_start', 5)
    d = data[data['time'] >= drive_start]
    sign = 1 if meta['target'] >= meta.get('origin', 0) else -1
    summary = {k: meta.get(k) for k in ('target','origin','speed_limit_mm_s','result',
                'arrival_seconds','settled_position','final_position','final_velocity','disabled')}
    summary.update(position_min=float(d['position'].min()),position_max=float(d['position'].max()),
                   peak_abs_velocity=float(np.abs(d['velocity']).max()),
                   peak_abs_current=float(np.abs(d['current']).max()),
                   overshoot_mm=max(0,float((sign*(d['position']-meta['target'])).max())))
    for label, mask in [('enabled',d['state']!=0),('disabled',d['state']==0)]:
        summary['peak_abs_velocity_'+label] = float(np.abs(d['velocity'][mask]).max()) if mask.any() else None
    (directory/'analysis.json').write_text(json.dumps(summary, indent=2))
    fig, axes = plt.subplots(3,1,figsize=(10,8),sharex=True)
    t=d['time']-drive_start
    axes[0].plot(t,d['position'],label='Position')
    axes[0].axhline(meta['target'],color='orange',linestyle='--',label='Target')
    axes[0].axhspan(meta['target']-.5,meta['target']+.5,alpha=.15,color='green')
    axes[0].set_ylabel('Position [configured mm]');axes[0].legend()
    axes[1].plot(t,d['velocity']);axes[1].set_ylabel('Velocity [configured mm/s]')
    axes[2].plot(t,d['current']);axes[2].set_ylabel('CAN current [A]')
    axes[2].set_xlabel('Time from enable [s]')
    for ax in axes:
        ax.grid(alpha=.3)
        stopped = np.flatnonzero((d['state'][1:]==0)&(d['state'][:-1]!=0))
        if len(stopped):
            ax.axvline(t[stopped[0]+1],color='gray',linestyle=':',label='Disable')
    fig.suptitle(f"C620 Mode5 target {meta['target']:.3f} mm, speed cap {meta['speed_limit_mm_s']:.0f} mm/s\n"
                 'Ideal velocity model time constant 33.3 ms, current cap 7 A')
    fig.tight_layout();fig.savefig(directory/'response.png',dpi=140);plt.close(fig)
    print(directory.name,json.dumps(summary))


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('directories',type=Path,nargs='+')
    for directory in p.parse_args().directories:
        analyze(directory)
