"""短い電流パルスの指令・実電流と変位を比較する。"""
import csv
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
for name in sys.argv[1:]:
    directory=Path(name)
    rows=[r for r in csv.DictReader((directory/'feedback.csv').open())
          if r['type']=='1' and r['id']=='1' and
          (r['stage'].startswith('current_') or r['stage'].startswith('coast_'))]
    t=np.array([float(r['time']) for r in rows]);t-=t[0]
    x=np.array([float(r['position']) for r in rows]);x-=x[0]
    current=np.array([float(r['current']) for r in rows])
    command=np.array([float(r['target']) for r in rows])
    fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
    axes[0].plot(t,x);axes[0].set_ylabel('Elbow displacement [deg]')
    axes[1].step(t,command,where='post',label='Current command')
    axes[1].plot(t,current,label='Measured current');axes[1].legend()
    axes[1].set_ylabel('Current [A]');axes[1].set_xlabel('Time [s]')
    for ax in axes:ax.grid(True)
    fig.suptitle('ID1: direct current pulse and zero-current recovery')
    fig.tight_layout();fig.savefig(directory/'current_probe.png',dpi=150);plt.close(fig)
