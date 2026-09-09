"""Analyze buffered 500 Hz ID4 firmware diagnostics after a pulse."""
import sys,json
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

src=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(exist_ok=True)
rows=[]
for line in src.read_text().splitlines():
    try:message=json.loads(line)['message']
    except (ValueError,KeyError):continue
    if message.startswith('ID4D,'):
        try:
            values=[float(x) for x in message.split(',')[1:]]
            if len(values)==8:rows.append(values)
        except ValueError:pass
d=np.array(rows)
if not len(d):raise SystemExit('No ID4D records yet')
# Separate tests by discontinuities; analyze most recent complete recording.
cuts=np.r_[0,np.flatnonzero(np.diff(d[:,0])>20)+1,len(d)]
blocks=[d[a:b] for a,b in zip(cuts[:-1],cuts[1:])]
d=blocks[-1]
if len(d)<400:raise SystemExit(f'Insufficient recording: {len(d)}/512')
t=(d[:,0]-d[0,0])*.001
dt=np.median(np.diff(t))
grid=np.arange(t[0],t[-1]+dt*.1,dt)
acc_grid=savgol_filter(np.interp(grid,t,d[:,2]),11,2,deriv=1,delta=dt)
acc=np.interp(t,grid,acc_grid)
on=np.abs(d[:,3])>.1
off=np.flatnonzero(~on)
offi=int(off[0]) if len(off) else len(d)-1
stats=dict(samples=len(d),median_dt_ms=dt*1000,max_dt_ms=float(max(np.diff(t))*1000),
           missing_or_corrupt_serial_records=512-len(d),
           position_range_mm=[min(d[:,1]),max(d[:,1])],peak_velocity_mm_s=float(max(abs(d[:,2]))),
           peak_current_command_A=float(max(abs(d[:,5]))),
           peak_unsaturated_current_A=float(max(abs(d[:,6]))),
           saturated_fraction=float(np.mean(d[:,7])),
           model_rmse_mm_s=float(np.sqrt(np.mean((d[:,2]-d[:,4])**2))),
           peak_acceleration_22ms_smooth_mm_s2=float(max(abs(acc))),
           zero_command_start_velocity_mm_s=float(d[offi,2]),
           travel_after_zero_mm=float(d[-1,1]-d[offi,1]))
np.savetxt(out/'firmware_500hz.csv',d,delimiter=',',header='tick_ms,position_mm,velocity_mm_s,target_mm_s,internal_model_mm_s,current_command_A,unsaturated_current_A,saturated',comments='')
(out/'firmware_analysis.json').write_text(json.dumps(stats,indent=2))
fig,ax=plt.subplots(4,1,figsize=(11,10),sharex=True)
ax[0].plot(t,d[:,3],label='Target');ax[0].plot(t,d[:,4],label='F7 internal model');ax[0].plot(t,d[:,2],label='Measured',lw=.8);ax[0].legend();ax[0].set_ylabel('Velocity [mm/s]')
ax[1].plot(t,d[:,1]);ax[1].set_ylabel('Position [mm]')
ax[2].plot(t,d[:,6],label='Before saturation',alpha=.6);ax[2].plot(t,d[:,5],label='CAN current command');ax[2].legend();ax[2].set_ylabel('Current [A]')
ax[3].plot(t,acc);ax[3].set_ylabel('Acceleration [mm/s²]\n22 ms smoothed');ax[3].set_xlabel('Time from pulse onset [s]')
for a in ax:a.grid(alpha=.3)
fig.suptitle('ID4 pulse: F7 internal model vs measured response (500 Hz buffer)')
fig.tight_layout();fig.savefig(out/'firmware_response.png',dpi=160)
print(json.dumps(stats,indent=2))
