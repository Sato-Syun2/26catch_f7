"""停止中に回収したF7 RAMの実記録を描画。MPCの再計算は行わない。"""
import argparse,csv,json,struct
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('directory',type=Path)
p.add_argument('--with-current',action='store_true');a=p.parse_args()
out=a.directory/'internal';raw=(out/'ram_snapshot.bin').read_bytes()
count,read_count=struct.unpack_from('<II',raw,16384)
active=raw[16392]
if active or not 1<count<=512:raise SystemExit('snapshot is active or invalid')
d=np.array([struct.unpack_from('<I6fI',raw,i*32) for i in range(count)])
if not np.all(np.isfinite(d)) or np.any(np.diff(d[:,0])<=0):raise SystemExit('invalid records')
with (a.directory/'feedback.csv').open() as f:r=list(csv.DictReader(f))
rt=np.array([float(q['f7_time']) for q in r]);rx=np.array([float(q['position']) for q in r])
if d[0,0]/1000<rt.min() or d[-1,0]/1000>rt.max():raise SystemExit('different boot/test timestamps')
delta=d[:,1]-np.interp(d[:,0]/1000,rt,rx)
if np.median(abs(delta))>2:raise SystemExit('RAM/ROS positions do not match')
metadata=json.loads((a.directory/'metadata.json').read_text());target=metadata['target']
t=(d[:,0]-d[0,0])/1000
np.savetxt(out/'firmware_velocity_command.csv',d,delimiter=',',
 header='f7_tick_ms,position_mm,measured_velocity_mm_s,applied_mpc_velocity_mm_s,dob_model_velocity_mm_s,controller_current_A,unsaturated_controller_current_A,saturated',comments='')
stats=dict(samples=count,span_s=float(t[-1]),median_interval_ms=float(np.median(np.diff(d[:,0]))),
 max_interval_ms=float(max(np.diff(d[:,0]))),peak_velocity_command=float(max(abs(d[:,3]))),
 peak_measured_velocity=float(max(abs(d[:,2]))),
 model_tracking_rmse=float(np.sqrt(np.mean((d[:,2]-d[:,4])**2))),
 ram_ros_position_median_difference_mm=float(np.median(abs(delta))),
 source='F7 RAM snapshot, not reconstructed',
 note='Applied MPC command after safety limiter; data ends when stationary deadband is entered')
(out/'analysis.json').write_text(json.dumps(stats,indent=2))
fig,ax=plt.subplots(4 if a.with_current else 3,1,
 figsize=(11,12 if a.with_current else 9),sharex=True,
 gridspec_kw={'height_ratios':[2,1,1,1.5] if a.with_current else [2,1,1]})
ax[0].step(t,d[:,3],where='post',label='MPC velocity command applied to DOB',color='tab:blue')
ax[0].plot(t,d[:,4],label='DOB internal reference (tau=0.05 s)',color='tab:orange',lw=2)
ax[0].plot(t,d[:,2],label='Measured velocity (F7 feedback)',color='tab:green',lw=1)
ax[0].set_ylabel('Velocity [mm/s]');ax[0].legend(loc='lower right')
ax[1].plot(t,d[:,2]-d[:,4],color='tab:purple');ax[1].axhline(0,color='gray',lw=.8)
ax[1].set_ylabel('Measured - DOB model\n[mm/s]')
ax[2].plot(t,d[:,1],label='Measured position');ax[2].axhline(target,color='gray',ls='--',label=f'Target {target:g} mm')
ax[2].set_ylabel('Position [mm]');ax[2].set_xlabel('Time from first buffered command [s]');ax[2].legend()
if a.with_current:
    # ID4はROT_CW。電流指令はCAN送信直前に反転されるがFB電流は反転されない。
    # 指令をモーター側の符号へ揃えて重ねる。ROS実測値は補間せず生の時刻で描画。
    mask=(rt>=d[0,0]/1000)&(rt<=d[-1,0]/1000)
    measured=np.array([float(q['current']) for q in r])
    ax[3].plot(t,-d[:,6],color='gray',ls='--',alpha=.65,label='Requested current before limit (motor sign)')
    ax[3].step(t,-d[:,5],where='post',label='Limited current command (motor sign)',color='tab:blue')
    ax[3].plot(rt[mask]-d[0,0]/1000,measured[mask],'.-',markersize=3,
               color='tab:red',label='Measured current (ROS feedback, ~100 Hz)')
    for limit in [-10,10]:ax[3].axhline(limit,color='orange',ls=':',lw=1)
    ax[3].set_ylabel('Motor current [A]');ax[3].legend(fontsize=8)
    ax[2].set_xlabel('');ax[3].set_xlabel('Time from first buffered command [s]')
for q in ax:q.grid(alpha=.3);q.set_xlim(0,t[-1])
fig.suptitle(f'ID4 Mode 5: 520 -> 220 mm | MPC limit 825 mm/s, current limit 10 A\nActual F7 internal recording: {count} samples / {t[-1]:.3f} s; no reconstructed commands')
filename='mpc_velocity_with_current.png' if a.with_current else 'mpc_velocity_command.png'
fig.tight_layout();fig.savefig(out/filename,dpi=160)
print(json.dumps(stats,indent=2));print(out/filename)
