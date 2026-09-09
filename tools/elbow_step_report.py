"""チャープのステップ換算（仮定付き）と実測時系列の表。実機へ通信しない。"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('directories',nargs=3,type=Path);p.add_argument('--output',type=Path,required=True)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
colors=['#0072B2','#D55E00','#009E73']
t=np.arange(76)*.02;ideal=1-np.exp(-t/.1);prior=np.diff(np.r_[0.,ideal])
C=np.array([np.ones(len(t)),np.eye(len(t))[0],np.eye(len(t))[-1]])
constraints=np.array([1.,0.,0.])
stepfig,axs=plt.subplots(2,2,figsize=(14,10))
wavefig,waxs=plt.subplots(3,3,figsize=(17,11))
currentfig,caxs=plt.subplots(3,2,figsize=(15,10))
fitfig,faxs=plt.subplots(2,3,figsize=(15,8))
results=[];summary=[];step_rows={'time_s':t,'ideal_unit_step_response':ideal}
for col,path in enumerate(a.directories):
    meta=json.loads((path/'metadata.json').read_text());amp=meta['velocity_amplitude_deg_s']
    frf=json.loads((path/'velocity_sweep.json').read_text())['windows']
    f=np.array([r['frequency_hz'] for r in frf])
    H=np.array([r['raw_input_gain']*np.exp(1j*np.deg2rad(r['raw_input_phase_deg'])) for r in frf])
    E=np.exp(-2j*np.pi*f[:,None]*t);weights=1/np.maximum(abs(H),.1)
    A=np.r_[E.real*weights[:,None],E.imag*weights[:,None]];b=np.r_[H.real*weights,H.imag*weights]
    solutions={};fits={}
    # 測定帯域外はDOBの理想モデルを事前値とする。帯の幅は信頼区間ではなく仮定感度。
    for lam in (1.,10.,100.):
        R=A.T@A+lam*np.eye(len(t))
        K=np.block([[R,C.T],[C,np.zeros((3,3))]])
        h=np.linalg.solve(K,np.r_[A.T@b+lam*prior,constraints])[:len(t)]
        assert abs(sum(h)-1)<1e-8 and abs(h[0])<1e-8 and np.isfinite(h).all()
        solutions[lam]=np.cumsum(h);fits[lam]=E@h
    y=solutions[10.];family=np.array(list(solutions.values()))
    lo,hi=family.min(axis=0),family.max(axis=0)
    ax=axs.flat[col+1]
    ax.plot(t,np.full_like(t,amp),color='.7',ls=':',label='Hypothetical target step')
    ax.plot(t,amp*ideal,'k--',lw=2,label='Ideal DOB: tau=0.1s')
    ax.fill_between(t,amp*lo,amp*hi,color=colors[col],alpha=.18,label='Regularization sensitivity (not CI)')
    ax.plot(t,amp*y,color=colors[col],lw=2,ls='-.' if amp==400 else '-',label='Conditional chirp-derived estimate')
    ax.set_title(f'{amp:g} deg/s step — NOT measured'+(' / strong saturation' if amp==400 else ''))
    ax.set_ylabel('Velocity [deg/s]')
    axs[0,0].plot(t,y,color=colors[col],ls='-.' if amp==400 else '-',label=f'{amp:g} deg/s chirp'+(' (prior-sensitive)' if amp==400 else ''))
    if amp==400:
        ax.text(.04,.94,'Low-frequency behavior is assumed, NOT identified.\nNear-ideal curve does NOT demonstrate good tracking.',
                transform=ax.transAxes,va='top',fontsize=9,color='darkred',bbox=dict(facecolor='white',alpha=.85,edgecolor='none'))
    step_rows[f'estimate_{amp:g}_deg_s']=amp*y
    step_rows[f'sensitivity_min_{amp:g}_deg_s']=amp*lo
    step_rows[f'sensitivity_max_{amp:g}_deg_s']=amp*hi
    error=float(np.sqrt(np.mean(abs((fits[10.]-H)/H)**2)))
    results.append(dict(amplitude=amp,source=str(path),frequency_fit_min=float(f.min()),frequency_fit_max=float(f.max()),
        method='Causal FIR 76 taps, dt=20ms, regularization towards ideal tau=0.1s impulse; DC gain constrained to 1; h[0]=h[-1]=0',
        lambda_main=10,lambda_sensitivity=[1,10,100],relative_complex_frf_rmse=error,
        measured_step=False,large_signal_step_prediction_valid=False if amp==400 else None))
    faxs[0,col].semilogx(f,abs(H),'o',color=colors[col],label='Measured chirp FRF')
    faxs[0,col].semilogx(f,abs(fits[10.]),color=colors[col],label='Regularized FIR fit')
    faxs[1,col].semilogx(f,np.angle(H,deg=True),'o',color=colors[col])
    faxs[1,col].semilogx(f,np.angle(fits[10.],deg=True),color=colors[col])
    faxs[0,col].set_title(f'{amp:g}deg/s; relative FRF RMS error {100*error:.1f}%')
    faxs[0,col].set_ylabel('Gain');faxs[1,col].set_ylabel('Phase [deg]');faxs[1,col].set_xlabel('Frequency [Hz]')
    faxs[0,col].legend(fontsize=8)
    for axf in faxs[:,col]: axf.grid(True)

    feedback=list(csv.DictReader((path/'feedback.csv').open()))
    active=[r for r in feedback if r['stage']=='velocity_chirp' and r['type']=='1' and r['id']=='1']
    t0=float(active[0]['f7_time']);rt=np.array([float(r['f7_time'])-t0 for r in active])
    u=np.array([float(r['target']) for r in active]);v=np.array([float(r['velocity']) for r in active])
    cur=np.array([float(r['current']) for r in active]);pos=np.array([float(r['position']) for r in active])
    ram=list(csv.DictReader((path/'dob.csv').open()))
    ram=[r for r in ram if t0<=float(r['tick'])/1000<=float(active[-1]['f7_time'])]
    mt=np.array([float(r['tick'])/1000-t0 for r in ram]);model=np.array([float(r['model']) for r in ram]);cmd=np.array([float(r['current']) for r in ram])
    # 表はROS同一メッセージ内の実測値を保存。F7ログは別表にして再サンプリングしない。
    fields=['time_s','f7_time_s','ros_target_deg_s','raw_type2_velocity_deg_s','position_deg','reference_current_Arms_equivalent']
    with (a.output/f'actual_chirp_{amp:g}.csv').open('w') as file:
        writer=csv.writer(file);writer.writerow(fields)
        writer.writerows(zip(rt,[r['f7_time'] for r in active],u,v,pos,cur))
    with (a.output/f'f7_chirp_{amp:g}.csv').open('w') as file:
        writer=csv.DictWriter(file,fieldnames=list(ram[0]));writer.writeheader();writer.writerows(ram)
    def metrics(x): return dict(min=float(min(x)),max=float(max(x)),abs_peak=float(max(abs(x))),rms=float(np.sqrt(np.mean(x*x))))
    s=dict(amplitude=amp,frequency_start=meta['frequency_hz'],frequency_end=meta['frequency_end_hz'],seconds=meta['duration'],
        ros_target=metrics(u),raw_velocity=metrics(v),reference_current=metrics(cur),command_current=metrics(cmd),
        command_saturation_percent=float(100*np.mean(abs(cmd)>=10.78)),source=str(path))
    summary.append(s)
    for axw in (waxs[0,col],waxs[1,col]):
        axw.plot(rt,u,color='.65',lw=.7,label='ROS target')
        axw.plot(mt,model,'k--',lw=.9,label='F7 ideal-model velocity')
        axw.plot(rt,v,color=colors[col],lw=.7,label='Raw Type2 actual velocity')
        axw.set_ylabel('Velocity [deg/s]')
    waxs[0,col].set_title(f'{amp:g}deg/s, {meta["frequency_hz"]:g}-10Hz, 30s')
    waxs[1,col].set_xlim(0,3);waxs[1,col].set_title('First 3 seconds (measured chirp, not step)')
    waxs[2,col].plot(mt,cmd,color='#d62728',lw=1.,label='F7 controller current command [A]')
    waxs[2,col].plot(rt,cur,color='#0072B2',lw=.8,label='Reference current: Type2 torque / 0.94 [Arms equiv.]')
    for limit in (-11,11): waxs[2,col].axhline(limit,color='k',ls='--',lw=.7)
    for j in (0,1):
        axc=caxs[col,j]
        axc.plot(mt,cmd,color='#d62728',lw=1.,label='F7 controller current command [A]')
        axc.plot(rt,cur,color='#0072B2',lw=.9,label='Type2 torque / Kt reference [Arms equiv.]')
        axc.axhline(11,color='k',ls='--',lw=.8,label='Command limit +/-11A')
        axc.axhline(-11,color='k',ls='--',lw=.8)
        axc.set_ylim(-12,12);axc.set_xlabel('Time from chirp start [s]');axc.set_ylabel('Current (see legend for conventions)')
        axc.set_title(f'{amp:g}deg/s, {meta["frequency_hz"]:g}-10Hz: '+('full 30s' if j==0 else 'first 3s'))
        if j==1: axc.set_xlim(0,3)
        axc.grid(True);axc.legend(fontsize=8)
    waxs[2,col].set_ylabel('Current [A; reference is Arms-equivalent]')
    for axw in waxs[:,col]: axw.grid(True);axw.set_xlabel('Time from chirp start [s]');axw.legend(fontsize=7)

axs[0,0].plot(t,ideal,'k--',lw=2,label='Ideal DOB: 1-exp(-t/0.1)')
axs[0,0].set_title('Normalized step estimates');axs[0,0].set_ylabel('Velocity / step amplitude')
for ax in axs.flat:
    ax.axvspan(0,.05,color='.8',alpha=.35);ax.grid(True);ax.set_xlabel('Time after hypothetical step [s]');ax.legend(fontsize=7)
stepfig.suptitle('Chirp-derived STEP ESTIMATES — NOT measured step tests',fontsize=15)
stepfig.text(.5,.015,'DC gain=1 and ideal-model prior are assumptions. Final convergence is imposed, not measured.\n'
             'Below measured frequencies and above 10Hz are unknown; gray first 50ms is not resolved.\n'
             '400deg/s is strongly saturated: its LTI conversion is illustrative, not a valid large-signal prediction.',ha='center',fontsize=9)
stepfig.tight_layout(rect=(0,.095,1,.95));stepfig.savefig(a.output/'estimated_step_responses.png',dpi=160)
wavefig.suptitle('Measured chirps: command, actual velocity, model and torque-derived reference current')
wavefig.tight_layout(rect=(0,0,1,.96));wavefig.savefig(a.output/'measured_chirps_and_current.png',dpi=160)
currentfig.suptitle('F7 controller output vs torque-derived reference current — NOT identical current definitions')
currentfig.tight_layout(rect=(0,0,1,.96));currentfig.savefig(a.output/'controller_current_comparison.png',dpi=160)
fitfig.tight_layout();fitfig.savefig(a.output/'step_fit_validation.png',dpi=160)
with (a.output/'estimated_steps.csv').open('w') as file:
    writer=csv.writer(file);writer.writerow(step_rows);writer.writerows(zip(*step_rows.values()))
(a.output/'step_estimation.json').write_text(json.dumps(results,indent=2))
(a.output/'measured_summary.json').write_text(json.dumps(summary,indent=2))
with (a.output/'measured_summary.csv').open('w') as file:
    fields=['amplitude','frequency_start','frequency_end','seconds']+[f'{name}_{stat}' for name in ['ros_target','raw_velocity','reference_current','command_current'] for stat in ['min','max','abs_peak','rms']]+['command_saturation_percent']
    writer=csv.DictWriter(file,fields);writer.writeheader()
    for s in summary:
        row={k:s[k] for k in fields if k in s}
        for name in ['ros_target','raw_velocity','reference_current','command_current']:
            row.update({name+'_'+stat:value for stat,value in s[name].items()})
        writer.writerow(row)
print(json.dumps(dict(fits=results,measured=summary),indent=2))
