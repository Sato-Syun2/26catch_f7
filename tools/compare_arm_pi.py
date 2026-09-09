"""PI候補の同一角度チャープを比較。FW設定は変更しない。"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('directories',nargs='+',type=Path);p.add_argument('--output',type=Path,required=True)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);results=[]
fig,axes=plt.subplots(2,2,figsize=(13,8))
for out in a.directories:
    meta=json.loads((out/'metadata.json').read_text());mid=meta['velocity_motor'];col=mid-1
    rows=list(csv.DictReader((out/'dob.csv').open()))
    cmd=np.array([float(r['current']) for r in rows]);v=np.array([float(r['velocity']) for r in rows]);model=np.array([float(r['model']) for r in rows]);target=np.array([float(r['target']) for r in rows])
    selected=abs(target)>.1
    cycles=json.loads((out/'angle_summary.json').read_text())['cycles_detail']
    good=[r for r in cycles if r['valid']];gain=np.array([r['physical_velocity_model_gain'] for r in good]);phase=np.deg2rad([r['physical_velocity_model_phase_deg'] for r in good]);freq=np.array([r['frequency_hz'] for r in good])
    metrics=dict(motor=mid,tag=meta['tag'],source=str(out),result=meta['result'],
        filtered_velocity_rmse_deg_s=float(np.sqrt(np.mean((v[selected]-model[selected])**2))),
        current_command_peak_A=float(abs(cmd).max()),current_command_rms_A=float(np.sqrt(np.mean(cmd**2))),
        mean_complex_tracking_error=float(np.mean(abs(gain*np.exp(1j*phase)-1))),
        low_frequency_gain=float(np.median(gain[freq<.5])),valid_cycles=len(good))
    common=freq<=(1.3 if mid==1 else .79)
    metrics['common_band_time_weighted_complex_error']=float(np.average(abs(gain[common]*np.exp(1j*phase[common])-1),weights=1/freq[common]))
    metrics['position_net_drift_deg']=meta['final'][str((1,mid))]['position']-meta['initial_position'][str(mid)]
    feedback=list(csv.DictReader((out/'feedback.csv').open()))
    zero=[float(r['f7_time']) for r in feedback if r['stage']=='velocity_zero' and r['type']=='1' and r['id']==str(mid)]
    if zero:
        # 旧試験の停止観測は1秒、新試験は5秒。評価窓も記録して混同を避ける。
        delay=1.0 if meta.get('settle_seconds',1)>1 else .3
        stopped=[r for r in rows if float(r['tick'])/1000>=min(zero)+delay]
        if stopped:
            metrics['stop_after_delay_s']=delay
            metrics['stop_samples']=len(stopped)
            metrics['stop_position_span_deg']=float(np.ptp([float(r['position']) for r in stopped]))
            metrics['stop_current_command_rms_A']=float(np.sqrt(np.mean([float(r['current'])**2 for r in stopped])))
    results.append(metrics)
    axes[0,col].plot(freq,gain,'o-',label=meta['tag'])
    axes[1,col].plot(freq,np.rad2deg(phase),'o-',label=meta['tag'])
for col in (0,1):
    axes[0,col].set_title(f'ID{col+1}: actual velocity / target model')
    axes[0,col].axhline(1,color='k',ls='--');axes[1,col].axhline(0,color='k',ls='--')
    axes[0,col].set_ylabel('Gain ratio');axes[1,col].set_ylabel('Phase difference [deg]')
    axes[1,col].set_xlabel('Frequency [Hz]')
    for ax in axes[:,col]:ax.grid(True);ax.legend(fontsize=7)
fig.tight_layout();fig.savefig(a.output/'pi_comparison.png',dpi=150)
(a.output/'pi_comparison.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
