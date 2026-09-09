"""有効周期から指令速度→位置由来速度の一次遅れ近似を求める（設定は変更しない）。"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('directories',nargs='+',type=Path);p.add_argument('--output',type=Path,required=True)
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
results={};fig,axes=plt.subplots(2,len(a.directories),figsize=(6*len(a.directories),7),squeeze=False)
for col,path in enumerate(a.directories):
    m=json.loads((path/'metadata.json').read_text());tau=m['angle_wave']['tau'];mid=m['velocity_motor']
    rows=[r for r in json.loads((path/'angle_summary.json').read_text())['cycles_detail'] if r['valid']]
    f=np.array([r['frequency_hz'] for r in rows]);omega=2*np.pi*f
    ratio=np.array([r['physical_velocity_model_gain']*np.exp(1j*np.deg2rad(r['physical_velocity_model_phase_deg'])) for r in rows])
    target=1/(1+1j*omega*tau)
    measured=ratio*target
    def residual(x):
        e=(x[0]/(1+1j*omega*x[1])-measured)/np.abs(measured)
        return np.r_[e.real,e.imag]
    fit=least_squares(residual,[1,tau],bounds=([.2,.001],[2.,2.]))
    K,T=fit.x;estimate=K/(1+1j*omega*T)
    relative=np.abs(estimate-measured)/np.abs(measured)
    results[mid]=dict(source=str(path),target_tau_s=tau,fit_gain=float(K),fit_tau_s=float(T),
        frequency_min_hz=float(f.min()),frequency_max_hz=float(f.max()),
        mean_complex_relative_error=float(relative.mean()),max_complex_relative_error=float(relative.max()),
        note='Empirical local first-order approximation, not proven fastest achievable model; no extrapolation outside measured posture/amplitude/frequency')
    for H,label,style in [(measured,'Measured from position','o'),(target,'F7 target model','--'),(estimate,'Local first-order fit','-')]:
        axes[0,col].plot(f,20*np.log10(abs(H)),style,label=label)
        axes[1,col].plot(f,np.angle(H,deg=True),style,label=label)
    axes[0,col].set_title(f'ID{mid}: fit K={K:.3f}, tau={T:.3f}s')
    axes[0,col].set_ylabel('Gain [dB]');axes[1,col].set_ylabel('Phase [deg]')
    axes[1,col].set_xlabel('Frequency [Hz]')
    for ax in axes[:,col]:ax.grid(True);ax.legend()
fig.tight_layout();fig.savefig(a.output/'identified_models.png',dpi=150)
(a.output/'identified_models.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
