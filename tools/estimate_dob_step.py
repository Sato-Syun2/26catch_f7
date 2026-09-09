"""Offline step extrapolation from chirp FRFs; never sends motor commands."""
import json
from pathlib import Path
import numpy as np
from scipy import signal, optimize
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parents[1]
out = root/'debug_logs/dob_comparison_20260909_143157'
records = json.loads((out/'summary.json').read_text())
t = np.linspace(-.05, 1.0, 2101)
ideal = np.where(t>=0, 1-np.exp(-10*np.maximum(t,0)), 0)
fig, axs = plt.subplots(2,2,figsize=(14,9))
fitfig, fitaxs = plt.subplots(2,1,figsize=(10,8),sharex=True)
colors = ['#0072B2','#D55E00','#009E73']
results = []
curves = [t, (t>=0).astype(float), ideal]

def transfer(p, f):
    k,tau,delay = p
    return k*np.exp(-2j*np.pi*f*delay)/(1+2j*np.pi*f*tau)

def step(p):
    k,tau,delay = p
    return k*(1-np.exp(-np.maximum(t-delay,0)/tau))

axs[0,0].step(t,(t>=0).astype(float),where='post',color='0.65',label='Target unit step')
axs[0,0].plot(t,ideal,'k--',lw=2,label='Ideal DOB: 10/(s+10)')
for i,rec in enumerate(records):
    p = Path(rec['path'])
    amp = rec['metadata']['amplitude']
    d = np.genfromtxt(p/'feedback.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
    d = d[d['stage']=='chirp']
    times = d['time']
    grid = np.arange(times[0],times[-1],.01)
    u = d['target'][np.searchsorted(times,grid,side='right')-1]
    v = np.interp(grid,times,d['velocity'])
    opts = dict(fs=100,nperseg=1024,noverlap=512)
    f,pu = signal.welch(u,**opts)
    _,pv = signal.welch(v,**opts)
    _,cross = signal.csd(u,v,**opts)
    coh = abs(cross)**2/np.maximum(pu*pv,1e-20)
    h = cross/np.maximum(pu,1e-20)
    valid = (f>=1.5)&(f<=8)&(coh>=.9)
    fv,hv = f[valid],h[valid]
    def residual(p):
        # Complex relative error, equal per retained frequency bin.
        e = (transfer(p,fv)-hv)/np.maximum(abs(hv),.02)
        return np.r_[e.real,e.imag]
    fitted = optimize.least_squares(residual,[1,.1,.005],bounds=([.1,.005,0],[2,1,.15]))
    unity = optimize.least_squares(lambda q:residual([1,*q]),[.1,.005],bounds=([.005,0],[1,.15]))
    params = fitted.x
    alt = np.r_[1,unity.x]
    y,yalt = step(params),step(alt)
    color = colors[i]
    axs[0,0].plot(t,y,color=color,label=f'{amp:g} mm/s chirp fit')
    axs[0,0].plot(t,yalt,color=color,ls=':',alpha=.7)
    ax = axs.flat[i+1]
    ax.step(t,amp*(t>=0),where='post',color='0.65',label='Target step (hypothetical)')
    ax.plot(t,amp*ideal,'k--',lw=2,label='Ideal DOB')
    ax.plot(t,amp*y,color=color,lw=2,label='Estimated: fitted DC gain')
    ax.plot(t,amp*yalt,color=color,ls=':',label='Alternative: DC gain fixed to 1')
    ax.set_title(f'{amp:g} mm/s step — model prediction, NOT a measured step')
    ax.set_ylabel('Velocity [mm/s]')
    ax.legend(fontsize=8,loc='lower right')
    results.append(dict(amplitude=amp,source=str(p),model='K exp(-Ls)/(1+tau s)',
                        fitted=dict(K=params[0],tau_s=params[1],delay_s=params[2]),
                        unity_dc_alternative=dict(K=1,tau_s=alt[1],delay_s=alt[2]),
                        relative_complex_frf_rmse=float(np.sqrt(np.mean(abs((transfer(params,fv)-hv)/hv)**2))),
                        used_bins=len(fv),frequency_min=float(fv.min()),frequency_max=float(fv.max())))
    curves.extend([y,yalt])
    fitaxs[0].plot(fv,20*np.log10(abs(hv)),'.',color=color,label=f'{amp:g} measured FRF')
    fitaxs[0].plot(fv,20*np.log10(abs(transfer(params,fv))),color=color,label=f'{amp:g} model')
    fitaxs[1].plot(fv,np.angle(hv,deg=True),'.',color=color)
    fitaxs[1].plot(fv,np.angle(transfer(params,fv),deg=True),color=color)
axs[0,0].set_title('Normalized comparison (each predicted step / its target)')
axs[0,0].set_ylabel('Normalized velocity')
axs[0,0].legend(fontsize=8,loc='lower right')
for ax in axs.flat:
    ax.set_xlabel('Time after target step [s]')
    ax.grid(alpha=.25)
    ax.set_xlim(-.05,1)
fig.suptitle('DOB step-response estimates from the three velocity chirps',fontsize=16)
fig.text(.5,.02,'Assumption: first-order + delay, fitted to 1.5–8 Hz bins with coherence >= 0.9.\n'
         'DC / long-term tracking and fast transients are extrapolated; dotted curves show DC=1 sensitivity, not confidence bounds.\n'
         'Includes ROS/CAN timing. Amplitude-dependent nonlinear behavior may make real steps different.',ha='center',fontsize=9)
fig.tight_layout(rect=(0,.105,1,.95))
fig.savefig(out/'estimated_step_response.png',dpi=180)
fig.savefig(out/'estimated_step_response.svg')
for ax in fitaxs:
    ax.grid(alpha=.25)
fitaxs[0].set_ylabel('Gain [dB]');fitaxs[0].legend(fontsize=8)
fitaxs[1].set_ylabel('Phase [deg]');fitaxs[1].set_xlabel('Frequency [Hz]')
fitfig.suptitle('Model-fit check: measured chirp FRF vs fitted first-order + delay')
fitfig.tight_layout()
fitfig.savefig(out/'step_model_fit_check.png',dpi=160)
np.savetxt(out/'estimated_step_response.csv',np.array(curves).T,delimiter=',',
           header='time_s,target_normalized,ideal_normalized,fit20,unity20,fit200,unity200,fit800,unity800',comments='')
(out/'step_model_parameters.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results,indent=2))
