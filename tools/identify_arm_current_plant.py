"""閉ループチャープの基準位相を使い、電流指令→速度の基本波を同定する。"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares

p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args();out=a.directory
m=json.loads((out/'metadata.json').read_text());mid=m['velocity_motor']
rows=[r for r in csv.DictReader((out/'feedback.csv').open()) if r['type']=='1' and int(r['id'])==mid
      and r['stage']=='angle_chirp' and r['chirp_phase']]
t=np.array([float(r['f7_time']) for r in rows]);phi=np.array([float(r['chirp_phase']) for r in rows]);x=np.deg2rad([float(r['position']) for r in rows])
ram=list(csv.DictReader((out/'dob.csv').open()));rt=np.array([float(r['tick'])*.001 for r in ram]);current=np.array([float(r['current']) for r in ram])
cycles=json.loads((out/'angle_summary.json').read_text())['cycles_detail'];measure=[]
for c in cycles:
    if not c['valid']:continue
    sel=(np.floor(phi/(2*np.pi))==c['cycle']);tt=t[sel];pp=phi[sel]
    A=np.column_stack((np.sin(pp),np.cos(pp),np.ones(len(pp)),tt-tt.mean()))
    q=np.linalg.lstsq(A,x[sel],rcond=None)[0]
    i=np.linalg.lstsq(A,np.interp(tt,rt,current),rcond=None)[0]
    omega=2*np.pi*c['frequency_hz'];iv=complex(i[1],-i[0])
    if abs(iv)<.05:continue
    H=omega*complex(q[0],q[1])/iv
    measure.append(dict(frequency_hz=c['frequency_hz'],current_fundamental_A=abs(iv),
                        gain_rad_s_per_A=abs(H),phase_deg=np.angle(H,deg=True)))
f=np.array([r['frequency_hz'] for r in measure]);w=2*np.pi*f
H=np.array([r['gain_rad_s_per_A']*np.exp(1j*np.deg2rad(r['phase_deg'])) for r in measure])
# P(s)=b/(s+a)*exp(-Ls)。局所記述関数であり摩擦の影響を含む。
def response(z):return z[0]/(1j*w+z[1])*np.exp(-1j*w*z[2])
def residual(z):
    e=(response(z)-H)/abs(H);return np.r_[e.real,e.imag]
fit=least_squares(residual,[3.,3.,.01],bounds=([.01,0.,0.],[1000.,200.,.2]))
b,damping,delay=fit.x;kt=.94 if mid==1 else 1.22
result=dict(motor=mid,source=str(out),b_rad_s2_per_A=float(b),damping_per_s=float(damping),delay_s=float(delay),
            effective_J=float(kt/b),effective_d=float(kt*damping/b),
            mean_complex_relative_error=float(np.mean(np.abs(response(fit.x)-H)/abs(H))),measurements=measure,
            warning='Closed-loop fundamental describing function at this posture and amplitude; not a globally validated linear plant')
# 位置に比例した復元力を許す候補。単純一次遅れと比較するだけで自動適用しない。
def second(z):
    s=1j*w
    return z[0]*s/(s*s+2*z[1]*z[2]*s+z[1]**2)*np.exp(-s*z[3])
def residual2(z):
    e=(second(z)-H)/abs(H);return np.r_[e.real,e.imag]
fits=[least_squares(residual2,[b0,wn,1.,.01],bounds=([.01,.01,.01,0.],[1000.,200.,30.,.2]))
      for b0,wn in [(2.,3.),(10.,6.),(20.,12.)]]
best=min(fits,key=lambda x:x.cost)
result['second_order_candidate']=dict(b=float(best.x[0]),wn=float(best.x[1]),zeta=float(best.x[2]),
    delay_s=float(best.x[3]),mean_complex_relative_error=float(np.mean(abs(second(best.x)-H)/abs(H))))
result['automatic_gain_design_accepted']=False
result['rejection_reason']='Frequency range is too narrow for crossover/margin certification; elbow fits also have large residuals/boundary solutions. Do not copy fitted J/d into firmware.'
(out/'current_plant.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='measurements'},indent=2))
