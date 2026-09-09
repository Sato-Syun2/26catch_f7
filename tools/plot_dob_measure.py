"""Plot logged DOB response without arbitrary time alignment."""
import sys
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p = Path(sys.argv[1])
meta = json.loads((p/'metadata.json').read_text())
d = np.genfromtxt(p/'feedback.csv', delimiter=',', names=True, dtype=None, encoding='utf8')
mask = np.isin(d['stage'], ['pre', 'chirp', 'post'])
d = d[mask]
t = d['time'] - d['time'][0]
u = d['target']
v = d['velocity']
y = np.zeros(len(t))
y[0] = v[0]
for i in range(1, len(t)):
    a = np.exp(-meta['alpha']*(t[i]-t[i-1]))
    y[i] = a*y[i-1] + (1-a)*u[i-1]
m = d['stage'] == 'chirp'
meta = json.loads((p/'metadata.json').read_text())
frequency = meta['f0_hz']*np.exp(np.log(meta['f1_hz']/meta['f0_hz'])*(t-t[m][0])/meta['duration'])
bands = []
for lo, hi in [(.2,.5),(.5,1),(1,2),(2,5),(5,10)]:
    b = m & (frequency >= lo) & (frequency < hi)
    if not b.any():
        continue
    bands.append(dict(frequency_hz=[lo,hi],
                      rmse_mm_s=float(np.sqrt(np.mean((v[b]-y[b])**2))),
                      measured_rms_mm_s=float(np.sqrt(np.mean(v[b]**2))),
                      reference_rms_mm_s=float(np.sqrt(np.mean(y[b]**2)))))
stats = dict(position_min=float(d['position'].min()), position_max=float(d['position'].max()),
             peak_current_A=float(np.abs(d['current']).max()),
             peak_velocity_mm_s=float(np.abs(v[m]).max()),
             current_near_limit_fraction=float(np.mean(np.abs(d['current'][m]) >= .95*meta.get('current_limit',4))),
             velocity_rmse_mm_s=float(np.sqrt(np.mean((v[m]-y[m])**2))),
             reference_rms_mm_s=float(np.sqrt(np.mean(y[m]**2))),
             feedback_hz=float(1/np.median(np.diff(t))),
             maximum_feedback_gap_s=float(np.max(np.diff(t))),
             frequency_bands=bands,
             note=f'Reference {meta["alpha"]}/(s+{meta["alpha"]}); host feedback receive timestamps; target is most recently published command, not acknowledged application time. No fitted time shift. Near-limit current is >=95% of configured limit, not the internal controller saturation flag.')
(p/'analysis.json').write_text(json.dumps(stats, indent=2))
fig, axs = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
axs[0].plot(t, v, label='Measured velocity', lw=.6, alpha=.7)
axs[0].plot(t, u, label='Velocity command', lw=.7)
axs[0].plot(t, y, label=f'Reference {meta["alpha"]}/(s+{meta["alpha"]})', lw=1)
axs[0].set_ylabel('Velocity [mm/s]')
axs[0].legend(loc='upper right')
axs[1].plot(t, d['position'])
axs[1].set_ylabel('Position [mm]')
axs[2].plot(t, d['current'], lw=.7)
axs[2].set_ylabel('Current [A]')
axs[2].set_xlabel('Time from pre-test [s]')
for ax in axs:
    ax.grid(True, alpha=.3)
fig.suptitle(f'ID4 velocity DOB: {meta["f0_hz"]}–{meta["f1_hz"]} Hz, amplitude {meta["amplitude"]} mm/s\n'
             f'RMSE vs reference = {stats["velocity_rmse_mm_s"]:.3f} mm/s; reference RMS = {stats["reference_rms_mm_s"]:.3f} mm/s')
fig.tight_layout()
fig.savefig(p/'dob_response.png', dpi=170)
print(json.dumps(stats, indent=2))
