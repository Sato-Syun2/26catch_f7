"""低電流ステップのFB比較。時刻はホスト受信時刻で、FOC帯域の測定ではない。"""
import csv
import json
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('paths', nargs='*')
parser.add_argument('--output', default='debug_logs/arm_current_tuning_20260909')
args = parser.parse_args()
summary = []
names = args.paths or [str(p.parent) for p in sorted(Path('debug_logs').glob('arm_step_20260909_*/metadata.json'))
                         if json.loads(p.read_text()).get('current_step')]
ids = sorted({json.loads((Path(p)/'metadata.json').read_text())['velocity_motor'] for p in names})
fig, aa = plt.subplots(len(ids), 1, figsize=(11, 3.5*len(ids)), sharex=True, squeeze=False)
axes = dict(zip(ids, aa[:, 0]))
for name in names:
    path = Path(name)
    meta = json.loads((path/'metadata.json').read_text())
    mid = meta['velocity_motor']
    rows = [r for r in csv.DictReader((path/'feedback.csv').open())
            if r['type']=='1' and int(r['id'])==mid]
    pulses = list(dict.fromkeys(r['stage'] for r in rows if r['stage'].startswith('current_')))
    result = dict(path=str(path), id=mid, tag=meta['tag'], result=meta['result'], pulses=[])
    for stage in pulses:
        rr = [r for r in rows if r['stage']==stage]
        t = np.array([float(r['time']) for r in rr]); t -= t[0]
        iq = np.array([float(r['current']) for r in rr])
        cmd = float(stage.rsplit('_',1)[1])
        plateau = float(np.median(iq[t>t[-1]-.08]))
        result['pulses'].append(dict(stage=stage, command_A=cmd, plateau_A=plateau,
                                    ratio=plateau/cmd, peak_abs_A=float(max(abs(iq))),
                                    observed_duration_s=float(t[-1]),
                                    reached_coast=any(r['stage']==stage.replace('current_', 'coast_', 1) for r in rows),
                                    plateau_std_A=float(np.std(iq[t>t[-1]-.08]))))
    if pulses:
        pp=result['pulses']
        result['mean_abs_plateau_error_A']=float(np.mean([abs(p['plateau_A']-p['command_A']) for p in pp]))
        result['within_5percent_count']=sum(abs(p['ratio']-1.)<=.05 for p in pp)
        result['peak_abs_current_A']=max(p['peak_abs_A'] for p in pp)
        first = next(float(r['time']) for r in rows if r['stage']==pulses[0])
        rr = [r for r in rows if r['stage'].startswith(('current_', 'coast_'))]
        pos=[float(r['position']) for r in rr]
        result['position_span_deg']=max(pos)-min(pos)
        axes[mid].plot([float(r['time'])-first for r in rr],
                        [float(r['current']) for r in rr], label=meta['tag'])
        if len(axes[mid].lines)==1:
            axes[mid].plot([float(r['time'])-first for r in rr],
                            [float(r['target']) for r in rr], 'k--', label='command')
    summary.append(result)
for mid, ax in axes.items():
    ax.set_ylabel(f'ID{mid} current [A]'); ax.grid()
    if ax.lines: ax.legend()
axes[ids[-1]].set_xlabel('Host receive time from first pulse [s]')
fig.suptitle('Current command vs filtered current feedback (iqf)')
fig.tight_layout()
out = Path(args.output)
out.mkdir(parents=True, exist_ok=True)
fig.savefig(out/'current_comparison.png', dpi=160)
(out/'comparison.json').write_text(json.dumps(summary, indent=2))
for r in summary:
    print(r['tag'], r['id'], 'plateaus:', [round(p['plateau_A'],4) for p in r['pulses']])
