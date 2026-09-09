"""Mode5記録を解析。位置誤差0.5mmかつ速度2mm/s以内の継続収束を評価。"""
import csv,json,sys
from pathlib import Path
import numpy as np
for name in sys.argv[1:]:
 p=Path(name);m=json.loads((p/'metadata.json').read_text())
 with (p/'feedback.csv').open() as f:r=[x for x in csv.DictReader(f) if x['stage']=='step']
 if not r:print(p.name,'no step');continue
 t=np.array([float(x['time']) for x in r]);t-=t[0]
 x=np.array([float(q['position']) for q in r]);v=np.array([float(q['velocity']) for q in r]);i=np.array([float(q['current']) for q in r])
 bad=np.flatnonzero((abs(x-m['target'])>.5)|(abs(v)>2))
 settle=0 if not len(bad) else (float(t[bad[-1]+1]) if bad[-1]+1<len(t) else None)
 posbad=np.flatnonzero(abs(x-m['target'])>.5)
 possettle=0 if not len(posbad) else (float(t[posbad[-1]+1]) if posbad[-1]+1<len(t) else None)
 result=dict(result=m['result'],initial=m.get('initial'),target=m['target'],final=float(x[-1]),
             settle_s=settle,position_settle_s=possettle,peak_speed=float(max(abs(v))),peak_current=float(max(abs(i))),
             position_min=float(min(x)),position_max=float(max(x)),
             final_second_max_error=float(max(abs(x[t>t[-1]-1]-m['target']))),
             fb_hz=float((len(t)-1)/(t[-1]-t[0])),fb_max_gap=float(max(np.diff(t))))
 (p/'analysis.json').write_text(json.dumps(result,indent=2));print(p.name,json.dumps(result))
