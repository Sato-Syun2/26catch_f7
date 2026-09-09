"""Disable後にiqf・VBUSの実応答リングを読み出す。制御への書き込みなし。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser(); p.add_argument('directory',type=Path)
out=p.parse_args().directory
meta=json.loads((out/'metadata.json').read_text())
elf=Path('build/Debug/26catch_f7.elf')
if hashlib.sha256(elf.read_bytes()).hexdigest()!=meta['local_elf_sha256']:
    raise SystemExit('ELF differs from measurement')
info=subprocess.check_output(['gdb','-batch',str(elf),'-ex','p/x &arm_current_log','-ex','p sizeof(arm_current_log)'],text=True)
addr=re.search(r'\$1 = (0x[0-9a-f]+)',info)[1]
size=int(re.search(r'\$2 = (\d+)',info)[1])
assert size==32772
programmer='/home/taiyo/snap/code/257/.local/share/stm32cube/bundles/programmer/2.23.0/bin/STM32_Programmer_CLI'
subprocess.run([programmer,'-c','port=SWD','mode=HOTPLUG','-u',addr,str(size),str(out/'current_diagnostic.bin')],check=True,stdout=subprocess.DEVNULL)
raw=(out/'current_diagnostic.bin').read_bytes()
count=struct.unpack_from('<I',raw)[0]
rows=[struct.unpack_from('<3If',raw,4+(i%2048)*16) for i in range(max(0,count-2048),count)]
rows=[r for r in rows if meta['measurement_start_f7']*1000<=r[0]<=meta['measurement_end_f7']*1000]
with (out/'current_diagnostic.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['tick','id','address','value']);w.writerows(rows)
dob=list(csv.DictReader((out/'dob.csv').open()))
t0=meta['measurement_start_f7']
fig,ax=plt.subplots(2,1,figsize=(12,7),sharex=True)
dt=np.array([float(r['tick'])/1000-t0 for r in dob])
for key in ['current','reference_current']:
    ax[0].plot(dt,[float(r[key]) for r in dob],label=key,lw=.7)
metrics={}
for address,name,index in [(0x701a,'iqf',0),(0x701c,'VBUS',1)]:
    selected=[r for r in rows if r[2]==address]
    if not selected: continue
    t=np.array([r[0]/1000-t0 for r in selected]);v=np.array([r[3] for r in selected])
    ax[index].plot(t,v,label=name,lw=.8)
    metrics[name]=dict(samples=len(v),min=float(v.min()),max=float(v.max()),rms=float(np.sqrt(np.mean(v*v))),max_gap_ms=float(np.diff(t).max()*1000) if len(t)>1 else None)
ax[0].set_ylabel('Current [A; torque/Kt reference differs]');ax[1].set_ylabel('Bus voltage [V]')
ax[1].set_xlabel('F7 elapsed [s]')
for a in ax:a.grid();a.legend()
fig.tight_layout();fig.savefig(out/'current_diagnostic.png',dpi=150)
(out/'current_diagnostic_summary.json').write_text(json.dumps(metrics,indent=2))
print(json.dumps(metrics,indent=2))
