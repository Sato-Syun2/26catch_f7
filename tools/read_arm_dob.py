"""Disable後にDOB内部目標・速度・電流指令をRAMから読み出して比較する。"""
import argparse
import csv
import hashlib
import json
import re
import struct
import subprocess
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args()
out=a.output;out.mkdir(parents=True,exist_ok=True)
elf=Path('build/Debug/26catch_f7.elf')
gdb=subprocess.check_output(['gdb','-batch',str(elf),'-ex','p/x &arm_dob_log','-ex','p sizeof(arm_dob_log)'],text=True)
address=re.search(r'\$1 = (0x[0-9a-f]+)',gdb).group(1)
size=int(re.search(r'\$2 = (\d+)',gdb).group(1))
if size!=131076: raise SystemExit(f'unexpected RAM schema size {size}')
programmer='/home/taiyo/snap/code/257/.local/share/stm32cube/bundles/programmer/2.23.0/bin/STM32_Programmer_CLI'
subprocess.run([programmer,'-c','port=SWD','mode=HOTPLUG','-u',address,str(size),str(out/'dob.bin')],check=True,stdout=subprocess.DEVNULL)
raw=(out/'dob.bin').read_bytes();count=struct.unpack_from('<I',raw)[0]
fields=['tick','id','target','model','velocity','current','position','reference_current']
rows=[dict(zip(fields,struct.unpack_from('<2I6f',raw,4+(i%4096)*32))) for i in range(max(0,count-4096),count)]
test_metadata=out/'metadata.json'
if test_metadata.exists():
    test=json.loads(test_metadata.read_text())
    if test.get('local_elf_sha256')!=hashlib.sha256(elf.read_bytes()).hexdigest():
        raise SystemExit('ELF differs from test metadata')
    if 'measurement_start_f7' in test:
        rows=[r for r in rows if r['id']==test['velocity_motor'] and
              test['measurement_start_f7']*1000<=r['tick']<=test['measurement_end_f7']*1000]
with (out/'dob.csv').open('w') as f:
    w=csv.DictWriter(f,fields);w.writeheader();w.writerows(rows)
metrics={'elf_sha256':hashlib.sha256(elf.read_bytes()).hexdigest(),'count':count,'axes':{}}
for mid in (1,2):
    data=[r for r in rows if r['id']==mid]
    if not data: continue
    t=np.array([r['tick'] for r in data],dtype=float);t=(t-t[0])*.001
    v={k:np.array([r[k] for r in data]) for k in fields[2:]}
    selected=np.abs(v['target'])>.1
    if not selected.any(): continue
    metrics['axes'][mid]={'rmse_deg_s':float(np.sqrt(np.mean((v['velocity'][selected]-v['model'][selected])**2))),
        'model_rms_deg_s':float(np.sqrt(np.mean(v['model'][selected]**2))),
        'peak_command_A':float(np.abs(v['current']).max()),
        'position_min_deg':float(v['position'].min()),'position_max_deg':float(v['position'].max())}
    fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
    for k in ('target','model','velocity'):axes[0].plot(t,v[k],label=k)
    axes[0].set_ylabel('Velocity [deg/s]');axes[0].legend()
    axes[1].plot(t,v['position']);axes[1].set_ylabel('Position [deg]')
    axes[2].plot(t,v['current'],label='F7 current command [A]')
    axes[2].plot(t,v['reference_current'],label='Torque/Kt [Arms equivalent]',alpha=.7)
    axes[2].legend();axes[2].set_xlabel('F7 elapsed time [s]')
    for ax in axes:ax.grid(True)
    sample_hz=1/np.median(np.diff(t))
    fig.suptitle(f'ID{mid}: actual F7 DOB states ({sample_hz:.0f}Hz RAM recording)')
    fig.tight_layout();fig.savefig(out/f'dob_id{mid}.png',dpi=150);plt.close(fig)
(out/'dob_summary.json').write_text(json.dumps(metrics,indent=2));print(json.dumps(metrics,indent=2))
