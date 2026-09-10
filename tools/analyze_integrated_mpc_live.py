"""実機CSVと同一ELFのHot Plug RAMログを集計する（実機へアクセスしない）。"""
import csv
import json
import re
import sys
import subprocess
from pathlib import Path

directory=Path(sys.argv[1]); raw=Path(sys.argv[2])
rows=list(csv.DictReader((directory/'feedback.csv').open()))
phase='continuous' if any(r['phase']=='continuous' for r in rows) else 'simultaneous'
active=[r for r in rows if r['phase']==phase]
start=min(float(r['f7_time']) for r in active)+.5
end=max(float(r['f7_time']) for r in active)
active=[r for r in active if start<=float(r['f7_time'])<=end]
times=sorted({float(r['host_time']) for r in active})
result={'phase':phase,'steady_f7_seconds':[start,end],
        'feedback_hz':(len(times)-1)/(times[-1]-times[0]),
        'max_steady_host_gap':max(b-a for a,b in zip(times,times[1:])), 'axes':{}}
for key in [(1,2),(1,1),(0,4),(0,3)]:
    selected=[r for r in active if (int(r['type']),int(r['id']))==key]
    result['axes'][str(key)]={'position_min':min(float(r['position']) for r in selected),
        'position_max':max(float(r['position']) for r in selected),
        'peak_abs_fb_current':max(abs(float(r['current'])) for r in selected),
        'states':sorted({int(r['state']) for r in selected}),
        'codes':sorted({int(r['code']) for r in selected})}
memory={}
for address,words in re.findall(r'0x([0-9A-Fa-f]{8})\s*:\s*((?:[0-9A-Fa-f]{8}(?:\s+|$))+)',raw.read_text()):
    for i,word in enumerate(words.split()): memory[int(address,16)+4*i]=int(word,16)
# Debug/Releaseで配置が異なるため、実際のELFからシンボルを解決する。
elf=sys.argv[3] if len(sys.argv)>3 else 'build/Debug/26catch_f7.elf'
symbols={}
for line in subprocess.check_output(['arm-none-eabi-nm','--defined-only',elf],text=True).splitlines():
    fields=line.split()
    if len(fields)==3: symbols[fields[2]]=int(fields[0],16)
base=symbols['arm_rate_log']; count=memory[base]; samples=[]
for index in range(max(0,count-120),count):
    address=base+4+(index%120)*100
    v=[memory[address+4*i] for i in range(25)]
    tick,interval=v[0],v[1]
    if interval and tick/1000<=end and (tick-interval)/1000>=start and v[11]==3:
        samples.append(v)
if samples:
    result['robstride_rate']={'samples':len(samples),
        'loops_hz_min':min(v[9]*1000/v[1] for v in samples),
        'loops_hz_max':max(v[9]*1000/v[1] for v in samples),
        'max_loop_gap_ms':max(v[10] for v in samples),
        'ring_overrun':sum(v[4] for v in samples),
        'priority_full':sum(v[5] for v in samples),
        'tx_errors':sum(v[6] for v in samples),
        'can_errors':sum(v[7] for v in samples),
        'type2_rx_hz_min_by_id':[min(v[22+i]*1000/v[1] for v in samples) for i in range(2)]}
result['stack_free_min_bytes']={'robstride':memory[symbols['robstride_stack_free_min_bytes']],
                              'robomaster':memory[symbols['robomas_stack_free_min_bytes']]}
(directory/'analysis.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
