"""停止後にF7の1秒集計リングをSWD経由で読み出す。MCUリセットなし。"""
import argparse
import csv
import json
import re
import struct
import subprocess
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument('output')
a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
elf='build/Debug/26catch_f7.elf'
gdb=subprocess.check_output(['gdb','-batch',elf,'-ex','p/x &arm_rate_log','-ex','p sizeof(arm_rate_log)'],text=True)
address=re.search(r'\$1 = (0x[0-9a-f]+)',gdb).group(1)
size=int(re.search(r'\$2 = (\d+)',gdb).group(1))
if size!=12004: raise SystemExit(f'unexpected RAM schema size {size}')
programmer='/home/taiyo/snap/code/257/.local/share/stm32cube/bundles/programmer/2.23.0/bin/STM32_Programmer_CLI'
subprocess.run([programmer,'-c','port=SWD','mode=HOTPLUG','-u',address,str(size),str(out/'rate.bin')],check=True)
raw=(out/'rate.bin').read_bytes()
count=struct.unpack_from('<I',raw)[0]
fields=['tick','interval','received','coalesced','ring_overrun','priority_full','tx_errors',
        'can_errors','can_error_code','loops','loop_gap_ms','enabled_mask',
        'target1','target2','target_gap1_ms','target_gap2_ms','iq1','iq2','position1','position2',
        'velocity1','velocity2','type2_1','type2_2','tx_complete']
rows=[dict(zip(fields,struct.unpack_from('<25I',raw,4+(i%120)*100))) for i in range(max(0,count-120),count)]
with (out/'rate.csv').open('w') as f:
    w=csv.DictWriter(f,fields);w.writeheader();w.writerows(rows)
(out/'metadata.json').write_text(json.dumps(dict(address=address,size=size,total_samples=count,elf=elf),indent=2))
for r in rows[-25:]: print(r)
