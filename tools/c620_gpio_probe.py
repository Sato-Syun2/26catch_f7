"""停止中の原点配線確認用。HOTPLUGでGPIO入力を読むだけ。リセット/書込なし。"""
import argparse
import json
from pathlib import Path
import re
import subprocess
import time

parser = argparse.ArgumentParser()
parser.add_argument('--duration', type=float, default=20)
args = parser.parse_args()
assert 0 < args.duration <= 60
programmer = '/home/taiyo/snap/code/257/.local/share/stm32cube/bundles/programmer/2.23.0/bin/STM32_Programmer_CLI'
ports = {f'P{chr(65+i)}': 0x40020010+0x400*i for i in range(7)}
command = [programmer, '-c', 'port=SWD', 'mode=HOTPLUG', 'sn=066DFF485570854967104926']
for address in ports.values():
    command += ['-r32', hex(address), '4']
root = Path(__file__).resolve().parents[1]
path = root/'debug_logs'/('c620_gpio_'+time.strftime('%Y%m%d_%H%M%S')+'.jsonl')
previous = None
start = time.monotonic()
print(path, flush=True)
with path.open('x') as log:
    while time.monotonic()-start < args.duration:
        completed = subprocess.run(command, text=True, capture_output=True, timeout=5, check=True)
        values = {}
        for port, address in ports.items():
            match = re.search(rf'0x{address:08X}\s*:\s*([0-9A-Fa-f]{{8}})', completed.stdout, re.I)
            if not match:
                raise RuntimeError('GPIO read failed')
            values[port] = int(match[1],16) & 0xffff
        changes = [f'{port}{bit}={int(bool(value & (1<<bit)))}'
                   for port,value in values.items() for bit in range(16)
                   if previous and (previous[port]^value)&(1<<bit) and (port,bit) not in [('PA',13),('PA',14)]]
        elapsed = round(time.monotonic()-start,3)
        log.write(json.dumps({'time':elapsed,'idr':values,'changes':changes})+'\n')
        log.flush()
        sensor_changes = [change for change in changes if change.split('=')[0] in ('PA3','PA4','PA5')]
        if previous is None:
            print(elapsed, {f'PA{bit}': int(bool(values['PA'] & (1<<bit))) for bit in (3,4,5)}, flush=True)
        elif sensor_changes:
            print(elapsed, sensor_changes, flush=True)
        previous = values
        time.sleep(.15)
