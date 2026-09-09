"""Short, bounded ID4 velocity-DOB pulse and zero-speed recovery."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from dob_measure import Test

p = argparse.ArgumentParser()
p.add_argument('--amplitude',type=float,required=True)
p.add_argument('--width',type=float,default=.12)
p.add_argument('--current-limit',type=float,default=4)
p.add_argument('--alpha',type=float,default=10)
p.add_argument('--kp',type=float,default=.5)
a = p.parse_args()
if not 0 < abs(a.amplitude) <= 800 or not 0 < a.width <= .15:
    p.error('pulse outside tested configuration envelope')
directory = Path(__file__).resolve().parents[1]/'debug_logs'/('dob_pulse_'+time.strftime('%Y%m%d_%H%M%S'))
directory.mkdir()
print(directory,flush=True)
meta = vars(a)|dict(current_limit_A=a.current_limit,alpha=a.alpha,result='not started')
rclpy.init()
n = Test(directory)
try:
    end=time.monotonic()+3
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
    n.check(True)
    if abs(n.fb.position-250)>3 or abs(n.fb.velocity)>10:
        raise RuntimeError('must start stopped at 250mm')
    n.service('disable');n.service('mode',4);n.send(0,True);n.service('enable')
    n.stage='pre';n.loop(.5,lambda t:n.send(0,True),True)
    n.stage='chirp';n.loop(a.width,lambda t:n.send(a.amplitude,True),True)
    n.stage='recovery';n.loop(2,lambda t:n.send(0,True),True)
    if abs(n.fb.velocity)>10:raise RuntimeError('velocity did not settle')
    meta['result']='complete'
except BaseException as e:
    meta['result']=repr(e)
finally:
    try:n.service('disable')
    except Exception as e:meta['disable_error']=repr(e)
    n.file.close()
    (directory/'metadata.json').write_text(json.dumps(meta,indent=2))
    print(meta,flush=True)
    n.destroy_node();rclpy.shutdown()
