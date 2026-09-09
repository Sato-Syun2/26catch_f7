"""Low-speed live validation of ID4 braking latch / command timeout."""
import argparse,json,time
from pathlib import Path
import rclpy
from dob_measure import Test
p=argparse.ArgumentParser()
p.add_argument('kind',choices=['upper','lower','timeout'])
a=p.parse_args()
expected={'upper':400,'lower':100,'timeout':250}[a.kind]
directory=Path(__file__).resolve().parents[1]/'debug_logs'/('id4_brake_'+a.kind+'_'+time.strftime('%Y%m%d_%H%M%S'))
directory.mkdir();print(directory,flush=True)
rclpy.init();n=Test(directory);result={'kind':a.kind,'result':'not started'}
try:
    end=time.monotonic()+3
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
    n.check(True)
    if abs(n.fb.position-expected)>3:raise RuntimeError('unexpected start position')
    n.service('disable');n.service('mode',4);n.send(0,True);n.service('enable')
    n.stage='pre';n.loop(.3,lambda t:n.send(0,True),True)
    if a.kind=='timeout':
        n.stage='pulse';n.loop(.06,lambda t:n.send(200,True),True)
        n.stage='no_commands';end=time.monotonic()+1
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.002);n.check(True)
    else:
        n.stage='guard';n.loop(1.5,lambda t:n.send(100 if a.kind=='upper' else -100,True),True)
    if n.fb.state!=0 or abs(n.fb.velocity)>5:
        raise RuntimeError('automatic stop/disable not confirmed')
    result.update(result='complete',final_position=n.fb.position,final_velocity=n.fb.velocity,final_state=n.fb.state)
except BaseException as e:result['result']=repr(e)
finally:
    try:n.service('disable')
    except Exception as e:result['disable_error']=repr(e)
    n.file.close();(directory/'metadata.json').write_text(json.dumps(result,indent=2))
    print(result,flush=True);n.destroy_node();rclpy.shutdown()
