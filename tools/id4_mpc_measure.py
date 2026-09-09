"""Mode5の単一位置ステップ試験。ID4のみ、FBを記録して最終Disable。"""
import argparse,json,time
from pathlib import Path
import rclpy
from dob_measure import Test
p=argparse.ArgumentParser();p.add_argument('target',type=float)
p.add_argument('--duration',type=float,default=8);a=p.parse_args()
if not 0<=a.target<=520:raise SystemExit('target must be 0..520')
directory=Path(__file__).resolve().parents[1]/'debug_logs'/('id4_mpc_'+time.strftime('%Y%m%d_%H%M%S'))
directory.mkdir();print(directory,flush=True)
rclpy.init();n=Test(directory);result={'target':a.target,'result':'not started'}
try:
    end=time.monotonic()+4
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
    n.check()
    if not -1<n.fb.position<521 or abs(n.fb.velocity)>2 or n.fb.state!=0:
        raise RuntimeError('initial stopped state not confirmed')
    result['initial']=n.fb.position
    n.service('disable');n.service('mode',5)
    n.send(max(0,min(520,n.fb.position)))
    n.loop(.1,lambda t:n.send(max(0,min(520,result['initial']))))
    n.service('enable');n.stage='step';begin=time.monotonic()
    while time.monotonic()-begin<a.duration:
        rclpy.spin_once(n,timeout_sec=.002);n.check()
        if n.fb.position<=-1 or n.fb.position>=525:
            raise RuntimeError('host boundary')
        if time.monotonic()-begin>.1 and n.fb.state!=1:
            raise RuntimeError('position control disabled / fault')
        n.send(a.target)
        until=time.monotonic()+.01
        while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.001)
    result.update(result='complete' if abs(n.fb.position-a.target)<=.5 else 'target not reached',
                  final_position=n.fb.position,final_velocity=n.fb.velocity)
except BaseException as e:result['result']=repr(e)
finally:
    try:n.service('disable')
    except Exception as e:result['disable_error']=repr(e)
    n.file.close();(directory/'metadata.json').write_text(json.dumps(result,indent=2))
    print(result,flush=True);n.destroy_node();rclpy.shutdown()
