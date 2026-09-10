"""統合版の起動保持観測／単軸小移動。終了時は必ず全軸Disableを試みる。"""
import argparse
import csv
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from catch26_interface.msg import UrosF7Feedback, UrosF7Command, UrosF7MotorUnitCommand
from catch26_interface.srv import UrosF7Param

p = argparse.ArgumentParser()
p.add_argument('--motor', choices=['robstride:1', 'robstride:2', 'robomaster:4'])
p.add_argument('--delta', type=float, default=5)
p.add_argument('--duration', type=float, default=8)
a = p.parse_args()
if not math.isfinite(a.delta) or abs(a.delta)>20 or not 2<=a.duration<=15:
    p.error('invalid small-step limits')
if a.motor and a.motor.startswith('robstride') and abs(a.delta)>5:
    p.error('arm step limited to 5 degrees')
out=Path('debug_logs')/time.strftime('integrated_smoke_%Y%m%d_%H%M%S')
out.mkdir()
print(out, flush=True)
rclpy.init()
n=rclpy.create_node('integrated_smoke')
c=n.create_client(UrosF7Param, '/uros_f7_param')
pub=n.create_publisher(UrosF7Command, '/uros_f7_command', qos_profile_sensor_data)
keys=[(1,2),(1,1),(0,4),(0,1),(0,3)]
fb={}; rows=[]; results={}; warnings=[]
def receive(msg):
    now=time.monotonic()
    for v in msg.feedback:
        k=(v.info.type,v.info.id)
        fb[k]=(now,v)
        rows.append([now,msg.local_time.sec+msg.local_time.nanosec*1e-9,*k,v.position,v.velocity,v.current,v.state,v.unit_message_code])
sub=n.create_subscription(UrosF7Feedback,'/uros_f7_feedback',receive,qos_profile_sensor_data)
def service(target,command):
    if not c.wait_for_service(timeout_sec=2): raise RuntimeError('service unavailable')
    f=c.call_async(UrosF7Param.Request(target=target,command=command,data=0.))
    rclpy.spin_until_future_complete(n,f,timeout_sec=2)
    r=f.result() if f.done() else None
    if not r or not r.success: raise RuntimeError(f'{target} {command}: {r}')
def name(k): return ('robstride' if k[0] else 'robomaster')+':'+str(k[1])
def stop_all():
    for k in keys:
        try: service(name(k),'disable')
        except Exception as e: warnings.append(str(e))
def check():
    for k in keys:
        t,v=fb[k]
        if time.monotonic()-t>.2 or v.unit_message_code!=1:
            raise RuntimeError(f'{k} stale/disconnected')
        if not all(math.isfinite(x) for x in [v.position,v.velocity,v.current]):
            raise RuntimeError(f'{k} nonfinite')
        if abs(v.current)>(8 if k[0] else 20.8): raise RuntimeError(f'{k} current')
def snapshot(): return {name(k):[v.position,v.velocity,v.current,v.state,v.unit_message_code] for k,(t,v) in fb.items()}
try:
    deadline=time.monotonic()+10
    while not all(k in fb for k in keys) and time.monotonic()<deadline:
        rclpy.spin_once(n,timeout_sec=.01)
    if not all(k in fb for k in keys): raise RuntimeError('missing FB')
    results['initial']=snapshot(); print(results['initial'],flush=True)
    if a.motor:
        stop_all()
        if pub.get_subscription_count()!=1 or n.count_publishers('/uros_f7_command')!=1:
            raise RuntimeError('unexpected command endpoints')
        k=next(k for k in keys if name(k)==a.motor)
        origin=fb[k][1].position; target=origin+a.delta
        if not ((-140<=target<=140) if k[0] else (0<=target<=520)):
            raise RuntimeError('target outside range')
        unit=UrosF7MotorUnitCommand(); unit.info.type,unit.info.id=k; unit.position=target
        def send(): pub.publish(UrosF7Command(command=[unit]))
        send()
        # 指令をモード変更せず送信し、起動Mode5のまま試験する。
        for _ in range(10): rclpy.spin_once(n,timeout_sec=.002)
        service(a.motor,'enable')
        start=due=time.monotonic(); stable=None
        while time.monotonic()-start<a.duration:
            rclpy.spin_once(n,timeout_sec=.002); check(); now=time.monotonic()
            v=fb[k][1]
            if now>=due: send(); due=now+.01
            if now-start>.1 and v.state!=1: raise RuntimeError('not position enabled')
            if not min(origin,target)-3<=v.position<=max(origin,target)+3:
                raise RuntimeError('excursion')
            if any(fb[o][1].state!=0 for o in keys if o!=k): raise RuntimeError('other motor enabled')
            if abs(v.position-target)<.5 and abs(v.velocity)<3:
                if stable is None: stable=now
                if now-stable>=1: break
            else: stable=None
        if stable is None or time.monotonic()-stable<1: raise RuntimeError('not settled')
        results.update(target=target,origin=origin,settled=fb[k][1].position,seconds=time.monotonic()-start)
    else:
        start=time.monotonic(); id1_stopped=False
        while time.monotonic()-start<a.duration:
            rclpy.spin_once(n,timeout_sec=.002); check()
            v=fb[(0,1)][1]
            if time.monotonic()-start>3 and v.state==2 and not id1_stopped:
                service('robomaster:1','disable'); id1_stopped=True
                warnings.append('ID1 still calibrating after observation grace; disabled')
        results['holding']=snapshot()
    results['result']='complete'
except Exception as e:
    results['result']=repr(e)
finally:
    stop_all()
    end=time.monotonic()+.3
    while time.monotonic()<end: rclpy.spin_once(n,timeout_sec=.01)
    results['final']=snapshot(); results['warnings']=warnings
    with (out/'feedback.csv').open('w') as f:
        w=csv.writer(f); w.writerow(['host_time','f7_time','type','id','position','velocity','current','state','code']);w.writerows(rows)
    (out/'summary.json').write_text(json.dumps(results,indent=2))
    print(json.dumps(results),flush=True)
    n.destroy_node();rclpy.shutdown()
if results['result']!='complete' or warnings: raise SystemExit(1)
