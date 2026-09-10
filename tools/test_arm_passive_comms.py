"""Enable/目標送信をせず、FB連続性とDisableサービスを30秒検証する。"""
import csv,json,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from catch26_interface.msg import UrosF7Feedback
from catch26_interface.srv import UrosF7Param
out=Path('debug_logs')/time.strftime('arm_passive_%Y%m%d_%H%M%S');out.mkdir()
rclpy.init();n=rclpy.create_node('arm_passive_comms_test');c=n.create_client(UrosF7Param,'/uros_f7_param')
rows=[];times=[];states={};failure=[];service_results=[]
def receive(msg):
    now=time.monotonic();ft=msg.local_time.sec+msg.local_time.nanosec*1e-9;times.append((now,ft))
    for x in msg.feedback:
        key=(x.info.type,x.info.id);states[key]=(now,x.state,x.unit_message_code)
        rows.append((now,ft,*key,x.position,x.velocity,x.current,x.state,x.unit_message_code))
        if x.state!=0:failure.append('unexpected enabled motor '+str(key))
s=n.create_subscription(UrosF7Feedback,'/uros_f7_feedback',receive,qos_profile_sensor_data)
try:
    if not c.wait_for_service(timeout_sec=5):raise RuntimeError('service unavailable')
    deadline=time.monotonic()+5
    expected={(1,1),(1,2),(0,1),(0,4),(0,3)}
    while not expected.issubset(states) and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.01)
    if not expected.issubset(states):raise RuntimeError('missing motor FB')
    start=time.monotonic();due=start;index=0;pending=None;sent=0
    while time.monotonic()-start<30:
        rclpy.spin_once(n,timeout_sec=.002);now=time.monotonic()
        if failure:raise RuntimeError(failure[0])
        if any(now-states[k][0]>.2 or states[k][2]!=1 for k in expected):raise RuntimeError('stale/disconnected FB')
        if pending is not None:
            if pending.done():
                result=pending.result();service_results.append(dict(target=target,latency=now-sent,success=bool(result and result.success)))
                if not result or not result.success:raise RuntimeError('disable rejected')
                pending=None
            elif now-sent>2:raise RuntimeError('disable timeout')
        if pending is None and now>=due:
            target=['robstride:1','robstride:2','robomaster:1','robomaster:4','robomaster:3'][index%5]
            pending=c.call_async(UrosF7Param.Request(target=target,command='disable',data=0.));sent=now;due=now+.1;index+=1
    if pending is not None:
        rclpy.spin_until_future_complete(n,pending,timeout_sec=2)
        result=pending.result() if pending.done() else None
        service_results.append(dict(target=target,latency=time.monotonic()-sent,success=bool(result and result.success)))
        if not result or not result.success:raise RuntimeError('last disable failed')
except Exception as e:failure.append(repr(e))
finally:
    with (out/'feedback.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['host_time','f7_time','type','id','position','velocity','current','state','code']);w.writerows(rows)
    result=dict(failures=failure,messages=len(times),services=service_results,max_host_gap=max((b[0]-a[0] for a,b in zip(times,times[1:])),default=None),max_f7_gap=max((b[1]-a[1] for a,b in zip(times,times[1:])),default=None),f7_time_monotonic=all(b[1]>a[1] for a,b in zip(times,times[1:])))
    (out/'summary.json').write_text(json.dumps(result,indent=2));print(out,{k:v for k,v in result.items() if k!='services'},'services',len(service_results))
    n.destroy_node();rclpy.shutdown()
if failure:raise SystemExit(1)
