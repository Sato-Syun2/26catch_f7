"""実機用。起動校正観測／4軸Mode5単独・同時小移動。終了時Disable確認。"""
import argparse
import csv
import hashlib
import json
import math
import time
from collections import deque
from pathlib import Path
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from catch26_interface.msg import UrosF7Feedback, UrosF7Command, UrosF7MotorUnitCommand
from catch26_interface.srv import UrosF7Param

p=argparse.ArgumentParser()
p.add_argument('action', choices=['boot','motion'])
p.add_argument('--seconds', type=float, default=60)
p.add_argument('--resume', action='store_true', help='初回指令試験後の再試験は明示Enableを使う')
p.add_argument('--elf', default='build/Debug/26catch_f7.elf', help='書き込み済みELFの記録用パス')
p.add_argument('--continuous', action='store_true', help='単軸試験後の追加負荷確認: 全4軸に0.4Hzの連続位置目標')
a=p.parse_args()
if not math.isfinite(a.seconds) or not 10<=a.seconds<=120: p.error('seconds must be 10..120')
out=Path('debug_logs')/time.strftime('integrated_mpc_live_%Y%m%d_%H%M%S')
out.mkdir(); print(out,flush=True)
result={'action':a.action,'elf':a.elf,'sha256':hashlib.sha256(Path(a.elf).read_bytes()).hexdigest(),'stages':[]}
keys=[(1,2),(1,1),(0,4),(0,1),(0,3)]; axes=[(1,2),(1,1),(0,4),(0,3)]
def name(k): return ('robstride' if k[0] else 'robomaster')+':'+str(k[1])
# Ctrl-CでもfinallyのDisable通信が終わるまでROS contextを維持する。
rclpy.init(signal_handler_options=SignalHandlerOptions.NO); n=rclpy.create_node('integrated_mpc_live')
c=n.create_client(UrosF7Param,'/uros_f7_param')
pub=n.create_publisher(UrosF7Command,'/uros_f7_command',qos_profile_sensor_data)
fb={}; history={k:deque() for k in keys}; targets={}; phase='initial'; gaps=[]; last=None; last_f7=None
disabled_at={}
log=(out/'feedback.csv').open('w'); writer=csv.writer(log)
writer.writerow(['host_time','f7_time','phase','type','id','target','position','velocity','current','state','code'])
def receive(msg):
    global last,last_f7
    now=time.monotonic(); ft=msg.local_time.sec+msg.local_time.nanosec*1e-9
    if last is not None: gaps.append(now-last)
    if last_f7 is not None and ft<last_f7: result['clock_regressed']=True
    last=now;last_f7=ft
    for v in msg.feedback:
        k=(v.info.type,v.info.id)
        if k not in history: continue
        fb[k]=(now,v); history[k].append((now,v.position,v.velocity))
        while history[k] and now-history[k][0][0]>1: history[k].popleft()
        writer.writerow([now,ft,phase,*k,targets.get(k,''),v.position,v.velocity,v.current,v.state,v.unit_message_code])
sub=n.create_subscription(UrosF7Feedback,'/uros_f7_feedback',receive,qos_profile_sensor_data)
due=0
def pump():
    global due
    now=time.monotonic()
    if targets and now>=due:
        units=[]
        for k,x in targets.items():
            u=UrosF7MotorUnitCommand();u.info.type,u.info.id=k;u.position=float(x);units.append(u)
        pub.publish(UrosF7Command(command=units));due=now+.01
    rclpy.spin_once(n,timeout_sec=.002)
def service(k,cmd,data=0):
    if not c.wait_for_service(timeout_sec=2): raise RuntimeError('service unavailable')
    sent=time.monotonic();f=c.call_async(UrosF7Param.Request(target=name(k),command=cmd,data=float(data)))
    while not f.done() and time.monotonic()-sent<2: pump()
    r=f.result() if f.done() else None
    if not r or not r.success: raise RuntimeError(f'{name(k)} {cmd}: {r}')
    if cmd=='disable': disabled_at[k]=time.monotonic()
    result.setdefault('services',[]).append([name(k),cmd,time.monotonic()-sent,r.message])
    return r.message
def snapshot(): return {name(k):[v.position,v.velocity,v.current,v.state,v.unit_message_code] for k,(t,v) in fb.items()}
def check():
    if result.get('clock_regressed'): raise RuntimeError('F7 reboot/time regression')
    for k in keys:
        if k not in fb: raise RuntimeError('missing '+name(k))
        t,v=fb[k]
        transition = v.state==0 and time.monotonic()-disabled_at.get(k,-1e9)<.15
        if time.monotonic()-t>.2 or (v.unit_message_code!=1 and not transition): raise RuntimeError('stale/disconnected '+name(k))
        if not all(math.isfinite(x) for x in [v.position,v.velocity,v.current]): raise RuntimeError('nonfinite '+name(k))
        cap=8 if k[0] else (5.8 if k==(0,3) and v.state==2 else 20.8 if k==(0,3) else 10.8 if k==(0,4) else 2.8)
        if abs(v.current)>cap: raise RuntimeError('current '+name(k))
        if a.action=='motion' and k in axes:
            lo,hi=bounds[k]
            if not lo<=v.position<=hi: raise RuntimeError('excursion '+name(k))
    if a.action=='motion' and fb[(0,1)][1].state!=0: raise RuntimeError('untuned ID1 enabled')
def stop():
    targets.clear(); errors=[]
    for k in keys:
        # 初回指令未送信のRobstrideを不要なDisableで消費しない。
        if k[0] and k in fb and fb[k][1].state==0 and a.action=='boot': continue
        try: service(k,'disable')
        except Exception as e: errors.append(str(e))
    end=time.monotonic()+.4
    while time.monotonic()<end: pump()
    for k in keys:
        if k not in fb or time.monotonic()-fb[k][0]>.2 or fb[k][1].state!=0:
            errors.append('stop unconfirmed '+name(k))
    return errors
def settled(k,target):
    h=history[k]
    return len(h)>=40 and h[-1][0]-h[0][0]>=.7 and all(abs(x-target)<.7 for t,x,v in h) and max(x for t,x,v in h)-min(x for t,x,v in h)<.5
try:
    deadline=time.monotonic()+10
    while not all(k in fb for k in keys) and time.monotonic()<deadline: pump()
    if not all(k in fb for k in keys): raise RuntimeError('missing initial FB')
    result['initial']=snapshot();print(result['initial'],flush=True)
    if a.action=='boot':
        phase='boot';start=report=time.monotonic()
        while time.monotonic()-start<65:
            pump();check()
            if any(fb[k][1].state!=0 for k in axes if k[0]): raise RuntimeError('Robstride enabled without first command')
            if time.monotonic()-start>10 and fb[(0,1)][1].state==2:
                service((0,1),'disable');result['id1_calibration_incomplete']=True
            if time.monotonic()-report>10:
                print('boot',round(time.monotonic()-start),snapshot(),flush=True);log.flush();report=time.monotonic()
            if fb[(0,3)][1].state==1:
                result['c620_status']=service((0,3),'c620_status')
                result['c620_origin']=service((0,3),'c620_origin')
                if not result['c620_status'].startswith('s=5 '): raise RuntimeError('C620 range not ready')
                result['holding']=snapshot();break
        else: raise RuntimeError('C620 calibration did not complete')
    else:
        if n.count_publishers('/uros_f7_command')!=1: raise RuntimeError('another command publisher')
        if not service((0,3),'c620_status').startswith('s=5 '): raise RuntimeError('C620 not calibrated')
        base={k:fb[k][1].position for k in axes}
        delta={k:(-2 if base[k]>0 else 2) if k[0] else 10 for k in axes}
        bounds={k:(max(-140 if k[0] else -1,min(base[k],base[k]+delta[k])-3),min(140 if k[0] else 175 if k==(0,3) else 520,max(base[k],base[k]+delta[k])+3)) for k in axes}
        for k in ([] if a.continuous else axes):
            phase='single_'+name(k);targets[k]=base[k]+delta[k]
            start=time.monotonic()
            # Robstrideの初回指令Enableをサービス無しで検証する。
            if not k[0] or a.resume: service(k,'enable')
            while time.monotonic()-start<8:
                pump();check()
                if time.monotonic()-start>.5 and fb[k][1].state!=1: raise RuntimeError('first command/enable failed '+name(k))
                if time.monotonic()-start>1 and settled(k,targets[k]): break
            else: raise RuntimeError('single not settled '+name(k))
            result['stages'].append({'phase':phase,'seconds':time.monotonic()-start,'target':targets[k],'position':fb[k][1].position})
            print(result['stages'][-1],flush=True)
            del targets[k];service(k,'disable')
        base={k:fb[k][1].position for k in axes}
        for k in axes:
            delta[k]=(-2 if base[k]>0 else 2) if k[0] else 10
            bounds[k]=(min(base[k],base[k]+delta[k])-3,max(base[k],base[k]+delta[k])+3)
        phase='continuous' if a.continuous else 'simultaneous';targets.update(base);pump()
        if a.continuous and not a.resume:
            # 起動直後の初回topic自動Enableと明示Enableを重ねない。
            deadline=time.monotonic()+2
            while time.monotonic()<deadline and any(fb[k][1].state!=1 for k in axes if k[0]):
                pump();check()
            if any(fb[k][1].state!=1 for k in axes if k[0]):
                raise RuntimeError('first-command enable did not complete')
        for k in axes:
            if not (a.continuous and not a.resume and k[0]): service(k,'enable')
        start=report=time.monotonic();last_leg=-1
        while time.monotonic()-start<a.seconds:
            elapsed=time.monotonic()-start;leg=int(elapsed//5)
            if a.continuous:
                targets.update({k:base[k]+delta[k]*.5*(1-math.cos(2*math.pi*.4*elapsed)) for k in axes})
            elif leg!=last_leg:
                if last_leg>=0 and not all(settled(k,targets[k]) for k in axes): raise RuntimeError('simultaneous not settled')
                targets.update({k:base[k]+delta[k]*(1 if leg%2==0 else 0) for k in axes});last_leg=leg
            pump();check()
            if elapsed>.3 and any(fb[k][1].state!=1 for k in axes): raise RuntimeError('simultaneous axis disabled')
            if time.monotonic()-report>10:
                print('simultaneous',round(elapsed),snapshot(),flush=True);log.flush();report=time.monotonic()
        if a.continuous:
            targets.update(base);phase='settle';end=time.monotonic()+4
            while time.monotonic()<end:
                pump();check()
                if all(settled(k,targets[k]) for k in axes): break
        if not all(settled(k,targets[k]) for k in axes): raise RuntimeError('final simultaneous not settled')
        result['simultaneous_seconds']=time.monotonic()-start
        result['legs']=last_leg+1
    result['result']='complete'
except BaseException as e:
    result['result']=repr(e)
finally:
    phase='stopping'
    result['stop_errors']=stop();result['final']=snapshot()
    result['max_host_gap']=max(gaps,default=0);result['feedback_messages']=len(gaps)+1
    log.close();(out/'summary.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True);n.destroy_node();rclpy.shutdown()
if result['result']!='complete' or result['stop_errors']: raise SystemExit(1)
