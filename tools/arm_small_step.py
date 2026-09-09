"""両軸FBを記録。--move指定時のみ、両軸保持から各軸を中央へ5度動かす。"""
import argparse
import csv
import json
import hashlib
import math
import time
from pathlib import Path
from arm_angle_chirp import AngleChirp
from arm_velocity_sweep import VelocitySweep

import rclpy
from rclpy.qos import qos_profile_sensor_data
from catch26_interface.msg import UrosF7Command, UrosF7MotorUnitCommand, UrosF7Feedback
from catch26_interface.srv import UrosF7Param

p = argparse.ArgumentParser()
p.add_argument('--move', action='store_true')
p.add_argument('--velocity-motor', type=int, choices=(1,2))
p.add_argument('--position-motor', type=int, choices=(1,2), help='通常PPで指定軸を試験中心へ移動')
p.add_argument('--position-target', type=float, default=0.)
p.add_argument('--dob', action='store_true')
p.add_argument('--both-axes', action='store_true', help='両軸をF7 DOB速度制御で同時に試験')
p.add_argument('--amplitude', type=float, default=3.)
p.add_argument('--test-seconds', type=float, default=10., help='正弦速度試験の長さ、最大30秒')
p.add_argument('--frequency-start', type=float, default=.5)
p.add_argument('--frequency-end', type=float, default=3.)
p.add_argument('--current-monitor-limit', type=float, default=2.5, help='参考電流による追加停止閾値。F7電流指令上限とは別')
p.add_argument('--retreat-elbow', action='store_true', help='肘だけ中央へ5度退避。開始範囲を±144度にする')
p.add_argument('--chirp', action='store_true', help='0.5..3Hz、20秒の対数速度チャープ')
p.add_argument('--chirp-seconds', type=float, default=20., help='速度チャープ本体秒数、10〜60秒')
p.add_argument('--wide-velocity-chirp', action='store_true', help='一定速度振幅の比較試験。相対±10度制限なし、絶対±145度維持')
p.add_argument('--chirp-sign', type=int, choices=(-1,1), default=1, help='一定速度チャープの初動方向')
p.add_argument('--chirp-phase-deg', type=float, choices=(0.,90.), default=0., help='速度波形の初期位相。90度は角度中心の片寄りを減らす')
p.add_argument('--angle-chirp', action='store_true', help='角度±5度・本体60秒・前後5秒の速度DOB同定')
p.add_argument('--shaped-velocity-chirp', action='store_true', help='基準モデルの角振幅を抑え、速度入力を最大600deg/sへ増加')
p.add_argument('--model-tau', type=float, default=.1, help='angle-chirpのF7一次遅れ時定数[s]')
p.add_argument('--settle-seconds', type=float, default=1., help='速度指令0での終了時確認、最大5秒')
p.add_argument('--current-probe', action='store_true', help='指定軸に正負0.2/0.4/0.6Aの80msパルス')
p.add_argument('--negative-probe', action='store_true', help='current-probeを-0.8A・80msの1回だけにする')
p.add_argument('--current-step', action='store_true', help='±0.2Aを各250ms・3往復。定常値と立上がりを比較')
p.add_argument('--step-current', type=float, default=.2, help='current-step振幅[A]、0超～0.4')
p.add_argument('--step-pairs', type=int, default=3, help='current-step往復数、1～6')
p.add_argument('--tag', default='')
a = p.parse_args()
if not 0<a.amplitude<=(600 if a.shaped_velocity_chirp or a.wide_velocity_chirp else 60): p.error('invalid velocity amplitude')
if not (.1 if a.angle_chirp else .5)<=a.frequency_start<a.frequency_end<=10: p.error('invalid frequency range')
if not 0<a.current_monitor_limit<=16: p.error('current monitor limit must be in (0,16]')
if not a.angle_chirp and not a.shaped_velocity_chirp and not a.wide_velocity_chirp and a.amplitude/(math.pi*(a.frequency_start if a.chirp else .5))>8: p.error('nominal velocity integral exceeds 8deg excursion budget')
if not 0<a.test_seconds<=30: p.error('test-seconds must be in (0,30]')
if not 10<=a.chirp_seconds<=60: p.error('chirp-seconds must be in [10,60]')
if not 1<=a.settle_seconds<=5: p.error('settle-seconds must be in [1,5]')
if a.position_motor and (not a.move or a.velocity_motor or a.dob or a.both_axes or not -120<=a.position_target<=120):
    p.error('position-motor requires ordinary single-axis move to [-120,120]')
if a.both_axes and (not a.move or not a.dob or not a.velocity_motor): p.error('both-axes requires active DOB test')
if a.retreat_elbow and (not a.move or a.velocity_motor): p.error('retreat requires --move and no velocity test')
if a.chirp and (not a.dob or not a.velocity_motor or not a.move): p.error('chirp requires active DOB velocity test')
if a.wide_velocity_chirp and (not a.chirp or a.shaped_velocity_chirp or a.angle_chirp or a.both_axes or a.current_probe):
    p.error('wide-velocity-chirp requires single-axis constant-amplitude DOB chirp')
if a.current_probe and (not a.velocity_motor or not a.move or a.dob or a.chirp): p.error('current probe requires motor and --move, without DOB/chirp')
if a.negative_probe and not a.current_probe: p.error('negative-probe requires current-probe')
if a.current_step and (not a.current_probe or a.negative_probe): p.error('current-step requires current-probe without negative-probe')
if not 0<a.step_current<=.4 or not 1<=a.step_pairs<=6: p.error('invalid current-step amplitude/pairs')
angle_wave=None
velocity_wave=None
if a.shaped_velocity_chirp:
    if not a.move or not a.dob or not a.velocity_motor or not a.chirp or a.both_axes or a.angle_chirp or a.current_probe:
        p.error('shaped-velocity-chirp requires single-axis DOB velocity chirp')
    velocity_wave=VelocitySweep(f0=a.frequency_start,f1=a.frequency_end,seconds=a.chirp_seconds,tau=a.model_tau,velocity_cap=a.amplitude)
    print('waveform preflight',velocity_wave.verify(),flush=True)
if a.angle_chirp:
    if not a.move or not a.dob or not a.velocity_motor or a.both_axes or a.current_probe or a.chirp:
        p.error('angle-chirp requires single-axis active DOB only')
    try:
        angle_wave=AngleChirp(f0=a.frequency_start,f1=a.frequency_end,tau=a.model_tau)
    except ValueError as e: p.error(str(e))
    if max(abs(angle_wave.sample(i*.002)[3]) for i in range(int(angle_wave.duration/.002)+1))>179:
        p.error('inverse model target exceeds firmware 180deg/s cap')
out = Path('debug_logs') / time.strftime('arm_step_%Y%m%d_%H%M%S')
out.mkdir(parents=True, exist_ok=False)
rclpy.init()
n = rclpy.create_node('arm_small_step')
pub = n.create_publisher(UrosF7Command, '/uros_f7_command', qos_profile_sensor_data)
cli = n.create_client(UrosF7Param, '/uros_f7_param')
fb, received, targets, initial = {}, {}, {}, {}
stage = 'observe'
velocity_motor = None
wave_start = None
center_started = None
pulse_current = 0.
enabled_at = None
publisher_check_due = 0.
last_f7_time = 0.
desired_position = None
chirp_phase = None
start = time.monotonic()
f = (out / 'feedback.csv').open('x')
w = csv.writer(f)
w.writerow(['time','f7_time','stage','type','id','target','position','velocity','current','state','code','desired_position','chirp_phase'])
def receive(msg):
    global last_f7_time
    now = time.monotonic()
    last_f7_time=msg.local_time.sec+msg.local_time.nanosec*1e-9
    for v in msg.feedback:
        key = (v.info.type, v.info.id)
        fb[key] = v
        received[key] = now
        w.writerow([now-start,msg.local_time.sec+msg.local_time.nanosec*1e-9,stage,*key,
                    targets.get(v.info.id) if v.info.type == 1 else None,
                    v.position,v.velocity,v.current,v.state,v.unit_message_code,
                    desired_position if key==(1,velocity_motor) else None,
                    chirp_phase if key==(1,velocity_motor) else None])
sub = n.create_subscription(UrosF7Feedback, '/uros_f7_feedback', receive, qos_profile_sensor_data)
def service(motor, command, data=0., kind='robstride', expected=True):
    if not cli.wait_for_service(timeout_sec=3): raise RuntimeError('service unavailable')
    req = UrosF7Param.Request(target=f'{kind}:{motor}', command=command, data=float(data))
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(n,future,timeout_sec=3)
    if not future.done() or future.result() is None: raise RuntimeError('service timeout')
    result = future.result()
    print(kind,motor,command,result,flush=True)
    if bool(result.success) != expected: raise RuntimeError('unexpected service result')
def loop(duration, publish=False):
    global publisher_check_due, desired_position, chirp_phase
    t = time.monotonic()
    due = t
    while time.monotonic()-t < duration:
        rclpy.spin_once(n,timeout_sec=.002)
        now = time.monotonic()
        if publish:
            if now>=publisher_check_due:
                if n.count_publishers('/uros_f7_command')>1: raise RuntimeError('another command publisher exists')
                publisher_check_due=now+.25
            for mid in (1,2):
                v = fb.get((1,mid))
                if v is None or now-received[(1,mid)] > .05 or v.unit_message_code != 1:
                    raise RuntimeError('arm feedback stale')
                if not all(math.isfinite(x) for x in (v.position,v.velocity,v.current)):
                    raise RuntimeError('nonfinite feedback')
                outside=abs(v.position-initial[mid])>10
                if a.wide_velocity_chirp:
                    outside=False  # 依頼により相対変位制限のみ解除。絶対角は下で常に確認。
                if mid==a.position_motor:
                    outside=not min(initial[mid],a.position_target)-5<=v.position<=max(initial[mid],a.position_target)+5
                if not -145 < v.position < 145 or outside:
                    raise RuntimeError('position excursion')
                if abs(v.current)>a.current_monitor_limit: raise RuntimeError('reference current monitor threshold exceeded')
                expected_state=(3 if a.current_probe else 2) if mid==velocity_motor or a.both_axes else 1
                if enabled_at is not None and now-enabled_at>.05 and v.state!=expected_state:
                    raise RuntimeError(f'ID{mid} control disabled or wrong mode')
            for mid in (1,4):
                v=fb.get((0,mid))
                if v is not None and v.state!=0: raise RuntimeError('RoboMaster unexpectedly enabled')
            if now>=due:
                units=[]
                for mid,target in targets.items():
                    u=UrosF7MotorUnitCommand();u.info.type=1;u.info.id=mid
                    if mid == velocity_motor or a.both_axes:
                        elapsed=now-wave_start if wave_start is not None else 0.
                        k=math.log(a.frequency_end/a.frequency_start)/a.chirp_seconds
                        phase=2.*math.pi*a.frequency_start*math.expm1(k*elapsed)/k if a.chirp else 2.*math.pi*.5*elapsed
                        target=a.amplitude*math.sin(phase) if wave_start is not None else 0.
                        if a.chirp and velocity_wave is None:
                            target=a.chirp_sign*a.amplitude*math.sin(phase+math.radians(a.chirp_phase_deg)) if wave_start is not None else 0.
                        if a.chirp:
                            chirp_phase=phase
                        if velocity_wave is not None:
                            target,chirp_phase=velocity_wave.sample(elapsed)
                        if angle_wave is not None:
                            q,_,_,target,chirp_phase=angle_wave.sample(elapsed)
                            desired_position=initial[mid]+q
                        if a.current_probe:
                            target=pulse_current;u.current=float(target)
                        else: u.velocity=float(target)
                        targets[mid]=target
                    else:
                        if mid==a.position_motor and center_started is not None:
                            delta=a.position_target-initial[mid]
                            target=initial[mid]+math.copysign(min(abs(delta),15.*(now-center_started)),delta)
                            targets[mid]=target
                        u.position=float(target)
                    units.append(u)
                pub.publish(UrosF7Command(command=units));due=now+.01
    f.flush()
result={'move':a.move,'dob':a.dob,'retreat_elbow':a.retreat_elbow,'velocity_motor':a.velocity_motor,'velocity_amplitude_deg_s':a.amplitude,'frequency_hz':.5}
result.update(chirp=a.chirp,frequency_hz=a.frequency_start if a.chirp else .5,frequency_end_hz=a.frequency_end if a.chirp else .5,duration=a.chirp_seconds if a.chirp else a.test_seconds)
result['reference_current_monitor_limit']=a.current_monitor_limit
result['settle_seconds']=a.settle_seconds
result.update(wide_velocity_chirp=a.wide_velocity_chirp,chirp_sign=a.chirp_sign,chirp_phase_deg=a.chirp_phase_deg)
result['current_probe']=a.current_probe
result['negative_probe']=a.negative_probe
result.update(current_step=a.current_step,tag=a.tag)
result.update(step_current_A=a.step_current,step_pairs=a.step_pairs)
result['both_axes']=a.both_axes
result['angle_chirp']=a.angle_chirp
result.update(position_motor=a.position_motor,position_target=a.position_target)
if velocity_wave is not None:
    result.update(velocity_wave=vars(velocity_wave),duration=velocity_wave.duration,
                  waveform_preflight=velocity_wave.verify())
if angle_wave is not None:
    result.update(angle_wave=vars(angle_wave),duration=angle_wave.duration,
                  frequency_hz=angle_wave.f0,frequency_end_hz=angle_wave.f1,
                  velocity_amplitude_deg_s=None)
if a.current_probe:
    count=2*a.step_pairs if a.current_step else (1 if a.negative_probe else 6)
    pulse_s=.25 if a.current_step else .08
    coast_s=.4 if a.current_step else .7
    result.update(frequency_hz=None,frequency_end_hz=None,velocity_amplitude_deg_s=None,
                  duration=count*(pulse_s+coast_s),pulse_count=count,
                  pulse_duration_s=pulse_s,zero_duration_s=coast_s)
result['local_elf_sha256']=hashlib.sha256(Path('build/Debug/26catch_f7.elf').read_bytes()).hexdigest()
result['configuration_source']=Path('Core/Src/can_devices.c').read_text()
result['arm_test_configuration']=Path('Core/Inc/arm_test_config.h').read_text()
result['velocity_estimator_source']=Path('canlib_robstride/CAN_Robstride/Control/ArmVelocityEstimate.h').read_text()
result['robstride_control_source']=Path('canlib_robstride/CAN_Robstride/CAN_Robstride.c').read_text()
result['robstride_system_source']=Path('canlib_robstride/CAN_Robstride_System/CAN_Robstride_System.c').read_text()
result['current_feedback_source']='Type2 torque / nominal Kt (Arms equivalent)' if a.dob else 'iqf'
try:
    loop(3)
    discovery_deadline=time.monotonic()+7.
    while any((1,mid) not in fb for mid in (1,2)) and time.monotonic()<discovery_deadline:
        loop(.25)
    print('initial',[(k,v.position,v.velocity,v.state) for k,v in fb.items()],flush=True)
    if a.move:
        for mid in (1,2):
            v=fb.get((1,mid))
            initial_limit=144. if a.retreat_elbow and mid==1 else 140.
            if v is None or v.state != 0 or not -initial_limit<v.position<initial_limit:
                raise RuntimeError('initial state/range unsuitable')
            initial[mid]=targets[mid]=v.position
        result['initial_position']=initial.copy()
        if a.wide_velocity_chirp:
            # 有限可動域のため、モデル積分だけでも絶対角を越える試験は開始しない。
            if not 0<a.model_tau<=.5: raise RuntimeError('invalid model tau')
            dt=.002;beta=math.exp(-dt/a.model_tau);vm=qm=0.;qmin=qmax=0.
            k=math.log(a.frequency_end/a.frequency_start)/a.chirp_seconds
            for i in range(int((a.chirp_seconds+a.settle_seconds)/dt)+1):
                t=i*dt
                phase=2*math.pi*a.frequency_start*math.expm1(k*t)/k
                u=a.chirp_sign*a.amplitude*math.sin(phase+math.radians(a.chirp_phase_deg)) if t<a.chirp_seconds else 0.
                vm=beta*vm+(1-beta)*u;qm+=vm*dt
                qmin=min(qmin,qm);qmax=max(qmax,qm)
            result['model_position_preflight']={'min':initial[a.velocity_motor]+qmin,'max':initial[a.velocity_motor]+qmax}
            if not -140<initial[a.velocity_motor]+qmin<=initial[a.velocity_motor]+qmax<140:
                raise RuntimeError('ideal model would approach absolute angle boundary')
        for mid in (1,4): service(mid,'disable',kind='robomaster')
        velocity_motor=a.velocity_motor
        # 全台Disable中にモード設定を終え、service中のFB一時停止を動作と重ねない。
        for mid in (1,2): service(mid,'mode',(2 if a.current_probe else (4 if a.dob else 1)) if mid==velocity_motor or a.both_axes else 0)
        loop(.3)
        stage='prepare';loop(.3,True)
        result['measurement_start_f7']=last_f7_time
        for mid in (1,2): service(mid,'enable')
        enabled_at=time.monotonic()
        stage='hold';loop(1,True)
        if a.current_probe:
            sequence=(a.step_current,-a.step_current)*a.step_pairs if a.current_step else ((-.8,) if a.negative_probe else (.2,-.2,.4,-.4,.6,-.6))
            for number,current in enumerate(sequence):
                stage=f'current_{number:02d}_{current:+.3f}';pulse_current=current;loop(.25 if a.current_step else .08,True)
                stage=f'coast_{number:02d}_{current:+.3f}';pulse_current=0.;loop(.4 if a.current_step else .7,True)
        elif a.velocity_motor:
            stage='angle_chirp' if angle_wave is not None else ('velocity_chirp' if a.chirp else 'velocity_sine')
            wave_start=time.monotonic();loop(angle_wave.duration if angle_wave is not None else (velocity_wave.duration if velocity_wave is not None else (a.chirp_seconds if a.chirp else a.test_seconds)),True)
            wave_start=None;stage='velocity_zero';loop(a.settle_seconds,True)
        elif a.position_motor:
            mid=a.position_motor;targets[mid]=a.position_target
            center_started=time.monotonic()
            stage=f'center_{mid}'
            loop(abs(initial[mid]-a.position_target)/15.+8.,True)
            if abs(fb[(1,mid)].position-a.position_target)>.5:
                raise RuntimeError('center target not reached')
        else:
            for mid in ((1,) if a.retreat_elbow else (2,1)):
                targets[mid]=initial[mid]-math.copysign(5.,initial[mid])
                stage=f'step_{mid}';loop(3,True)
                if abs(fb[(1,mid)].position-targets[mid])>1.: raise RuntimeError('target not reached')
        result['result']='complete'
    else: result['result']='observed'
except Exception as e:
    result['result']='abort';result['error']=repr(e)
finally:
    if a.move:
        for mid in (1,2):
            try: service(mid,'disable')
            except Exception as e: result[f'disable_error_{mid}']=repr(e)
        stage='disabled';loop(1)
    result['final']={str(k):{'position':v.position,'velocity':v.velocity,'state':v.state} for k,v in fb.items()}
    result['measurement_end_f7']=last_f7_time
    f.close();(out/'metadata.json').write_text(json.dumps(result,indent=2))
    print(out,{k:v for k,v in result.items() if k not in ('configuration_source','velocity_estimator_source','arm_test_configuration')},flush=True)
    n.destroy_node();rclpy.shutdown()
if result.get('result')=='abort' or any(k.startswith('disable_error') for k in result):
    raise SystemExit(1)
