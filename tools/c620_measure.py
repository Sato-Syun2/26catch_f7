"""ID3/C620専用の有限時間試験。例外・終了時はDisable。起動だけでは動かさない。"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from catch26_interface.msg import UrosF7Command, UrosF7MotorUnitCommand, UrosF7Feedback
from catch26_interface.srv import UrosF7Param


class C620Test(Node):
    def __init__(self, directory, action, survey_current=1.0):
        super().__init__('c620_measure')
        self.action = action
        self.survey_current = survey_current
        self.start = time.monotonic()
        self.fb = None
        self.received = 0.0
        self.target = 0.0
        self.file = (directory/'feedback.csv').open('x')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time','f7_time','action','target','position','velocity','current','state','code'])
        self.create_subscription(UrosF7Feedback, '/uros_f7_feedback', self.receive, qos_profile_sensor_data)
        self.pub = self.create_publisher(UrosF7Command, '/uros_f7_command', qos_profile_sensor_data)
        self.cli = self.create_client(UrosF7Param, '/uros_f7_param')

    def receive(self, msg):
        for fb in msg.feedback:
            if fb.info.type == 0 and fb.info.id == 3:
                self.fb = fb
                self.received = time.monotonic()
                self.writer.writerow([self.received-self.start,
                    msg.local_time.sec+msg.local_time.nanosec*1e-9, self.action, self.target,
                    fb.position, fb.velocity, fb.current, fb.state, fb.unit_message_code])

    def service(self, command, data=0.0):
        if not self.cli.wait_for_service(timeout_sec=3):
            raise RuntimeError('F7 service unavailable')
        request = UrosF7Param.Request(target='robomaster:3', command=command, data=float(data))
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)
        result = future.result() if future.done() else None
        if result is None or not result.success:
            raise RuntimeError(f'{command}: {result}')
        print(command, result.message, flush=True)
        return result.message

    def status(self):
        match = re.fullmatch(r's=(\d+) end=([\d.-]+) max=([\d.-]+)', self.service('c620_status'))
        if not match:
            raise RuntimeError('firmware status format mismatch')
        return int(match[1]), float(match[2]), float(match[3])

    def check(self):
        if self.fb is None or time.monotonic()-self.received > .1 or self.fb.unit_message_code != 1:
            raise RuntimeError('C620 feedback missing/stale/disconnected')
        if not all(math.isfinite(x) for x in (self.fb.position,self.fb.velocity,self.fb.current)):
            raise RuntimeError('nonfinite feedback')
        current_guard = 10.5 if self.action == 'calibrate' else 2.5
        if self.action == 'survey':
            current_guard = max(2.5, self.survey_current+.5)
        if abs(self.fb.current) > current_guard or abs(self.fb.velocity) > 150:
            raise RuntimeError('host current/velocity guard')

    def send(self, target, mode):
        self.target = float(target)
        unit = UrosF7MotorUnitCommand()
        unit.info.type, unit.info.id = 0, 3
        if mode == 2:
            unit.current = float(target)
        elif mode in (1, 4):
            unit.velocity = float(target)
        else:
            unit.position = float(target)
        self.pub.publish(UrosF7Command(command=[unit]))

    def loop(self, duration, command=None, stop_when_disabled=False):
        begin = due = time.monotonic()
        while time.monotonic()-begin < duration:
            rclpy.spin_once(self, timeout_sec=.001)
            self.check()
            now = time.monotonic()
            if stop_when_disabled and now-begin > .2 and self.fb.state == 0:
                break
            if command and now >= due:
                command(now-begin)
                due = now+.01
        self.file.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['inspect','calibrate','survey','configure','move','dob','mpc'])
    parser.add_argument('--duration', type=float, default=5)
    parser.add_argument('--target', type=float, default=0)
    parser.add_argument('--command', default='c620_status')
    parser.add_argument('--survey-current', type=float, default=1.0)
    args = parser.parse_args()
    if not math.isfinite(args.survey_current) or not 0 < args.survey_current <= 5:
        parser.error('survey current must be within 0..5A')
    if not math.isfinite(args.duration) or not 0 < args.duration <= 60 or not math.isfinite(args.target):
        parser.error('invalid duration/target')
    directory = Path(__file__).resolve().parents[1]/'debug_logs'/('c620_'+args.action+'_'+time.strftime('%Y%m%d_%H%M%S'))
    directory.mkdir(parents=True)
    print(directory, flush=True)
    metadata = vars(args).copy()
    firmware = directory.parents[1]/'build/Debug/26catch_f7.elf'
    metadata['local_elf_sha256'] = hashlib.sha256(firmware.read_bytes()).hexdigest()
    rclpy.init()
    node = C620Test(directory, args.action, args.survey_current)
    result = 1
    try:
        end = time.monotonic()+5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        node.check()
        metadata['initial_status'] = node.status()
        metadata['switch'] = node.service('c620_switch')
        if args.action == 'inspect':
            node.loop(args.duration)
        else:
            if node.count_publishers('/uros_f7_command') != 1:
                raise RuntimeError('another command publisher exists')
            node.service('disable')
            if args.action == 'configure':
                if args.command not in ('c620_range','c620_current','c620_kp','c620_ki','c620_dob','c620_alpha'):
                    raise RuntimeError('not a tuning configuration command')
                node.service(args.command, args.target)
            elif args.action == 'calibrate':
                node.service('c620_calibrate')
                node.loop(11, stop_when_disabled=True)
                if node.status()[0] != 2:
                    raise RuntimeError('calibration did not complete')
            elif args.action == 'survey':
                node.service('c620_survey', args.survey_current)
                node.loop(61, lambda t: node.send(40.0,1), stop_when_disabled=True)
                if node.status()[0] != 4:
                    raise RuntimeError('survey did not finish at contact candidate')
            else:
                state, contact, maximum = metadata['initial_status']
                if state != 5:
                    raise RuntimeError('confirmed range required')
                mode = {'move':3,'dob':4,'mpc':5}[args.action]
                if mode == 4 and abs(args.target) > 10:
                    raise RuntimeError('initial DOB velocity maximum 10')
                if mode != 4 and not 5 <= args.target <= maximum:
                    raise RuntimeError('target outside range')
                node.service('mode', mode)
                node.send(args.target, mode)
                node.service('enable')
                node.loop(args.duration, lambda t: node.send(args.target,mode), stop_when_disabled=True)
                if node.fb.state == 0:
                    raise RuntimeError('firmware stopped trial')
            metadata['final_status'] = node.status()
        metadata['result'] = 'complete'
        result = 0
    except BaseException as exc:
        metadata['result'] = repr(exc)
        print('ABORT', repr(exc), flush=True)
    finally:
        if args.action != 'inspect':
            try:
                node.service('disable')
                node.loop(.3)
                metadata['disabled'] = node.fb.state == 0
            except Exception as exc:
                metadata['disable_error'] = repr(exc)
        node.file.close()
        (directory/'metadata.json').write_text(json.dumps(metadata, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
