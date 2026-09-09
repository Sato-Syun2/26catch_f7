"""ID4 test: explicit move/chirp actions; record feedback with monotonic time."""
import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from catch26_interface.msg import UrosF7Command, UrosF7MotorUnitCommand, UrosF7Feedback
from catch26_interface.srv import UrosF7Param


class Test(Node):
    def __init__(self, directory):
        super().__init__('id4_dob_measure')
        self.start = time.monotonic()
        self.fb = None
        self.received = 0
        self.stage = 'idle'
        self.target = 0.0
        self.file = open(directory / 'feedback.csv', 'x')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time', 'f7_time', 'stage', 'target', 'position', 'velocity', 'current', 'state', 'code'])
        self.pub = self.create_publisher(UrosF7Command, '/uros_f7_command', qos_profile_sensor_data)
        self.sub = self.create_subscription(UrosF7Feedback, '/uros_f7_feedback', self.receive, qos_profile_sensor_data)
        self.cli = self.create_client(UrosF7Param, '/uros_f7_param')

    def receive(self, msg):
        for fb in msg.feedback:
            if fb.info.type == 0 and fb.info.id == 4:
                self.fb = fb
                self.received = time.monotonic()
                self.writer.writerow([self.received-self.start, msg.local_time.sec+msg.local_time.nanosec*1e-9,
                                      self.stage, self.target, fb.position, fb.velocity, fb.current, fb.state, fb.unit_message_code])

    def service(self, command, data=0.0):
        if not self.cli.wait_for_service(timeout_sec=3):
            raise RuntimeError('service unavailable')
        req = UrosF7Param.Request()
        req.target = 'robomaster:4'
        req.command = command
        req.data = float(data)
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError(f'service {command}: {future.result() if future.done() else "timeout"}')
        print(command, data, future.result().message, flush=True)

    def send(self, target, velocity=False):
        self.target = float(target)
        unit = UrosF7MotorUnitCommand()
        unit.info.type = 0
        unit.info.id = 4
        if velocity:
            unit.velocity = self.target
        else:
            unit.position = self.target
        msg = UrosF7Command()
        msg.command = [unit]
        self.pub.publish(msg)

    def check(self, bounded=False):
        if self.fb is None or time.monotonic()-self.received > 0.05 or self.fb.unit_message_code != 1:
            raise RuntimeError('feedback missing/disconnected')
        if not all(math.isfinite(x) for x in [self.fb.position, self.fb.velocity, self.fb.current]):
            raise RuntimeError('invalid feedback')
        if bounded and not 80 < self.fb.position < 420:
            raise RuntimeError('host position boundary')
        if self.stage == 'chirp' and self.fb.state != 2:
            raise RuntimeError('velocity mode feedback lost')

    def loop(self, duration, fn, bounded=False):
        begin = time.monotonic()
        due = begin
        while time.monotonic()-begin < duration:
            rclpy.spin_once(self, timeout_sec=0.001)
            now = time.monotonic()
            self.check(bounded)
            if now >= due:
                fn(now-begin)
                due = now+0.01
        self.file.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['move', 'chirp'])
    parser.add_argument('--amplitude', type=float, default=1.0)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--f0', type=float, default=0.2)
    parser.add_argument('--f1', type=float, default=10.0)
    parser.add_argument('--alpha', type=float, default=10.0)
    parser.add_argument('--kp', type=float, default=.5)
    parser.add_argument('--current-limit', type=float, default=4.0)
    parser.add_argument('--position', type=float, default=250.0)
    args = parser.parse_args()
    if not (0 < args.f0 < args.f1 <= 10 and 0 < args.amplitude <= 800 and args.duration > 0):
        parser.error('invalid chirp parameters')
    if not 80 < args.position < 420:
        parser.error('move target must be inside 80..420mm')
    directory = Path(__file__).resolve().parents[1] / 'debug_logs' / ('dob_'+args.action+'_'+time.strftime('%Y%m%d_%H%M%S'))
    directory.mkdir()
    print(directory, flush=True)
    metadata = vars(args) | {'f0_hz': args.f0, 'f1_hz': args.f1, 'alpha': args.alpha, 'command_hz': 100, 'control_nominal_hz': 500}
    rclpy.init()
    node = Test(directory)
    try:
        end = time.monotonic()+5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.check()
        node.service('disable')
        if args.action == 'move':
            initial = node.fb.position
            if not -10 < initial < 450:
                raise RuntimeError('unexpected initial position')
            node.service('mode', 0)
            node.send(initial)
            node.service('enable')
            node.stage = 'move'
            duration = abs(args.position-initial)/20+1
            node.loop(duration, lambda t: node.send(initial+(args.position-initial)*min(t/(duration-1), 1)))
            node.stage = 'settle'
            node.loop(5, lambda t: node.send(args.position), True)
            if abs(node.fb.position-args.position) > 2:
                raise RuntimeError('move target not reached')
        else:
            if abs(node.fb.position-250) > 8:
                raise RuntimeError('must start near 250mm')
            node.service('mode', 4)
            node.send(0, True)
            node.service('enable')
            node.stage = 'pre'
            node.loop(2, lambda t: node.send(0, True), True)
            node.stage = 'chirp'
            k = math.log(args.f1/args.f0)/args.duration
            node.loop(args.duration, lambda t: node.send(args.amplitude*math.sin(2*math.pi*args.f0*math.expm1(k*t)/k), True), True)
            node.stage = 'post'
            node.loop(2, lambda t: node.send(0, True), True)
        metadata['result'] = 'complete'
        print('complete', node.fb.position, flush=True)
    except BaseException as exc:
        metadata['result'] = repr(exc)
        print('ABORT', repr(exc), flush=True)
    finally:
        try:
            node.service('disable')
        except Exception as exc:
            metadata['disable_error'] = repr(exc)
        node.file.close()
        (directory/'metadata.json').write_text(json.dumps(metadata, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
