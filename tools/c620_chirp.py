"""C620速度DOBの有限時間チャープ。中央で実行し、移動量と速度を監視する。"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import rclpy
from c620_measure import C620Test


def reference(t, duration, amplitude, f0, f1):
    taper = min(1.0, t / 1.0, max(0.0, duration - t) / 1.0)
    phase = 2 * math.pi * (f0 * t + (f1 - f0) * t * t / (2 * duration))
    return amplitude * taper * math.sin(phase)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--center', type=float, default=90)
    p.add_argument('--position-only', action='store_true')
    p.add_argument('--amplitude', type=float, default=20)
    p.add_argument('--duration', type=float, default=30)
    p.add_argument('--f0', type=float, default=.5)
    p.add_argument('--f1', type=float, default=4)
    p.add_argument('--kp', type=float, default=.1)
    p.add_argument('--ki', type=float, default=.1)
    p.add_argument('--bandwidth', type=float, default=5)
    model = p.add_mutually_exclusive_group()
    model.add_argument('--tau-ms', type=float, help='理想速度モデルの時定数[ms]。既定33.333ms')
    model.add_argument('--alpha', type=float, help='旧指定との互換用。通常は--tau-msを使用')
    args = p.parse_args()
    if args.tau_ms is not None:
        if not math.isfinite(args.tau_ms) or args.tau_ms <= 0:
            p.error('invalid model time constant')
        args.alpha = 1000.0 / args.tau_ms
    elif args.alpha is None:
        args.alpha = 30.0
    if not math.isfinite(args.alpha) or not 1 <= args.alpha <= 60:
        p.error('model time constant must be within 16.667..1000ms')
    args.tau_ms = 1000.0 / args.alpha
    if not all(math.isfinite(v) for v in vars(args).values()):
        p.error('nonfinite argument')
    if not (40 <= args.center <= 140 and 0 < args.amplitude <= 20 and
            5 <= args.duration <= 30 and .5 <= args.f0 <= args.f1 <= 4 and
            0 <= args.kp <= 1 and 0 <= args.ki <= 2 and
            0 <= args.bandwidth <= 30 and 1 <= args.alpha <= 60):
        p.error('argument outside commissioning limits')
    directory = Path(__file__).resolve().parents[1] / 'debug_logs' / (
        'c620_chirp_' + time.strftime('%Y%m%d_%H%M%S'))
    directory.mkdir()
    print(directory, flush=True)
    print(f'Ideal velocity model time constant: {args.tau_ms:.3f} ms', flush=True)
    metadata = vars(args).copy()
    metadata['local_elf_sha256'] = hashlib.sha256(
        (directory.parents[1] / 'build/Debug/26catch_f7.elf').read_bytes()).hexdigest()
    rclpy.init()
    node = C620Test(directory, 'chirp')
    result = 1
    try:
        until = time.monotonic() + 5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02)
        node.check()
        metadata['initial_status'] = node.status()
        if metadata['initial_status'][0] != 5:
            raise RuntimeError('confirmed range required')
        if node.count_publishers('/uros_f7_command') != 1:
            raise RuntimeError('another command publisher exists')
        node.service('disable')
        origin = node.fb.position
        metadata['origin'] = origin
        if not args.position_only and abs(origin - args.center) > 3:
            raise RuntimeError('reposition with --position-only before chirp')
        for command, value in [('c620_current', 7), ('c620_kp', args.kp),
                               ('c620_ki', args.ki), ('c620_dob', args.bandwidth),
                               ('c620_alpha', args.alpha)]:
            node.service(command, value)
        node.service('mode', 4)
        node.send(0, 4)
        node.service('enable')
        metadata['drive_start'] = time.monotonic() - node.start

        def command(t):
            x = node.fb.position
            if args.position_only:
                # 原点直後と伸長端からは内側への移動だけを許す。
                if not min(origin - 3, 20) <= x <= max(origin + 3, 155):
                    raise RuntimeError('positioning excursion guard')
                target = max(-5, min(5, .8 * (args.center - x)))
                if abs(args.center - x) < 1:
                    target = 0
            else:
                if not 20 <= x <= 155 or abs(x - origin) > 15:
                    raise RuntimeError('chirp position/excursion guard')
                target = reference(t, args.duration, args.amplitude, args.f0, args.f1)
            node.send(target, 4)

        node.loop(args.duration, command, stop_when_disabled=True)
        if node.fb.state == 0:
            raise RuntimeError('firmware stopped trial')
        metadata['drive_end'] = time.monotonic() - node.start
        # 終端で速度指令をゼロにし、残留運動もログへ残す。
        node.loop(1, lambda t: node.send(0, 4), stop_when_disabled=True)
        if node.fb.state == 0:
            raise RuntimeError('firmware stopped settling')
        metadata['result'] = 'complete'
        result = 0
    except BaseException as exc:
        metadata['result'] = repr(exc)
        print('ABORT', repr(exc), flush=True)
    finally:
        try:
            node.service('disable')
            node.loop(.5)
            metadata['disabled'] = node.fb.state == 0
            if not metadata['disabled']:
                raise RuntimeError('Disable not confirmed by feedback')
            metadata['final_position'] = node.fb.position
            metadata['final_velocity'] = node.fb.velocity
            metadata['final_status'] = node.status()
        except Exception as exc:
            metadata['disable_error'] = repr(exc)
            result = 1
        node.file.close()
        (directory / 'metadata.json').write_text(json.dumps(metadata, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
