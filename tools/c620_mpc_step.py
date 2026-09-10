"""C620 Mode5位置ステップ。到達・保持を確認してDisableし、未到達は失敗とする。"""
import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import time
import rclpy
from c620_measure import C620Test


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--target', type=float, required=True)
    p.add_argument('--duration', type=float, default=40)
    p.add_argument('--speed-limit', type=float, default=20)
    p.add_argument('--current-limit', type=float, default=7)
    p.add_argument('--ki', type=float, default=.1)
    p.add_argument('--kp', type=float, default=.1)
    args = p.parse_args()
    if not math.isfinite(args.target) or not 0 <= args.target <= 175.15:
        p.error('target outside confirmed mechanism range')
    if not math.isfinite(args.duration) or not 2 <= args.duration <= 60:
        p.error('duration outside 2..60s')
    if not math.isfinite(args.speed_limit) or not 1 <= args.speed_limit <= 200:
        p.error('speed limit outside 1..200mm/s')
    if not math.isfinite(args.current_limit) or not 0 < args.current_limit <= 20:
        p.error('current limit outside 0..20A')
    if not math.isfinite(args.ki) or not 0 <= args.ki <= 2:
        p.error('invalid velocity integral gain')
    if not math.isfinite(args.kp) or not 0 <= args.kp <= 1:
        p.error('invalid velocity proportional gain')
    directory = Path(__file__).resolve().parents[1]/'debug_logs'/(
        'c620_mode5_'+time.strftime('%Y%m%d_%H%M%S'))
    directory.mkdir()
    print(directory, flush=True)
    metadata = vars(args).copy()
    metadata.update(mode=5, model_time_constant_ms=1000/30, current_limit_a=args.current_limit,
                    speed_limit_mm_s=args.speed_limit, acceleration_mm_s2=100000,
                    target_braking_mm_s2=100000,
                    braking_envelope_mm_s2=200, tolerance_mm=.5, hold_seconds=1)
    metadata['local_elf_sha256'] = hashlib.sha256(
        (directory.parents[1]/'build/Debug/26catch_f7.elf').read_bytes()).hexdigest()
    rclpy.init()
    node = C620Test(directory, 'mpc')
    node.current_guard = args.current_limit + .8
    node.velocity_guard = max(30, 1.25*args.speed_limit)
    result = 1
    try:
        end = time.monotonic()+5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        node.check()
        state, contact, maximum = node.status()
        metadata['initial_status'] = [state, contact, maximum]
        if state != 5 or args.target > maximum:
            raise RuntimeError('confirmed range required')
        if node.count_publishers('/uros_f7_command') != 1:
            raise RuntimeError('another command publisher exists')
        node.service('disable')
        node.service('c620_current', args.current_limit)
        node.service('c620_speed', args.speed_limit)
        node.service('c620_ki', args.ki)
        node.service('c620_kp', args.kp)
        metadata['origin'] = origin = node.fb.position
        node.service('mode', 5)
        node.send(args.target, 5)
        node.service('enable')
        begin = due = time.monotonic()
        metadata['drive_start'] = begin-node.start
        window = deque()
        settled_since = None
        last_sample = None
        while time.monotonic()-begin < args.duration:
            rclpy.spin_once(node, timeout_sec=.002)
            node.check()
            now = time.monotonic()
            x = node.fb.position
            lower = max(-.5, min(origin, args.target)-1)
            upper = min(contact-3, max(origin, args.target)+1)
            if not lower <= x <= upper:
                raise RuntimeError('position overshoot/excursion guard')
            if now >= due and node.fb.state != 0:
                node.send(args.target, 5)
                due = now+.01
            if node.received != last_sample:
                last_sample = node.received
                window.append((now, x, node.fb.velocity))
                while window and now-window[0][0] > .5:
                    window.popleft()
            stable = (len(window) >= 30 and now-window[0][0] >= .45 and
                      all(abs(row[1]-args.target) <= .5 for row in window) and
                      sum(abs(row[2]) for row in window)/len(window) < 2)
            if stable:
                if settled_since is None:
                    settled_since = now
                    metadata['arrival_seconds'] = now-begin
                if now-settled_since >= 1:
                    break
            else:
                settled_since = None
            if now-begin > .2 and node.fb.state == 0:
                # 原点接触でFWが押し込みを停止した場合だけ到達確認を続ける。
                if args.target != 0 or abs(x) > .5:
                    raise RuntimeError('firmware stopped before target')
                metadata['origin_switch_stop'] = True
        else:
            raise RuntimeError('position did not settle within timeout')
        metadata['drive_end'] = time.monotonic()-node.start
        metadata['settled_position'] = node.fb.position
        metadata['result'] = 'complete'
        result = 0
    except BaseException as exc:
        metadata['result'] = repr(exc)
        print('ABORT', repr(exc), flush=True)
    finally:
        try:
            node.service('disable')
            metadata['fault'] = node.service('c620_fault')
            # Disable直後の戻り動作も記録して、駆動時の速度異常と区別する。
            node.velocity_guard = max(150, node.velocity_guard)
            node.loop(.5)
            metadata['disabled'] = node.fb.state == 0
            if not metadata['disabled']:
                raise RuntimeError('Disable not confirmed')
            metadata['final_position'] = node.fb.position
            metadata['final_velocity'] = node.fb.velocity
            metadata['final_status'] = node.status()
        except Exception as exc:
            metadata['disable_error'] = repr(exc)
            result = 1
        node.file.close()
        (directory/'metadata.json').write_text(json.dumps(metadata, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(metadata), flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
