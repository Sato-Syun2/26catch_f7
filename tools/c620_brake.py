"""中間区間で短い速度パルス後にゼロ速度を指令し、能動制動を測る。"""
import argparse
import json
from pathlib import Path
import time
import rclpy
from c620_measure import C620Test


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--speed', type=float, choices=[-200, -100, 100, 200], required=True)
    args = parser.parse_args()
    direction = 1 if args.speed > 0 else -1
    directory = Path(__file__).resolve().parents[1]/'debug_logs'/('c620_brake_'+time.strftime('%Y%m%d_%H%M%S'))
    directory.mkdir()
    print(directory, flush=True)
    data = dict(speed=args.speed, current_limit_a=20, model_time_constant_ms=1000/30)
    rclpy.init()
    node = C620Test(directory, 'mpc')
    node.current_guard = 20.8
    node.velocity_guard = 250
    result = 1
    try:
        # DDS探索・初回FB待ちは駆動前のみ。運転中の鮮度監視は緩めない。
        ready_deadline = time.monotonic()+5
        while time.monotonic() < ready_deadline:
            rclpy.spin_once(node,timeout_sec=.02)
        node.check()
        if node.status()[0] != 5 or node.count_publishers('/uros_f7_command') != 1:
            raise RuntimeError('ready state and exclusive command publisher required')
        origin = node.fb.position
        if not ((40 <= origin <= 55) if direction > 0 else (120 <= origin <= 135)):
            raise RuntimeError('start position outside interior test window')
        node.service('disable')
        for command, value in [('c620_current',20),('c620_speed',200),('c620_kp',.1),('c620_ki',0),('mode',4)]:
            node.service(command,value)
        node.send(args.speed,4)
        node.service('enable')
        begin = due = time.monotonic()
        data.update(origin=origin, drive_start=begin-node.start)
        stopped = None
        stable = None
        furthest = origin
        while time.monotonic()-begin < 2:
            rclpy.spin_once(node,timeout_sec=.001)
            node.check()
            now = time.monotonic()
            x, v = node.fb.position, node.fb.velocity
            if not 25 <= x <= 150 or (now-begin > .1 and node.fb.state == 0):
                raise RuntimeError('interior position/enable guard')
            if stopped is None and (direction*(x-origin) >= 30 or now-begin >= .45):
                node.send(0,4)
                stopped = now
                furthest = x
                data.update(stop_command_time=now-node.start, stop_position=x, stop_velocity=v)
            if stopped is not None:
                if direction*(x-furthest)>0:
                    furthest=x
                if direction*(x-data['stop_position']) > 25:
                    raise RuntimeError('braking distance guard')
                if abs(v)<2:
                    if stable is None:
                        stable=now
                    if now-stable >= .2:
                        data.update(stop_time_s=stable-stopped,
                                    braking_distance_mm=direction*(furthest-data['stop_position']),
                                    stopped_position=x)
                        break
                else:
                    stable=None
                if now-stopped > 1:
                    raise RuntimeError('braking timeout')
            if now >= due:
                node.send(0 if stopped is not None else args.speed,4)
                due=now+.005
        else:
            raise RuntimeError('trial timeout')
        data['result']='complete'
        result=0
    except BaseException as exc:
        data['result']=repr(exc)
        print('ABORT',repr(exc),flush=True)
    finally:
        try:
            node.service('disable')
            node.loop(.5)
            data.update(disabled=node.fb.state==0, final_position=node.fb.position,
                        final_velocity=node.fb.velocity, fault=node.service('c620_fault'))
            if not data['disabled']:
                result=1
        except Exception as exc:
            data['disable_error']=repr(exc)
            result=1
        node.file.close()
        (directory/'metadata.json').write_text(json.dumps(data,indent=2))
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(data),flush=True)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
