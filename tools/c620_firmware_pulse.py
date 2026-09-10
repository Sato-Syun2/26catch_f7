"""F7の有限距離・有限時間速度パルスを起動して記録。再送で再始動しない。"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import rclpy
from c620_measure import C620Test


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--speed',type=float,choices=[-800,-400,-200,200,400,800],required=True)
    args=p.parse_args()
    root=Path(__file__).resolve().parents[1]
    directory=root/'debug_logs'/('c620_fw_pulse_'+time.strftime('%Y%m%d_%H%M%S'))
    directory.mkdir()
    print(directory,flush=True)
    data=dict(speed=args.speed,current_limit_a=20,
              local_elf_sha256=hashlib.sha256((root/'build/Debug/26catch_f7.elf').read_bytes()).hexdigest())
    rclpy.init()
    node=C620Test(directory,'mpc')
    node.current_guard=20.8
    node.velocity_guard=1.25*abs(args.speed)
    result=1
    try:
        end=time.monotonic()+5
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
        node.check()
        if node.status()[0]!=5 or node.count_publishers('/uros_f7_command')!=1:
            raise RuntimeError('ready/exclusive publisher required')
        node.service('disable')
        for cmd,value in [('c620_current',20),('c620_kp',.1),('c620_ki',0)]:
            node.service(cmd,value)
        data['origin']=node.fb.position
        data['start_time']=time.monotonic()-node.start
        node.service('c620_pulse',args.speed)
        # ゼロ指令を通信監視用に送る。実速度目標はF7試験状態機械が毎周期上書き。
        node.loop(2,lambda t:node.send(0,4),stop_when_disabled=True)
        data['pulse_status']=node.service('c620_pulse_fb')
        data['pulse_stop']=node.service('c620_pulse_fb',1)
        data['pulse_braking']=node.service('c620_pulse_fb',2)
        if 'done=1' not in data['pulse_status']:
            raise RuntimeError('firmware pulse did not complete')
        data['result']='complete'
        result=0
    except BaseException as exc:
        data['result']=repr(exc)
        print('ABORT',repr(exc),flush=True)
    finally:
        try:
            node.service('disable')
            node.loop(.5)
            data.update(disabled=node.fb.state==0,final_position=node.fb.position,
                        final_velocity=node.fb.velocity,fault=node.service('c620_fault'))
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


if __name__=='__main__':
    raise SystemExit(main())
