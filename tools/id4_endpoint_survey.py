"""ID4端探索用ファーム専用。1Aの強制制限はF7側で実施する。"""
import json
import time
from pathlib import Path
import rclpy
from dob_measure import Test

directory = Path(__file__).resolve().parents[1] / 'debug_logs' / (
    'id4_endpoint_' + time.strftime('%Y%m%d_%H%M%S'))
directory.mkdir()
print(directory, flush=True)
rclpy.init()
n = Test(directory)
result = dict(target_mm_s=40, firmware_current_limit_a=1, result='not started')
try:
    end = time.monotonic() + 5
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.02)
    n.check()
    if not -1 < n.fb.position < 1 or abs(n.fb.velocity) > 2 or n.fb.state != 0:
        raise RuntimeError('calibrated zero / disabled state not confirmed')
    n.service('disable')
    n.service('mode', 1)
    n.send(40, True)
    n.loop(.1, lambda t: n.send(40, True))
    n.service('enable')
    n.stage = 'survey'
    start = time.monotonic()
    while time.monotonic() - start < 21:
        rclpy.spin_once(n, timeout_sec=.002)
        n.check()
        if not -2 < n.fb.position < 550 or abs(n.fb.velocity) > 60:
            raise RuntimeError('host boundary / velocity limit')
        if n.fb.state == 0 and time.monotonic()-start > .2:
            result.update(result='F7 stopped; inspect contact log',
                          position=n.fb.position, velocity=n.fb.velocity)
            break
        n.send(40, True)
        # ROS側は100Hz。F7側の監視は500Hz。
        until = time.monotonic() + .01
        while time.monotonic() < until:
            rclpy.spin_once(n, timeout_sec=.001)
    else:
        raise RuntimeError('host timeout')
except BaseException as e:
    result['result'] = repr(e)
finally:
    try:
        n.service('disable')
    except Exception as e:
        result['disable_error'] = repr(e)
    n.file.close()
    (directory/'metadata.json').write_text(json.dumps(result, indent=2))
    print(result, flush=True)
    n.destroy_node()
    rclpy.shutdown()
