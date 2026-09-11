"""実機に接続しない回帰テスト: IOC整合、実CANパック処理、MPC。"""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
ioc = dict(line.split('=', 1) for line in (root/'26catch_f7.ioc').read_text().splitlines() if '=' in line)
assert ioc['PA4.GPIO_Label'] == 'sensor2'
assert ioc['PA4.GPIO_PuPd'] == 'GPIO_PULLUP'
assert ioc['PA4.Signal'] == 'GPIO_Input'
assert ioc['PB12.Signal'] == 'CAN2_RX'
assert ioc['PB6.Signal'] == 'CAN2_TX'
assert ioc['CAN3.TXFP'] == 'ENABLE'
assert ioc['ProjectManager.KeepUserCode'] == 'true'
header = (root/'Core/Inc/main.h').read_text()
assert '#define sensor2_Pin GPIO_PIN_4' in header
assert '#define sensor2_GPIO_Port GPIOA' in header
gpio = (root/'Core/Src/gpio.c').read_text()
assert 'sensor1_Pin|sensor2_Pin|sensor3_Pin' in gpio
assert 'GPIO_MODE_INPUT' in gpio and 'GPIO_PULLUP' in gpio
base = 'canlib_robomaster/CAN_RoboMas'
includes = ['tools/c620_test_stubs', 'canlib_robomaster', base, base+'/Control',
            'canlib_robomaster/CAN_RoboMas_Def', 'canlib_robomaster/CAN_RoboMas_System']
common = ['gcc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror'] + ['-I'+p for p in includes]
with tempfile.TemporaryDirectory(prefix='c620-tests-') as directory:
    executable = str(pathlib.Path(directory)/'commission')
    subprocess.run(common + ['tools/test_c620_commission.c', base+'/CAN_RoboMas.c',
        base+'/RoboMas_Control.c', base+'/Control/Actuator_VelocityDob.c',
        base+'/Control/PositionMpc.c', base+'/Control/C620Commission.c', '-lm', '-o', executable], cwd=root, check=True)
    for case in [[], ['stale'], ['timeout'], ['cancel'], ['no_stall'], ['survey5'], ['overspeed'],
                 ['survey_speed'], ['survey_stale'], ['survey_timeout']]:
        subprocess.run([executable]+case, check=True)
    subprocess.run(common + ['-DC620_AUTO_CALIBRATE_ON_BOOT=1',
        'tools/test_c620_commission.c', base+'/CAN_RoboMas.c',
        base+'/RoboMas_Control.c', base+'/Control/Actuator_VelocityDob.c',
        base+'/Control/PositionMpc.c', base+'/Control/C620Commission.c', '-lm', '-o', executable], cwd=root, check=True)
    for case in ['boot', 'boot_cancel', 'stale', 'timeout']:
        subprocess.run([executable, case], check=True)
    subprocess.run(common + ['tools/test_id4_position_mpc.c', base+'/Control/PositionMpc.c',
                            '-lm', '-o', executable], cwd=root, check=True)
    subprocess.run([executable], check=True)
    subprocess.run(common + ['tools/test_c620_mpc.c', base+'/Control/PositionMpc.c',
                            '-lm', '-o', executable], cwd=root, check=True)
    subprocess.run([executable], check=True)
print('IOC/generated GPIO and firmware regression tests passed')
