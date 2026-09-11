"""実際のwatchdog関数をホストで実行し、受信更新競合とtick周回を検証する。"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'Core/Src/microros_app.c').read_text()
start = source.index('static bool command_watchdog_expired(', source.index('static void end_control_transaction(void)\n{'))
end = source.index('\nstatic void reset_robstride_command_watchdog', start)
function = source[start:end]
stub = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define MICROROS_COMMAND_TIMEOUT_MS 100U
static uint32_t clock_tick, mask;
static bool microros_command_watchdog_initialized = true;
static uint32_t __get_PRIMASK(void) {return mask;}
static void __disable_irq(void) {mask=1;}
static void __set_PRIMASK(uint32_t m) {mask=m;}
static uint32_t HAL_GetTick(void) {assert(mask); return clock_tick;}
'''
test = r'''
int main(void) {
 volatile uint32_t received = 1001;
 /* 呼出側が1000msを取得した後に1001msの指令が入った場合。 */
 clock_tick=1001; assert(!command_watchdog_expired(&received));
 clock_tick=1100; assert(!command_watchdog_expired(&received));
 clock_tick=1101; assert(command_watchdog_expired(&received));
 received=UINT32_MAX-49; clock_tick=49;
 assert(!command_watchdog_expired(&received));
 clock_tick=50; assert(command_watchdog_expired(&received));
 microros_command_watchdog_initialized=false;
 assert(!command_watchdog_expired(&received));
 assert(mask==0);
}
'''
with tempfile.TemporaryDirectory(prefix='watchdog-test-') as directory:
 path=Path(directory); (path/'test.c').write_text(stub+function+test)
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',str(path/'test.c'),'-o',str(path/'test')],check=True)
 subprocess.run([str(path/'test')],check=True)
print('PASS: fresh receive race, timeout boundary, tick wrap, uninitialized watchdog')
