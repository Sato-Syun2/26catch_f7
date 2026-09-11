"""実際の接続監視関数をホスト実行し、一時欠落・切断・tick周回を確認する。"""
from pathlib import Path
import subprocess
import tempfile

source = (Path(__file__).resolve().parents[1] / 'Core/Src/microros_app.c').read_text()
start = source.index('static bool agent_session_lost(')
function = source[start:source.index('\nvoid MicroRosTask_Run', start)]
stub = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define RMW_RET_OK 0
static uint32_t tick;
static int ping_result, calls;
static uint32_t HAL_GetTick(void) { return tick; }
static int rmw_uros_ping_agent(int timeout, int attempts) {
 assert(timeout == 10 && attempts == 1); ++calls; return ping_result;
}
'''
test = r'''
int main(void) {
 uint32_t last = 0; uint8_t failures = 0;
 tick=999; assert(!agent_session_lost(&last,&failures)); assert(calls==0);
 ping_result=1;
 tick=1000; assert(!agent_session_lost(&last,&failures)); assert(failures==1);
 tick=2000; assert(!agent_session_lost(&last,&failures)); assert(failures==2);
 ping_result=0;
 tick=3000; assert(!agent_session_lost(&last,&failures)); assert(failures==0);
 ping_result=1;
 tick=4000; assert(!agent_session_lost(&last,&failures));
 tick=5000; assert(!agent_session_lost(&last,&failures));
 tick=6000; assert(agent_session_lost(&last,&failures));
 failures=0; last=UINT32_MAX-499; tick=499;
 assert(!agent_session_lost(&last,&failures)); assert(failures==0);
 tick=500; assert(!agent_session_lost(&last,&failures)); assert(failures==1);
}
'''
with tempfile.TemporaryDirectory(prefix='agent-reconnect-') as directory:
    path = Path(directory)
    (path / 'test.c').write_text(stub + function + test)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    str(path / 'test.c'), '-o', str(path / 'test')], check=True)
    subprocess.run([str(path / 'test')], check=True)
print('PASS: polling interval, transient loss, recovery, persistent loss, tick wrap')
