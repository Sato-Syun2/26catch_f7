"""Production mode predicates: position holds through missing ROS commands."""
from pathlib import Path
import subprocess
import tempfile
source = (Path(__file__).resolve().parents[1] / 'Core/Src/microros_app.c').read_text()
start = source.index('static bool robstride_timeout_mode(const ')
end = source.index('\ntypedef enum', start)
code = '''
#include <stdbool.h>
#include <assert.h>
typedef enum {ROBSTRIDE_CTRL_POS, ROBSTRIDE_CTRL_POS_MPC, ROBSTRIDE_CTRL_VEL,
 ROBSTRIDE_CTRL_VEL_DOB, ROBSTRIDE_CTRL_CURRENT} ROBSTRIDE_CTRL_TYPE;
typedef enum {ROBOMAS_CTRL_POS, ROBOMAS_CTRL_POS_AW, ROBOMAS_CTRL_POS_MPC,
 ROBOMAS_CTRL_VEL, ROBOMAS_CTRL_VEL_DOB, ROBOMAS_CTRL_CURRENT} ROBOMAS_CTRL_TYPE;
''' + source[start:end] + '''
int main(void) {
 assert(!robstride_timeout_mode(ROBSTRIDE_CTRL_POS));
 assert(!robstride_timeout_mode(ROBSTRIDE_CTRL_POS_MPC));
 assert(!robomas_timeout_mode(ROBOMAS_CTRL_POS));
 assert(!robomas_timeout_mode(ROBOMAS_CTRL_POS_AW));
 assert(!robomas_timeout_mode(ROBOMAS_CTRL_POS_MPC));
 assert(robstride_timeout_mode(ROBSTRIDE_CTRL_VEL));
 assert(robstride_timeout_mode(ROBSTRIDE_CTRL_VEL_DOB));
 assert(robstride_timeout_mode(ROBSTRIDE_CTRL_CURRENT));
 assert(robomas_timeout_mode(ROBOMAS_CTRL_VEL));
 assert(robomas_timeout_mode(ROBOMAS_CTRL_VEL_DOB));
 assert(robomas_timeout_mode(ROBOMAS_CTRL_CURRENT));
}
'''
with tempfile.TemporaryDirectory() as directory:
 p = Path(directory)
 (p/'test.c').write_text(code)
 subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', str(p/'test.c'), '-o', str(p/'test')], check=True)
 subprocess.run([str(p/'test')], check=True)
print('PASS: all position modes exempt; velocity and current watchdogs retained')
