#include <assert.h>
#include <stdio.h>
#include "command_auto_enable.h"
int main(void)
{
    for (int first=0; first<2; ++first) {
        /* 初回・停止後とも位置指令で復帰し、有効中はEnableを重複送信しない。 */
        assert(CommandAutoEnable_Required(first, true, false));
        assert(!CommandAutoEnable_Required(first, true, true));
        assert(!CommandAutoEnable_Required(first, false, true));
    }
    assert(CommandAutoEnable_Required(true, false, false));
    assert(!CommandAutoEnable_Required(false, false, false));
    puts("PASS: position command auto-enable after stop; no duplicate enable");
}
