#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "control_period.h"
int main(void)
{
    assert(ControlPeriod_Rebase(100,101,2)==100);
    assert(ControlPeriod_Rebase(100,102,2)==100);
    assert(ControlPeriod_Rebase(100,103,2)==103);
    assert(ControlPeriod_Rebase(UINT32_MAX,1,2)==UINT32_MAX);
    assert(ControlPeriod_Rebase(UINT32_MAX,2,2)==2);
    puts("PASS: control deadline equality, overrun and tick wrap");
}
