#include "PositionMpc.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
uint32_t Id4PositionMpc_ClockMs(void) { return 0; }
int main(void) {
    Id4PositionMpc m;
    Id4PositionMpc_Reset(&m);
    float x=20, v=0;
    for (unsigned i=0;i<15000;i++) {
        float u=PositionMpc_UpdateBounded(&m,x,v,60,5,.002f,5,95,10);
        assert(isfinite(u) && fabsf(u)<=10);
        float a=expf(-5*.002f), c=(1-a)/5;
        x+=c*v+(.002f-c)*u;v=a*v+(1-a)*u;
        assert(x>=5 && x<=95);
    }
    assert(fabsf(x-60)<.5f && fabsf(v)<.5f);
    assert(PositionMpc_UpdateBounded(&m,x,v,96,5,.002f,5,95,10)==0);
    assert(PositionMpc_UpdateBounded(&m,x,v,60,5,.002f,5,95,NAN)==0);
    puts("C620 bounded MPC nominal-model checks passed (not hardware validation)");
}
