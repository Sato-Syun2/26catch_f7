#include "PositionMpc.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#ifndef TEST_ACCELERATION
#define TEST_ACCELERATION 100000.0f
#endif
#ifndef TEST_PLANT_ALPHA
#define TEST_PLANT_ALPHA 30.0f
#endif
uint32_t Id4PositionMpc_ClockMs(void) { return 0; }

int main(void)
{
    const float maximum=180.145263671875f-5.0f, dt=.002f, alpha=30;
    float x=0, v=0;
    const float speeds[]={20,50,100,200};
    const float targets[]={100,0,maximum,50,150};
    for (unsigned speed_index=0;speed_index<4;speed_index++) {
    const float speed=speeds[speed_index];x=0;v=0;
    for (unsigned j=0;j<5;j++) {
        Id4PositionMpc s;Id4PositionMpc_Reset(&s);
        const float start=x;
        float previous_ref=0;
        for (unsigned k=0;k<15000;k++) {
            float ref=PositionMpc_UpdateBounded(&s,x,v,targets[j],alpha,dt,0,maximum,speed);
            ref=fmaxf(previous_ref-TEST_ACCELERATION*dt,fminf(previous_ref+TEST_ACCELERATION*dt,ref));
            const float arrival_speed=sqrtf(2*TEST_ACCELERATION*fabsf(targets[j]-x));
            ref=fmaxf(-arrival_speed,fminf(arrival_speed,ref));
            ref=fmaxf(-sqrtf(400*fmaxf(0,x)),fminf(sqrtf(400*fmaxf(0,maximum-x)),ref));
            previous_ref=ref;
            assert(isfinite(ref) && fabsf(ref)<=speed+.001f);
            const float a=expf(-TEST_PLANT_ALPHA*dt),c=(1-a)/TEST_PLANT_ALPHA;
            x+=c*v+(dt-c)*ref;v=a*v+(1-a)*ref;
            if (!(x>=fminf(start,targets[j])-.5f && x<=fmaxf(start,targets[j])+.5f)) {
                printf("FAIL accel=%.0f tau_ms=%.2f cap=%.0f target=%.4f t=%.3f x=%.4f v=%.4f\n",
                       (double)TEST_ACCELERATION,1000.0/TEST_PLANT_ALPHA,speed,targets[j],k*dt,x,v);
                return 1;
            }
        }
        printf("nominal MPC cap=%.0f target=%.4f final=%.4f velocity=%.4f\n",speed,targets[j],x,v);
        assert(fabsf(x-targets[j])<.1f && fabsf(v)<.1f);
    }
    }
    Id4PositionMpc s;Id4PositionMpc_Reset(&s);
    assert(PositionMpc_UpdateBounded(&s,x,v,-1,alpha,dt,0,maximum,20)==0);
    assert(PositionMpc_UpdateBounded(&s,x,v,maximum+1,alpha,dt,0,maximum,20)==0);
    puts("C620 nominal MPC origin/100/end-minus-5 checks passed");
}
