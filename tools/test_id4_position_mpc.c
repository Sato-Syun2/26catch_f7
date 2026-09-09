#include "PositionMpc.h"
#include "id4_velocity_safety.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static bool budget_test=false;
uint32_t Id4PositionMpc_ClockMs(void) {
    static uint32_t tick=0;
    return budget_test ? tick++ : 0;
}
static void trial(float initial,float target) {
    Id4PositionMpc m; Id4PositionMpc_Reset(&m);
    float x=initial,v=0,settled=-1;
    for(int i=0;i<10000;i++) {
        float u=Id4PositionMpc_Update(&m,x,v,target,20,.002f);
        assert(isfinite(u)); assert(fabsf(u)<=ID4_MPC_SPEED_MAX+.01f);
        float a=expf(-20*.002f),c=(1-a)/20;
        x+=c*v+(.002f-c)*u;v=a*v+(1-a)*u;
        assert(x>-2 && x<ID4_HARD_STOP_MAX_MM);
        if (fabsf(x-target)<.5f && fabsf(v)<2) {
            if(settled<0)settled=i*.002f;
        }else settled=-1;
    }
    printf("%.1f -> %.1f final=%.5f v=%.4f settle=%.3f\n",initial,target,x,v,settled);
    assert(fabsf(x-target)<.5f);assert(fabsf(v)<2);assert(settled>=0);
}
int main(void) {
    assert(ID4_MPC_SPEED_MAX == 825.0f);
    trial(0,100);trial(100,400);trial(400,100);trial(100,520);trial(520,0);
    /* 毎回1反復で打ち切られた場合にも収束する初期軌道を確認。 */
    budget_test=true;trial(100,520);trial(520,0);
    Id4PositionMpc m;Id4PositionMpc_Reset(&m);
    assert(Id4PositionMpc_Update(&m,NAN,0,100,20,.002f)==0);
    assert(Id4PositionMpc_Update(&m,100,0,521,20,.002f)==0);
    return 0;
}
