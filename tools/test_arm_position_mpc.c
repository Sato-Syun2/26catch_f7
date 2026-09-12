#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "ArmPositionMpc.h"
int main(void) {
    {
        ArmPositionMpc s={0}; unsigned solves=0;
        for(unsigned i=0;i<500;i++) {
            const float target=5*sinf(i*.002f*6.2831853f*.4f);
            (void)ArmPositionMpc_Update(&s,0,0,target,10,.002f);
            if(s.elapsed==0) ++solves;
        }
        assert(solves>=45 && solves<=51);
        printf("Continuous target MPC solves per second: %u\n",solves);
    }
    for(int direction=-1;direction<=1;direction+=2) {
        ArmPositionMpc s={0}; float x=0,v=0,target=direction*90;
        for(int i=0;i<5000;i++) {
            /* 外側速度FBに高周波成分を重畳しても位置モデルで収束すること。 */
            float noisy_v=v+30*sinf(i*.002f*6.2831853f*20);
            float u=ArmPositionMpc_Update(&s,x,noisy_v,target,10,.002f);
            assert(isfinite(u)&&fabsf(u)<=ARM_MPC_SPEED);
            float b=1-expf(-.02f),c=b/10;
            x+=c*v+(.002f-c)*u;v=(1-b)*v+b*u;
            assert(fabsf(x)<145);
        }
        printf("target %.1f position %.5f velocity %.5f\n",target,x,v);
        assert(fabsf(x-target)<.1f && fabsf(v)<.1f);
        assert(ArmPositionMpc_Update(&s,x,v,NAN,10,.002f)==0 && !s.initialized);
        assert(ArmPositionMpc_Update(&s,x,v,146,10,.002f)==0 && !s.initialized);
        assert(ArmPositionMpc_Update(&s,x,v,0,0,.002f)==0 && !s.initialized);
    }
    /* 範囲超過や不正指令の後でも、次の正常指令で内側へ戻れる。 */
    for (int direction=-1; direction<=1; direction+=2) {
        ArmPositionMpc s={0};
        float x=direction*146.0f, v=0, target=direction*130.0f;
        assert(ArmPositionMpc_Update(&s,x,v,direction*150.0f,10,.002f)==0);
        for (int i=0; i<5000; ++i) {
            const float u=ArmPositionMpc_Update(&s,x,v,target,10,.002f);
            assert(isfinite(u) && fabsf(u)<=ARM_MPC_SPEED);
            if (fabsf(x)>145) assert(direction*u<=0);
            const float b=1-expf(-.02f),c=b/10;
            x+=c*v+(.002f-c)*u;v=(1-b)*v+b*u;
        }
        assert(fabsf(x-target)<.1f);
        assert(ArmPositionMpc_Update(&s,NAN,0,0,10,.002f)==0);
        assert(ArmPositionMpc_Update(&s,0,0,0,10,.002f)==0);
        assert(s.initialized);
        const float far_u=ArmPositionMpc_Update(&s,direction*10000.0f,0,0,10,.002f);
        assert(fabsf(far_u)<=ARM_MPC_SPEED && direction*far_u<=0);
    }
    assert(ArmPositionMpc_TargetAllowed(-145) && ArmPositionMpc_TargetAllowed(145));
    assert(!ArmPositionMpc_TargetAllowedForDevice(1,146));
    assert(!ArmPositionMpc_TargetAllowedForDevice(1,-146));
    assert(ArmPositionMpc_TargetAllowedForDevice(2,720));
    assert(ArmPositionMpc_TargetAllowedForDevice(2,-720));
    assert(!ArmPositionMpc_TargetAllowedForDevice(2,NAN));
    assert(!ArmPositionMpc_TargetAllowedForDevice(2,INFINITY));
    for (int direction=-1;direction<=1;direction+=2) {
        ArmPositionMpc s={0}; float x=direction*200.0f,v=0,target=direction*360.0f;
        for(int i=0;i<5000;i++) {
            float u=ArmPositionMpc_UpdateForDevice(2,&s,x,v,target,10,.002f);
            assert(isfinite(u) && fabsf(u)<=ARM_MPC_SPEED);
            float b=1-expf(-.02f),c=b/10;
            x+=c*v+(.002f-c)*u;v=(1-b)*v+b*u;
        }
        assert(fabsf(x-target)<.1f);
        assert(ArmPositionMpc_UpdateForDevice(2,&s,x,v,NAN,10,.002f)==0 && !s.initialized);
    }
    puts("Arm MPC nominal plant / limits / invalid reset: PASS");
}
