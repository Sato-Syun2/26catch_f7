#include <assert.h>
#include <stdio.h>
#include "ArmVelocityEstimate.h"
int main(void) {
    ArmVelocityEstimate s={0};
    for(uint32_t t=0;t<=1000;t+=10) ArmVelocityEstimate_Update(&s,3.0f*t*.001f,t);
    assert(fabsf(s.velocity-3.0f)<.001f);
    float before=s.velocity;
    assert(ArmVelocityEstimate_Update(&s,100.0f,1000)==before);
    assert(ArmVelocityEstimate_Update(&s,10.0f,1100)==0.0f);
    for(uint32_t t=1110;t<1500;t+=10) assert(ArmVelocityEstimate_Update(&s,10.0f,t)==0.0f);
    assert(ArmVelocityEstimate_Update(&s,NAN,1500)==0.0f);
    s=(ArmVelocityEstimate){0};
    ArmVelocityEstimate_Update(&s,0.0f,UINT32_MAX-4U);
    assert(ArmVelocityEstimate_Update(&s,.03f,5U)>0.0f);
    puts("arm velocity estimate: PASS");
    s=(ArmVelocityEstimate){0};
    for(uint32_t t=0;t<=1000;t+=2) ArmVelocityEstimate_UpdateVelocity(&s,3.0f,t);
    assert(fabsf(s.velocity-3.0f)<.001f);
    before=s.velocity;
    assert(ArmVelocityEstimate_UpdateVelocity(&s,30.0f,1000)==before);
    assert(ArmVelocityEstimate_UpdateVelocity(&s,30.0f,1100)==0.0f);
    assert(ArmVelocityEstimate_UpdateVelocity(&s,NAN,1102)==0.0f);
    s=(ArmVelocityEstimate){0};
    ArmVelocityEstimate_UpdateVelocity(&s,3.0f,UINT32_MAX-4U);
    assert(ArmVelocityEstimate_UpdateVelocity(&s,3.0f,5U)>0.0f);
    puts("standard feedback velocity filter: PASS");
    s=(ArmVelocityEstimate){0};
    ArmVelocityEstimate_UpdateVelocityWithTau(&s,3.f,0,.01f);
    for(uint32_t t=2;t<=10;t+=2) ArmVelocityEstimate_UpdateVelocityWithTau(&s,3.f,t,.01f);
    assert(fabsf(s.velocity-3.f*(1.f-expf(-1.f)))<1.e-5f);
    assert(ArmVelocityEstimate_UpdateVelocityWithTau(&s,3.f,12,0.f)==0.f);
    assert(ArmVelocityEstimate_UpdateVelocityWithTau(&s,3.f,14,NAN)==0.f);
    puts("configurable velocity filter: PASS");
}
