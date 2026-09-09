#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "Actuator_VelocityDob.h"

int main(void) {
    for (int motor=1;motor<=2;++motor) {
        Robstride_Actuator_VelocityDob_Parameters p={
            .J=motor==1?.121497f:.607f,.d=.02f,.K_tau=motor==1?.94f:1.22f,
            .dob_bandwidth=3.f,.velocity_kp=motor==1?14.f:12.f,.velocity_ki=motor==1?20.f:12.f,
            .velocity_kd=0.f,.reference_alpha=motor==1?10.f:5.f,
            .velocity_reference_limit=10.471975512f,.current_limit=motor==1?11.f:16.f,
            .velocity_reference_limit_enable=true,.current_limit_enable=true,
            .velocity_unit_to_rad_s=.01745329252f,.control_period=.002f};
        Robstride_Actuator_VelocityDob_State s={0};
        for (int i=0;i<5000;++i) {
            float u=Robstride_Actuator_VelocityDob_Update(&p,&s,i<2500?600.f:-600.f,0.f,.002f);
            assert(isfinite(u));assert(fabsf(u)<=p.current_limit+.00001f);
            assert(isfinite(s.integral));
        }
        assert(Robstride_Actuator_VelocityDob_Update(&p,&s,NAN,0.f,.002f)==0.f);
        assert(!s.initialized);
        assert(Robstride_Actuator_VelocityDob_Update(&p,&s,1.f,0.f,0.f)==0.f);
    }
    puts("Robstride DOB PI finite output/current bound/reset: PASS");
}
