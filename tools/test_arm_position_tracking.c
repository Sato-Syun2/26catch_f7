#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "arm_position_tracking.h"
#include "ArmPositionMpc.h"

static const float rad = 0.017453292519943295f;

int main(void)
{
    /* Blueの実測事象と、Red側の逆符号を両方再現する。
     * 起動時保持指令とEnable直後のMPC位置が360度ずれないこと。 */
    for (int side=-1; side<=1; side+=2) {
        float position = ArmPositionTracking_Update(side*257.3129f*rad,0,false);
        const float target = -position/rad;
        ArmPositionMpc mpc={0};
        for (int i=0;i<500;i++) {
            /* Type17とType2の表現を交互に入力する。 */
            float sample = side*(i%2 ? -102.6871f : 257.3129f)*rad;
            position = ArmPositionTracking_Update(sample,position,true);
            float measured = -position/rad;
            assert(fabsf(measured-target)<.001f);
            float velocity = ArmPositionMpc_UpdateForDevice(2,&mpc,measured,0,target,10,.002f);
            assert(fabsf(velocity)<.01f);
        }
    }
    /* 符号化境界と複数回転を跨いでも連続値を維持する。 */
    for (int direction=-1;direction<=1;direction+=2) {
        float previous=0;
        for (int i=0;i<=3000;i++) {
            float actual=direction*i*.5f*rad;
            float type2=remainderf(actual,8*3.141592653589793f);
            previous=ArmPositionTracking_Update(type2,previous,i!=0);
            assert(fabsf(previous-actual)<.001f);
            float mech=remainderf(actual,2*3.141592653589793f);
            previous=ArmPositionTracking_Update(mech,previous,true);
            assert(fabsf(previous-actual)<.001f);
        }
    }
    assert(isnan(ArmPositionTracking_Update(NAN,0,false)));
    assert(isnan(ArmPositionTracking_Update(INFINITY,0,true)));
    assert(isnan(ArmPositionTracking_Update(0,NAN,true)));
    puts("PASS: Blue/Red feedback handoff holds position; multi-turn tracking; invalid samples");
}
