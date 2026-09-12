#include "ArmPositionMpc.h"
#include "arm_test_config.h"
#include <math.h>
#include <string.h>

/* 位置[deg]・速度[deg/s]、一次遅れ速度モデルの有限ホライズンMPC。
 * 固定メモリ/反復数。PIゲインとは独立した外側制御器。最短時間保証ではない。 */
static float clip(float v,float limit) { return fmaxf(-limit,fminf(limit,v)); }
void ArmPositionMpc_Reset(ArmPositionMpc *s) { if(s) memset(s,0,sizeof(*s)); }
bool ArmPositionMpc_TargetAllowed(float target) {
    return isfinite(target) && target>=ARM_TEST_POSITION_MIN_DEG && target<=ARM_TEST_POSITION_MAX_DEG;
}
/* ID2は角度制限なし。非有限値、速度・電流制限は引き続き検査する。 */
bool ArmPositionMpc_TargetAllowedForDevice(uint8_t id, float target) {
    return id == 2U ? isfinite(target) : ArmPositionMpc_TargetAllowed(target);
}
float ArmPositionMpc_Update(ArmPositionMpc *s,float x,float v,float target,float alpha,float dt) {
    return ArmPositionMpc_UpdateForDevice(1U,s,x,v,target,alpha,dt);
}
float ArmPositionMpc_UpdateForDevice(uint8_t id, ArmPositionMpc *s,float x,float v,float target,float alpha,float dt) {
    if(!s) return 0;
    if(!ArmPositionMpc_TargetAllowedForDevice(id,target)||!isfinite(x)||
       !isfinite(v)||!isfinite(alpha)||alpha<=0||!isfinite(dt)||dt<=0||dt>.05f) {
        ArmPositionMpc_Reset(s); return 0;
    }
    if(!s->initialized) {
        ArmPositionMpc_Reset(s); s->initialized=true; s->target=target; s->elapsed=ARM_MPC_PERIOD;
        s->observer_position=x;
        s->observer_velocity=clip(v,ARM_MPC_SPEED);
    } else if(fabsf(target-s->target)>.001f) {
        /* 連続目標でも外側MPCは20ms周期。目標変更で時計・観測器をリセットしない。 */
        s->target=target;
        s->bias=0;
    }
    s->elapsed+=dt;
    if(s->elapsed>=ARM_MPC_PERIOD) {
        s->elapsed=0;
        const float a=expf(-alpha*ARM_MPC_PERIOD),b=1-a,c=b/alpha,d=ARM_MPC_PERIOD-c;
        /* 外側MPCだけにモデル予測+位置イノベーションの速度推定を使用。
         * 内側DOBの40ms速度FBは変更しない。位置コストには生の位置xを使う。 */
        s->observer_position+=c*s->observer_velocity+d*s->reference;
        s->observer_velocity=a*s->observer_velocity+b*s->reference;
        const float innovation=x-s->observer_position;
        s->observer_position+=.4f*innovation;
        s->observer_velocity=clip(s->observer_velocity+2.0f*innovation,ARM_MPC_SPEED);
        v=s->observer_velocity;
        if(fabsf(target-x)<2 && fabsf(v)<10)
            s->bias=clip(s->bias+.3f*(target-x)*ARM_MPC_PERIOD,.5f);
        float goal=id == 2U ? target+s->bias : fmaxf(ARM_TEST_POSITION_MIN_DEG,fminf(ARM_TEST_POSITION_MAX_DEG,target+s->bias));
        float wx=x,wv=v;
        for(int k=0;k<ARM_MPC_N;k++) {
            s->u[k]=clip(6*(goal-wx)-wv,ARM_MPC_SPEED);
            wx+=c*wv+d*s->u[k]; wv=a*wv+b*s->u[k];
        }
        for(int iteration=0;iteration<32;iteration++) {
            s->x[0]=x;s->v[0]=v;
            for(int k=0;k<ARM_MPC_N;k++) {
                s->x[k+1]=s->x[k]+c*s->v[k]+d*s->u[k];
                s->v[k+1]=a*s->v[k]+b*s->u[k];
            }
            float lx=40*(s->x[ARM_MPC_N]-goal),lv=.3f*s->v[ARM_MPC_N];
            for(int k=ARM_MPC_N-1;k>=0;k--) {
                float g=.002f*s->u[k]+d*lx+b*lv;
                float nv=.03f*s->v[k]+c*lx+a*lv;
                lx+=2*(s->x[k]-goal);lv=nv;
                s->u[k]=clip(s->u[k]-.05f*g,ARM_MPC_SPEED);
            }
        }
        s->reference=s->u[0];
    }
    /* 毎2ms更新する端接近時の速度包絡。範囲外からは内側方向だけ許可。
     * 実機の制動距離保証ではなく、初期試験の追加抑制。 */
    if (id == 2U) return clip(s->reference,ARM_MPC_SPEED);
    const float upper=clip((ARM_TEST_POSITION_MAX_DEG-x)/(1/alpha+.15f),ARM_MPC_SPEED);
    const float lower=clip((x-ARM_TEST_POSITION_MIN_DEG)/(1/alpha+.15f),ARM_MPC_SPEED);
    return fmaxf(-lower,fminf(upper,s->reference));
}
