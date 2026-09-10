#include "PositionMpc.h"
#include <math.h>
static float clip(float x,float a){return fmaxf(-a,fminf(a,x));}
float PositionMpc_UpdateBounded(Id4PositionMpc *s, float x, float v,
    float target, float alpha, float dt, float minimum, float maximum, float speed)
{
    if (!isfinite(x) || !isfinite(v) || !isfinite(target) ||
        !isfinite(minimum) || !isfinite(maximum) || minimum >= maximum ||
        target < minimum || target > maximum || !isfinite(speed) || speed <= 0 ||
        !isfinite(alpha) || alpha<=0 || !isfinite(dt) || dt<=0) {
        Id4PositionMpc_Reset(s); return 0;
    }
    s->elapsed += dt;
    if (!s->initialized || fabsf(target-s->target)>.01f) {
        Id4PositionMpc_Reset(s);
        s->initialized=true; s->target=target; s->elapsed=ID4_MPC_PERIOD_S;
    }
    if (s->elapsed < ID4_MPC_PERIOD_S) return s->reference;
    s->elapsed=0;
    /* 微小なモデル/摩擦誤差のみ補償。大移動中の積分蓄積は禁止する。 */
    if (fabsf(target-x)<3 && fabsf(v)<20)
        s->bias=clip(s->bias + .4f*(target-x)*ID4_MPC_PERIOD_S, 1.0f);
    const float goal=fmaxf(minimum, fminf(maximum,target+s->bias));
    const float a=expf(-alpha*ID4_MPC_PERIOD_S), b=1-a;
    const float c=b/alpha, d=ID4_MPC_PERIOD_S-c;
    /* 計算打切り時でも停止へ向かう初期軌道を毎回生成する。
     * 未収束な前回解をシフトするだけでは端付近で停止偏差が残る。 */
    float warm_x=x,warm_v=v;
    for (int k=0;k<ID4_MPC_HORIZON;k++) {
        s->u[k]=clip(20*(goal-warm_x)-warm_v,speed);
        warm_x+=c*warm_v+d*s->u[k];warm_v=a*warm_v+b*s->u[k];
    }
    s->iterations=0;
    const uint32_t begin=Id4PositionMpc_ClockMs();
    for (int iteration=0;iteration<40;iteration++) {
        /* 時間上限で打ち切り、途中解を使う。CAN/ROS処理を占有しない。 */
        if (iteration>0 && Id4PositionMpc_ClockMs()-begin>=1U) break;
        s->iterations++;
        s->x[0]=x; s->v[0]=v;
        for (int k=0;k<ID4_MPC_HORIZON;k++) {
            /* QP内は一定の入力箱制約にする。予測状態依存クリップを
             * 勾配に混ぜると停留点が壊れる。機構包絡は実出力で適用する。 */
            s->u[k]=clip(s->u[k],speed);
            s->x[k+1]=s->x[k]+c*s->v[k]+d*s->u[k];
            s->v[k+1]=a*s->v[k]+b*s->u[k];
        }
        float lx=40*(s->x[ID4_MPC_HORIZON]-goal);
        float lv=.2f*s->v[ID4_MPC_HORIZON];
        for (int k=ID4_MPC_HORIZON-1;k>=0;k--) {
            const float gradient=.0008f*s->u[k]+d*lx+b*lv;
            const float next_lv=.005f*s->v[k]+c*lx+a*lv;
            lx+=2*(s->x[k]-goal); lv=next_lv;
            s->u[k]=clip(s->u[k]-.15f*gradient,speed);
        }
    }
    s->reference=s->u[0];
    return s->reference;
}
