#include "PositionMpc.h"
#include "id4_velocity_safety.h"
#include <math.h>
#include <string.h>

/* 位置+一次遅れ速度モデルの有限ホライズン最適化。
 * 固定反復数・固定メモリ。予測は実測位置/速度から毎回更新する。
 * 各予測入力を速度上限へ射影し、実出力に機構端減速包絡を適用する。
 * 厳密な最短時間保証ではない。 */
void Id4PositionMpc_Reset(Id4PositionMpc *s) { memset(s, 0, sizeof(*s)); }
static float clip(float x, float a) { return fmaxf(-a, fminf(a,x)); }
/* v_dot=alpha(u-v), |u|<=limit の位置・速度終端条件を満たす
 * 一回切替のbang-bang解。積分関係 dx=integral(u)+v0/alpha を利用する。
 * 飽和電流/機構端制約を含まないため、実機最短時間の保証ではない。 */
bool Id4MinimumTimePlan(float x,float v,float target,float alpha,float limit,
                       float *switch_time,float *arrival_time,float *direction)
{
    if (!isfinite(x)||!isfinite(v)||!isfinite(target)||!isfinite(alpha)||
        !isfinite(limit)||alpha<=0||limit<=0) return false;
    const float stop_time=log1pf(fabsf(v)/limit)/alpha;
    const float stop_distance=v/alpha-copysignf(limit*stop_time,v);
    const float sign=(target-x-stop_distance)>=0 ? 1.0f : -1.0f;
    float lo=stop_time,hi=stop_time+fabsf(target-x-stop_distance)/limit+1.0f;
    for (int i=0;i<28;i++) {
        const float t=.5f*(lo+hi);
        const float arg=.5f*(1+(1-v/(sign*limit))*expf(-alpha*t));
        if (arg<=0 || !isfinite(arg)) return false;
        const float brake=-logf(arg)/alpha;
        const float dx=sign*limit*(t-2*brake)+v/alpha;
        if (sign*(dx-(target-x))>=0) hi=t;else lo=t;
    }
    const float t=.5f*(lo+hi);
    const float brake=-logf(.5f*(1+(1-v/(sign*limit))*expf(-alpha*t)))/alpha;
    *switch_time=fmaxf(0,t-brake);*arrival_time=t;*direction=sign;
    return isfinite(t)&&brake>=-1e-5f&&*switch_time<=t+1e-5f;
}
static float time_profile(Id4PositionMpc *s,float x,float dt)
{
    /* 500Hz内で切替。2ms区間の平均入力で切替時刻の量子化を抑える。 */
    const float begin=s->phase_time,end=begin+dt;
    const float positive=fmaxf(0,fminf(end,s->switch_time)-begin);
    const float negative=fmaxf(0,fminf(end,s->arrival_time)-fmaxf(begin,s->switch_time));
    return id4_safe_velocity(x,s->first_direction*ID4_MPC_SPEED_MAX*(positive-negative)/dt);
}
float Id4PositionMpc_Update(Id4PositionMpc *s, float x, float v,
                           float target, float alpha, float dt)
{
    if (!isfinite(x) || !isfinite(v) || !id4_position_target_allowed(target) ||
        !isfinite(alpha) || alpha<=0 || !isfinite(dt) || dt<=0) {
        Id4PositionMpc_Reset(s); return 0;
    }
    s->elapsed += dt;
    if (!s->initialized || fabsf(target-s->target)>.01f) {
        Id4PositionMpc_Reset(s);
        s->initialized=true; s->target=target; s->elapsed=ID4_MPC_PERIOD_S;
    }
    if (s->elapsed < ID4_MPC_PERIOD_S) {
        if (s->time_priority_active) {
            s->phase_time+=dt;s->reference=time_profile(s,x,dt);
        }
        return s->reference;
    }
    s->elapsed=0;
    s->time_priority_active=false;
#if ID4_MPC_TIME_PRIORITY
    /* 終端の位置精度は既存MPCで確保し、遠方の移動時間を直接短縮する。 */
    /* 初回比較は中央目標に限定。端付近は実績ある二次評価を維持する。 */
    if (target>=70.0f && target<=450.0f &&
        (fabsf(target-x)>8.0f || fabsf(v)>80.0f) &&
        Id4MinimumTimePlan(x,v,target,alpha,ID4_MPC_SPEED_MAX,
                          &s->switch_time,&s->arrival_time,&s->first_direction)) {
        s->time_priority_active=true;s->phase_time=0;
        s->reference=time_profile(s,x,dt);return s->reference;
    }
#endif
    /* 微小なモデル/摩擦誤差のみ補償。大移動中の積分蓄積は禁止する。 */
    if (fabsf(target-x)<3 && fabsf(v)<20)
        s->bias=clip(s->bias + .4f*(target-x)*ID4_MPC_PERIOD_S, 1.0f);
    const float goal=fmaxf(0, fminf(ID4_POSITION_COMMAND_MAX_MM,target+s->bias));
    const float a=expf(-alpha*ID4_MPC_PERIOD_S), b=1-a;
    const float c=b/alpha, d=ID4_MPC_PERIOD_S-c;
    /* 計算打切り時でも停止へ向かう初期軌道を毎回生成する。
     * 未収束な前回解をシフトするだけでは端付近で停止偏差が残る。 */
    float warm_x=x,warm_v=v;
    for (int k=0;k<ID4_MPC_HORIZON;k++) {
        s->u[k]=clip(20*(goal-warm_x)-warm_v,ID4_MPC_SPEED_MAX);
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
            s->u[k]=clip(s->u[k],ID4_MPC_SPEED_MAX);
            s->x[k+1]=s->x[k]+c*s->v[k]+d*s->u[k];
            s->v[k+1]=a*s->v[k]+b*s->u[k];
        }
        float lx=40*(s->x[ID4_MPC_HORIZON]-goal);
        float lv=.2f*s->v[ID4_MPC_HORIZON];
        for (int k=ID4_MPC_HORIZON-1;k>=0;k--) {
            const float gradient=.0008f*s->u[k]+d*lx+b*lv;
            const float next_lv=.005f*s->v[k]+c*lx+a*lv;
            lx+=2*(s->x[k]-goal); lv=next_lv;
            s->u[k]=clip(s->u[k]-.15f*gradient,ID4_MPC_SPEED_MAX);
        }
    }
    s->reference=id4_safe_velocity(x,s->u[0]);
    return s->reference;
}
