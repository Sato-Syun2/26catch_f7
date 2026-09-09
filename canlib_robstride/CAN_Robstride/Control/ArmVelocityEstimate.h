#ifndef ARM_VELOCITY_ESTIMATE_H
#define ARM_VELOCITY_ESTIMATE_H
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* mechPosの新着時刻だけで微分する。運転中の角度wrapは行わない。
 * 40ms一次LPFの位相遅れを含むため、同定前に高帯域DOBへ流用しない。 */
typedef struct {
    float position, velocity;
    uint32_t tick;
    bool initialized;
} ArmVelocityEstimate;
static inline float ArmVelocityEstimate_Update(ArmVelocityEstimate *s, float p, uint32_t tick)
{
    if (!isfinite(p)) { *s=(ArmVelocityEstimate){0}; return 0.0f; }
    uint32_t elapsed=tick-s->tick;
    if (!s->initialized || elapsed>50U) {
        *s=(ArmVelocityEstimate){.position=p,.tick=tick,.initialized=true};
    } else if (elapsed) {
        float dt=elapsed*.001f;
        float beta=expf(-dt/.04f);
        s->velocity=beta*s->velocity+(1.0f-beta)*(p-s->position)/dt;
        s->position=p;s->tick=tick;
    }
    return s->velocity;
}
/* 標準FBは微分せず40msで平滑化。角度振幅一定チャープの基準設定。 */
static inline float ArmVelocityEstimate_UpdateVelocityWithTau(ArmVelocityEstimate *s, float velocity, uint32_t tick, float tau)
{
    if (!isfinite(velocity) || !isfinite(tau) || tau<=0.0f) { *s=(ArmVelocityEstimate){0}; return 0.0f; }
    const uint32_t elapsed=tick-s->tick;
    if (!s->initialized || elapsed>50U) {
        *s=(ArmVelocityEstimate){.velocity=0.0f,.tick=tick,.initialized=true};
    } else if (elapsed) {
        const float beta=expf(-(float)elapsed*.001f/tau);
        s->velocity=beta*s->velocity+(1.0f-beta)*velocity;
        s->tick=tick;
    }
    return s->velocity;
}
/* 既存経路は40msを維持する。 */
static inline float ArmVelocityEstimate_UpdateVelocity(ArmVelocityEstimate *s, float velocity, uint32_t tick)
{
    return ArmVelocityEstimate_UpdateVelocityWithTau(s,velocity,tick,.04f);
}
#endif
