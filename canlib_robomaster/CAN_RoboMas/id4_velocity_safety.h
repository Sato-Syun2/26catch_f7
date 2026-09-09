#ifndef ID4_VELOCITY_SAFETY_H
#define ID4_VELOCITY_SAFETY_H
#include <stdbool.h>
#include <math.h>

/* 2026-09-09: 原点校正後、ユーザーが手で真の機械端へ移動して測定。
 * 自動探索の500.688/503.841mmではなく、この値を機構上限とする。 */
#define ID4_MECHANICAL_MAX_MM 528.878173828125f
#define ID4_POSITION_COMMAND_MAX_MM 520.0f
#define ID4_CALIBRATION_CURRENT_A 2.0f
#define ID4_NORMAL_CURRENT_A 10.0f
#define ID4_HARD_STOP_MAX_MM (ID4_MECHANICAL_MAX_MM - 2.0f)
#define ID4_HARD_STOP_MIN_MM (-2.0f)
/* DOB tau=.05s、実測約769mm/sから36.84mmで停止。
 * 0.065|v|を非常判定、0.10|u|を通常速度包絡に使用する。
 * 負荷/電流制限変更時の物理保証ではなく、実測からの設計余裕。 */
#define ID4_BRAKE_DELAY_S 0.065f
#define ID4_NORMAL_STOP_TIME_S 0.10f
#define ID4_BRAKE_MARGIN_MM 2.0f

static inline bool id4_position_target_allowed(float x)
{
    return isfinite(x) && x >= 0.0f && x <= ID4_POSITION_COMMAND_MAX_MM;
}
static inline float id4_brake_distance(float v)
{
    return ID4_BRAKE_DELAY_S*fabsf(v);
}
/* 目標速度に事前制約を掛け、位置制御の515mm目標は変更しない。 */
static inline float id4_safe_velocity(float x, float v)
{
    if (!isfinite(x) || !isfinite(v)) return 0.0f;
    const float distance = v >= 0.0f
        ? ID4_HARD_STOP_MAX_MM-ID4_BRAKE_MARGIN_MM-x
        : x-ID4_HARD_STOP_MIN_MM-ID4_BRAKE_MARGIN_MM;
    const float cap = fmaxf(0.0f, distance/ID4_NORMAL_STOP_TIME_S);
    return fmaxf(-cap, fminf(cap, v));
}
static inline bool id4_velocity_stop_required(float x, float v)
{
    if (!isfinite(x) || !isfinite(v)) return true;
    return (v > 0.0f && x + id4_brake_distance(v) >= ID4_HARD_STOP_MAX_MM) ||
           (v < 0.0f && x - id4_brake_distance(v) <= ID4_HARD_STOP_MIN_MM);
}
static inline float id4_velocity_reference_limit(float v)
{
    return isfinite(v) ? fmaxf(-800.0f, fminf(800.0f, v)) : 0.0f;
}
#endif
