#pragma once
#include <stdbool.h>
#include <stdint.h>

/* 停止履歴は保持しない。毎回、現在の角度とFB鮮度で判定する。
 * 一時的なFB欠落中は既存MPC/DOBが電流ゼロを出す。新規Enableは待つ。 */
static inline bool ArmFeedbackGuard_Check(bool measured, uint32_t age_ms,
                                         bool in_range, bool enabled)
{
    if (!measured || age_ms > 50U) return enabled;
    return in_range;
}
