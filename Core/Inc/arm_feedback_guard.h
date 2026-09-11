#pragma once
#include <stdbool.h>
#include <stdint.h>

/* 一時的なFB欠落を角度超過ラッチにしない。欠落中のMPC/DOBは
 * 電流ゼロを出し、FB復帰後に保持目標へ復帰する。新規Enableは待つ。 */
static inline bool ArmFeedbackGuard_Check(bool measured, uint32_t age_ms,
                                         bool in_range, bool enabled, bool *latched)
{
    if (*latched) return false;
    if (!measured || age_ms > 50U) return enabled;
    if (!in_range) {
        if (enabled) *latched = true;
        return false;
    }
    return true;
}
