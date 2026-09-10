#ifndef CONTROL_PERIOD_H
#define CONTROL_PERIOD_H
#include <stdint.h>
/* 期限ちょうどなら次の周期を即開始。期限超過時だけ追い付き連続計算を抑える。 */
static inline uint32_t ControlPeriod_Rebase(uint32_t previous, uint32_t now, uint32_t period)
{
    return (uint32_t)(now-previous)>period ? now : previous;
}
#endif
