#ifndef ARM_POSITION_TRACKING_H
#define ARM_POSITION_TRACKING_H

#include <stdbool.h>
#include <math.h>

/* Type17 mechPosとType2は同じ姿勢を異なる周回で返すことがある。
 * 通信符号化範囲(8*pi)ではなく機械角の1回転(2*pi)で前回位置へ揃える。
 * 両入力で同じ連続位置を更新し、Enable/Disableで周回を変えない。 */
static inline float ArmPositionTracking_Update(float sample, float previous, bool valid)
{
    if (!isfinite(sample) || (valid && !isfinite(previous))) return NAN;
    if (!valid) return sample;
    const float turn = 6.2831853071795864769f;
    return previous + remainderf(sample - previous, turn);
}

#endif
