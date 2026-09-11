#include "arm_feedback_guard.h"
#include <assert.h>
int main(void)
{
    bool latch = false;
    assert(!ArmFeedbackGuard_Check(false, 0, true, false, &latch));
    assert(!ArmFeedbackGuard_Check(true, 51, true, false, &latch));
    assert(ArmFeedbackGuard_Check(true, 10, true, true, &latch));
    assert(ArmFeedbackGuard_Check(true, 51, true, true, &latch));
    assert(ArmFeedbackGuard_Check(false, 1000, false, true, &latch));
    assert(!latch);
    assert(ArmFeedbackGuard_Check(true, 10, true, true, &latch));
    assert(!ArmFeedbackGuard_Check(true, 10, false, true, &latch));
    assert(latch);
    assert(!ArmFeedbackGuard_Check(true, 10, true, false, &latch));
}
