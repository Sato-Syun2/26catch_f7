#include "arm_feedback_guard.h"
#include <assert.h>
int main(void)
{
    assert(!ArmFeedbackGuard_Check(false, 0, true, false));
    assert(!ArmFeedbackGuard_Check(true, 51, true, false));
    assert(ArmFeedbackGuard_Check(true, 10, true, true));
    assert(ArmFeedbackGuard_Check(true, 51, true, true));
    assert(ArmFeedbackGuard_Check(false, 1000, false, true));
    /* 範囲超過で停止しても、正常な最新FBへ戻れば再Enableできる。 */
    for (int i = 0; i < 10; ++i) {
        assert(!ArmFeedbackGuard_Check(true, 10, false, true));
        assert(!ArmFeedbackGuard_Check(true, 10, false, false));
        assert(!ArmFeedbackGuard_Check(true, 51, true, false));
        assert(ArmFeedbackGuard_Check(true, 50, true, false));
    }
}
