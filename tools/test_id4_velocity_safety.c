#include <assert.h>
#include "id4_velocity_safety.h"
int main(void) {
    assert(!id4_velocity_stop_required(250, 100));
    assert(id4_velocity_stop_required(500, 500));
    assert(id4_velocity_stop_required(20, -400));
    assert(!id4_velocity_stop_required(500, -100));
    assert(!id4_velocity_stop_required(20, 100));
    assert(id4_position_target_allowed(515));
    assert(id4_position_target_allowed(0));
    assert(id4_position_target_allowed(520));
    assert(!id4_position_target_allowed(520.01f));
    assert(!id4_position_target_allowed(NAN));
    assert(!id4_velocity_stop_required(515, 0));
    assert(id4_safe_velocity(515, 10) == 10);
    assert(id4_safe_velocity(525, 100) == 0);
    assert(id4_safe_velocity(525, -10) == -10);
    for (float x=0; x<=520; x+=.25f) {
        const float v=id4_safe_velocity(x, 800);
        assert(v>=0 && v<=800);
        assert(!id4_velocity_stop_required(x, v));
    }
    assert(id4_velocity_stop_required(NAN, 0));
    assert(id4_velocity_stop_required(250, NAN));
    assert(id4_velocity_reference_limit(1000) == 800);
    assert(id4_velocity_reference_limit(-1000) == -800);
    assert(id4_velocity_reference_limit(NAN) == 0);
    return 0;
}
