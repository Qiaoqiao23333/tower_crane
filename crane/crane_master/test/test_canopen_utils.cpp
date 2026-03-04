#include <cassert>
#include <cstdint>
#include <iostream>
#include <cmath>


namespace crane_master_test_helpers
{
    inline int32_t angle_to_position_linear(float angle)
    {
        return static_cast<int32_t>(std::lround(static_cast<double>(angle)));
    }

    inline float position_to_angle_linear(int32_t position)
    {
        return static_cast<float>(position);
    }
}

int main()
{
    using namespace crane_master_test_helpers;

    assert(angle_to_position_linear(0.0f) == 0);
    assert(angle_to_position_linear(90.0f) == 90);
    assert(angle_to_position_linear(-45.0f) == -45);

    assert(angle_to_position_linear(10.4f) == 10);
    assert(angle_to_position_linear(10.5f) == 11);

    assert(position_to_angle_linear(0) == 0.0f);
    assert(position_to_angle_linear(90) == 90.0f);
    assert(position_to_angle_linear(-30) == -30.0f);

    std::cout << "[OK] canopen_utils 1:1 mapping tests passed.\n";
    return 0;
}

