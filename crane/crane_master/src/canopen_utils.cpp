#include "crane_master/canopen_ros2_node.hpp"
#include <cmath>
#include <numeric>

// ======================================================================
// 🔬 Helper functions used by both production code and tests
// ======================================================================

namespace crane_master_test_helpers
{
    /**
     * @brief 1:1 映射：角度 → 位置单位
     * @details 输入多少度，就返回多少位置单位（四舍五入到最近整数）
     */
    inline int32_t angle_to_position_linear(float angle)
    {
        return static_cast<int32_t>(std::lround(static_cast<double>(angle)));
    }

    /**
     * @brief 1:1 映射：位置单位 → 角度
     * @details 输入多少位置单位，就返回多少度
     */
    inline float position_to_angle_linear(int32_t position)
    {
        return static_cast<float>(position);
    }
} // namespace crane_master_test_helpers

using namespace crane_master_test_helpers;

/**
 * @brief 📐 Convert angle to position command units
 * @param angle Angle (degrees)
 * @return Position command units
 *
 * @details
 *  改为「1:1 直通」关系：
 *    - 输入多少度，就在驱动器对象 0x607A 中写多少「位置单位」
 *    - 例如：angle = 90.0f → 写入 90
 */
int32_t CANopenROS2::angle_to_position(float angle)
{
    return angle_to_position_linear(angle);
}

/**
 * @brief 📐 Convert position command units to angle
 * @param position Position command units
 * @return Angle (degrees)
 *
 * @details
 *  与 angle_to_position 保持 1:1 关系：
 *    - 位置单位是多少，就认为是多少度
 */
float CANopenROS2::position_to_angle(int32_t position)
{
    return position_to_angle_linear(position);
}

/**
 * @brief 📐 Convert velocity to command units
 * @param velocity_rev_per_sec Velocity (revolutions/second of the output/driving shaft)
 * @return Command units/second  (1 command unit = 1 output-shaft revolution)
 * @details
 *  Position units and velocity units share the same 1:1 scale
 *  (1 command unit = 1 driving-shaft revolution, matching angle_to_position).
 *  The drive's internal gear ratio (0x6091) handles the motor-side multiplication.
 *  Formula: velocity_units = round(velocity_rev_per_sec)
 */
int32_t CANopenROS2::velocity_to_units(float velocity_rev_per_sec)
{
    int32_t velocity_units_per_sec = static_cast<int32_t>(std::lround(static_cast<double>(velocity_rev_per_sec)));
    return velocity_units_per_sec;
}

/**
 * @brief 📐 Convert command units to velocity
 * @param velocity_units_per_sec Command units/second
 * @return Velocity (revolutions/second of the output/driving shaft)
 * @details 1 command unit = 1 driving-shaft revolution → direct cast.
 */
float CANopenROS2::units_to_velocity(int32_t velocity_units_per_sec)
{
    float velocity_rev_per_sec = static_cast<float>(velocity_units_per_sec);
    return velocity_rev_per_sec;
}

/**
 * @brief 📐 Convert acceleration to command units
 * @param acceleration_rev_per_sec2 Acceleration (revolutions/second² of the output/driving shaft)
 * @return Command units/second²  (1 command unit = 1 output-shaft revolution)
 * @details Same 1:1 scale as velocity_to_units.
 */
int32_t CANopenROS2::acceleration_to_units(float acceleration_rev_per_sec2)
{
    int32_t acceleration_units_per_sec2 = static_cast<int32_t>(std::lround(static_cast<double>(acceleration_rev_per_sec2)));
    return acceleration_units_per_sec2;
}
