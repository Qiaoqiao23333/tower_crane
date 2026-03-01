#include "crane_master/canopen_ros2_node.hpp"
#include <cmath>
#include <numeric>

/**
 * @brief 📐 Convert angle to position command units
 * @param angle Angle (degrees)
 * @return Position command units
 * @details Formula: (angle / 360°) × ((position_factor_num / position_factor_den) × gear_ratio_) = command units
 *          Example (slewing): 360° → (360/360) × (10000/1 × 10) = 1 × 100000 = 100000 units
 */
int32_t CANopenROS2::angle_to_position(float angle)
{
    int32_t position = static_cast<int32_t>(std::lround(static_cast<double>(angle) * units_per_degree_));
    return position;
}

/**
 * @brief 📐 Convert position command units to angle
 * @param position Position command units
 * @return Angle (degrees)
 * @details Formula: position × 360° / ((position_factor_num / position_factor_den) × gear_ratio_) = angle
 *          Example (slewing): 100000 units → 100000 × 360 / (10000 × 10) = 360°
 */
float CANopenROS2::position_to_angle(int32_t position)
{
    float angle = static_cast<float>(static_cast<double>(position) * degrees_per_unit_);
    return angle;
}

/**
 * @brief 📐 Convert velocity to command units
 * @param velocity_deg_per_sec Velocity (degrees/second)
 * @return Command units/second
 * @details Formula: velocity_deg_per_sec * units_per_degree_ = command units/second
 */
int32_t CANopenROS2::velocity_to_units(float velocity_deg_per_sec)
{
    int32_t velocity_units_per_sec = static_cast<int32_t>(std::lround(static_cast<double>(velocity_deg_per_sec) * units_per_degree_));
    return velocity_units_per_sec;
}

/**
 * @brief 📐 Convert command units to velocity
 * @param velocity_units_per_sec Command units/second
 * @return Velocity (degrees/second)
 * @details Formula: velocity_units_per_sec * degrees_per_unit_ = velocity (degrees/second)
 */
float CANopenROS2::units_to_velocity(int32_t velocity_units_per_sec)
{
    float velocity_deg_per_sec = static_cast<float>(static_cast<double>(velocity_units_per_sec) * degrees_per_unit_);
    return velocity_deg_per_sec;
}

/**
 * @brief 📐 Convert acceleration to command units
 * @param acceleration_deg_per_sec2 Acceleration (degrees/second²)
 * @return Command units/second²
 * @details Formula: acceleration_deg_per_sec2 * units_per_degree_ = command units/second²
 */
int32_t CANopenROS2::acceleration_to_units(float acceleration_deg_per_sec2)
{
    int32_t acceleration_units_per_sec2 = static_cast<int32_t>(std::lround(static_cast<double>(acceleration_deg_per_sec2) * units_per_degree_));
    return acceleration_units_per_sec2;
}

/**
 * @brief ⚙️ Calculate Greatest Common Divisor (GCD)
 * @param a First number
 * @param b Second number
 * @return Greatest Common Divisor
 */
static uint32_t calculate_gcd(uint32_t a, uint32_t b) {
    while (b != 0) {
        uint32_t temp = b;
        b = a % b;
        a = temp;
    }
    return a;
}

/**
 * @brief ⚙️ Calculate electronic gear ratio parameters
 * @param gear_ratio Physical gear reduction ratio
 * @param numerator    Position factor numerator   (0x6093:01)
 * @param denominator  Position factor denominator (0x6093:02)
 * @return Electronic gear ratio {Numerator, Denominator} corresponding to 0x6091:01 and 0x6091:02
 * @details Formula: (Target_Units) * (Num / Den) = Gear_Ratio
 *          i.e.: Num / Den = Gear_Ratio / Target_Units
 */
std::pair<uint32_t, uint32_t> CANopenROS2::calculate_gear_ratio_params(float gear_ratio, uint32_t numerator, uint32_t denominator)
{
    double num = static_cast<double>(gear_ratio);
    double den = static_cast<double>(numerator) / static_cast<double>(denominator);

    // 🔢 Scale up to remove decimal point (supports up to 6 decimal places)
    // Example: 10.5 / 10000 -> 105 / 100000
    int max_iter = 6;
    while (std::abs(num - std::round(num)) > 1e-6 && max_iter > 0) {
        num *= 10.0;
        den *= 10.0;
        max_iter--;
    }

    uint32_t result_numerator = static_cast<uint32_t>(std::round(num));
    uint32_t result_denominator = static_cast<uint32_t>(std::round(den));

    // ➗ Reduce fraction
    uint32_t common = calculate_gcd(result_numerator, result_denominator);
    
    if (common > 0) {
        result_numerator /= common;
        result_denominator /= common;
    }

    return std::make_pair(result_numerator, result_denominator);
}
