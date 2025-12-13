#include "crane_master/canopen_ros2_node.hpp"
#include <cmath>
#include <numeric>

int32_t CANopenROS2::angle_to_position(float angle)
{
    // Convert angle (degrees) to command units
    // Formula: (angle / 360°) × (target_units_per_rev_ / gear_ratio_) = command units
    // Example: 90度 → (90/360) × (10000/10) = 0.25 × 1000 = 250 units
    int32_t position = static_cast<int32_t>(angle * units_per_degree_);
    return position;
}

float CANopenROS2::position_to_angle(int32_t position)
{
    // Convert command units to angle (degrees)
    // Formula: position × (gear_ratio_ / target_units_per_rev_) × 360° = angle
    // Example: 250 units → 250 × (10/10000) × 360 = 250 × 0.001 × 360 = 90度
    float angle = static_cast<float>(position) * degrees_per_unit_;
    return angle;
}

int32_t CANopenROS2::velocity_to_units(float velocity_deg_per_sec)
{
    // Convert deg/s to command units/s using cached ratio from calculate_gear_ratio_params
    // Formula: velocity_deg_per_sec * units_per_degree_ = command units/s
    int32_t velocity_units_per_sec = static_cast<int32_t>(velocity_deg_per_sec * units_per_degree_);
    return velocity_units_per_sec;
}

int32_t CANopenROS2::acceleration_to_units(float acceleration_deg_per_sec2)
{
    // Convert deg/s² to command units/s² using cached ratio from calculate_gear_ratio_params
    // Formula: acceleration_deg_per_sec2 * units_per_degree_ = command units/s²
    int32_t acceleration_units_per_sec2 = static_cast<int32_t>(acceleration_deg_per_sec2 * units_per_degree_);
    return acceleration_units_per_sec2;
}

// Helper function for Greatest Common Divisor
static uint32_t calculate_gcd(uint32_t a, uint32_t b) {
    while (b != 0) {
        uint32_t temp = b;
        b = a % b;
        a = temp;
    }
    return a;
}

std::pair<uint32_t, uint32_t> CANopenROS2::calculate_gear_ratio_params(float gear_ratio, int32_t target_units_per_rev)
{
    // Formula: (Target_Units) * (Num / Den) = Gear_Ratio
    // Therefore: Num / Den = Gear_Ratio / Target_Units
    
    double num = static_cast<double>(gear_ratio);
    double den = static_cast<double>(target_units_per_rev);

    // Scale up to remove decimals from gear_ratio if necessary
    // e.g. 10.5 / 10000 -> 105 / 100000
    int max_iter = 6; // Support up to 6 decimal places
    while (std::abs(num - std::round(num)) > 1e-6 && max_iter > 0) {
        num *= 10.0;
        den *= 10.0;
        max_iter--;
    }

    uint32_t numerator = static_cast<uint32_t>(std::round(num));
    uint32_t denominator = static_cast<uint32_t>(std::round(den));

    // Reduce fraction
    uint32_t common = calculate_gcd(numerator, denominator);
    
    if (common > 0) {
        numerator /= common;
        denominator /= common;
    }

    return std::make_pair(numerator, denominator);
}
