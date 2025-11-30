#include "crane_master/canopen_ros2_node.hpp"

int32_t CANopenROS2::angle_to_position(float angle)
{
    int32_t position = static_cast<int32_t>((angle / 360.0) * ENCODER_RESOLUTION * gear_ratio_);
    return position;
}

float CANopenROS2::position_to_angle(int32_t position)
{
    float angle = (static_cast<float>(position) / (ENCODER_RESOLUTION * gear_ratio_)) * 360.0;
    return angle;
}

int32_t CANopenROS2::velocity_to_pulse(float velocity_deg_per_sec)
{
    // Convert deg/s to pulses/s using encoder counts per revolution
    int32_t velocity_pulse_per_sec = static_cast<int32_t>((velocity_deg_per_sec / 360.0f) * ENCODER_RESOLUTION * gear_ratio_);
    return velocity_pulse_per_sec;
}

int32_t CANopenROS2::acceleration_to_pulse(float acceleration_deg_per_sec2)
{
    int32_t acceleration_pulse_per_sec2 = static_cast<int32_t>((acceleration_deg_per_sec2 / 360.0f) * ENCODER_RESOLUTION * gear_ratio_);
    return acceleration_pulse_per_sec2;
}

