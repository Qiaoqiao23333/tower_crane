#include "crane_master/canopen_ros2_node.hpp"
#include <cmath>
#include <numeric>

/**
 * @brief 📐 角度转换为位置命令单位
 * @param angle 角度 (度)
 * @return 位置命令单位
 * @details 公式: (angle / 360°) × (target_units_per_rev_ / gear_ratio_) = 命令单位
 *          例如: 90度 → (90/360) × (10000/10) = 0.25 × 1000 = 250 units
 */
int32_t CANopenROS2::angle_to_position(float angle)
{
    int32_t position = static_cast<int32_t>(angle * units_per_degree_);
    return position;
}

/**
 * @brief 📐 位置命令单位转换为角度
 * @param position 位置命令单位
 * @return 角度 (度)
 * @details 公式: position × (gear_ratio_ / target_units_per_rev_) × 360° = 角度
 *          例如: 250 units → 250 × (10/10000) × 360 = 90度
 */
float CANopenROS2::position_to_angle(int32_t position)
{
    float angle = static_cast<float>(position) * degrees_per_unit_;
    return angle;
}

/**
 * @brief 📐 速度转换为命令单位
 * @param velocity_deg_per_sec 速度 (度/秒)
 * @return 命令单位/秒
 * @details 公式: velocity_deg_per_sec * units_per_degree_ = 命令单位/秒
 */
int32_t CANopenROS2::velocity_to_units(float velocity_deg_per_sec)
{
    int32_t velocity_units_per_sec = static_cast<int32_t>(velocity_deg_per_sec * units_per_degree_);
    return velocity_units_per_sec;
}

/**
 * @brief 📐 命令单位转换为速度
 * @param velocity_units_per_sec 命令单位/秒
 * @return 速度 (度/秒)
 * @details 公式: velocity_units_per_sec * degrees_per_unit_ = 速度(度/秒)
 */
float CANopenROS2::units_to_velocity(int32_t velocity_units_per_sec)
{
    float velocity_deg_per_sec = static_cast<float>(velocity_units_per_sec) * degrees_per_unit_;
    return velocity_deg_per_sec;
}

/**
 * @brief 📐 加速度转换为命令单位
 * @param acceleration_deg_per_sec2 加速度 (度/秒²)
 * @return 命令单位/秒²
 * @details 公式: acceleration_deg_per_sec2 * units_per_degree_ = 命令单位/秒²
 */
int32_t CANopenROS2::acceleration_to_units(float acceleration_deg_per_sec2)
{
    int32_t acceleration_units_per_sec2 = static_cast<int32_t>(acceleration_deg_per_sec2 * units_per_degree_);
    return acceleration_units_per_sec2;
}

/**
 * @brief ⚙️ 计算最大公约数 (GCD)
 * @param a 第一个数
 * @param b 第二个数
 * @return 最大公约数
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
 * @brief ⚙️ 计算电子齿轮比参数
 * @param gear_ratio 物理减速比
 * @param target_units_per_rev 每输出轴圈的目标命令单位
 * @return 电子齿轮比 {Numerator, Denominator} 对应 0x6091:01 和 0x6091:02
 * @details 公式: (Target_Units) * (Num / Den) = Gear_Ratio
 *          即: Num / Den = Gear_Ratio / Target_Units
 */
std::pair<uint32_t, uint32_t> CANopenROS2::calculate_gear_ratio_params(float gear_ratio, int32_t target_units_per_rev)
{
    double num = static_cast<double>(gear_ratio);
    double den = static_cast<double>(target_units_per_rev);

    // 🔢 放大以去除小数点 (支持最多 6 位小数)
    // 例如: 10.5 / 10000 -> 105 / 100000
    int max_iter = 6;
    while (std::abs(num - std::round(num)) > 1e-6 && max_iter > 0) {
        num *= 10.0;
        den *= 10.0;
        max_iter--;
    }

    uint32_t numerator = static_cast<uint32_t>(std::round(num));
    uint32_t denominator = static_cast<uint32_t>(std::round(den));

    // ➗ 约分
    uint32_t common = calculate_gcd(numerator, denominator);
    
    if (common > 0) {
        numerator /= common;
        denominator /= common;
    }

    return std::make_pair(numerator, denominator);
}
