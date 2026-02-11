#ifndef CANOPEN_ROS2_NODE_HPP
#define CANOPEN_ROS2_NODE_HPP

/**
 * @file canopen_ros2_node.hpp
 * @brief 🏗️ 塔吊 CANopen ROS2 控制节点头文件
 * @details 定义了 CANopen 协议相关的宏、CiA402 状态机控制字、对象字典索引等
 */

#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <chrono>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <utility>  // 用于 std::pair

// ==================== 📡 CANopen COB-ID 基础值 ====================
#define COB_NMT      0x000   // 🎛️ 网络管理 (NMT)
#define COB_SYNC     0x080   // 🔄 同步帧
#define COB_RPDO1    0x200   // 📥 接收PDO1
#define COB_RPDO2    0x300   // 📥 接收PDO2
#define COB_RSDO     0x600   // 📥 接收SDO (客户端->服务器)
#define COB_TSDO     0x580   // 📤 发送SDO (服务器->客户端)
#define COB_TPDO1    0x180   // 📤 发送PDO1

// ==================== 🎮 NMT 网络管理命令 ====================
#define NMT_START_REMOTE_NODE    0x01   // ▶️ 启动远程节点
#define NMT_STOP_REMOTE_NODE     0x02   // ⏹️ 停止远程节点
#define NMT_RESET_NODE           0x81   // 🔄 复位节点
#define NMT_RESET_COMM           0x82   // 🔄 复位通信

// ==================== ⚙️ CiA402 控制字 ====================
#define CONTROL_SHUTDOWN         0x06   // 🔌 关机 (准备切换到 Switch On)
#define CONTROL_SWITCH_ON        0x07   // ⚡ 开机 (已开启但未使能)
#define CONTROL_ENABLE_OPERATION 0x0F   // ✅ 使能操作 (电机可运行)
#define CONTROL_DISABLE_VOLTAGE  0x00   // 🔴 禁用电压
#define CONTROL_FAULT_RESET      0x80   // 🚨 故障复位
#define CONTROL_NEW_SET_POINT    0x10   // 🎯 新设定点 (位4, 触发位置运动)

// ==================== 🚀 CiA402 操作模式 ====================
#define MODE_PROFILE_POSITION    1      // 📍 轮廓位置模式
// #define MODE_VELOCITY         2      // ❌ 不支持 (DSY-C.eds 中未定义)
#define MODE_PROFILE_VELOCITY    3      // 🏃 轮廓速度模式 (用于速度控制)
#define MODE_PROFILE_TORQUE      4      // 💪 轮廓力矩模式
#define MODE_HOMING              6      // 🏠 回零模式
#define MODE_INTERPOLATED_POS    7      // 📐 插补位置模式

// ==================== 📚 对象字典索引 ====================
#define OD_CYCLE_PERIOD          0x1006 // ⏱️ 通信周期 (微秒)
#define OD_CONTROL_WORD          0x6040 // 🎮 控制字
#define OD_STATUS_WORD           0x6041 // 📊 状态字
#define OD_OPERATION_MODE        0x6060 // 🔧 操作模式 (写入)
#define OD_OPERATION_MODE_DISPLAY 0x6061 // 👁️ 操作模式显示 (只读)
#define OD_TARGET_POSITION       0x607A // 🎯 目标位置
#define OD_TARGET_VELOCITY       0x60FF // 🏃 目标速度
#define OD_ACTUAL_POSITION       0x6064 // 📍 实际位置
#define OD_ACTUAL_VELOCITY       0x606C // 📈 实际速度
#define OD_PROFILE_VELOCITY      0x6081 // 🚀 轮廓速度
#define OD_PROFILE_ACCELERATION  0x6083 // ⬆️ 轮廓加速度
#define OD_PROFILE_DECELERATION  0x6084 // ⬇️ 轮廓减速度

// ==================== 🛡️ 安全限制 (EDS 中定义) ====================
#define OD_MAX_MOTOR_SPEED       0x6080 // 🔒 最大电机转速 (默认: 5000 RPM)
#define OD_MAX_PROFILE_VELOCITY  0x607F // 🔒 最大轮廓速度 (默认: 600,000)

// ==================== ⚙️ 电子齿轮比 ====================
// 许多 DSY 电机需要显式设置为 1:1 (如果不是默认值)
#define OD_GEAR_RATIO            0x6091 // ⚙️ 齿轮比 (子索引01=分子, 02=分母)

// ==================== 💾 保存参数 ====================
// 如果修改了设置，可能需要保存到 EEPROM
#define OD_STORE_PARAMETERS      0x1010 // 💾 存储参数
#define STORE_SIGNATURE          0x65766173 // 🔑 保存签名 (ASCII: "save")


/**
 * @class CANopenROS2
 * @brief 🏗️ 塔吊 CANopen ROS2 控制节点类
 * @details 实现 CANopen 协议通信、CiA402 电机控制、ROS2 接口等功能
 */
class CANopenROS2 : public rclcpp::Node
{
public:
    CANopenROS2();   // 🛠️ 构造函数
    ~CANopenROS2();  // 🗑️ 析构函数

private:
    // ==================== 📡 CAN 通信函数 ====================
    void init_can_socket();                                              // 🔌 初始化CAN套接字
    void send_nmt_command(uint8_t command);                              // 🎮 发送NMT命令
    void send_sync_frame();                                              // 🔄 发送同步帧
    void write_sdo(uint16_t index, uint8_t subindex, int32_t data, uint8_t size);  // ✏️ 写SDO
    int32_t read_sdo(uint16_t index, uint8_t subindex);                  // 📖 读SDO
    void receive_can_frames();                                           // 📥 接收CAN帧
    
    // ==================== ⚙️ 电机控制函数 ====================
    void initialize_node();                                              // 🚀 初始化节点
    void configure_pdo();                                                // 📋 配置PDO映射
    void start_node();                                                   // ▶️ 启动节点
    void set_immediate_effect(bool immediate);                           // ⚡ 设置立即生效
    void clear_fault();                                                  // 🚨 清除故障
    void enable_motor();                                                 // ✅ 使能电机
    void stop_motor();                                                   // ⏹️ 停止电机
    void initialize_motor();                                             // 🔧 初始化电机
    void set_operation_mode(uint8_t mode);                               // 🎛️ 设置操作模式
    void set_profile_velocity(float velocity_deg_per_sec);               // 🏃 设置轮廓速度
    void set_profile_acceleration(float acceleration_deg_per_sec2);      // ⬆️ 设置轮廓加速度
    void set_profile_deceleration(float deceleration_deg_per_sec2);      // ⬇️ 设置轮廓减速度
    void set_profile_parameters(float velocity_deg_per_sec, float acceleration_deg_per_sec2, float deceleration_deg_per_sec2);  // 📏 设置轮廓参数
    void set_control_word(uint16_t control_word);                        // 🎮 设置控制字
    void set_target_velocity(int32_t velocity_units_per_sec);            // 🎯 设置目标速度
    void go_to_position(float angle);                                    // 📍 移动到指定位置
    void set_velocity(float velocity_deg_per_sec);                       // 🏃 设置速度 (SDO)
    void set_velocity_pdo(float velocity_deg_per_sec);                   // 🚀 设置速度 (PDO)
    
    // ==================== 🤖 ROS2 接口函数 ====================
    void publish_status();                                               // 📊 发布状态
    void position_callback(const std_msgs::msg::Float32::SharedPtr msg); // 📍 位置回调
    void velocity_callback(const std_msgs::msg::Float32::SharedPtr msg); // 🏃 速度回调
    void handle_start(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);   // ▶️ 处理启动请求
    void handle_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);    // ⏹️ 处理停止请求
    void handle_reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);   // 🔄 处理复位请求
    void handle_set_mode(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response); // 🎛️ 处理模式设置请求
    
    // ==================== 🧮 工具函数 ====================
    int32_t angle_to_position(float angle);                              // 📐 角度转换为位置命令单位
    float position_to_angle(int32_t position);                           // 📐 位置命令单位转换为角度
    int32_t velocity_to_units(float velocity_deg_per_sec);               // 📐 速度转换为命令单位
    float units_to_velocity(int32_t velocity_units_per_sec);             // 📐 命令单位转换为速度
    int32_t acceleration_to_units(float acceleration_deg_per_sec2);      // 📐 加速度转换为命令单位
    
    // ⚙️ 电子齿轮比计算 - 返回 {Numerator, Denominator} 对应 0x6091:01 和 0x6091:02
    std::pair<uint32_t, uint32_t> calculate_gear_ratio_params(float gear_ratio, int32_t target_units_per_rev);
    
    // 🚨 错误处理
    void check_and_clear_error();

    // ==================== 📦 成员变量 ====================
    std::string can_interface_;                    // 📡 CAN接口名称 (e.g., "can0")
    uint8_t node_id_;                              // 🆔 CANopen节点ID
    float gear_ratio_ = 1.0;                       // ⚙️ 物理减速比
    int32_t target_units_per_rev_ = 10000;         // 📊 每输出轴圈的命令单位数 (默认: 10,000)
    float units_per_degree_ = 0.0;                 // 📐 缓存: 每度的命令单位数
    float degrees_per_unit_ = 0.0;                 // 📐 缓存: 每命令单位的度数
    int can_socket_ = -1;                          // 🔌 CAN套接字文件描述符
    uint16_t status_word_ = 0;                     // 📊 当前状态字
    int32_t position_ = 0;                         // 📍 当前位置 (命令单位)
    int32_t velocity_ = 0;                         // 🏃 当前速度 (命令单位)
    
    // ==================== 🤖 ROS2 接口 ====================
    rclcpp::TimerBase::SharedPtr timer_;                                    // ⏱️ CAN帧接收定时器
    rclcpp::TimerBase::SharedPtr status_timer_;                             // ⏱️ 状态发布定时器
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;        // 📤 状态发布器
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_pub_;     // 📤 位置发布器
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;     // 📤 速度发布器
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_sub_;  // 📥 位置订阅器
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_sub_;  // 📥 速度订阅器
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;      // ▶️ 启动服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;       // ⏹️ 停止服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;      // 🔄 复位服务
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_mode_service_;   // 🎛️ 模式设置服务
    
    // ==================== 🔒 SDO 等待机制 ====================
    std::mutex sdo_mutex_;                         // 🔐 保护SDO响应变量的互斥锁
    volatile bool sdo_response_received_ = false;  // 📩 SDO响应接收标志
    uint16_t expected_sdo_index_ = 0;              // 🎯 期望的SDO索引
    uint8_t expected_sdo_subindex_ = 0;            // 🎯 期望的SDO子索引
    int32_t sdo_read_value_ = 0;                   // 📊 SDO读取值
};

#endif // CANOPEN_ROS2_NODE_HPP

