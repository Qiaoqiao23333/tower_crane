#ifndef CANOPEN_ROS2_NODE_HPP
#define CANOPEN_ROS2_NODE_HPP

/**
 * @file canopen_ros2_node.hpp
 * @brief 🏗️ Tower crane CANopen ROS2 control node header file
 * @details Defines CANopen protocol related macros, CiA402 state machine control words, object dictionary indices, etc.
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

// ==================== 📡 CANopen COB-ID base values ====================
#define COB_NMT      0x000   // 🎛️ Network Management (NMT)
#define COB_SYNC     0x080   // 🔄 Sync frame
#define COB_RPDO1    0x200   // 📥 Receive PDO1
#define COB_RPDO2    0x300   // 📥 Receive PDO2
#define COB_RSDO     0x600   // 📥 Receive SDO (client->server)
#define COB_TSDO     0x580   // 📤 Transmit SDO (server->client)
#define COB_TPDO1    0x180   // 📤 Transmit PDO1

// ==================== 🎮 NMT Network Management commands ====================
#define NMT_START_REMOTE_NODE    0x01   // ▶️ Start remote node
#define NMT_STOP_REMOTE_NODE     0x02   // ⏹️ Stop remote node
#define NMT_RESET_NODE           0x81   // 🔄 Reset node
#define NMT_RESET_COMM           0x82   // 🔄 Reset communication

// ==================== ⚙️ CiA402 Control words ====================
#define CONTROL_SHUTDOWN         0x06   // 🔌 Shutdown (prepare to switch to Switch On)
#define CONTROL_SWITCH_ON        0x07   // ⚡ Switch on (switched on but not enabled)
#define CONTROL_ENABLE_OPERATION 0x0F   // ✅ Enable operation (motor can run)
#define CONTROL_DISABLE_VOLTAGE  0x00   // 🔴 Disable voltage
#define CONTROL_FAULT_RESET      0x80   // 🚨 Fault reset
#define CONTROL_NEW_SET_POINT    0x10   // 🎯 New setpoint (bit 4, trigger position movement)

// ==================== 🚀 CiA402 Operation modes ====================
#define MODE_PROFILE_POSITION    1      // 📍 Profile position mode
// #define MODE_VELOCITY         2      // ❌ Not supported (not defined in DSY-C.eds)
#define MODE_PROFILE_VELOCITY    3      // 🏃 Profile velocity mode (for velocity control)
#define MODE_PROFILE_TORQUE      4      // 💪 Profile torque mode
#define MODE_HOMING              6      // 🏠 Homing mode
#define MODE_INTERPOLATED_POS    7      // 📐 Interpolated position mode

// ==================== 📚 Object dictionary indices ====================
#define OD_CYCLE_PERIOD          0x1006 // ⏱️ Communication cycle (microseconds)
#define OD_CONTROL_WORD          0x6040 // 🎮 Control word
#define OD_STATUS_WORD           0x6041 // 📊 Status word
#define OD_OPERATION_MODE        0x6060 // 🔧 Operation mode (write)
#define OD_OPERATION_MODE_DISPLAY 0x6061 // 👁️ Operation mode display (read-only)
#define OD_TARGET_POSITION       0x607A // 🎯 Target position
#define OD_POSITION_RANGE_LIMIT  0x607B // 📐 Position range limit (sub1=max, sub2=min)
#define OD_TARGET_VELOCITY       0x60FF // 🏃 Target velocity
#define OD_ACTUAL_POSITION       0x6064 // 📍 Actual position
#define OD_ACTUAL_VELOCITY       0x606C // 📈 Actual velocity
#define OD_PROFILE_VELOCITY      0x6081 // 🚀 Profile velocity
#define OD_PROFILE_ACCELERATION  0x6083 // ⬆️ Profile acceleration
#define OD_PROFILE_DECELERATION  0x6084 // ⬇️ Profile deceleration

// ==================== 🛡️ Safety limits (defined in EDS) ====================
#define OD_MAX_MOTOR_SPEED       0x6080 // 🔒 Maximum motor speed (default: 5000 RPM)
#define OD_MAX_PROFILE_VELOCITY  0x607F // 🔒 Maximum profile velocity (default: 600,000)

// ==================== ⚙️ Electronic gear ratio ====================
// Many DSY motors need to be explicitly set to 1:1 (if not default)
#define OD_GEAR_RATIO            0x6091 // ⚙️ Gear ratio (sub-index 01=numerator, 02=denominator)

// ==================== 💾 Save parameters ====================
// If settings are modified, may need to save to EEPROM
#define OD_STORE_PARAMETERS      0x1010 // 💾 Store parameters
#define STORE_SIGNATURE          0x65766173 // 🔑 Save signature (ASCII: "save")


/**
 * @class CANopenROS2
 * @brief 🏗️ Tower crane CANopen ROS2 control node class
 * @details Implements CANopen protocol communication, CiA402 motor control, ROS2 interface, etc.
 */
class CANopenROS2 : public rclcpp::Node
{
public:
    CANopenROS2();   // 🛠️ Constructor
    ~CANopenROS2();  // 🗑️ Destructor

private:
    // ==================== 📡 CAN communication functions ====================
    void init_can_socket();                                              // 🔌 Initialize CAN socket
    void send_nmt_command(uint8_t command);                              // 🎮 Send NMT command
    void send_sync_frame();                                              // 🔄 Send sync frame
    void write_sdo(uint16_t index, uint8_t subindex, int32_t data, uint8_t size);  // ✏️ Write SDO
    int32_t read_sdo(uint16_t index, uint8_t subindex);                  // 📖 Read SDO
    void receive_can_frames();                                           // 📥 Receive CAN frames
    
    // ==================== ⚙️ Motor control functions ====================
    void initialize_node();                                              // 🚀 Initialize node
    void configure_pdo();                                                // 📋 Configure PDO mapping
    void start_node();                                                   // ▶️ Start node
    void set_immediate_effect(bool immediate);                           // ⚡ Set immediate effect
    void clear_fault();                                                  // 🚨 Clear fault
    void enable_motor();                                                 // ✅ Enable motor
    void stop_motor();                                                   // ⏹️ Stop motor
    void initialize_motor();                                             // 🔧 Initialize motor
    void set_operation_mode(uint8_t mode);                               // 🎛️ Set operation mode
    void set_profile_velocity(float velocity_deg_per_sec);               // 🏃 Set profile velocity
    void set_profile_acceleration(float acceleration_deg_per_sec2);      // ⬆️ Set profile acceleration
    void set_profile_deceleration(float deceleration_deg_per_sec2);      // ⬇️ Set profile deceleration
    void set_position_range_limit(int32_t max_val, int32_t min_val);         // 📐 Set position range limit (0x607B)
    void set_profile_parameters(float velocity_deg_per_sec, float acceleration_deg_per_sec2, float deceleration_deg_per_sec2);  // 📏 Set profile parameters
    void set_control_word(uint16_t control_word);                        // 🎮 Set control word
    void set_target_velocity(int32_t velocity_units_per_sec);            // 🎯 Set target velocity
    void go_to_position(float angle);                                    // 📍 Move to specified position
    void set_velocity(float velocity_deg_per_sec);                       // 🏃 Set velocity (SDO)
    void set_velocity_pdo(float velocity_deg_per_sec);                   // 🚀 Set velocity (PDO)
    
    // ==================== 🤖 ROS2 interface functions ====================
    void publish_status();                                               // 📊 Publish status
    void position_callback(const std_msgs::msg::Float32::SharedPtr msg); // 📍 Position callback
    void velocity_callback(const std_msgs::msg::Float32::SharedPtr msg); // 🏃 Velocity callback
    void handle_start(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);   // ▶️ Handle start request
    void handle_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);    // ⏹️ Handle stop request
    void handle_reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);   // 🔄 Handle reset request
    void handle_set_mode(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response); // 🎛️ Handle mode set request
    
    // ==================== 🧮 Utility functions ====================
    int32_t angle_to_position(float angle);                              // 📐 Convert angle to position command units
    float position_to_angle(int32_t position);                           // 📐 Convert position command units to angle
    int32_t velocity_to_units(float velocity_deg_per_sec);               // 📐 Convert velocity to command units
    float units_to_velocity(int32_t velocity_units_per_sec);             // 📐 Convert command units to velocity
    int32_t acceleration_to_units(float acceleration_deg_per_sec2);      // 📐 Convert acceleration to command units
    
    // ⚙️ Electronic gear ratio calculation - returns {Numerator, Denominator} corresponding to 0x6091:01 and 0x6091:02
    std::pair<uint32_t, uint32_t> calculate_gear_ratio_params(float gear_ratio, int32_t target_units_per_rev);
    
    // 🚨 Error handling
    void check_and_clear_error();

    // ==================== 📦 Member variables ====================
    std::string can_interface_;                    // 📡 CAN interface name (e.g., "can0")
    uint8_t node_id_;                              // 🆔 CANopen node ID
    float gear_ratio_ = 1.0;                       // ⚙️ Physical gear reduction ratio
    int32_t target_units_per_rev_ = 10000;         // 📊 Command units per output shaft revolution (default: 10,000)
    float units_per_degree_ = 0.0;                 // 📐 Cache: command units per degree
    float degrees_per_unit_ = 0.0;                 // 📐 Cache: degrees per command unit
    float profile_velocity_ = 30.0;                // 🏃 Default profile velocity (°/s)
    float profile_acceleration_ = 30.0;            // ⬆️ Default profile acceleration (°/s²)
    float profile_deceleration_ = 30.0;            // ⬇️ Default profile deceleration (°/s²)
    int32_t position_range_limit_max_ = 2147483647;   // 📐 Max position range limit (0x607B:01) [position units]
    int32_t position_range_limit_min_ = -2147483648;  // 📐 Min position range limit (0x607B:02) [position units]
    int can_socket_ = -1;                          // 🔌 CAN socket file descriptor
    uint16_t status_word_ = 0;                     // 📊 Current status word
    int32_t position_ = 0;                         // 📍 Current position (command units)
    int32_t velocity_ = 0;                         // 🏃 Current velocity (command units)
    
    // ==================== 🤖 ROS2 interface ====================
    rclcpp::TimerBase::SharedPtr timer_;                                    // ⏱️ CAN frame receive timer
    rclcpp::TimerBase::SharedPtr status_timer_;                             // ⏱️ Status publish timer
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;        // 📤 Status publisher
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_pub_;     // 📤 Position publisher
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;     // 📤 Velocity publisher
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_sub_;  // 📥 Position subscriber
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_sub_;  // 📥 Velocity subscriber
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;      // ▶️ Start service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;       // ⏹️ Stop service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;      // 🔄 Reset service
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_mode_service_;   // 🎛️ Mode set service
    
    // ==================== 🔒 SDO wait mechanism ====================
    std::mutex sdo_mutex_;                         // 🔐 Mutex protecting SDO response variables
    volatile bool sdo_response_received_ = false;  // 📩 SDO response received flag
    uint16_t expected_sdo_index_ = 0;              // 🎯 Expected SDO index
    uint8_t expected_sdo_subindex_ = 0;            // 🎯 Expected SDO sub-index
    int32_t sdo_read_value_ = 0;                   // 📊 SDO read value
};

#endif // CANOPEN_ROS2_NODE_HPP

