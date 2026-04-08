#ifndef CANOPEN_ROS2_NODE_HPP
#define CANOPEN_ROS2_NODE_HPP

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
#include <utility>  // For std::pair

// CANopen COB-ID base values
#define COB_NMT      0x000
#define COB_SYNC     0x080
#define COB_RPDO1    0x200
#define COB_RPDO2    0x300
#define COB_RSDO     0x600
#define COB_TSDO     0x580
#define COB_TPDO1    0x180

// NMT commands
#define NMT_START_REMOTE_NODE    0x01
#define NMT_STOP_REMOTE_NODE     0x02
#define NMT_RESET_NODE           0x81
#define NMT_RESET_COMM           0x82

// CiA402 control word
#define CONTROL_SHUTDOWN         0x06
#define CONTROL_SWITCH_ON        0x07
#define CONTROL_ENABLE_OPERATION 0x0F
#define CONTROL_DISABLE_VOLTAGE  0x00
#define CONTROL_FAULT_RESET      0x80
#define CONTROL_NEW_SET_POINT    0x10  // Bit 4 for new set point

// CiA402 operation mode
#define MODE_PROFILE_POSITION    1
// #define MODE_VELOCITY         2  <-- REMOVE THIS (Not supported by DSY-C.eds)
#define MODE_PROFILE_VELOCITY    3  // <-- USE THIS for Speed Control
#define MODE_PROFILE_TORQUE      4
#define MODE_HOMING              6  // Supported by your EDS
#define MODE_INTERPOLATED_POS    7  // Supported by your EDS

// Object dictionary index
#define OD_CYCLE_PERIOD          0x1006 // Renamed from SYNC_MANAGER for accuracy
#define OD_CONTROL_WORD          0x6040
#define OD_STATUS_WORD           0x6041
#define OD_OPERATION_MODE        0x6060
#define OD_OPERATION_MODE_DISPLAY 0x6061
#define OD_TARGET_POSITION       0x607A
#define OD_TARGET_VELOCITY       0x60FF
#define OD_ACTUAL_POSITION       0x6064
#define OD_ACTUAL_VELOCITY       0x606C
#define OD_PROFILE_VELOCITY      0x6081
#define OD_PROFILE_ACCELERATION  0x6083
#define OD_PROFILE_DECELERATION  0x6084

// 1. Safety limits (Defined in EDS)
#define OD_MAX_MOTOR_SPEED       0x6080 // EDS Default: 5000 RPM
#define OD_MAX_PROFILE_VELOCITY  0x607F // EDS Default: 600,000

// 2. Electronic gear ratio (Crucial for correct speed/position units)
// Many DSY motors need these set to 1:1 explicitly if not default
#define OD_GEAR_RATIO            0x6091

// 3. Save parameters
// If you change settings, you might need to save them to EEPROM
#define OD_STORE_PARAMETERS      0x1010
#define STORE_SIGNATURE          0x65766173 // ASCII for "save"


class CANopenROS2 : public rclcpp::Node
{
public:
    CANopenROS2();
    ~CANopenROS2();

private:
    // CAN Communication functions
    void init_can_socket();
    void send_nmt_command(uint8_t command);
    void send_sync_frame();
    void write_sdo(uint16_t index, uint8_t subindex, int32_t data, uint8_t size);
    int32_t read_sdo(uint16_t index, uint8_t subindex);
    void receive_can_frames();
    
    // Motor control functions
    void initialize_node();
    void configure_pdo();
    void start_node();
    void set_immediate_effect(bool immediate);
    void clear_fault();
    void enable_motor();
    void stop_motor();
    void initialize_motor();
    void set_operation_mode(uint8_t mode);
    void set_profile_velocity(float velocity_deg_per_sec);
    void set_profile_acceleration(float acceleration_deg_per_sec2);
    void set_profile_deceleration(float deceleration_deg_per_sec2);
    void set_profile_parameters(float velocity_deg_per_sec, float acceleration_deg_per_sec2, float deceleration_deg_per_sec2);
    void set_control_word(uint16_t control_word);
    void set_target_velocity(int32_t velocity_units_per_sec);
    void go_to_position(float angle);
    void set_velocity(float velocity_deg_per_sec);
    void set_velocity_pdo(float velocity_deg_per_sec);
    
    // ROS 2 interface functions
    void publish_status();
    void position_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void handle_start(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void handle_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void handle_reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void handle_set_mode(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    
    // Utility functions
    int32_t angle_to_position(float angle);
    float position_to_angle(int32_t position);
    int32_t velocity_to_units(float velocity_deg_per_sec);
    int32_t acceleration_to_units(float acceleration_deg_per_sec2);
    
    // Electronic Gear Ratio Calculation
    // Returns {Numerator, Denominator} pair for 0x6091:01 and 0x6091:02
    std::pair<uint32_t, uint32_t> calculate_gear_ratio_params(float gear_ratio, int32_t target_units_per_rev);
    
    // Error handling
    void check_and_clear_error();

    // Member variables
    std::string can_interface_;
    uint8_t node_id_;
    float gear_ratio_ = 1.0;
    int32_t target_units_per_rev_ = 10000;  // Default: 10,000 units per output shaft revolution
    float units_per_degree_ = 0.0;  // Cached: gear_ratio_ / target_units_per_rev_ / 360.0
    float degrees_per_unit_ = 0.0;  // Cached: target_units_per_rev_ / gear_ratio_ * 360.0
    bool auto_start_ = true;
    float profile_velocity_ = 30.0;         // deg/s
    float profile_acceleration_ = 30.0;     // deg/s²
    float profile_deceleration_ = 30.0;     // deg/s²
    int32_t cycle_period_us_ = 1000;        // Communication cycle period (microseconds)
    int can_socket_ = -1;
    uint16_t status_word_ = 0;
    int32_t position_ = 0;
    int32_t prev_position_ = 0;
    rclcpp::Time prev_position_time_;
    bool sync_producer_ = false;
    int sync_period_ms_ = 20;
    
    // ROS 2 interfaces
    rclcpp::CallbackGroup::SharedPtr status_cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr sync_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_mode_service_;
    
    // SDO waiting mechanism
    std::mutex sdo_mutex_;  // Protect SDO response variable
    volatile bool sdo_response_received_ = false;
    uint16_t expected_sdo_index_ = 0;
    uint8_t expected_sdo_subindex_ = 0;
    int32_t sdo_read_value_ = 0;
};

#endif // CANOPEN_ROS2_NODE_HPP

