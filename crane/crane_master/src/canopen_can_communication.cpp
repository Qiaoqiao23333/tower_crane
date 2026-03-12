#include "crane_master/canopen_ros2_node.hpp"
#include <errno.h>

void CANopenROS2::init_can_socket()
{
    // Create socket
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "😩 Failed to create CAN socket");
        return;
    }
    
    // Get interface index
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface_.c_str());
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "😩 Failed to get CAN interface index");
        close(can_socket_);
        can_socket_ = -1;
        return;
    }
    
    // Bind socket
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "😩 Failed to bind CAN socket");
        close(can_socket_);
        can_socket_ = -1;
        return;
    }
    
    // Set non-blocking mode
    int flags = fcntl(can_socket_, F_GETFL, 0);
    if (flags < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "😩 Failed to get socket flags");
        close(can_socket_);
        can_socket_ = -1;
        return;
    }
    
    flags |= O_NONBLOCK;
    if (fcntl(can_socket_, F_SETFL, flags) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "😩 Failed to set non-blocking mode");
        close(can_socket_);
        can_socket_ = -1;
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "👍👍👍 CAN socket initialized successfully");
}

void CANopenROS2::send_nmt_command(uint8_t command)
{
    struct can_frame frame;
    frame.can_id = COB_NMT;
    frame.can_dlc = 2;
    frame.data[0] = command;
    frame.data[1] = node_id_;
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "😩 Failed to send NMT command");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "⏩ NMT command sent: 0x%02X", command);
    }
}

void CANopenROS2::send_sync_frame()
{
    struct can_frame frame;
    frame.can_id = COB_SYNC;
    frame.can_dlc = 0;
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "😩 Failed to send sync frame");
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "🔊🔊🔊 Sync frame sent");
    }
}

void CANopenROS2::write_sdo(uint16_t index, uint8_t subindex, int32_t data, uint8_t size)
{
    struct can_frame frame;
    frame.can_id = COB_RSDO + node_id_;
    frame.can_dlc = 8;
    
    // Command byte
    uint8_t command = 0x22;  // download request
    if (size == 1)
    {
        command |= 0x0F;  // 1 byte
    }
    else if (size == 2)
    {
        command |= 0x0B;  // 2 bytes
    }
    else if (size == 4)
    {
        command |= 0x03;  // 4 bytes
    }
    
    frame.data[0] = command;
    frame.data[1] = index & 0xFF;  // index low byte
    frame.data[2] = (index >> 8) & 0xFF;  // index high byte
    frame.data[3] = subindex;  // subindex
    frame.data[4] = data & 0xFF;  // data low byte
    frame.data[5] = (data >> 8) & 0xFF;
    frame.data[6] = (data >> 16) & 0xFF;
    frame.data[7] = (data >> 24) & 0xFF;  // data high byte
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "😩 Failed to write SDO [Node ID=%d]: Index=0x%04X, Subindex=0x%02X", node_id_, index, subindex);
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "🤝🤝🤝 SDO written [Node ID=%d]: Index=0x%04X, Subindex=0x%02X, Data=0x%08X", node_id_, index, subindex, data);
    }
}

int32_t CANopenROS2::read_sdo(uint16_t index, uint8_t subindex)
{
    struct can_frame frame;
    frame.can_id = COB_RSDO + node_id_;
    frame.can_dlc = 8;
    frame.data[0] = 0x40;  // upload request
    frame.data[1] = index & 0xFF;  // index low byte
    frame.data[2] = (index >> 8) & 0xFF;  // index high byte
    frame.data[3] = subindex;  // subindex
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    
    // Protect shared variables with mutex, reset flags and expected index
    {
        std::lock_guard<std::mutex> lock(sdo_mutex_);
        sdo_response_received_ = false;
        expected_sdo_index_ = index;
        expected_sdo_subindex_ = subindex;
        sdo_read_value_ = 0;
    }
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "😩 Failed to send SDO read request [Node ID=%d]: Index=0x%04X, Subindex=0x%02X", node_id_, index, subindex);
        return -1;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "🤝🤝🤝 Sent SDO read request [Node ID=%d]: Index=0x%04X, Subindex=0x%02X, waiting for response COB-ID=0x%03X", 
                 node_id_, index, subindex, COB_TSDO + node_id_);
    
    // Wait for response, up to 500 ms
    int retry = 0;
    const int max_retries = 50;  // 50 * 10ms = 500ms
    bool response_received = false;
    
    while (retry < max_retries)
    {
        // Within each loop, try reading multiple times to increase chance of catching response
        for (int i = 0; i < 20; i++)
        {
            receive_can_frames();
            
            // Check whether response received (protected by mutex)
            {
                std::lock_guard<std::mutex> lock(sdo_mutex_);
                if (sdo_response_received_)
                {
                    response_received = true;
                    break;
                }
            }
            
            if (response_received)
                break;
        }
        
        if (response_received)
        {
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        retry++;
    }
    
    if (!response_received)
    {
        RCLCPP_WARN(this->get_logger(), "😅 SDO read timeout [Node ID=%d]: Index=0x%04X, Subindex=0x%02X (waited %d times, total %d ms)", 
                   node_id_, index, subindex, retry, retry * 10);
        return 0;
    }
    
    // Read final value with mutex protection
    int32_t result;
    {
        std::lock_guard<std::mutex> lock(sdo_mutex_);
        result = sdo_read_value_;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "🤝🤝🤝 SDO read successful [Node ID=%d]: Index=0x%04X, Subindex=0x%02X, Value=0x%08X (%d)", 
                 node_id_, index, subindex, result, result);
    
    return result;
}

void CANopenROS2::receive_can_frames()
{
    struct can_frame frame;
    ssize_t nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
    
    if (nbytes < 0)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            RCLCPP_ERROR(this->get_logger(), "😩 Failed to receive CAN frame: %s", strerror(errno));
        }
        return;
    }
    
    // Process received CAN frame
    uint32_t cob_id = frame.can_id & 0x780;  // function code
    uint8_t node_id = frame.can_id & 0x7F;   // node ID
    
    if (node_id != node_id_)
    {
        return;  // not our node
    }
    
    RCLCPP_DEBUG(this->get_logger(), "🔊🔊🔊 Received CAN frame: ID=0x%03X, DLC=%d, Data=0x%02X%02X%02X%02X%02X%02X%02X%02X",
        frame.can_id, frame.can_dlc,
        frame.data[0], frame.data[1], frame.data[2], frame.data[3],
        frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
    
    // Detect SDO response (COB-ID = 0x580 + node_id)
    if (frame.can_id == static_cast<canid_t>(COB_TSDO + node_id_))
    {
        // Handle SDO response
        uint8_t command = frame.data[0];
        uint16_t index = frame.data[1] | (frame.data[2] << 8);
        uint8_t subindex = frame.data[3];
        
        RCLCPP_DEBUG(this->get_logger(), "🤝🤝🤝 Received SDO response [Node ID=%d]: COB-ID=0x%03X, Command=0x%02X, Index=0x%04X, Subindex=0x%02X", 
                     node_id_, frame.can_id, command, index, subindex);
        
        // Protect shared variables with mutex
        std::lock_guard<std::mutex> lock(sdo_mutex_);
        
        if (command == 0x80)  // SDO abort
        {
            uint32_t abort_code = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);
            RCLCPP_ERROR(this->get_logger(), "😩 SDO abort [Node ID=%d]: Index=0x%04X, Subindex=0x%02X, Error code=0x%08X", node_id_, index, subindex, abort_code);
            
            // If this is the SDO response we are waiting for
            if (!sdo_response_received_ && index == expected_sdo_index_ && subindex == expected_sdo_subindex_)
            {
                sdo_read_value_ = 0;  // return 0 on error
                sdo_response_received_ = true;
                RCLCPP_DEBUG(this->get_logger(), "🤝🤝🤝 SDO abort response matched, setting flag");
            }
        }
        else 
        {
            // Extract data (for SDO response data is in bytes 4–7)
            int32_t data = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);
            
            // If this is the response we expect, set flag and data
            if (!sdo_response_received_ && index == expected_sdo_index_ && subindex == expected_sdo_subindex_)
            {
                sdo_read_value_ = data;
                sdo_response_received_ = true;
                RCLCPP_DEBUG(this->get_logger(), "🤝🤝🤝 SDO response matched, setting flag and data: 0x%08X (%d)", data, data);
            }
            
            // Also handle specific SDO updates for status monitoring
            if (index == OD_STATUS_WORD && subindex == 0x00)  // status word
            {
                uint16_t status_word = data & 0xFFFF;
                status_word_ = status_word;
                
                // Check target reached bit
                if (status_word & 0x0400)
                {
                    RCLCPP_INFO(this->get_logger(), "🎯 Target position reached");
                }
            }
            else if (index == OD_ACTUAL_POSITION && subindex == 0x00)  // actual position
            {
                position_ = data;
                float angle = position_to_angle(position_);
                
                // Publish position (if publisher is initialized)
                if (position_pub_)
                {
                    auto msg = std_msgs::msg::Float32();
                    msg.data = angle;
                    position_pub_->publish(msg);
                }
            }
        }
    }
    else if (cob_id == COB_TPDO1)
    {
        // Handle TPDO1 response
        if (frame.can_dlc >= 6)  // status word (2 bytes) + actual position (4 bytes)
        {
            uint16_t status_word = frame.data[0] | (frame.data[1] << 8);
            int32_t position = frame.data[2] | (frame.data[3] << 8) | (frame.data[4] << 16) | (frame.data[5] << 24);
            
            status_word_ = status_word;
            position_ = position;
            
            float angle = position_to_angle(position);
            
            // Publish position (if publisher is initialized)
            if (position_pub_)
            {
                auto pos_msg = std_msgs::msg::Float32();
                pos_msg.data = angle;
                position_pub_->publish(pos_msg);
            }
            
            // Check target reached bit
            if (status_word & 0x0400)
            {
                RCLCPP_INFO(this->get_logger(), "🎯 Target position reached");
            }
        }
    }
}

