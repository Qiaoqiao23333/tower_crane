#include "crane_master/canopen_ros2_node.hpp"
#include <errno.h>

void CANopenROS2::init_can_socket()
{
    // 创建套接字
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "无法创建CAN套接字");
        return;
    }
    
    // 获取接口索引
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface_.c_str());
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "无法获取CAN接口索引");
        close(can_socket_);
        can_socket_ = -1;
        return;
    }
    
    // 绑定套接字
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "无法绑定CAN套接字");
        close(can_socket_);
        can_socket_ = -1;
        return;
    }
    
    // 设置非阻塞模式
    int flags = fcntl(can_socket_, F_GETFL, 0);
    if (flags < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "无法获取套接字标志");
        close(can_socket_);
        can_socket_ = -1;
        return;
    }
    
    flags |= O_NONBLOCK;
    if (fcntl(can_socket_, F_SETFL, flags) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "无法设置非阻塞模式");
        close(can_socket_);
        can_socket_ = -1;
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "CAN套接字初始化成功");
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
        RCLCPP_ERROR(this->get_logger(), "发送NMT命令失败");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "NMT命令已发送: 0x%02X", command);
    }
}

void CANopenROS2::send_sync_frame()
{
    struct can_frame frame;
    frame.can_id = COB_SYNC;
    frame.can_dlc = 0;
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "发送同步帧失败");
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "同步帧已发送");
    }
}

void CANopenROS2::write_sdo(uint16_t index, uint8_t subindex, int32_t data, uint8_t size)
{
    struct can_frame frame;
    frame.can_id = COB_RSDO + node_id_;
    frame.can_dlc = 8;
    
    // 命令字节
    uint8_t command = 0x22;  // 下载请求
    if (size == 1)
    {
        command |= 0x0F;  // 1字节
    }
    else if (size == 2)
    {
        command |= 0x0B;  // 2字节
    }
    else if (size == 4)
    {
        command |= 0x03;  // 4字节
    }
    
    frame.data[0] = command;
    frame.data[1] = index & 0xFF;  // 索引低字节
    frame.data[2] = (index >> 8) & 0xFF;  // 索引高字节
    frame.data[3] = subindex;  // 子索引
    frame.data[4] = data & 0xFF;  // 数据低字节
    frame.data[5] = (data >> 8) & 0xFF;
    frame.data[6] = (data >> 16) & 0xFF;
    frame.data[7] = (data >> 24) & 0xFF;  // 数据高字节
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "写入SDO失败: 索引=0x%04X, 子索引=0x%02X", index, subindex);
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "SDO已写入: 索引=0x%04X, 子索引=0x%02X, 数据=0x%08X", index, subindex, data);
    }
}

int32_t CANopenROS2::read_sdo(uint16_t index, uint8_t subindex)
{
    struct can_frame frame;
    frame.can_id = COB_RSDO + node_id_;
    frame.can_dlc = 8;
    frame.data[0] = 0x40;  // 上传请求
    frame.data[1] = index & 0xFF;  // 索引低字节
    frame.data[2] = (index >> 8) & 0xFF;  // 索引高字节
    frame.data[3] = subindex;  // 子索引
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    
    // 重置标志和期望的索引
    sdo_response_received_ = false;
    expected_sdo_index_ = index;
    expected_sdo_subindex_ = subindex;
    sdo_read_value_ = 0;
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "读取SDO请求失败: 索引=0x%04X, 子索引=0x%02X", index, subindex);
        return -1;
    }
    
    // 等待响应，最多等待200ms
    int retry = 0;
    const int max_retries = 20;  // 20 * 10ms = 200ms
    while (retry < max_retries && !sdo_response_received_)
    {
        // 在等待期间，需要处理接收到的CAN帧
        receive_can_frames();
        
        if (sdo_response_received_)
        {
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        retry++;
    }
    
    if (!sdo_response_received_)
    {
        RCLCPP_WARN(this->get_logger(), "读取SDO超时: 索引=0x%04X, 子索引=0x%02X", index, subindex);
        return 0;
    }
    
    return sdo_read_value_;
}

void CANopenROS2::receive_can_frames()
{
    struct can_frame frame;
    ssize_t nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
    
    if (nbytes < 0)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            RCLCPP_ERROR(this->get_logger(), "接收CAN帧失败: %s", strerror(errno));
        }
        return;
    }
    
    // 处理接收到的CAN帧
    uint32_t cob_id = frame.can_id & 0x780;  // 提取功能码
    uint8_t node_id = frame.can_id & 0x7F;  // 提取节点ID
    
    if (node_id != node_id_)
    {
        return;  // 不是我们关心的节点
    }
    
    RCLCPP_DEBUG(this->get_logger(), "接收到CAN帧: ID=0x%03X, DLC=%d, Data=0x%02X%02X%02X%02X%02X%02X%02X%02X",
        frame.can_id, frame.can_dlc,
        frame.data[0], frame.data[1], frame.data[2], frame.data[3],
        frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
    
    // 检测SDO响应 (COB-ID = 0x580 + node_id)
    if (frame.can_id == (COB_TSDO + node_id_))
    {
        // 处理SDO响应
        uint8_t command = frame.data[0];
        uint16_t index = frame.data[1] | (frame.data[2] << 8);
        uint8_t subindex = frame.data[3];
        
        if (command == 0x80)  // SDO中止
        {
            uint32_t abort_code = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);
            RCLCPP_ERROR(this->get_logger(), "SDO中止: 索引=0x%04X, 子索引=0x%02X, 错误码=0x%08X", index, subindex, abort_code);
            
            // 如果是我们正在等待的SDO响应
            if (!sdo_response_received_ && index == expected_sdo_index_ && subindex == expected_sdo_subindex_)
            {
                sdo_read_value_ = 0;  // 错误时返回0
                sdo_response_received_ = true;
            }
        }
        else 
        {
            // 提取数据 (根据SDO响应格式，数据在字节4-7)
            int32_t data = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);
            
            // 如果是我们正在等待的SDO响应，设置标志和数据
            if (!sdo_response_received_ && index == expected_sdo_index_ && subindex == expected_sdo_subindex_)
            {
                sdo_read_value_ = data;
                sdo_response_received_ = true;
            }
            
            // 同时也处理特定的SDO更新（用于状态监控）
            if (index == OD_STATUS_WORD && subindex == 0x00)  // 状态字
            {
                uint16_t status_word = data & 0xFFFF;
                status_word_ = status_word;
                
                // 检查目标到达位
                if (status_word & 0x0400)
                {
                    RCLCPP_INFO(this->get_logger(), "目标位置已到达");
                }
            }
            else if (index == OD_ACTUAL_POSITION && subindex == 0x00)  // 实际位置
            {
                position_ = data;
                float angle = position_to_angle(position_);
                
                // 发布位置（检查发布器是否已初始化）
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
        // 处理TPDO1响应
        if (frame.can_dlc >= 6)  // 状态字(2字节) + 实际位置(4字节)
        {
            uint16_t status_word = frame.data[0] | (frame.data[1] << 8);
            int32_t position = frame.data[2] | (frame.data[3] << 8) | (frame.data[4] << 16) | (frame.data[5] << 24);
            
            status_word_ = status_word;
            position_ = position;
            
            float angle = position_to_angle(position);
            
            // 发布位置（检查发布器是否已初始化）
            if (position_pub_)
            {
                auto pos_msg = std_msgs::msg::Float32();
                pos_msg.data = angle;
                position_pub_->publish(pos_msg);
            }
            
            // 检查目标到达位
            if (status_word & 0x0400)
            {
                RCLCPP_INFO(this->get_logger(), "目标位置已到达");
            }
        }
    }
}

