#include "crane_master/canopen_ros2_node.hpp"
#include <errno.h>

/**
 * @brief 🔌 初始化 CAN 套接字
 * @details 创建、配置并绑定 CAN 套接字，设置非阻塞模式
 */
void CANopenROS2::init_can_socket()
{
    // 🔧 创建 CAN 套接字
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 无法创建 CAN 套接字");
        return;
    }
    
    // 🔍 获取接口索引
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface_.c_str());
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 无法获取 CAN 接口索引");
        close(can_socket_);
        can_socket_ = -1;
        return;
    }
    
    // 🔗 绑定套接字
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 无法绑定 CAN 套接字");
        close(can_socket_);
        can_socket_ = -1;
        return;
    }
    
    // ⚡ 设置非阻塞模式
    int flags = fcntl(can_socket_, F_GETFL, 0);
    if (flags < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 无法获取套接字标志");
        close(can_socket_);
        can_socket_ = -1;
        return;
    }
    
    flags |= O_NONBLOCK;
    if (fcntl(can_socket_, F_SETFL, flags) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 无法设置非阻塞模式");
        close(can_socket_);
        can_socket_ = -1;
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ CAN 套接字初始化成功");
}

/**
 * @brief 🎮 发送 NMT 命令
 * @param command NMT 命令码 (如 NMT_START_REMOTE_NODE, NMT_RESET_NODE 等)
 */
void CANopenROS2::send_nmt_command(uint8_t command)
{
    struct can_frame frame;
    frame.can_id = COB_NMT;
    frame.can_dlc = 2;
    frame.data[0] = command;
    frame.data[1] = node_id_;
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 发送 NMT 命令失败");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "🎮 NMT 命令已发送: 0x%02X", command);
    }
}

/**
 * @brief 🔄 发送同步帧
 * @details CANopen 同步帧 (COB-ID 0x080)，用于同步 PDO 通信
 */
void CANopenROS2::send_sync_frame()
{
    struct can_frame frame;
    frame.can_id = COB_SYNC;
    frame.can_dlc = 0;
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 发送同步帧失败");
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "🔄 同步帧已发送");
    }
}

/**
 * @brief ✏️ 写入 SDO (服务数据对象)
 * @param index 对象字典索引
 * @param subindex 子索引
 * @param data 要写入的数据
 * @param size 数据大小 (字节数: 1, 2, 或 4)
 */
void CANopenROS2::write_sdo(uint16_t index, uint8_t subindex, int32_t data, uint8_t size)
{
    struct can_frame frame;
    frame.can_id = COB_RSDO + node_id_;
    frame.can_dlc = 8;
    
    // 📝 命令字节 (SDO 下载请求)
    uint8_t command = 0x22;  // 下载请求基础值
    if (size == 1)
    {
        command |= 0x0F;  // 1字节: 设置 n=3, e=1, s=1
    }
    else if (size == 2)
    {
        command |= 0x0B;  // 2字节: 设置 n=2, e=1, s=1
    }
    else if (size == 4)
    {
        command |= 0x03;  // 4字节: 设置 n=0, e=1, s=1
    }
    
    frame.data[0] = command;
    frame.data[1] = index & 0xFF;         // 📍 索引低字节
    frame.data[2] = (index >> 8) & 0xFF;  // 📍 索引高字节
    frame.data[3] = subindex;             // 📍 子索引
    frame.data[4] = data & 0xFF;          // 📦 数据字节0 (最低位)
    frame.data[5] = (data >> 8) & 0xFF;   // 📦 数据字节1
    frame.data[6] = (data >> 16) & 0xFF;  // 📦 数据字节2
    frame.data[7] = (data >> 24) & 0xFF;  // 📦 数据字节3 (最高位)
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 写入 SDO 失败 [节点%d]: 索引=0x%04X, 子索引=0x%02X", 
                     node_id_, index, subindex);
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "✏️ SDO 已写入 [节点%d]: 索引=0x%04X, 子索引=0x%02X, 数据=0x%08X", 
                     node_id_, index, subindex, data);
    }
}

/**
 * @brief 📖 读取 SDO (服务数据对象)
 * @param index 对象字典索引
 * @param subindex 子索引
 * @return 读取到的数据值，失败返回 -1 或 0
 */
int32_t CANopenROS2::read_sdo(uint16_t index, uint8_t subindex)
{
    struct can_frame frame;
    frame.can_id = COB_RSDO + node_id_;
    frame.can_dlc = 8;
    frame.data[0] = 0x40;                 // 📤 SDO 上传请求命令
    frame.data[1] = index & 0xFF;         // 📍 索引低字节
    frame.data[2] = (index >> 8) & 0xFF;  // 📍 索引高字节
    frame.data[3] = subindex;             // 📍 子索引
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    
    // 🔒 使用互斥锁保护共享变量，重置标志和期望的索引
    {
        std::lock_guard<std::mutex> lock(sdo_mutex_);
        sdo_response_received_ = false;
        expected_sdo_index_ = index;
        expected_sdo_subindex_ = subindex;
        sdo_read_value_ = 0;
    }
    
    if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 读取 SDO 请求失败 [节点%d]: 索引=0x%04X, 子索引=0x%02X", 
                     node_id_, index, subindex);
        return -1;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "📤 发送 SDO 读取请求 [节点%d]: 索引=0x%04X, 子索引=0x%02X, 等待响应 COB-ID=0x%03X", 
                 node_id_, index, subindex, COB_TSDO + node_id_);
    
    // ⏳ 等待响应，最多等待 500ms
    int retry = 0;
    const int max_retries = 50;  // 50 * 10ms = 500ms
    bool response_received = false;
    
    while (retry < max_retries)
    {
        // 🔄 在每次循环中多次尝试读取，提高响应捕获概率
        for (int i = 0; i < 20; i++)
        {
            receive_can_frames();
            
            // 🔍 检查响应是否已收到
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
        RCLCPP_WARN(this->get_logger(), "⏰ 读取 SDO 超时 [节点%d]: 索引=0x%04X, 子索引=0x%02X (等待了 %dms)", 
                   node_id_, index, subindex, retry * 10);
        return 0;
    }
    
    // 🔓 使用互斥锁读取最终值
    int32_t result;
    {
        std::lock_guard<std::mutex> lock(sdo_mutex_);
        result = sdo_read_value_;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "✅ SDO 读取成功 [节点%d]: 索引=0x%04X, 子索引=0x%02X, 值=0x%08X (%d)", 
                 node_id_, index, subindex, result, result);
    
    return result;
}

/**
 * @brief 📥 接收并处理 CAN 帧
 * @details 处理 SDO 响应和 TPDO1 反馈，更新状态字和位置信息
 */
void CANopenROS2::receive_can_frames()
{
    struct can_frame frame;
    ssize_t nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
    
    if (nbytes < 0)
    {
        if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ 接收 CAN 帧失败: %s", strerror(errno));
        }
        return;
    }
    
    // 🔍 分析 CAN 帧
    uint32_t cob_id = frame.can_id & 0x780;  // 提取功能码
    uint8_t node_id = frame.can_id & 0x7F;   // 提取节点 ID
    
    if (node_id != node_id_)
    {
        return;  // 🚫 不是我们关心的节点
    }
    
    RCLCPP_DEBUG(this->get_logger(), "📥 接收到 CAN 帧: ID=0x%03X, DLC=%d, Data=0x%02X%02X%02X%02X%02X%02X%02X%02X",
        frame.can_id, frame.can_dlc,
        frame.data[0], frame.data[1], frame.data[2], frame.data[3],
        frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
    
    // 📩 检测 SDO 响应 (COB-ID = 0x580 + node_id)
    if (frame.can_id == static_cast<canid_t>(COB_TSDO + node_id_))
    {
        // 📝 处理 SDO 响应
        uint8_t command = frame.data[0];
        uint16_t index = frame.data[1] | (frame.data[2] << 8);
        uint8_t subindex = frame.data[3];
        
        RCLCPP_DEBUG(this->get_logger(), "📩 收到 SDO 响应 [节点%d]: COB-ID=0x%03X, 命令=0x%02X, 索引=0x%04X, 子索引=0x%02X", 
                     node_id_, frame.can_id, command, index, subindex);
        
        // 🔒 使用互斥锁保护共享变量
        std::lock_guard<std::mutex> lock(sdo_mutex_);
        
        if (command == 0x80)  // 🚨 SDO 中止
        {
            uint32_t abort_code = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);
            RCLCPP_ERROR(this->get_logger(), "🚨 SDO 中止 [节点%d]: 索引=0x%04X, 子索引=0x%02X, 错误码=0x%08X", 
                         node_id_, index, subindex, abort_code);
            
            // 如果是我们正在等待的 SDO 响应
            if (!sdo_response_received_ && index == expected_sdo_index_ && subindex == expected_sdo_subindex_)
            {
                sdo_read_value_ = 0;  // 错误时返回 0
                sdo_response_received_ = true;
                RCLCPP_DEBUG(this->get_logger(), "🎯 SDO 中止响应已匹配，设置标志");
            }
        }
        else 
        {
            // 📦 提取数据 (根据 SDO 响应格式，数据在字节 4-7)
            int32_t data = frame.data[4] | (frame.data[5] << 8) | (frame.data[6] << 16) | (frame.data[7] << 24);
            
            // 如果是我们正在等待的 SDO 响应，设置标志和数据
            if (!sdo_response_received_ && index == expected_sdo_index_ && subindex == expected_sdo_subindex_)
            {
                sdo_read_value_ = data;
                sdo_response_received_ = true;
                RCLCPP_DEBUG(this->get_logger(), "✅ SDO 响应已匹配，设置标志和数据: 0x%08X (%d)", data, data);
            }
            
            // 📊 同时也处理特定的 SDO 更新（用于状态监控）
            if (index == OD_STATUS_WORD && subindex == 0x00)  // 状态字
            {
                uint16_t status_word = data & 0xFFFF;
                status_word_ = status_word;
                
                // 检查目标到达位
                if (status_word & 0x0400)
                {
                    RCLCPP_INFO(this->get_logger(), "🎯 目标位置已到达");
                }
            }
            else if (index == OD_ACTUAL_POSITION && subindex == 0x00)  // 实际位置
            {
                position_ = data;
                float angle = position_to_angle(position_);
                
                // 📤 发布位置
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
        // 📥 处理 TPDO1 响应 (状态字 + 实际位置)
        if (frame.can_dlc >= 6)  // 状态字(2字节) + 实际位置(4字节)
        {
            uint16_t status_word = frame.data[0] | (frame.data[1] << 8);
            int32_t position = frame.data[2] | (frame.data[3] << 8) | (frame.data[4] << 16) | (frame.data[5] << 24);
            
            status_word_ = status_word;
            position_ = position;
            
            float angle = position_to_angle(position);
            
            // 📤 发布位置
            if (position_pub_)
            {
                auto pos_msg = std_msgs::msg::Float32();
                pos_msg.data = angle;
                position_pub_->publish(pos_msg);
            }
            
            // 检查目标到达位
            if (status_word & 0x0400)
            {
                RCLCPP_INFO(this->get_logger(), "🎯 目标位置已到达");
            }
        }
    }
}

