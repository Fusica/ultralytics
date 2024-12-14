// camera_control_node.cpp

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// 定义UDP发送参数
#define RECV_BUUF_SIZE 64
#define DEFAULT_SERVER_PORT 37260
#define DEFAULT_SERVER_IP "192.168.1.25"

// CRC16表（基于G(X) = X^16+X^12+X^5+1）
const uint16_t crc16_tab[256] =
    {0x0, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
     0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
     0x1231, 0x210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
     0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
     0x2462, 0x3443, 0x420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
     0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
     0x3653, 0x2672, 0x1611, 0x630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
     0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
     0x48c4, 0x58e5, 0x6886, 0x78a7, 0x840, 0x1861, 0x2802, 0x3823,
     0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
     0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0xa50, 0x3a33, 0x2a12,
     0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
     0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0xc60, 0x1c41,
     0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
     0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0xe70,
     0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
     0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
     0x1080, 0xa1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
     0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
     0x2b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
     0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
     0x34e2, 0x24c3, 0x14a0, 0x481, 0x7466, 0x6447, 0x5424, 0x4405,
     0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
     0x26d3, 0x36f2, 0x691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
     0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
     0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x8e1, 0x3882, 0x28a3,
     0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
     0x4a75, 0x5a54, 0x6a37, 0x7a16, 0xaf1, 0x1ad0, 0x2ab3, 0x3a92,
     0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
     0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0xcc1,
     0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
     0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0xed1, 0x1ef0};

// CRC16计算函数
uint16_t CRC16_cal(uint8_t *ptr, uint32_t len, uint16_t crc_init)
{
    uint16_t crc = crc_init;
    while (len-- != 0)
    {
        uint8_t temp = (crc >> 8) & 0xFF;
        crc = (crc << 8) ^ crc16_tab[*ptr++ ^ temp];
    }
    return crc;
}

// 发送UDP控制命令的函数
bool send_udp_command(unsigned char *send_buf, size_t len, const std::string &server_ip, int server_port)
{
    int sockfd;
    struct sockaddr_in send_addr;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket");
        return false;
    }

    memset(&send_addr, 0, sizeof(send_addr));
    send_addr.sin_family = AF_INET;
    send_addr.sin_addr.s_addr = inet_addr(server_ip.c_str());
    send_addr.sin_port = htons(server_port);

    if (sendto(sockfd, send_buf, len, 0, (struct sockaddr *)&send_addr, sizeof(struct sockaddr_in)) < 0)
    {
        perror("sendto");
        close(sockfd);
        return false;
    }

    close(sockfd);
    return true;
}

// 全局变量，用于存储图像中心
int image_center_x = 1920 / 2; // 默认图像宽度为1920像素
int image_center_y = 1080 / 2; // 默认图像高度为1080像素

// 偏航���仰的角度范围
const int YAW_MIN = -135;  // 最小偏航角度
const int YAW_MAX = 135;   // 最大偏航角度
const int PITCH_MIN = -90; // 最小俯仰角度
const int PITCH_MAX = 25;  // 最大俯仰角度

// 添加常量定义
const float DEADZONE_RATIO = 0.3f;     // 死区范围
const float MAX_ADJUST_RATE = 5.0f;    // 最大单次调整速率（度/次）

// 回调函数，接收目标位置信息
void targetCallback(const geometry_msgs::Point::ConstPtr &msg, const std::string &server_ip, int server_port)
{
    // 提取目标位置信息
    int target_x = static_cast<int>(msg->x);
    int target_y = static_cast<int>(msg->y);

    // 计算偏移量（归一化到[-1, 1]）
    float offset_x = (float)(target_x - image_center_x) / image_center_x;
    float offset_y = (float)(target_y - image_center_y) / image_center_y;

    // 检查是否在死区内
    if (fabs(offset_x) <= DEADZONE_RATIO && fabs(offset_y) <= DEADZONE_RATIO) {
        return;
    }

    // 计算完整的需要调整的角度
    float yaw_adjust_f = -offset_x * YAW_MAX;
    float pitch_adjust_f = -offset_y * PITCH_MAX;

    // 限制调整速率（保持运动方向，但限制单次调整的大小）
    if (fabs(yaw_adjust_f) > MAX_ADJUST_RATE) {
        yaw_adjust_f = (yaw_adjust_f > 0) ? MAX_ADJUST_RATE : -MAX_ADJUST_RATE;
    }
    if (fabs(pitch_adjust_f) > MAX_ADJUST_RATE) {
        pitch_adjust_f = (pitch_adjust_f > 0) ? MAX_ADJUST_RATE : -MAX_ADJUST_RATE;
    }

    // 四舍五入并转换为整数
    int yaw_adjust = static_cast<int>(round(yaw_adjust_f));
    int pitch_adjust = static_cast<int>(round(pitch_adjust_f));

    // 限制总角度范围
    yaw_adjust = std::max(YAW_MIN, std::min(YAW_MAX, yaw_adjust));
    pitch_adjust = std::max(PITCH_MIN, std::min(PITCH_MAX, pitch_adjust));

    // 将角度值转换为0.1度单位
    int16_t yaw_adjust_scaled = yaw_adjust * 10;
    int16_t pitch_adjust_scaled = pitch_adjust * 10;

    // 构建控制命令
    unsigned char send_buf[14] = {
        0x55, 0x66,  // 帧头
        0x01, 0x04,  // 命令字
        0x00, 0x00, 0x00, 0x0E,  // 数据长度
        0x00, 0x00,  // yaw
        0x00, 0x00,  // pitch
        0x00, 0x00   // CRC16
    };

    // 小端存储角度值（包括负值的补码）
    send_buf[8] = yaw_adjust_scaled & 0xFF;
    send_buf[9] = (yaw_adjust_scaled >> 8) & 0xFF;
    send_buf[10] = pitch_adjust_scaled & 0xFF;
    send_buf[11] = (pitch_adjust_scaled >> 8) & 0xFF;

    // 计算CRC16并插入到send_buf中（假设CRC16放在最后两个字节）
    // CRC计算时不包括CRC本身的位置，即前12字节
    uint16_t crc_result = CRC16_cal(send_buf, 12, 0);
    send_buf[12] = crc_result & 0xFF;
    send_buf[13] = (crc_result >> 8) & 0xFF;

    // 发送UDP控制命令
    if (send_udp_command(send_buf, sizeof(send_buf), server_ip, server_port))
    {
        ROS_INFO("Control command sent successfully. Yaw: %d, Pitch: %d, CRC: 0x%04X", yaw_adjust, pitch_adjust, crc_result);
    }
    else
    {
        ROS_ERROR("Failed to send control command.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_control_node");
    ros::NodeHandle nh("~"); // 使用私有命名空间，以便通过参数服务器设置参数

    // 从参数服务器获取图像中心坐标（默认为640x480）
    nh.param("image_center_x", image_center_x, image_center_x);
    nh.param("image_center_y", image_center_y, image_center_y);

    // 从参数服务器获取服务器IP和端口
    std::string server_ip;
    int server_port;
    nh.param<std::string>("server_ip", server_ip, DEFAULT_SERVER_IP);
    nh.param("server_port", server_port, DEFAULT_SERVER_PORT);

    // 检查服务器IP和端口的有效性
    if (server_ip.empty())
    {
        ROS_ERROR("Server IP is empty. Please set 'server_ip' parameter.");
        return 1;
    }
    if (server_port <= 0 || server_port > 65535)
    {
        ROS_ERROR("Invalid server port: %d. Please set 'server_port' between 1 and 65535.", server_port);
        return 1;
    }

    // 创建一个带有参数的订阅器
    auto targetCallbackWithParams = [&](const geometry_msgs::Point::ConstPtr &msg)
    {
        targetCallback(msg, server_ip, server_port);
    };

    // 订阅目标位置信息的主题，假设为"/target_position"
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Point>("/pixel_coordinates", 10, targetCallbackWithParams);

    ROS_INFO("Camera Control Node Started.");
    ROS_INFO("Server IP: %s, Port: %d", server_ip.c_str(), server_port);
    ROS_INFO("Image Center: (%d, %d)", image_center_x, image_center_y);
    ROS_INFO("Yaw Range: [%d°, %d°], Pitch Range: [%d°, %d°]", YAW_MIN, YAW_MAX, PITCH_MIN, PITCH_MAX);

    ros::spin();

    return 0;
}
