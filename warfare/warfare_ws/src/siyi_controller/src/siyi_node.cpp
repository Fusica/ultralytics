#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define RECV_BUUF_SIZE 64
#define SERVER_PORT 37260
#define SERVER_IP "192.168.1.25"


bool parse_attitude_data(unsigned char *data, int len, double &roll, double &pitch, double &yaw)
{
    if (len < 22)
    {
        ROS_ERROR("Data length too short to parse.");
        return false;
    }

    // 只解析 RPY 数据
    int16_t yaw_raw = (int16_t)(data[8] | (data[9] << 8));
    int16_t pitch_raw = (int16_t)(data[10] | (data[11] << 8));
    int16_t roll_raw = (int16_t)(data[12] | (data[13] << 8));

    // 转换为实际角度值
    roll = roll_raw / 10.0;
    pitch = pitch_raw / 10.0;
    yaw = yaw_raw / 10.0;

    return true;
}

class SiyiController {
private:
    ros::NodeHandle nh_;
    ros::Publisher rpy_pub_;
    int sockfd_;
    struct sockaddr_in server_addr_;

public:
    SiyiController() : nh_("~") {
        rpy_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/siyi/rpy", 10);

        if ((sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            ROS_ERROR("Failed to create socket: %s", strerror(errno));
            return;
        }

        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_addr.s_addr = inet_addr(SERVER_IP);
        server_addr_.sin_port = htons(SERVER_PORT);

        // Send initial command
        unsigned char init_cmd[] = {
            0x55, 0x66, 0x01, 0x04, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x6a, 0xff, 0xf9, 0x3d};

        if (sendto(sockfd_, init_cmd, sizeof(init_cmd), 0,
                  (struct sockaddr *)&server_addr_, sizeof(server_addr_)) < 0) {
            ROS_ERROR("Failed to send initial command: %s", strerror(errno));
        }
    }

    ~SiyiController() {
        if (sockfd_ >= 0) {
            close(sockfd_);
        }
    }

    void spin() {
        ros::Rate loop_rate(50);
        unsigned char send_buf[] = {0x55, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x0d, 0xe8, 0x05};
        unsigned char recv_buf[RECV_BUUF_SIZE] = {0};
        socklen_t addr_len = sizeof(struct sockaddr_in);

        while (ros::ok()) {
            if (sendto(sockfd_, send_buf, sizeof(send_buf), 0,
                      (struct sockaddr *)&server_addr_, addr_len) < 0) {
                ROS_ERROR("Failed to send request: %s", strerror(errno));
                continue;
            }

            int recv_len = recvfrom(sockfd_, recv_buf, RECV_BUUF_SIZE, 0,
                                  (struct sockaddr *)&server_addr_, &addr_len);

            if (recv_len > 0) {
                double roll, pitch, yaw;
                if (parse_attitude_data(recv_buf, recv_len, roll, pitch, yaw)) {
                    std_msgs::Float64MultiArray rpy_msg;
                    rpy_msg.data = {roll, pitch, yaw};
                    rpy_pub_.publish(rpy_msg);
                }
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "siyi_controller");
    SiyiController controller;
    controller.spin();
    return 0;
}
