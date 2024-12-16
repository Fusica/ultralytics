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

void parse_attitude_data(unsigned char *data, int len)
{
    if (len < 22)
    {
        fprintf(stderr, "Error: Data length too short to parse.\n");
        return;
    }

    // 跳过前8个字节 (STX + CTRL + Data_len + SEQ + CMD_ID)
    // 从第9个字节(索引8)开始是实际数据
    int16_t yaw = (int16_t)(data[8] | (data[9] << 8));
    int16_t pitch = (int16_t)(data[10] | (data[11] << 8));
    int16_t roll = (int16_t)(data[12] | (data[13] << 8));
    int16_t yaw_velocity = (int16_t)(data[14] | (data[15] << 8));
    int16_t pitch_velocity = (int16_t)(data[16] | (data[17] << 8));
    int16_t roll_velocity = (int16_t)(data[18] | (data[19] << 8));

    // 数据单位为 0.1° 或 0.1°/s，需除以 10 转为实际值
    printf("Yaw: %.1f°, Pitch: %.1f°, Roll: %.1f°\n", 
           yaw / 10.0, pitch / 10.0, roll / 10.0);
    printf("Yaw Velocity: %.1f°/s, Pitch Velocity: %.1f°/s, Roll Velocity: %.1f°/s\n",
           yaw_velocity / 10.0, pitch_velocity / 10.0, roll_velocity / 10.0);
}

int main(int argc, char *argv[])
{
    int sockfd;
    int ret, i, recv_len;
    struct sockaddr_in send_addr, recv_addr;
    unsigned char send_buf[] = {0x55, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x0d, 0xe8, 0x05}; // 对应功能的帧协议,十六进制数据
    unsigned char recv_buf[RECV_BUUF_SIZE] = {0};

    /* 创建 UDP 套接字
        AF_INET: ipv4 地址
        SOCK_DGRAM: UDP 协议
        0: 自动选择类型对应的默认协议
    */

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket");
        exit(1);
    }

    /* 设置云台相机的 ip 和端口号
        sin_family: ipv4 地址
        sin_addr.s_addr: 云台相机 IP 地址
        sin_port： 云台相机端口号
    */
    memset(&send_addr, 0, sizeof(send_addr));
    send_addr.sin_family = AF_INET;
    send_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    send_addr.sin_port = htons(SERVER_PORT);

    /* 发送帧数据
        sockfd: socket 套接字文件描述符
        send_buf： 要发送的数据在内存中的首地址
        sizeof(send_buf)： 要发送的数据的长度
        0： 发送标志，一般为 0
        (struct sockaddr *)&send_addr: 数据接收端的地址（包含 IP 地址和端口号） 的结构体指针
        addr_len: 数据接收端地址结构体的大小
    */

    printf("Send HEX data\n");

    socklen_t addr_len = sizeof(struct sockaddr_in);

    if (sendto(sockfd, send_buf, sizeof(send_buf), 0, (struct sockaddr *)&send_addr, addr_len) < 0)

    {
        perror("sendto");
        exit(1);
    }

    /* 接收云台相机的返回数据
        sockfd: sockfd 套接字文件描述符
        recv_buf: 接收到的数据存放在内存中的位置
        RECV_BUUF_SIZE: 指 buf 缓冲区的大小，即期望接收的最大数据的长度
        0: 接收标志，一般为 0
        (struct sockaddr *)&recv_addr: 指向的结构体将被数据发送端的地址（含 IP 地址和端口号）所填充
        &addr_len: 所指的存储位置，调用前应填入 src_addr 和 addrlen 的结构体大小，调用后则将被填入发送端的地址的实际大小
    */

    recv_len = recvfrom(sockfd, recv_buf, RECV_BUUF_SIZE, 0, (struct sockaddr *)&recv_addr, &addr_len);
    if (recv_len < 0)
    {
        perror("recvfrom");
        exit(1);
    }

    // 在接收到数据后添加解析
    if (recv_len > 0)
    {
        printf("Received HEX data: ");
        for (int i = 0; i < recv_len; i++)
        {
            printf("%02x ", recv_buf[i]);
        }
        printf("\n");

        // 解析接收到的数据
        parse_attitude_data(recv_buf, recv_len);
    }

    // 关闭套接字
    close(sockfd);

    return 0;
}