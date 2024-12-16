#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define RECV_BUUF_SIZE 64
#define SERVER_PORT 37260
#define SERVER_IP "192.168.1.25"

int main(int argc, char *argv[])
{
    int sockfd;
    struct sockaddr_in send_addr;
    // Control command data
    unsigned char send_buf[] = {0x55, 0x66, 0x01, 0x04, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x3e, 0xfe, 0xa3, 0xef};

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket");
        exit(1);
    }

    memset(&send_addr, 0, sizeof(send_addr));
    send_addr.sin_family = AF_INET;
    send_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    send_addr.sin_port = htons(SERVER_PORT);

    // Send control command
    if (sendto(sockfd, send_buf, sizeof(send_buf), 0, (struct sockaddr *)&send_addr, sizeof(struct sockaddr_in)) < 0)
    {
        perror("sendto");
        exit(1);
    }

    close(sockfd);
    return 0;
}
