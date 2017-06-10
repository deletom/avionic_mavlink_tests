/**
 SERVER
 */
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h> /* close */
#include <netdb.h> /* gethostbyname */
#include "../lib/mavlink/standard/mavlink.h"

#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket(s) close(s)
typedef int SOCKET;
typedef struct sockaddr_in SOCKADDR_IN;
typedef struct sockaddr SOCKADDR;
typedef struct in_addr IN_ADDR;

#define CRLF  "\r\n"
#define PORT  1977

#define BUFFER_LENGTH 2041

int main(void) {
    SOCKET sock;
    SOCKADDR_IN sin = {0};
    SOCKADDR_IN from = {0};

    socklen_t fromsize = sizeof from;
    ssize_t recsize;

    unsigned int temp = 0;

    char buffer[BUFFER_LENGTH];
    char bufferReturn[BUFFER_LENGTH];
    int i = 0;

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == INVALID_SOCKET) {
        perror("socket()");
        exit(errno);
    }

    sin.sin_addr.s_addr = htonl(INADDR_ANY);
    sin.sin_family = AF_INET;
    sin.sin_port = htons(PORT);

    if (bind(sock, (SOCKADDR *) & sin, sizeof sin) == SOCKET_ERROR) {
        perror("bind()");
        exit(errno);
    }

    while (1) {

        memset(buffer, 0, BUFFER_LENGTH);

        if ((recsize = recvfrom(sock, buffer, sizeof buffer - 1, 0, (SOCKADDR *) & from, &fromsize)) < 0) {
            perror("recvfrom()");
            exit(errno);
        }

        mavlink_message_t msg;
        mavlink_status_t status;

        printf("Bytes Received: %d\nDatagram: ", (int) recsize);
        for (i = 0; i < recsize; ++i) {
            temp = buffer[i];
            printf("%02x ", (unsigned char) temp);
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                // Packet received
                printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
            }
        }
        printf("\n");

        bufferReturn[0] = 'O';
        bufferReturn[1] = 'K';

        if (sendto(sock, bufferReturn, strlen(bufferReturn), 0, (SOCKADDR *) & from, fromsize) < 0) {
            perror("sendto()");
            exit(errno);
        }
    }


    closesocket(sock);

    return EXIT_SUCCESS;
}


