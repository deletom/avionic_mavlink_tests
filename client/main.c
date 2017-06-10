/**
 CLIENT
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

#define BUF_SIZE 1024

int main(void) {
    SOCKET sock;
    int n = 0;
    uint8_t buffer[2041];
    struct hostent *hostinfo = NULL;
    SOCKADDR_IN to = {0};
    const char *hostname = "127.0.0.1";
    socklen_t tosize = sizeof to;
    int indexElementSend = 0;
    mavlink_message_t msg;
    uint16_t len;

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == INVALID_SOCKET) {
        perror("socket()");
        exit(errno);
    }

    hostinfo = gethostbyname(hostname);
    if (hostinfo == NULL) {
        fprintf(stderr, "Unknown host %s.\n", hostname);
        exit(EXIT_FAILURE);
    }

    to.sin_addr = *(IN_ADDR *) hostinfo->h_addr;
    to.sin_port = htons(PORT);
    to.sin_family = AF_INET;

    while (1) {

        /* Envoi Heartbeat */
        mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
        len = mavlink_msg_to_send_buffer(buffer, &msg);

        if (sendto(sock, buffer, len, 0, (SOCKADDR *) & to, tosize) < 0) {
            perror("sendto()");
            exit(errno);
        }

        /* Envoi Statut */
        mavlink_msg_sys_status_pack(1, 200, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
        len = mavlink_msg_to_send_buffer(buffer, &msg);

        if (sendto(sock, buffer, len, 0, (SOCKADDR *) & to, tosize) < 0) {
            perror("sendto()");
            exit(errno);
        }

        /* Reception acquittement du serveur */
        if ((n = recvfrom(sock, buffer, sizeof buffer - 1, 0, (SOCKADDR *) & to, &tosize)) < 0) {
            perror("recvfrom()");
            exit(errno);
        }

        buffer[n] = '\0';

        indexElementSend++;

        printf("buffer: %s \r\n", buffer);
    }

    closesocket(sock);

    return EXIT_SUCCESS;
}


