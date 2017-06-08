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
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket(s) close(s)
typedef int SOCKET;
typedef struct sockaddr_in SOCKADDR_IN;
typedef struct sockaddr SOCKADDR;
typedef struct in_addr IN_ADDR;

#define CRLF	 "\r\n"
#define PORT	 1977

#define BUF_SIZE 1024

int main(void)
{
    SOCKET sock;
    SOCKADDR_IN sin = { 0 };
    SOCKADDR_IN from = { 0 };
    
    socklen_t fromsize = sizeof from;
    
    char buffer[1024];
    int n = 0;
    
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock == INVALID_SOCKET)
    {
        perror("socket()");
        exit(errno);
    }

    sin.sin_addr.s_addr = htonl(INADDR_ANY);
    sin.sin_family = AF_INET;
    sin.sin_port = htons(PORT);

    if(bind (sock, (SOCKADDR *) &sin, sizeof sin) == SOCKET_ERROR)
    {
        perror("bind()");
        exit(errno);
    }
    
    while(1) {
    
        if((n = recvfrom(sock, buffer, sizeof buffer - 1, 0, (SOCKADDR *)&from, &fromsize)) < 0)
        {
            perror("recvfrom()");
            exit(errno);
        }

        buffer[n] = '\0';

        printf("buffer: %s \r\n", buffer);



        if(sendto(sock, buffer, strlen(buffer), 0, (SOCKADDR *)&from, fromsize) < 0)
        {
            perror("sendto()");
            exit(errno);
        }
    }
    
    
    closesocket(sock);

   return EXIT_SUCCESS;
}


