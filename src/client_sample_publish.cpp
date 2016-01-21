#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <iostream>

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int main(int argc, char *argv[])
{
    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256];
    if (argc < 3) {
        fprintf(stderr,"usage %s hostname port\n", argv[0]);
        exit(0);
    }
    portno = atoi(argv[2]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");
    server = gethostbyname(argv[1]);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
        error("ERROR connecting");


    int i=0;
    char const * re;
    while(1)
    {
        switch (i % 4) {
        case 0:
            std::cout<<" Case 0 "<<std::endl;
            re="<Position M1=\"0.01\" M2=\"0.02\" M3=\"0.03\" M4=\"0.04\" M5=\"0.05\" M6=\"0.06\" M7=\"0.07\" M8=\"0.08\"/>\n\0";
            break;
        case 1:
            std::cout<<" Case 1 "<<std::endl;
            re="<Velocity M1=\"0.1\" M4=\"0.4\" M5=\"0.5\" M6=\"0.6\" M7=\"0.7\" M2=\"0.2\" M3=\"0.3\"  M8=\"0.8\"/>\n\0";
            break;
        case 2:
            std::cout<<" Case 2 "<<std::endl;
             re="<Torque M1=\"1\" M2=\"2\" M3=\"3\" M4=\"4\" M5=\"5\" M6=\"6\" M7=\"7\" M8=\"8\"/>\n\0";
            break;
          default:
            break;
        }
        std::cout<<"i = "<<i<<" i % 3"<<i % 3<<std::endl;

        n = write(sockfd,re,strlen(re));
        if (n < 0)
            error("ERROR writing to socket");
        bzero(buffer,256);
        n = read(sockfd,buffer,255);
        if (n < 0)
            error("ERROR reading from socket");
        printf("%s\n",buffer);
        sleep(1);
        i++;

    }
    close(sockfd);
    return 0;
}
