#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <iostream>
#include <tinyxml.h>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include "boost/lexical_cast.hpp"


void error(const char *msg)
{
    perror(msg);
    exit(0);
}

float RandomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}


std::vector<double> q{-0.87,-71.98,-116.38,-80.73,90.10,131.21,-54.0,10.131};
std::vector<double> dq{0.001,0.001,0.003,-0.001,0.005,0.001,0.0012,-0.0045};

std::string pack_message()
{


    std::string message;
    float floatvalue;
    TiXmlDocument doc;
    TiXmlElement* ROS_PLC=new TiXmlElement("PLC_ROS");
    TiXmlElement* Cables_xml=new TiXmlElement("Cables_FB");

    doc.LinkEndChild(ROS_PLC);
    ROS_PLC->LinkEndChild(Cables_xml);

    for (int i = 0; i < 8; ++i) {
        std::string element_name="Q"+
                boost::lexical_cast<std::string>(i+1);
        TiXmlElement* Qi=new TiXmlElement(element_name);
        TiXmlElement* Position=new TiXmlElement("Position");
        TiXmlElement* Velocity=new TiXmlElement("Vitesse");
        TiXmlElement* Torque=new TiXmlElement("Couple");
        floatvalue=RandomFloat(q[i]-dq[i],q[i]+1.5*dq[i]);
        q[i]=floatvalue;
        Position->SetAttribute("V",boost::lexical_cast<std::string>(floatvalue));
        floatvalue=RandomFloat(-3.0,3.0);
        Velocity->SetAttribute("V",boost::lexical_cast<std::string>(floatvalue));
        floatvalue=RandomFloat(-10.0,10.0);
        Torque->SetAttribute("V",boost::lexical_cast<std::string>(floatvalue));
        Cables_xml->LinkEndChild(Qi);
        Qi->LinkEndChild(Position);
        Qi->LinkEndChild(Velocity);
        Qi->LinkEndChild(Torque);
    }


    TiXmlPrinter printer;
    //doc.Print();
    doc.Accept( &printer );
    message = printer.CStr();
    //std::cout<<"message "<<message<<std::endl;

    return message;
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



    char const * re;
    std::string msg;
    while(1)
    {
        msg="";
        msg=pack_message();
        msg=msg+"\0";
        re=msg.c_str();
        std::cout<<strlen(re)<<std::endl;
        n = write(sockfd,re,strlen(re));

        std::cout<<"Writing to socket"<<msg<<std::endl;
        if (n < 0)
            error("ERROR writing to socket");
        bzero(buffer,10000);
        //n = read(sockfd,buffer,255);
        //if (n < 0)
          //  error("ERROR reading from socket");

        //usleep(5000);
        usleep(10000);


    }
    close(sockfd);
    return 0;
}
