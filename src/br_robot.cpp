#include "br_driver/br_robot.h"
#include <thread>


#include <sys/socket.h> // Needed for the socket functions
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>


#include <tinyxml.h>


// I need this class to
//      1. open a socket
//      2. provide a mechanism to read and write over said socket
//      3. publish the receved data to defined topics
// Note as a test I can run it with python

BRrobot::BRrobot(ros::NodeHandle nh_, std::string host, int reverse_port) : nh(nh_) ,REVERSE_PORT_(reverse_port)
{
    kill_signal=false;
    debug_=1;
    struct sockaddr_in serv_addr; // a structure containing the socket information
    socklen_t clilen;
    int n, flag;

    incoming_sockfd_ = socket(AF_INET, SOCK_STREAM, 0); // This defines a TCP socket

    if (incoming_sockfd_ < 0)         ROS_ERROR("ERROR opening socket");

    bzero((char *) &serv_addr, sizeof(serv_addr)); // fills the socket address with zeros

    serv_addr.sin_family = AF_INET; // address family: AF_INET
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(REVERSE_PORT_); // port

    flag = 1;

    // The setsockopt function sets the current value for a socket option associated with a socket of any
    setsockopt(incoming_sockfd_,// socket affected
               IPPROTO_TCP, // set option at TCP level
               TCP_NODELAY, // name of option bypasses Nagle's delay
               (char *) &flag, //
               sizeof(int)); // length of option value

    setsockopt(incoming_sockfd_,
               SOL_SOCKET, // Manipulate sockets at socket level
               SO_REUSEADDR, // The SO_REUSEADDR socket option allows a socket to forcibly bind to a port in use by another socket
               &flag,
               sizeof(int));

    if (bind(incoming_sockfd_, (struct sockaddr *) &serv_addr,
             sizeof(serv_addr)) < 0) {
        ROS_ERROR("ERROR on binding socket for reverse communication");
    }
    listen(incoming_sockfd_, 5);
    keepalive_=false;

    joint_sub = nh.subscribe("/desired_joint_position", 1, &BRrobot::JointDeviationCallback, this);
    robot_state_pub=nh.advertise<sensor_msgs::JointState>("/joint_state",1);

    // Names of joints
    robot_state.name.push_back("q1");
    robot_state.name.push_back("q2");
    robot_state.name.push_back("q3");
    robot_state.name.push_back("q4");
    robot_state.name.push_back("q5");
    robot_state.name.push_back("q6");
    robot_state.name.push_back("q7");
    robot_state.name.push_back("q8");

    Motor_Control.resize(8);
    All_Motor_Control=true;


    robot_state.position.resize(8);
    robot_state.velocity.resize(8);
    robot_state.effort.resize(8);

    SetStatus(PENDING_CONNECTION);
}

// Commuinication Functions

bool BRrobot::startCommuinication() // function to start commuinication
{
    keepalive_=true;
    int n;
    struct sockaddr_in cli_addr;
    socklen_t clilen;
    // Put this part in another function
    clilen = sizeof(cli_addr);
    new_sockfd_= accept(incoming_sockfd_,
                        (struct sockaddr *) &cli_addr,
                        &clilen);
    SetStatus(robot_status::CONNECTING);
    if (new_sockfd_< 0){
        ROS_ERROR("ERROR on accept");
        SetStatus(robot_status::DISCONNECTED);
        return false;
    }
    readingThread_ = std::thread(&BRrobot::readData, this);
    writingThread_ = std::thread(&BRrobot::writeData, this);
    statePublisherThread_ = std::thread(&BRrobot::statePublisher, this);

    SetStatus(robot_status::CONNECTED);

    return true;
}

bool BRrobot::halt(int restart=0) // function to halt commuinication
{
    keepalive_=false;
    readingThread_.join();
    writingThread_.join();
    statePublisherThread_.join();
    SetStatus(DISCONNECTED);

    if(restart)
    {
        ROS_INFO("Restarting Connection");
        SetStatus(PENDING_CONNECTION);
    }
    else
    {
        ROS_WARN("Closing Socket");
        close(new_sockfd_);
        close(incoming_sockfd_);
    }

    return true;
}


void BRrobot::readData() // function to start commuinication
{
    int n;
    char buffer[256];
    bzero(buffer, 256);
    struct timeval timeout;
    fd_set readfds; // adds file descriptor to a set
    FD_ZERO(&readfds); // Clears all entries from the set
    FD_SET(new_sockfd_, &readfds); // Adds new_sockfd_ to the set

    while(keepalive_ && ros::ok())
    {

        timeout.tv_sec = 0; //do this each loop as selects modifies timeout
        timeout.tv_usec = 500000; // timeout of 0.5 sec

        select(new_sockfd_ + 1, &readfds, NULL, NULL, &timeout);
        n = read(new_sockfd_,buffer,1500); // this was 1000 and working BR

        if (n < 0)
        {
            ROS_ERROR("ERROR reading from socket");
            SetStatus(robot_status::DISCONNECTING);
        }

        std::string p(buffer);

        //std::cout<<"p "<<p<<std::endl;

        std::vector<std::string> strs;
        boost::algorithm::split_regex(strs, p, boost::regex("</PLC_ROS>"));

        for (int i = 0; i < strs.size()-1; ++i) {
        //    std::cout<<"strs[ "<<i<<" ]= "<<strs[i]<<std::endl;
            unpack_message(strs[i]);
        }

    }
    close(new_sockfd_);
}

//
void BRrobot::unpack_message(std::string p)
{
    TiXmlDocument doc;
    const char* pTest =doc.Parse(p.c_str(), 0, TIXML_ENCODING_UTF8);
    bool bool_value;
    float float_value;

    TiXmlElement* PLC_ROS=doc.FirstChildElement("PLC_ROS");

  //  doc.Print();

    // Looking under the PLC_ROS tag
    if(PLC_ROS)
    {
    //    ROS_INFO("PLC_ROS exists");
        TiXmlElement* Cables_FB=PLC_ROS->FirstChildElement("Cables_FB");
        // Looking under the Cable tag
        if(Cables_FB)
        {
         //  ROS_INFO("Cables_FB exists");
            // Check control tabs
            TiXmlElement* All_Motors_Controlled=PLC_ROS->FirstChildElement("All_Motors_Controlled");
            if(All_Motors_Controlled)
            {
                check_attribute(All_Motors_Controlled->
                                QueryBoolAttribute("V",&bool_value));
                All_Motor_Control=bool_value;
            }
            // For each motor, get q qdot and tau, and control flag
            for (int i = 0; i < 8; ++i) {
                std::string element_name="Q"+
                        boost::lexical_cast<std::string>(i+1);
                TiXmlElement* Qi=Cables_FB->FirstChildElement(element_name);
                if(Qi)
                {
                    TiXmlElement* Controlled=Qi->FirstChildElement("Controlled");
                    TiXmlElement* Position_i=Qi->FirstChildElement("Position");
                    TiXmlElement* Velocity_i=Qi->FirstChildElement("Vitesse");
                    TiXmlElement* Torque_i=Qi->FirstChildElement("Couple");
                    if(Controlled)
                    {
                        check_attribute(Controlled->
                                        QueryBoolAttribute("V",&bool_value));
                        Motor_Control[i]=bool_value;

                    }
                    if(Position_i)
                    {
                       // std::cout<<"Getting position"<<std::endl;
                        check_attribute(Position_i->
                                        QueryFloatAttribute("V",&float_value));
                        robot_state.position[i]=float_value;

                    }
                    if(Velocity_i)
                    {
                        check_attribute(Velocity_i->
                                        QueryFloatAttribute("V",&float_value));
                        robot_state.velocity[i]=float_value;
                    }
                    if(Torque_i)
                    {
                        check_attribute(Torque_i->
                                        QueryFloatAttribute("V",&float_value));
                        robot_state.effort[i]=float_value;
                    }
                }

            }
        }

    }
}


void BRrobot::check_attribute(int attribute)
{
    switch (attribute){
    case TIXML_NO_ATTRIBUTE:
        std::cout<<"TIXML_NO_ATTRIBUTE"<<std::endl;
        break;
    case TIXML_WRONG_TYPE:
        std::cout<<"TIXML_WRONG_TYPE"<<std::endl;
        break;
    case TIXML_SUCCESS:
        break;
    default:
        ROS_INFO("who knows");
    }
}


//void BRrobot::unpack_message(std::string p)
//{

//    TiXmlDocument doc;
//    TiXmlElement* root;
//    const char* pTest =doc.Parse(p.c_str(), 0, TIXML_ENCODING_UTF8);
//    std::vector<std::string> data;
//    data.push_back("Position");
//    data.push_back("Velocity");
//    data.push_back("Torque");

//    for (int i = 0; i < 3; ++i) {
//        root = doc.FirstChildElement( data[i]);
//        if(root)
//        {
//            extract_robot_state(root,data[i]);
//        }
//    }
//}

//void BRrobot::extract_robot_state(TiXmlElement* root,std::string State)
//{
//    double d=-1.0;
//    for (int counter =0; counter < 8; ++counter) {

//        std::string value="q"+boost::lexical_cast<std::string>(counter+1);
//        switch (root->QueryDoubleAttribute(value,&d)) {
//        case TIXML_NO_ATTRIBUTE:
//            std::cout<<"TIXML_NO_ATTRIBUTE"<<std::endl;
//            break;
//        case TIXML_WRONG_TYPE:
//            std::cout<<"TIXML_WRONG_TYPE"<<std::endl;
//            break;
//        case 0:
//            if(State=="Position")
//            {
//                robot_state.position[counter]=d;
//            }
//            else if(State=="Velocity")
//            {
//                robot_state.velocity[counter]=d;
//            }
//            else if(State=="Torque")
//            {
//                robot_state.effort[counter]=d;
//            }
//            else
//            {
//                ROS_ERROR("Error defining data type");
//            }

//            break;
//        default:
//            break;
//        }
//    }
//}


std::string BRrobot::pack_joint_message()
{
    double q[8]; // converted deviation joint position
    double tau[8]; // converted torque
    std::string message=" ";


    if(desired_joint_position.position.size()==robot_state.position.size())
    {
        //ROS_INFO_COND(debug_,"joint name check");

        if(desired_joint_position.name!=robot_state.name)
        {
          //  ROS_INFO_COND(debug_,"In if loop");
            // Assign Joints according to index
            for (int i = 0; i < 8; ++i) {
                for (int j = 0; j < 8; ++j) {
                    if(robot_state.name[i]==desired_joint_position.name[j])
                    {
                        q[i]=desired_joint_position.position[j];
                        tau[i]=desired_joint_position.effort[j];
                    }
                }
            }
        }
        else // Assign Joints in simple manner
        {
            for (int var = 0; var < 8; ++var) {
                q[var]=desired_joint_position.position[var];
                tau[var]=desired_joint_position.effort[var];
            }
        }
      //  ROS_INFO_COND(debug_,"Creating Document");
        float floatvalue;
        TiXmlDocument doc;
        TiXmlElement* ROS_PLC=new TiXmlElement("ROS_PLC");
        TiXmlElement* Cables_xml=new TiXmlElement("Cables");

        doc.LinkEndChild(ROS_PLC);
        ROS_PLC->LinkEndChild(Cables_xml);

        for (int i = 0; i < 8; ++i) {
            std::string element_name="Q"+
                    boost::lexical_cast<std::string>(i+1);
            TiXmlElement* Qi=new TiXmlElement(element_name);
            TiXmlElement* Position=new TiXmlElement("Position");
            TiXmlElement* Torque=new TiXmlElement("Couple");
            floatvalue=q[i];
            Position->SetAttribute("V",boost::lexical_cast<std::string>(floatvalue));
            floatvalue=tau[i];
            Torque->SetAttribute("V",boost::lexical_cast<std::string>(floatvalue));
            Cables_xml->LinkEndChild(Qi);
            Qi->LinkEndChild(Position);
            Qi->LinkEndChild(Torque);
        }


        TiXmlPrinter printer;
        //doc.Print();
        doc.Accept( &printer );
        message = printer.CStr();
        //std::cout<<"message "<<message<<std::endl;
    }
    else
    {
        message="\0";
    }
    return message;

}


void BRrobot::writeData() // function to start commuinication
{
    int n;
    int counter=0;
    char buffer[256];
    bzero(buffer, 256);
    struct timeval timeout;
    fd_set writefds; // adds file descriptor to a set
    FD_ZERO(&writefds); // Clears all entries from the set
    FD_SET(new_sockfd_, &writefds); // Adds new_sockfd_ to the set
    std::string message;
    while(keepalive_ && ros::ok())
    {
        timeout.tv_sec = 0; //do this each loop as selects modifies timeout
        timeout.tv_usec = 500000; // timeout of 0.5 sec

        select(new_sockfd_ + 1, &writefds, NULL, NULL, &timeout);
        boost::lexical_cast<std::string>(counter);
        char const * msg;

        if(GetJointFlag())
        {

            SetJointFlag(false); // Reset the joint flag

            message=pack_joint_message();
            //std::cout<<"Sending="<<message<<std::endl;
            msg=message.c_str();


            n = write(new_sockfd_,msg,strlen(msg));

            if (n < 0)
            {
                ROS_ERROR("ERROR writing to socket");
                SetStatus(robot_status::DISCONNECTING);

            }
        }

    }
    close(new_sockfd_);
}




void BRrobot::statePublisher()
{
    ros::Rate r(200);
    while(keepalive_ && ros::ok())
    {
        robot_state.header.stamp=ros::Time::now();
        robot_state_pub.publish(robot_state);
        r.sleep();
    }
}


void BRrobot::JointDeviationCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    desired_joint_position=*msg; // joint is eqaul to the value pointed to by msg
    this->SetJointFlag(true); // Note that value has been receieved
}

// ========= Get & Set Flags ========= //

bool BRrobot::GetJointFlag()
{
    return jointStateReceived;
}

int BRrobot::GetStatus()
{
    return status;
}

void BRrobot::SetJointFlag(bool Flag)
{
    jointStateReceived=Flag;
}

void BRrobot::SetStatus(int s)
{
    status=s;
}







