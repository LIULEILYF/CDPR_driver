// This driver is based loosely on the ur_modern_driver by Thomas Timm Andersen

#include "ros/ros.h"
#include <br_driver/br_robot.h>
#include <thread>

int main(int argc, char **argv) {

    bool use_sim_time = false;
    std::string host;
    int reverse_port;

    ros::init(argc, argv, "br_driver");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(3);
    int restart_connection=1; // if this parameter is 1, connection will restart automatically

    if (ros::param::get("use_sim_time", use_sim_time)) {
        ROS_WARN("use_sim_time is set!!");
    }

    if (!(ros::param::get("~robot_ip_address", host))) {
        if (argc > 1) {
            ROS_WARN(
                        "Please set the parameter robot_ip_address instead of by command line"
                        );
            host = argv[1];
        }
        else {
            ROS_ERROR(
                        "Could not get robot ip. Supply it on parameter server as robot_ip"
                        );
            exit(1);
        }

    }

    if ((ros::param::get("~/commuinication_port", reverse_port))) {
        if((reverse_port <= 0) or (reverse_port >= 65535)) {
            ROS_WARN(
                        "Reverse port value is not valid (Use number between 1 and 65534"
                        );
        }
    }
    else
    {
        ROS_WARN(
                    "No port given default to 50001"
                    );
        reverse_port = 50001;
    }

    BRrobot interface(nh,host, reverse_port);
    spinner.start();


    while(ros::ok())
    {
//        ROS_INFO("in loop");

        switch (interface.GetStatus()) {
        case interface.PENDING_CONNECTION:
            if(!interface.startCommuinication())
            {
                ROS_ERROR("Ros_ driver : Error on startup");
            }
            break;
        case interface.CONNECTED:
            //ROS_INFO("Connected");
            break;
        case interface.CONNECTING:
            ROS_INFO("Connecting to Client");
            break;
        case interface.DISCONNECTING:
            ROS_INFO("Disconnecting Client");
            interface.halt(restart_connection);
            break;
        case interface.DISCONNECTED:
            ROS_INFO("Disconnected Client");
            ros::Duration(0.1).sleep();
            break;
        default:
            ROS_INFO("Default case");
            break;
        }



    }






    ros::waitForShutdown();

    interface.halt(0);

    exit(0);


}
