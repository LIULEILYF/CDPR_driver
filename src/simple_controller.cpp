// This file subscibes to a position and converts this into a joint velocity which is publish to the driver

// a callback from asking for Cartesian position

#include <ros/ros.h>
#include <stdlib.h>     /* srand, rand */
#include "sensor_msgs/JointState.h"


int main(int argc, char **argv) {

    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(200);

    sensor_msgs::JointState joint_state_out,joint_deviation;

    joint_deviation.name.push_back("q1");
    joint_deviation.name.push_back("q4");
    joint_deviation.name.push_back("q2");
    joint_deviation.name.push_back("q5");
    joint_deviation.name.push_back("q3");
    joint_deviation.name.push_back("q6");
    joint_deviation.name.push_back("q7");
    joint_deviation.name.push_back("q8");

    joint_deviation.position.push_back(1.1);
    joint_deviation.position.push_back(4.4);
    joint_deviation.position.push_back(2.2);
    joint_deviation.position.push_back(-5.55555);
    joint_deviation.position.push_back(3);
    joint_deviation.position.push_back(6.666);
    joint_deviation.position.push_back(77);
    joint_deviation.position.push_back(80.0);

    ros::Publisher joint_deviation_publisher=nh.advertise<sensor_msgs::JointState>("joint_deviation",1);


    while(ros::ok())
    {


        joint_deviation_publisher.publish(joint_deviation);

        ros::spinOnce();
        r.sleep();

    }

    return 0;
}
