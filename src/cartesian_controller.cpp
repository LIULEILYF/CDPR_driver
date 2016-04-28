// This file subscibes to a position and converts this into a joint velocity which is publish to the driver

// a callback from asking for Cartesian position

#include <ros/ros.h>
#include <stdlib.h>     /* srand, rand */
#include "sensor_msgs/JointState.h"


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <visp/vpHomogeneousMatrix.h>

#include <br_driver/CartesianTrajectory.h>
#include <br_driver/CartesianTrajectoryPoint.h>

#include <br_driver/controller_class.h>


// Function to load attachment points


int main(int argc, char **argv) {
    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh;
    ros::init(argc, argv, "Controller");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(10);
    int number_of_cables;

    nh.getParam("number_of_cables",number_of_cables);

    controller_class CableRobot(nh,number_of_cables);

    sensor_msgs::JointState current_joint_position,desired_joint_position;
    desired_joint_position.name.push_back("q1");
    desired_joint_position.name.push_back("q4");
    desired_joint_position.name.push_back("q2");
    desired_joint_position.name.push_back("q5");
    desired_joint_position.name.push_back("q3");
    desired_joint_position.name.push_back("q6");
    desired_joint_position.name.push_back("q7");
    desired_joint_position.name.push_back("q8");

    desired_joint_position.position.resize(8);
    current_joint_position=desired_joint_position;

    ros::Publisher joint_deviation_publisher=
            nh.advertise<sensor_msgs::JointState>("joint_deviation",1);

    //Transform of attachment points w.r.t to platform centre in platform frame

    double ratio=9/3.1428;  // 360/Pi/Diameter

    tf::TransformListener tflistener;
    tf::StampedTransform tfstam;

    ros::Time now = ros::Time(0);

    tflistener.waitForTransform("world","platform",
                                now, ros::Duration(4.0));

    tflistener.lookupTransform("world","platform",
                               now,tfstam);

    vpTranslationVector trans(tfstam.getOrigin().getX(),
                              tfstam.getOrigin().getY(),
                              tfstam.getOrigin().getZ());
    vpQuaternionVector quat(tfstam.getRotation().getX(),
                            tfstam.getRotation().getY(),
                            tfstam.getRotation().getZ(),
                            tfstam.getRotation().getW());

    // actually this is a desired value that comes from trajectory
    vpHomogeneousMatrix wTp(trans,quat);
    vpHomogeneousMatrix wTp_desired;
    std::vector<double> cable_length;
    std::vector<double> cable_length_desired;
    std::vector<double> dq(number_of_cables);
    CableRobot.SetPlatformFrameName("Cartesian_Controller_Frame");

    cable_length=CableRobot.calculate_cable_length(wTp);


    // *************************

    // Two loops one outer that waits for desired transform and checks that its is different to other

    // Second loop plots a simply trajectory between the two and sends a desired joint position to the robot
    // Loop exits upon completion or cancel flag
    // Loop details :
    //      Check current joint position
    //      Update transformation using last known and current joint position
    //      Define new desired position


    while(ros::ok())
    {
        if(CableRobot.GetDesiredTransformFlag()) // I have recieved a desired position
         {
            // Check if the desired position is far away from initial position
            CableRobot.GetDesiredPlatformTransformation(wTp_desired);
            CableRobot.printfM(wTp_desired,"wTp_desired= ");
        }

    r.sleep();

    }
    return 0;
}



