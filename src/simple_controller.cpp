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



int main(int argc, char **argv) {

    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(200);
    double ratio=9/3.1428;  // 360/Pi/Diameter

    int number_of_cables;
    nh.getParam("number_of_cables",number_of_cables);

    controller_class CableRobot(nh,number_of_cables);

    sensor_msgs::JointState desired_joint_state,current_joint_state;



    ros::Publisher desired_joint_position_publisher=
            nh.advertise<sensor_msgs::JointState>("desired_joint_position",1);



    //Transform of attachment points w.r.t to platform centre in platform frame
    vpHomogeneousMatrix wTp,wTp_desired,pTp_desired;


    std::vector<double> joint_deviation{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};



    tf::TransformListener tflistener;
    tf::StampedTransform tfstam;

    vpTranslationVector trans;
    vpQuaternionVector quat;

    ros::Time now = ros::Time(0);
    CableRobot.GetPlatformTransformation(wTp); //Current frame
    CableRobot.printfM(wTp,"wTp");

    while(ros::ok())
    {


        if(CableRobot.GetJointFlag())
        {
            tflistener.waitForTransform("world","desired_platform",
                                        now, ros::Duration(4.0));
            try{
            tflistener.lookupTransform("world","desired_platform",
                                       now,tfstam);

            trans.buildFrom(tfstam.getOrigin().getX(),
                            tfstam.getOrigin().getY(),
                            tfstam.getOrigin().getZ());
            quat.buildFrom(tfstam.getRotation().getX(),
                           tfstam.getRotation().getY(),
                           tfstam.getRotation().getZ(),
                           tfstam.getRotation().getW());
            // actually this is a desired value that comes from trajectory
            pTp_desired.buildFrom(trans,quat); // difference in world frame
            CableRobot.GetPlatformTransformation(wTp); //Current frame
            wTp_desired=wTp*pTp_desired; //desired
            CableRobot.printfM(wTp_desired,"wTp_desired");
             }
            catch(tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                    wTp_desired=wTp;
              }

            CableRobot.GetRobotJointState(current_joint_state);
            CableRobot.GetRobotJointState(desired_joint_state);
            joint_deviation=CableRobot.calculate_motor_change(wTp_desired,ratio);
            for (int i = 0; i < current_joint_state.position.size(); ++i) {
                desired_joint_state.position[i]+=joint_deviation[i];
            }
            CableRobot.UpdatePlatformTransformation(wTp_desired);
        }

        // Only publish if the joint state is full
        if(desired_joint_state.position.size()==8)
        {
        desired_joint_position_publisher.publish(desired_joint_state);
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;

}

