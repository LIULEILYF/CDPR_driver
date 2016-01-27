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



// Function to load attachment points


std::vector<vpHomogeneousMatrix> get_attachment_parameters(
        std::string param_name,ros::NodeHandle n);
void printfM(vpHomogeneousMatrix M, const char *intro="Matrix");

std::vector<double> calculate_cable_length(vpHomogeneousMatrix wTp,
                                           std::vector<vpHomogeneousMatrix> pTbi,
                                           std::vector<vpHomogeneousMatrix> wTai);

int main(int argc, char **argv) {

    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(1);
    int number_of_cables;
    nh.getParam("number_of_cables",number_of_cables);

    sensor_msgs::JointState joint_state_out,joint_deviation;
    joint_deviation.name.push_back("q1");
    joint_deviation.name.push_back("q4");
    joint_deviation.name.push_back("q2");
    joint_deviation.name.push_back("q5");
    joint_deviation.name.push_back("q3");
    joint_deviation.name.push_back("q6");
    joint_deviation.name.push_back("q7");
    joint_deviation.name.push_back("q8");

    joint_deviation.position.push_back(0.);
    joint_deviation.position.push_back(0.);
    joint_deviation.position.push_back(0.);
    joint_deviation.position.push_back(0.);
    joint_deviation.position.push_back(0.);
    joint_deviation.position.push_back(0.);
    joint_deviation.position.push_back(0.);
    joint_deviation.position.push_back(0.0);
    ros::Publisher joint_deviation_publisher=
            nh.advertise<sensor_msgs::JointState>("joint_deviation",1);

    //Transform of attachment points w.r.t to platform centre in platform frame

    std::vector<vpHomogeneousMatrix> pTbi(number_of_cables);
    // Transform of attachment points w.r.t world frame
    std::vector<vpHomogeneousMatrix> wTbi(number_of_cables);
    // Transform of base points w.r.t to world frame
    std::vector<vpHomogeneousMatrix> wTai(number_of_cables);
    // Transform of platform points w.r.t to corresponding base
    std::vector<vpHomogeneousMatrix> aiTbi(number_of_cables);
    std::vector<double> cable_length(number_of_cables);
    std::vector<double> cable_length_desired(number_of_cables);
    double ratio=9/3.1428;  // 360/Pi/Diameter

    tf::TransformListener tflistener;
    tf::StampedTransform tfstam;
    wTai=get_attachment_parameters("base_attachment_points",nh);
    pTbi=get_attachment_parameters("platform_attachment_points",nh);

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
    vpTranslationVector L;
    cable_length_desired=calculate_cable_length(wTp,pTbi,wTai);//initialisation

    br_driver::CartesianTrajectoryPoint p1;
    std::vector<br_driver::CartesianTrajectoryPoint> Points;
    p1.time_from_start=0.0;
    p1.desired_pose.pose.position.x=0.0;
    br_driver::CartesianTrajectory traj;

// Ok so eveything is in place to do the actual control
// This program recieves a Cartesian Trajectroy intepoltes between it to obtain
// the desired position at each iteration and then calulates the current position
//


    while(ros::ok())
    {

        //Simple Controller
        cable_length=calculate_cable_length(wTp,pTbi,wTai);//initialisation

        for (int var = 0; var < number_of_cables; ++var) {
            joint_deviation.position[var]=ratio*cable_length[var];
        }

        cable_length_desired=cable_length;
        ros::spinOnce();
        joint_deviation_publisher.publish(joint_deviation);
        // Here we can change wTp if desired.

        r.sleep();
    }

    return 0;

}



std::vector<double> calculate_cable_length(vpHomogeneousMatrix wTp,
                                     std::vector<vpHomogeneousMatrix> pTbi,
                                     std::vector<vpHomogeneousMatrix> wTai)
{

    std::vector<vpHomogeneousMatrix> wTbi(pTbi.size());
    std::vector<vpHomogeneousMatrix> aiTbi(pTbi.size());
    std::vector<double> cable_length(pTbi.size());
    vpTranslationVector L;

    for (int i = 0; i < pTbi.size(); ++i) {
        // Platform position w.r.t world frame
        wTbi[i]=wTp*pTbi[i];
        // Platform position w.r.t corresponding base
        aiTbi[i]= (wTai[i].inverse())*wTbi[i];
        aiTbi[i].extract(L);
        cable_length[i]=L.euclideanNorm();
    }
    return cable_length;
}


std::vector<vpHomogeneousMatrix> get_attachment_parameters(
        std::string param_name,ros::NodeHandle n)
{
    XmlRpc::XmlRpcValue Axml;
    n.getParam(param_name,Axml);
    std::vector<vpHomogeneousMatrix> T(Axml.size());


    for (int i = 0; i < Axml.size(); ++i) {
        double xa=Axml[i][0];     double ya=Axml[i][1];
        double za=Axml[i][2];
        T[i].buildFrom(xa/1000.,ya/1000.,za/1000,0,0,0);
    }

    return T;
}

void printfM(vpHomogeneousMatrix M,const char* intro)
{
    printf(intro,"\n");
    printf("%8.3f %8.3f %8.3f %8.3f\n", M[0][0], M[0][1], M[0][2], M[0][3]);
    printf("%8.3f %8.3f %8.3f %8.3f\n", M[1][0], M[1][1], M[1][2], M[1][3]);
    printf("%8.3f %8.3f %8.3f %8.3f\n", M[2][0], M[2][1], M[2][2], M[2][3]);
    printf("%8.3f %8.3f %8.3f %8.3f\n", M[3][0], M[3][1], M[3][2], M[3][3]);
}
