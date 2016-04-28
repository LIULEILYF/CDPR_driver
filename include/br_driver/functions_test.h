#include <ros/ros.h>
#include <stdlib.h>     /* srand, rand */
#include "sensor_msgs/JointState.h"


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpTranslationVector.h>

#include <br_driver/CartesianTrajectory.h>
#include <br_driver/CartesianTrajectoryPoint.h>


#ifndef FUNCTIONS_TEST_H
#define FUNCTIONS_TEST_H

class functions_test
{

private:
    ros::NodeHandle n;
    std::vector<vpHomogeneousMatrix> get_attachment_parameters(
            std::string param_name,ros::NodeHandle n);
    int nbr;
    ros::Subscriber joint_sub; // joint states
    sensor_msgs::JointState joint; // Declaration of message
    bool jointStateReceived;

    void JointSensorCallback(const sensor_msgs::JointState::ConstPtr& msg); // Callback to get value of joint Sensor


public:

    functions_test(
            ros::NodeHandle nh_,
            int number_of_cables
            );

    std::vector<vpHomogeneousMatrix> pTbi;
    // Transform of attachment points w.r.t world frame
    std::vector<vpHomogeneousMatrix> wTbi;
    // Transform of base points w.r.t to world frame
    std::vector<vpHomogeneousMatrix> wTai;
    // Transform of platform points w.r.t to corresponding base
    std::vector<vpHomogeneousMatrix> aiTbi;


void printfM(vpHomogeneousMatrix M, const char *intro="Matrix");
void calculate_inv_jacobian(vpHomogeneousMatrix wTp,vpMatrix& W);


std::vector<double> calculate_cable_length(vpHomogeneousMatrix wTp);
std::vector<vpTranslationVector> calculate_cable_vectors
          (vpHomogeneousMatrix wTp);
std::vector<vpTranslationVector> calculate_normalized_cable_vectors
          (vpHomogeneousMatrix wTp);
std::vector<double> calculate_motor_change(vpHomogeneousMatrix wTp,
                                           std::vector<double> cable_length,
                                           double ratio);


    void GetRobotJointState(sensor_msgs::JointState& return_joint);
    void SetJointFlag(bool Flag); // Set jointrecieved flag
    bool GetJointFlag(); // Get the joint recieved flag
};

#endif
