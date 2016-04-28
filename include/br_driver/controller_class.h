#include <ros/ros.h>
#include <stdlib.h>     /* srand, rand */
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TransformStamped.h"


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpTranslationVector.h>

#include <thread>
#ifndef CONTROLLER_CLASS_H
#define CONTROLLER_CLASS_H

class controller_class
{

private:
    ros::NodeHandle n;
    std::vector<vpHomogeneousMatrix> get_attachment_parameters(
            std::string param_name,ros::NodeHandle n);
    void get_initial_location(std::string param_name,ros::NodeHandle n);
    int nbr;
    ros::Subscriber joint_sub,desired_transform_sub; // joint states
    sensor_msgs::JointState joint; // Declaration of message
    bool jointStateReceived,DesiredTransformReceived;
    vpHomogeneousMatrix wTp_,wTp_desired_;
    void JointSensorCallback(const sensor_msgs::JointState::ConstPtr& msg); // Callback to get value of joint Sensor
    void DesiredFrameCallback(const tf2_msgs::TFMessageConstPtr& msg);
    std::thread PublisherThread;
    std::string frame_name_;
    void tfPublisher();

public: 

    controller_class(
            ros::NodeHandle nh_,
            int number_of_cables,
            std::string frame_name="platform"
            );

    std::vector<vpHomogeneousMatrix> pTbi;
    // Transform of attachment points w.r.t world frame
    std::vector<vpHomogeneousMatrix> wTbi;
    // Transform of base points w.r.t to world frame
    std::vector<vpHomogeneousMatrix> wTai;
    // Transform of platform points w.r.t to corresponding base
    std::vector<vpHomogeneousMatrix> aiTbi;
    // Current Platform Location in world frame



    void printfM(vpHomogeneousMatrix M, const char *intro="Matrix");

    void calculate_inv_jacobian(vpHomogeneousMatrix wTp,vpMatrix& W);
    void calculate_inv_jacobian(vpMatrix& W);

    std::vector<double> calculate_cable_length(vpHomogeneousMatrix wTp);
    std::vector<double> calculate_cable_length();

    std::vector<vpTranslationVector> calculate_cable_vectors
    (vpHomogeneousMatrix wTp);
    std::vector<vpTranslationVector> calculate_cable_vectors();

    std::vector<vpTranslationVector> calculate_normalized_cable_vectors
    (vpHomogeneousMatrix wTp);
    std::vector<vpTranslationVector> calculate_normalized_cable_vectors();


    std::vector<double> calculate_motor_change(vpHomogeneousMatrix wTp_desired, double ratio);


    // Update the pose of the platform
    void UpdatePlatformTransformation(vpHomogeneousMatrix M);
    void UpdatePlatformTransformation(vpTranslationVector t,vpQuaternionVector Q);
    void UpdatePlatformTransformation(double x,double y,double z
                                      ,double Rx, double Ry,double Rz);

    void convert_omega_to_quaternion_dot(vpHomogeneousMatrix wTp,vpColVector omega,
                                         vpColVector& quaternion_dot);
    void convert_omega_to_quaternion_dot(vpHomogeneousMatrix wTp,
                                         double omega_x,
                                         double omega_y,
                                         double omega_z,
                                         vpColVector& quaternion_dot);

    void GetPlatformTransformation(vpHomogeneousMatrix& M);
    void GetDesiredPlatformTransformation(vpHomogeneousMatrix& M);
    void GetRobotJointState(sensor_msgs::JointState& return_joint);
    void SetJointFlag(bool Flag); // Set jointrecieved flag
    void SetDesiredTransformFlag(bool Flag); // Set jointrecieved flag
    void SetPlatformFrameName(std::string frame_name); // // Set the frame name, this
    // instance of class publishes
    bool GetJointFlag(); // Get the joint recieved flag
    bool GetDesiredTransformFlag(); // Get the joint recieved flag
};

#endif
