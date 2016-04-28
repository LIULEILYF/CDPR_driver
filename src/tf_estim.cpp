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

// Node which completes the odmetry it assumes that robot is functioning perfectly and
// all constraints have been respected.


int main(int argc, char **argv) {

    ros::init(argc, argv, "Controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(10);
    int number_of_cables;

    nh.getParam("number_of_cables",number_of_cables);
    std::string frame_name="estimated_platform_frame";
    controller_class CableRobot(nh,number_of_cables,frame_name);

    //Transform of attachment points w.r.t to platform centre in platform frame

    //double ratio=9/3.1428;  // 360/Pi/Diameter ration*dl=dq

    // Define a ration that transforms from dq (deg) to l (m)
    // dq *(180/pi)*radius of drum 20mm
    //       deg2rad    radius of drum
    // dq * (pi/180) * (0.02)
    double ratio=0.00034906585039886593;


    tf::TransformListener tflistener;
    tf::StampedTransform tfstam;

    ros::Time now = ros::Time(0);

    tflistener.waitForTransform("world",frame_name,
                                now, ros::Duration(4.0));

    tflistener.lookupTransform("world",frame_name,
                               now,tfstam);

    vpTranslationVector trans(tfstam.getOrigin().getX(),
                              tfstam.getOrigin().getY(),
                              tfstam.getOrigin().getZ());
    vpQuaternionVector quat(tfstam.getRotation().getX(),
                            tfstam.getRotation().getY(),
                            tfstam.getRotation().getZ(),
                            tfstam.getRotation().getW());

    sensor_msgs::JointState current_joint_state,last_joint_state,initial_joint_state;
    last_joint_state.position.resize(number_of_cables);
    current_joint_state.position.resize(number_of_cables);
    initial_joint_state.position.resize(number_of_cables);
    std::vector<double> l(number_of_cables);
    std::vector<double> l_init(number_of_cables);
    std::vector<double> l_last(number_of_cables);
    vpHomogeneousMatrix wTp(trans,quat);
    vpHomogeneousMatrix wTp_last(trans,quat);
    vpHomogeneousMatrix wdiffp;
    vpMatrix W(6,8); // -inverse transpose of jacobian matrix
    vpMatrix WT(8,6); // inverse jacobian matrix
    vpMatrix J(6,8); // jacobian matrix

    vpTranslationVector w_P_p;
    vpQuaternionVector w_Quaternion_p,w_Quat_p_diff;
    vpColVector Quaternion_dot(4); // convert
    vpColVector dq(number_of_cables);
    vpColVector dl(number_of_cables);
    vpColVector dl_check(number_of_cables);
    vpColVector dX(6);

    CableRobot.UpdatePlatformTransformation(wTp);
    bool InitialStep=true;
    double tol=0.01; // tolerance is in metres


    while(ros::ok())
    {

        if(CableRobot.GetJointFlag())
        {

            if(InitialStep)
            {
                CableRobot.UpdatePlatformTransformation(wTp);
                l=CableRobot.calculate_cable_length();
                l_init=l;
                l_last=l;

                CableRobot.GetRobotJointState(current_joint_state);
                initial_joint_state=current_joint_state;
                last_joint_state=current_joint_state;

                InitialStep=false;
            }
            else
            {

                CableRobot.UpdatePlatformTransformation(wTp);
                l=CableRobot.calculate_cable_length();
                CableRobot.GetRobotJointState(current_joint_state);

                // Obtain how much the joint has changed
                for (int i = 0; i < number_of_cables; ++i) {
                    dq[i]=current_joint_state.position[i]-last_joint_state.position[i];
                }

                dl=dq*ratio; // converts from degrees to m

                if(dl.euclideanNorm()>tol)
                {
                    ROS_WARN("Norm dl is large %f. Linearization may not be valid",dl.euclideanNorm());
                }

                dl.print(std::cout,8,"dl desired= ");

                CableRobot.calculate_inv_jacobian(W);

                WT=W.transpose()*-1.0; // find negative transpose l_dot=Wt*dx
                J=WT.pseudoInverse(); // find jacobian

                J.print(std::cout,8,"Jacobian = ");

                dX=J*dl; // find velocity vx vy vz wx wy wz

                dX.print(std::cout,8,"dX= ");

                dl_check=WT*dX;
                dl_check.print(std::cout,8,"dl checking= ");
                // integrate the signal
                // CableRobot.printfM(wTp,"wTp= ");
                CableRobot.convert_omega_to_quaternion_dot(wTp,dX[3],dX[4],dX[5],Quaternion_dot);
                Quaternion_dot.print(std::cout,8,"Quaternion dot");
                wTp.extract(w_Quaternion_p);
                wTp.extract(w_P_p);
                w_Quaternion_p.buildFrom(w_Quaternion_p.x()+Quaternion_dot[1],
                        w_Quaternion_p.y()+Quaternion_dot[2],
                        w_Quaternion_p.z()+Quaternion_dot[3],
                        w_Quaternion_p.w()+Quaternion_dot[0]);
                w_Quaternion_p.normalize();

                w_P_p.buildFrom(w_P_p[0]+dX[0],
                        w_P_p[1]+dX[1],
                        w_P_p[2]+dX[2]);

                wTp.buildFrom(w_P_p,w_Quaternion_p);
                CableRobot.UpdatePlatformTransformation(wTp);

                // find cable lengthsat new position
                //l=CableRobot.calculate_cable_length();
                for (int i = 0; i < number_of_cables; ++i) {
                    dl[i]=l[i]-l_last[i];
                }
                dl.print(std::cout,8,"Actual dl= ");
                // Need to find actual wTp
                wdiffp=wTp_last*wTp.inverse();
                CableRobot.printfM(wTp,"wTp= ");
                CableRobot.printfM(wTp_last,"wTp_last= ");

                wdiffp.extract(w_Quat_p_diff);
                std::cout<<"w_diff_p= "<<wdiffp[0][3]<<","<<wdiffp[1][3]<<","<<wdiffp[2][3]<<","<<w_Quat_p_diff.x()<<","<<w_Quat_p_diff.y()<<","<<w_Quat_p_diff.z()<<","<<w_Quat_p_diff.w()<<","<<std::endl;

                wTp_last=wTp;
                last_joint_state=current_joint_state; // update last position
                l_last=l;
                std::cout<<"=================================================="<<std::endl;
            }

        }


        r.sleep();

    }
    return 0;
}



