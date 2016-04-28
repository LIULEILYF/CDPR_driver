

#include "br_driver/controller_class.h"


#ifndef PI
#define PI 3.14159265359
#endif



controller_class::controller_class(ros::NodeHandle nh_, int number_of_cables,std::string frame_name)
    : n(nh_),
      wTbi(number_of_cables),
      wTai(number_of_cables),
      pTbi(number_of_cables),
      aiTbi(number_of_cables),
      frame_name_(frame_name)
{
    nbr=number_of_cables;
    wTai=get_attachment_parameters("base_attachment_points",n);
    pTbi=get_attachment_parameters("platform_attachment_points",n);
    get_initial_location("initial_platform_location",n);

    this->SetJointFlag(false);
    this->SetDesiredTransformFlag(false);
    joint_sub = n.subscribe("/joint_state", 1, &controller_class::JointSensorCallback, this);
    desired_transform_sub = n.subscribe("/tf", 1, &controller_class::DesiredFrameCallback, this);
    PublisherThread = std::thread(&controller_class::tfPublisher, this);
}


void controller_class::JointSensorCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint=*msg; // joint is eqaul to the value pointed to by msg
    this->SetJointFlag(true); // Note that value has been receieved
}

void controller_class::DesiredFrameCallback(const tf2_msgs::TFMessageConstPtr& msg)
{

    for (int i = 0; i < msg->transforms.size(); ++i) {

        if(msg->transforms[i].child_frame_id=="desired_platform")
        {
            vpTranslationVector trans(msg->transforms[i].transform.translation.x,
                                      msg->transforms[i].transform.translation.y,
                                      msg->transforms[i].transform.translation.z);
            vpQuaternionVector quat(msg->transforms[i].transform.rotation.x,
                                    msg->transforms[i].transform.rotation.y,
                                    msg->transforms[i].transform.rotation.z,
                                    msg->transforms[i].transform.rotation.w);
            wTp_desired_.buildFrom(trans,quat);
            this->SetDesiredTransformFlag(true); // Note that value has been receieved
            break;
        }
    }

}


void controller_class::SetDesiredTransformFlag(bool Flag)
{
    DesiredTransformReceived=Flag;
}


void controller_class::SetJointFlag(bool Flag)
{
    jointStateReceived=Flag;
}


void controller_class::SetPlatformFrameName(std::string frame_name)
{
    frame_name_=frame_name;
}

// Public functions to get receive flags
bool controller_class::GetJointFlag()
{
    return jointStateReceived;
}
bool controller_class::GetDesiredTransformFlag()
{
    return DesiredTransformReceived;
}

void controller_class::GetRobotJointState(sensor_msgs::JointState& return_joint)
{
    return_joint=joint;
}




std::vector<vpHomogeneousMatrix> controller_class::get_attachment_parameters(
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

void controller_class::get_initial_location(std::string param_name,ros::NodeHandle n)
{
    XmlRpc::XmlRpcValue Axml;
    n.getParam(param_name,Axml);
    vpRotationMatrix R;
    vpTranslationVector t;
    vpRxyzVector Phi;
    // x y z Rx Ry Rz
    Phi[0]=Axml[3];
    Phi[1]=Axml[4];
    Phi[2]=Axml[5];
    double x=Axml[0];
    double y=Axml[1];
    double z=Axml[2];
    t.buildFrom(x/1.,y/1.,z/1.);
    R.buildFrom(Phi);
    wTp_.buildFrom(t,R);
}


void controller_class::printfM(vpHomogeneousMatrix M,const char* intro)
{
    printf(intro,"\n");
    printf("%8.3f %8.3f %8.3f %8.8f\n", M[0][0], M[0][1], M[0][2], M[0][3]);
    printf("%8.3f %8.3f %8.3f %8.8f\n", M[1][0], M[1][1], M[1][2], M[1][3]);
    printf("%8.3f %8.3f %8.3f %8.8f\n", M[2][0], M[2][1], M[2][2], M[2][3]);
    printf("%8.3f %8.3f %8.3f %8.3f\n", M[3][0], M[3][1], M[3][2], M[3][3]);
}

void controller_class::GetPlatformTransformation(vpHomogeneousMatrix& M)
{
    M=wTp_;
}
void controller_class::GetDesiredPlatformTransformation(vpHomogeneousMatrix& M)
{
    M=wTp_desired_;
}

void controller_class::UpdatePlatformTransformation(vpHomogeneousMatrix M)
{
    wTp_=M;
}

void controller_class::UpdatePlatformTransformation(vpTranslationVector t,vpQuaternionVector Q)
{
    wTp_.buildFrom(t,Q);
}

void controller_class::UpdatePlatformTransformation(double x,double y,double z,double Rx, double Ry,double Rz)
{
    vpRotationMatrix R;
    vpTranslationVector t;
    vpRxyzVector Phi;
    Phi[0]=Rx;
    Phi[1]=Ry;
    Phi[2]=Rz;
    t.buildFrom(x,y,z);
    R.buildFrom(Phi);
    wTp_.buildFrom(t,R);
}

// Calculations
/*===========================================================================================

        Each of these functions are overloaded with one using the tracked platform pose
        and a second allowing user input

  ===========================================================================================*/


// calculate_cable_length() obtain the lenght of each cable at current platform pose
std::vector<double> controller_class::calculate_cable_length()
{
    std::vector<double> cable_length(nbr);
    vpTranslationVector L;
    vpTranslationVector wPai,wPbi;

    for (int i = 0; i < nbr; ++i) {
        // Platform  attament position w.r.t world frame
        wTbi[i]=wTp_*pTbi[i];
        // Platform position w.r.t corresponding base

        // Extract position of base attachment points w.r.t world frame
        wTai[i].extract(wPai);
        // Extract position of platform attachment points w.r.t world frame
        wTbi[i].extract(wPbi);
        // Cable Vector
        L=wPbi-wPai;
        cable_length[i]=L.euclideanNorm();
    }
    return cable_length;
}



std::vector<double> controller_class::calculate_cable_length(vpHomogeneousMatrix wTp)
{


    std::vector<double> cable_length(nbr);
    vpTranslationVector L;
    vpTranslationVector wPai,wPbi;
    for (int i = 0; i < nbr; ++i) {
        // Platform position w.r.t world frame
        wTbi[i]=wTp*pTbi[i];

        // Extract position of base attachment points w.r.t world frame
        wTai[i].extract(wPai);
        // Extract position of platform attachment points w.r.t world frame
        wTbi[i].extract(wPbi);
        // Cable Vector
        L=wPbi-wPai;
        cable_length[i]=L.euclideanNorm();
    }
    return cable_length;
}



// calculate_cable_vectors() : obtain cable vector in anchor frame at current
// platform pose
std::vector<vpTranslationVector> controller_class::calculate_cable_vectors()
{

    std::vector<vpTranslationVector> L(nbr);
    vpTranslationVector wPai,wPbi;

    for (int i = 0; i < nbr; ++i) {
        // Platform position w.r.t world frame
        wTbi[i]=wTp_*pTbi[i];
        // Platform position w.r.t corresponding base
        wTai[i].extract(wPai);
        wTbi[i].extract(wPbi);
        L[i]=wPbi-wPai;

    }
    return L;
}
// calculate_cable_vectors(vpHomogeneousMatrix wTp) : obtain cable vector in anchor frame at wTp
std::vector<vpTranslationVector> controller_class::calculate_cable_vectors(vpHomogeneousMatrix wTp)
{

    std::vector<vpTranslationVector> L(nbr);
    vpTranslationVector wPai,wPbi;

    for (int i = 0; i < nbr; ++i) {
        // Platform position w.r.t world frame
        wTbi[i]=wTp*pTbi[i];
        // Platform position w.r.t corresponding base
        wTai[i].extract(wPai);
        wTbi[i].extract(wPbi);
        L[i]=wPbi-wPai;

    }
    return L;
}


/*
 This function extracts the matrix necessary to transform angular velocity to quaternion dot

  */
void controller_class::convert_omega_to_quaternion_dot(vpHomogeneousMatrix wTp,vpColVector omega,vpColVector& quaternion_dot)
{
    vpQuaternionVector Q;
    vpMatrix C(4,3);
    wTp.extract(Q);
    double Q1=Q.w();
    double Q2=Q.x();
    double Q3=Q.y();
    double Q4=Q.z();


    // Eq. 5.69 Khalil
    C[0][0]=-Q2; C[0][1]=-Q3; C[0][2]=-Q4;
    C[1][0]= Q1; C[1][1]= Q4; C[1][2]=-Q3;
    C[2][0]=-Q4; C[2][1]= Q1; C[2][2]= Q2;
    C[3][0]= Q3; C[3][1]=-Q2; C[3][2]= Q1;

    C=C*0.5;

    quaternion_dot=C*omega;

}

void controller_class::convert_omega_to_quaternion_dot(vpHomogeneousMatrix wTp,double omega_x,double omega_y,double omega_z,vpColVector& quaternion_dot)
{
    vpQuaternionVector Q;
    vpMatrix C(4,3);
    vpColVector omega(3);
    omega[0]=omega_x;
    omega[1]=omega_y;
    omega[2]=omega_z;
    wTp.extract(Q);
    double Q1=Q.w();
    double Q2=Q.x();
    double Q3=Q.y();
    double Q4=Q.z();


    // Eq. 5.69 Khalil
    C[0][0]=-Q2; C[0][1]=-Q3; C[0][2]=-Q4;
    C[1][0]= Q1; C[1][1]= Q4; C[1][2]=-Q3;
    C[2][0]=-Q4; C[2][1]= Q1; C[2][2]= Q2;
    C[3][0]= Q3; C[3][1]=-Q2; C[3][2]= Q1;

    C=C*0.5;

    quaternion_dot=C*omega;

}

// calculate_normalized_cable_vectors() : obtain cable normalized vector in anchor frame at current
// platform pose
std::vector<vpTranslationVector> controller_class::calculate_normalized_cable_vectors()
{
    std::vector<vpTranslationVector> L(nbr);
    vpTranslationVector wPai,wPbi;

    for (int i = 0; i < nbr; ++i) {
        // Platform position w.r.t world frame
        wTbi[i]=wTp_*pTbi[i];
        // Platform position w.r.t corresponding base
        wTai[i].extract(wPai);
        wTbi[i].extract(wPbi);
        L[i]=wPbi-wPai;
        //L[i].normalize();
        for (int j = 0; j < L[i].size(); ++j) {
            L[i][j]=L[i][j]/L[i].euclideanNorm();
        }
    }
    return L;
}

// calculate_normalized_cable_vectors(vpHomogeneousMatrix wTp) : obtain cable normalized vector
// in anchor frame at wTp
std::vector<vpTranslationVector> controller_class::calculate_normalized_cable_vectors(vpHomogeneousMatrix wTp)
{   
    std::vector<vpTranslationVector> L(nbr);
    vpTranslationVector wPai,wPbi;

    for (int i = 0; i < nbr; ++i) {
        // Platform position w.r.t world frame
        wTbi[i]=wTp*pTbi[i];
        // Platform position w.r.t corresponding base
        wTai[i].extract(wPai);
        wTbi[i].extract(wPbi);
        L[i]=wPbi-wPai;
        //L[i].normalize();
        for (int j = 0; j < L[i].size(); ++j) {
            L[i][j]=L[i][j]/L[i].euclideanNorm();
        }
    }
    return L;
}



// Find joint deviation required to move to wTp_desired from current location
std::vector<double> controller_class::calculate_motor_change(vpHomogeneousMatrix wTp_desired,double ratio)
{

    std::vector<double> desired_cable_length(nbr);
    std::vector<double> current_cable_length(nbr);
    std::vector<double> q(nbr);

    for (int i = 0; i < nbr; ++i) {
        current_cable_length=calculate_cable_length(); // current_cable_length
        desired_cable_length=calculate_cable_length(wTp_desired); // current_cable_length
        q[i]=ratio*(desired_cable_length[i]-current_cable_length[i]);
    }
    return q;
}

/*
  Caluclates the inverse jacobian robot of the system @ current platform pose
  NOTE: This is the transpose of the inverse jacobian matrix

  W=-J^{T}

  W \tau = f;

  */
void controller_class::calculate_inv_jacobian(vpMatrix& W)
{

    std::vector<vpTranslationVector> L;
    std::vector<vpTranslationVector> u(nbr);
    vpRotationMatrix wRp;
    vpTranslationVector pPbi;
    L=calculate_normalized_cable_vectors();
    wTp_.extract(wRp);



    for (int i = 0; i < nbr; ++i) {
        pTbi[i].extract(pPbi);
        u[i] = vpTranslationVector::cross(L[i], wRp*pPbi);

        for (int j = 0; j < 6; ++j) {

            // First 3 rows = normalized length
            // Last three rows equal u vector
            j<=2 ? W[j][i]=L[i][j] : W[j][i]=u[i][j-3];

        }
    }

}

/*
  Caluclates the inverse jacobian robot of the system @ wTp
  
  */
void controller_class::calculate_inv_jacobian(vpHomogeneousMatrix wTp,vpMatrix& W)
{

    std::vector<vpTranslationVector> L;
    std::vector<vpTranslationVector> u(nbr);
    vpRotationMatrix wRp;
    vpTranslationVector pPbi;
    L=calculate_normalized_cable_vectors(wTp);
    wTp.extract(wRp);



    for (int i = 0; i < nbr; ++i) {
        pTbi[i].extract(pPbi);
        u[i] = vpTranslationVector::cross(L[i], wRp*pPbi);

        for (int j = 0; j < 6; ++j) {

            // First 3 rows = normalized length
            // Last three rows equal u vector
            j<=2 ? W[j][i]=L[i][j] : W[j][i]=u[i][j-3];

        }
    }
    
}



/*
======================================================================

void security_publisher();
Runs in a seperate thread to publish the data.

======================================================================
*///
void controller_class::tfPublisher()
{

    ros::Rate r(10);
    geometry_msgs::TransformStamped T;
    vpQuaternionVector Q;
    vpTranslationVector t;
    static tf::TransformBroadcaster br;
    T.header.frame_id="world";
    T.child_frame_id=frame_name_;

    std::cout<<"frame_name_= "<<frame_name_<<std::endl;
    ros::Time t_n;

    while(ros::ok())
    {
        t_n=ros::Time::now();
        T.header.stamp.sec=t_n.sec;
        T.header.stamp.nsec=t_n.nsec;

        wTp_.extract(Q);
        wTp_.extract(t);
        T.transform.translation.x=t[0];
        T.transform.translation.y=t[1];
        T.transform.translation.z=t[2];
        T.transform.rotation.w=Q.w();
        T.transform.rotation.x=Q.x();
        T.transform.rotation.y=Q.y();
        T.transform.rotation.z=Q.z();
        br.sendTransform(T);
        r.sleep();
        ros::spinOnce();
    }
}
