

#include "br_driver/functions_test.h"


#ifndef PI
#define PI 3.14159265359
#endif



functions_test::functions_test(ros::NodeHandle nh_,int number_of_cables)
    : n(nh_),
      wTbi(number_of_cables),
      wTai(number_of_cables),
      pTbi(number_of_cables),
      aiTbi(number_of_cables)
{
    nbr=number_of_cables;
    wTai=get_attachment_parameters("base_attachment_points",n);
    pTbi=get_attachment_parameters("platform_attachment_points",n);
    this->SetJointFlag(false);
    joint_sub = n.subscribe("/joint_states", 1, &functions_test::JointSensorCallback, this);
}



void functions_test::JointSensorCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint=*msg; // joint is eqaul to the value pointed to by msg
    this->SetJointFlag(true); // Note that value has been receieved
}

void functions_test::SetJointFlag(bool Flag)
{
    jointStateReceived=Flag;
}

// Public functions to get receive flags
bool functions_test::GetJointFlag()
{
    return jointStateReceived;
}


void functions_test::GetRobotJointState(sensor_msgs::JointState& return_joint)
{
    return_joint=joint;
}




std::vector<double> functions_test::calculate_cable_length(vpHomogeneousMatrix wTp)
{


    std::vector<double> cable_length(nbr);
    vpTranslationVector L;
    for (int i = 0; i < nbr; ++i) {
        // Platform position w.r.t world frame
        wTbi[i]=wTp*pTbi[i];
        // Platform position w.r.t corresponding base
        aiTbi[i]= (wTai[i])*(wTbi[i].inverse());
        aiTbi[i].extract(L);
        cable_length[i]=L.euclideanNorm();
    }
    return cable_length;
}

std::vector<vpTranslationVector> functions_test::calculate_cable_vectors(vpHomogeneousMatrix wTp)
{

    std::vector<vpTranslationVector> L(nbr);

    for (int i = 0; i < nbr; ++i) {
        // Platform position w.r.t world frame
        wTbi[i]=wTp*pTbi[i];
        // Platform position w.r.t corresponding base
        aiTbi[i]= (wTai[i])*(wTbi[i].inverse());
        aiTbi[i].extract(L[i]);

    }
    return L;
}

std::vector<vpTranslationVector> functions_test::calculate_normalized_cable_vectors(vpHomogeneousMatrix wTp)
{
    std::vector<vpTranslationVector> L(nbr);

    for (int i = 0; i < nbr; ++i) {
        // Platform position w.r.t world frame
        wTbi[i]=wTp*pTbi[i];
        // Platform position w.r.t corresponding base
        aiTbi[i]= (wTai[i])*(wTbi[i].inverse());
        aiTbi[i].extract(L[i]);
        //L[i].normalize();
        for (int j = 0; j < L[i].size(); ++j) {
            L[i][j]=L[i][j]/L[i].euclideanNorm();
        }
    }
    return L;
}

std::vector<double> functions_test::calculate_motor_change(vpHomogeneousMatrix wTp,std::vector<double> cable_length,
                                                             double ratio)
{

    std::vector<double> cable_length_desired(nbr);
    vpTranslationVector L;
    std::vector<double> q(nbr);

    for (int i = 0; i < nbr; ++i) {
        // Platform position w.r.t world frame
        wTbi[i]=wTp*pTbi[i];
        // Platform position w.r.t corresponding base
        aiTbi[i]= (wTai[i])*(wTbi[i].inverse());
        aiTbi[i].extract(L);
        cable_length_desired[i]=L.euclideanNorm();
        q[i]=ratio*(cable_length_desired[i]-cable_length[i]);
    }
    return cable_length;
}

/*
  Caluclates the inverse jacobian robot of the system

  */
void functions_test::calculate_inv_jacobian(vpHomogeneousMatrix wTp,vpMatrix& W)
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


std::vector<vpHomogeneousMatrix> functions_test::get_attachment_parameters(
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

void functions_test::printfM(vpHomogeneousMatrix M,const char* intro)
{
    printf(intro,"\n");
    printf("%8.3f %8.3f %8.3f %8.3f\n", M[0][0], M[0][1], M[0][2], M[0][3]);
    printf("%8.3f %8.3f %8.3f %8.3f\n", M[1][0], M[1][1], M[1][2], M[1][3]);
    printf("%8.3f %8.3f %8.3f %8.3f\n", M[2][0], M[2][1], M[2][2], M[2][3]);
    printf("%8.3f %8.3f %8.3f %8.3f\n", M[3][0], M[3][1], M[3][2], M[3][3]);
}
