/*
 * File: get_jacobian.cpp
 * Author: Heiko Lenz
 * 
 * Created on 20. August 2020
 * 
 * Source file providing method calculating the geometric jacobian matrix
 * 
*/
#include <mur_kinematics/get_jacobian.h>

//others
#include <eigen3/Eigen/Dense>
#include <math.h>


//constructor
calculate_jacobian::GetJacobian::GetJacobian()
{
    this->nh_=ros::NodeHandle("~");
    this->get_joint_angles_ = nh_.subscribe("/robot1_ns/joint_states",10 , &calculate_jacobian::GetJacobian::callbackJointAngles, this);
}

//destructor
calculate_jacobian::GetJacobian::~GetJacobian(){}

int calculate_jacobian::GetJacobian::mapIndizes(std::string name_)
{
        for(int i=0;i<6;i++)
        {
                if (names[i]==name_)
                return i;
        }    
}

void calculate_jacobian::GetJacobian::callbackJointAngles(sensor_msgs::JointState joint_msg_){
        //sub_force_ = nh_.subscribe("/robot1_ns/ee_force_torque_sensor", 100, &CalculateJacobians::callbackWrenchState, this); //from gazebo ft_sensor plugin
        //sub_pose_ = nh_.subscribe("/cartesian/endeffector_pose/data", 100, &CalculateJacobians::callbackCurrentPose,  this); //from listen_frames_node
        std::string name;
        for(unsigned i = 0; i<6; i++){
                name=joint_msg_.name[i];
                theta[mapIndizes(name)] = joint_msg_.position[i];
        }
        //std::cout<<"Inside callback!"<<std::endl;

        //std::cout<<"Angles:"<<std::endl;
        //std::cout<<"["<<theta[0]<<","<<theta[1]<<","<<theta[2]<<","<<theta[3]<<","<<theta[4]<<","<<theta[5]<<"]"<<std::endl;
        //forwardKinematics();
}
void calculate_jacobian::GetJacobian::manipulationMeasure(Eigen::MatrixXd J_ur)
{
        /***** eigenvalues and eigenvectors *****/
        Eigen::EigenSolver<Eigen::MatrixXd> es;

        Eigen::MatrixXd J_v(3,6);
        J_v = J_ur.block(0,0,3,6);

        Eigen::Matrix3d JJ;
        JJ = J_v*J_v.transpose();
        
        es.compute(J_v*J_v.transpose(), true);
        Eigen::VectorXd eigen_values = es.eigenvalues().real(); //just the real part
        Eigen::MatrixXd eigen_vectors = es.eigenvectors().real(); //just the real part
        

        /***** Manipulation measure *****/
        double w = sqrt(JJ.determinant());
        std::cout<<"Manipulation measure is: "<<w<<std::endl;
}

void calculate_jacobian::GetJacobian::forwardKinematics()
{ 
        /**** Get endeffector pose ****/
        ur_kinematics::forward(theta, T);

        /**** Store endeffector pose ****/
        Eigen::MatrixXd T_E(4,4);
        int i = 0;
        for (int k = 0; k<4; k++){
                for (int l = 0; l<4; l++){
                        T_E(k,l)=T[i];
                        i++;
                }
        }
        //std::cout<<"Endeffector pose is: \n"<<T_E<<std::endl;

        /**** Calculate transformation mtarices ****/
        ur_kinematics::forward_all(theta, T1, T2, T3, T4, T5, T6);

        Eigen::MatrixXd _T01(4,4), _T02(4,4), _T03(4,4), _T04(4,4), _T05(4,4), _T06(4,4);
        i = 0;
        
        for (int k = 0; k<4; k++){
                for (int l = 0; l<4; l++){
                _T01(k,l) = T1[i];
                _T02(k,l) = T2[i];
                _T03(k,l) = T3[i];
                _T04(k,l) = T4[i];
                _T05(k,l) = T5[i];
                _T06(k,l) = T6[i];
                i++;
                }
        }

        /**** Calculate relative transformation matrices ****/
        Eigen::MatrixXd T1_(4,4), T2_(4,4), T3_(4,4), T4_(4,4), T5_(4,4), T6_(4,4);
        T1_ = _T01;
        T2_ = _T01.inverse()*_T02;
        //std::cout<<"T2_: \n"<<T2_<<std::endl;
        T3_ = _T02.inverse()*_T03;
        //std::cout<<"T3_: \n"<<T3_<<std::endl;
        T4_ = _T03.inverse()*_T04;
        //std::cout<<"T4_: \n"<<T4_<<std::endl;
        T5_ = _T04.inverse()*_T05;
        //std::cout<<"T5_: \n"<<T5_<<std::endl;
        T6_ = _T05.inverse()*_T06;
        //std::cout<<"T6_: \n"<<T6_<<std::endl;

        /**** Calculate Jacobian ****/
        Eigen::MatrixXd J_ur(6,6);
        J_ur = urJacobian(T_E, T1_, T2_, T3_, T4_, T5_);

        /**** Manipulation measure ****/
        manipulationMeasure(J_ur);
}

Eigen::MatrixXd calculate_jacobian::GetJacobian::urJacobian(Eigen::MatrixXd T_E, Eigen::MatrixXd T1_, Eigen::MatrixXd T2_, 
                            Eigen::MatrixXd T3_, Eigen::MatrixXd T4_, Eigen::MatrixXd T5_)
{

        Eigen::Vector3d ez_00, ez_01, ez_02, ez_03, ez_04, ez_05;
        Eigen::Vector3d r_0E, r_1E, r_2E, r_3E, r_4E, r_5E;
        Eigen::Vector4d r(0.0, 0.0, 0.0, 1.0);

        /***** spin axis (z) relative to ur5_base *****/
        ez_00 << 0.0, 0.0, 1.0;
        ez_01 = T1_.block(0,0,3,3)*ez_00;
        ez_02 = (T1_*T2_).block(0,0,3,3)*ez_00;
        ez_03 = (T1_*T2_*T3_).block(0,0,3,3)*ez_00;
        ez_04 = (T1_*T2_*T3_*T4_).block(0,0,3,3)*ez_00;
        ez_05 = (T1_*T2_*T3_*T4_*T5_).block(0,0,3,3)*ez_00;

        /***** homogenous directional vectors *****/
        r_0E = (T_E * r - r).head(3);
        r_1E = (T_E * r - T1_*r).head(3);
        r_2E = (T_E * r - (T1_*T2_)*r).head(3);
        r_3E = (T_E * r - (T1_*T2_*T3_)*r).head(3);
        r_4E = (T_E * r - (T1_*T2_*T3_*T4_)*r).head(3);
        r_5E = (T_E * r - (T1_*T2_*T3_*T4_*T5_)*r).head(3);

        /***** Jacobian matrix *****/
        Eigen::MatrixXd J_ur_(6,6);
        J_ur_ << ez_00.cross(r_0E), ez_01.cross(r_1E), ez_02.cross(r_2E),
                ez_03.cross(r_3E), ez_04.cross(r_4E), ez_05.cross(r_5E),
                ez_00, ez_01, ez_02, ez_03, ez_04, ez_05;
        //std::cout <<"Jacobian matrix of UR=\n"<<J_ur_<<std::endl;

        return J_ur_;
}

