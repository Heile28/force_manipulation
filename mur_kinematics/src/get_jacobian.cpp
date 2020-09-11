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
<<<<<<< HEAD
calculate_jacobian::GetJacobian::GetJacobian()
{
    this->nh_=ros::NodeHandle("~");
    /*
    move_compliant::MurBase *joint_angles = new move_compliant::MurBase();
    this->get_joint_angles_ = joint_angles->joint_angles_;
    */
=======
calculate_jacobian::GetJacobian::GetJacobian() //: move_compliant::MurBase()
{
    this->nh_=ros::NodeHandle("~");
    this->get_joint_angles_ = nh_.subscribe("/robot1_ns/joint_states", 10, &GetJacobian::callbackJointAngles, this);
>>>>>>> d0cfbdd073d435640bd6d40c8463e73396a3eb84
}

//destructor
calculate_jacobian::GetJacobian::~GetJacobian(){}

Eigen::MatrixXd calculate_jacobian::GetJacobian::urJacobian()
{
        /***** Initialise variables *****/
        Eigen::MatrixXd T1(4,4), T2(4,4), T3(4,4), T4(4,4), T5(4,4), T6(4,4);
        //-> Using Eigen:MatrixXd <double, XxX> see: http://eigen.tuxfamily.org/dox/GettingStarted.html 
        //(2) wie in Skript S.51 verfahren
        Eigen::MatrixXd T_E(4,4);
        //Eigen::VectorXd endeff_new(6);
        //Eigen::Matrix3d endeff_rotation;

        Eigen::Vector3d ez_00, ez_01, ez_02, ez_03, ez_04, ez_05;
        Eigen::Vector3d r_0E, r_1E, r_2E, r_3E, r_4E, r_5E;
        Eigen::Vector4d r(0.0, 0.0, 0.0, 1.0);

        theta_ = base_.getAngles();

        /***** Calculation of Transformation matrices *****/
<<<<<<< HEAD
        /*std::cout<<"Joint angles: "<<theta_[0]<<", "<<theta_[1]<<", "
        <<theta_[2]<<", "<<theta_[3]<<", "<<theta_[4]<<", "<<theta_[5]<<std::endl;*/
=======
        /*
        std::cout<<"Joint angles: "<<theta1_<<", "<<theta2_<<", "
        <<theta3_<<", "<<theta4_<<", "<<theta5_<<", "<<theta6_<<std::endl;
        */
>>>>>>> d0cfbdd073d435640bd6d40c8463e73396a3eb84

        T1 << cos(theta1_), 0, sin(theta1_), 0,
                sin(theta1_), 0, -cos(theta1_), 0,
                0, 1, 0, d1,
                0, 0, 0, 1;
        T2 << cos(theta2_), -sin(theta2_), 0, a2*cos(theta2_),
                sin(theta2_), cos(theta2_), 0, a2*sin(theta2_),
                0, 0, 1, 0,
                0, 0, 0, 1;
        T3 << cos(theta3_), -sin(theta3_), 0, a3*cos(theta3_),
                sin(theta3_), cos(theta3_), 0, a3*sin(theta3_),
                0, 0, 1, 0,
                0, 0, 0, 1;
        T4 << cos(theta4_), 0, sin(theta4_), 0,
                sin(theta4_), 0, -cos(theta4_), 0,
                0, 1, 0, d4,
                0, 0, 0, 1;
        T5 << cos(theta5_), 0, -sin(theta5_), 0,
                sin(theta5_), 0, cos(theta5_), 0,
                0, -1, 0, d5,
                0, 0, 0, 1;
        T6 << cos(theta6_), -sin(theta6_), 0, 0,
                sin(theta6_), cos(theta6_), 0, 0,
                0, 0, 1, d6,
                0, 0, 0, 1;
        
                
        /*
        std::cout <<"(0)_T_1 =\n "<< T1 <<std::endl;
        std::cout <<"(1)_T_2 =\n "<< T2 <<std::endl;
        std::cout <<"(2)_T_3 =\n "<< T3 <<std::endl;
        std::cout <<"(3)_T_4 =\n "<< T4 <<std::endl;
        std::cout <<"(4)_T_5 =\n "<< T5 <<std::endl;
        std::cout <<"(5)_T_E =\n "<< T6 <<std::endl;
        */

        //Endeffector transformation
        T_E << T1*T2*T3*T4*T5*T6;
        std::cout << "Homogenous transformation of Endeffector=\n" <<T_E<<std::endl;

        /***** spin axis (z) relative to ur5_base *****/
        ez_00 << 0.0, 0.0, 1.0;
        ez_01 = T1.block(0,0,3,3)*ez_00;
        ez_02 = (T1*T2).block(0,0,3,3)*ez_00;
        ez_03 = (T1*T2*T3).block(0,0,3,3)*ez_00;
        ez_04 = (T1*T2*T3*T4).block(0,0,3,3)*ez_00;
        ez_05 = (T1*T2*T3*T4*T5).block(0,0,3,3)*ez_00;

        /***** homogenous directional vectors *****/
        r_0E = (T_E * r - r).head(3);
        r_1E = (T_E * r - T1*r).head(3);
        r_2E = (T_E * r - (T1*T2)*r).head(3);
        r_3E = (T_E * r - (T1*T2*T3)*r).head(3);
        r_4E = (T_E * r - (T1*T2*T3*T4)*r).head(3);
        r_5E = (T_E * r - (T1*T2*T3*T4*T5)*r).head(3);

        /***** Jacobian matrix *****/
        Eigen::MatrixXd J_ur_(6,6);
        J_ur_ << ez_00.cross(r_0E), ez_01.cross(r_1E), ez_02.cross(r_2E),
                ez_03.cross(r_3E), ez_04.cross(r_4E), ez_05.cross(r_5E),
                ez_00, ez_01, ez_02, ez_03, ez_04, ez_05;
        //std::cout <<"Jacobian matrix of UR=\n"<<J_ur_<<std::endl;

        return J_ur_;
}

void calculate_jacobian::GetJacobian::manipulationMeasure()
{
        /***** Manipulation ellipsoid *****/
        //eigenvalues and eigenvectors
        Eigen::EigenSolver<Eigen::MatrixXd> es;
        
        Eigen::MatrixXd J_ur(6,6);
        J_ur = urJacobian();

        Eigen::MatrixXd J_v(3,6);
        J_v = J_ur.block(0,0,3,6);

        Eigen::Matrix3d JJ;
        JJ = J_v*J_v.transpose();
        
        es.compute(J_v*J_v.transpose(), true);
        Eigen::VectorXd eigen_values = es.eigenvalues().real(); //just the real part
        Eigen::MatrixXd eigen_vectors = es.eigenvectors().real(); //just the real part
        //std::cout<<"The eigenvalues of Jacobian are: \n"<<eigen_values<<std::endl;
<<<<<<< HEAD
        std::cout<<"\n";
=======
        //std::cout<<"\n";
>>>>>>> d0cfbdd073d435640bd6d40c8463e73396a3eb84
        
        //Manipulation measure
        w = sqrt(JJ.determinant());
        //std::cout<<"Manipulation measure is: "<<w<<std::endl;
<<<<<<< HEAD
=======
}

void calculate_jacobian::GetJacobian::callbackJointAngles(const sensor_msgs::JointState::ConstPtr& joint_msg_)
{
        this->theta1_ = joint_msg_->position[0];
        this->theta2_ = joint_msg_->position[1];
        this->theta3_ = joint_msg_->position[2];
        this->theta4_ = joint_msg_->position[3];
        this->theta5_ = joint_msg_->position[4];
        this->theta6_ = joint_msg_->position[5];
        
        std::cout<<"Winkel:"<<std::endl;
        std::cout<<"["<<theta1_<<","<<theta2_<<","<<theta3_<<","<<theta4_<<","<<theta5_<<","<<theta6_<<"]"<<std::endl;

        manipulationMeasure();
>>>>>>> d0cfbdd073d435640bd6d40c8463e73396a3eb84
}