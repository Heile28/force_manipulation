/*
 * File: get_jacobian.h
 * Author: Heiko Lenz
 * 
 * Created on 20. August 2020
 * 
 * Source file providing method calculating the geometric jacobian matrix
 * 
*/

#include <mur_kinematics/get_jacobian.h>
#include <mur_force_controller/move_mir_compliant.h>

//others
#include <eigen3/Eigen/Dense>
#include <math.h>

using namespace calculate_jacobian;

//constructor
GetJacobian::GetJacobian()
{
    this->nh_=ros::NodeHandle("~");
}

Eigen::MatrixXd GetJacobian::urJacobian()
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

        /***** Calculation of Transformation matrices *****/

        //std::cout<<"Joint angles: "<<theta1<<", "<<theta2<<", "<<theta3<<", "<<theta4<<", "<<theta5<<", "<<theta6<<std::endl;

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
        std::cout <<"Jacobian matrix of UR=\n"<<J_ur_<<std::endl;

        return J_ur_;
}

void GetJacobian::manipulationMeasure()
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
        std::cout<<"The eigenvalues of Jacobian are: \n"<<eigen_values<<std::endl;
        std::cout<<"\n";
        
        //Manipulation measure
        w = sqrt(JJ.determinant());
        std::cout<<"Manipulation measure is: "<<w<<std::endl;
}