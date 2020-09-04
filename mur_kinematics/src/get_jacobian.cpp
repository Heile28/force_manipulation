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
#include <mur_force_controller/move_compliant.h>

//others
#include <eigen3/Eigen/Dense>
#include <math.h>

using namespace calculate_jacobian;

//constructor
GetJacobian::GetJacobian()
{
    this->nh_=ros::NodeHandle("~");
}

void GetJacobian::urJacobian(){
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

        T1 <<   cos(theta1), -sin(theta1)*cos(alpha1), sin(theta1)*sin(alpha1), a[0]*cos(theta1),
                sin(theta1), cos(theta1)*cos(alpha1), -cos(theta1)*sin(alpha1), a[0]*sin(theta1),
                0, sin(alpha1), cos(alpha1), d[0],
                0, 0, 0, 1;
        T2 <<   cos(theta2), -sin(theta2)*cos(alpha2), sin(theta2)*sin(alpha2), a[1]*cos(theta2),
                sin(theta2), cos(theta2)*cos(alpha2), -cos(theta2)*sin(alpha2), a[1]*sin(theta2),
                0, sin(alpha2), cos(alpha2), d[1],
                0, 0, 0, 1;
        T3 <<   cos(theta3), -sin(theta3)*cos(alpha3), sin(theta3)*sin(alpha3), a[2]*cos(theta3),
                sin(theta3), cos(theta3)*cos(alpha3), -cos(theta3)*sin(alpha3), a[2]*sin(theta3),
                0, sin(alpha3), cos(alpha3), d[2],
                0, 0, 0, 1;
        T4 <<   cos(theta4), -sin(theta4)*cos(alpha4), sin(theta4)*sin(alpha4), a[3]*cos(theta4),
                sin(theta4), cos(theta4)*cos(alpha4), -cos(theta4)*sin(alpha4), a[3]*sin(theta4),
                0, sin(alpha4), cos(alpha4), d[3],
                0, 0, 0, 1;
        T5 <<   cos(theta5), -sin(theta5)*cos(alpha5), sin(theta5)*sin(alpha5), a[4]*cos(theta5),
                sin(theta5), cos(theta5)*cos(alpha5), -cos(theta5)*sin(alpha5), a[4]*sin(theta5),
                0, sin(alpha5), cos(alpha5), d[4],
                0, 0, 0, 1;
        T6 <<   cos(theta6), -sin(theta6)*cos(alpha6), sin(theta6)*sin(alpha6), a[5]*cos(theta6),
                sin(theta6), cos(theta6)*cos(alpha6), -cos(theta6)*sin(alpha6), a[5]*sin(theta6),
                0, sin(alpha6), cos(alpha6), d[5],
                0, 0, 0, 1;
        std::cout <<"(0)_T_1 =\n "<< T1 <<std::endl;
        std::cout <<"(1)_T_2 =\n "<< T2 <<std::endl;
        //std::cout <<"(2)_T_3 =\n "<< T3 <<std::endl;
        //std::cout <<"(3)_T_4 =\n "<< T4 <<std::endl;
        //std::cout <<"(4)_T_5 =\n "<< T5 <<std::endl;
        //std::cout <<"(5)_T_E =\n "<< T6 <<std::endl;
        //std::cout << "Homogenous transformation of Endeffector=\n" <<T1*T2*T3*T4*T5*T6 <<std::endl <<std::endl;

        //Endeffector transformation
        T_E << T1*T2*T3*T4*T5*T6;
        //std::cout << "Inverse Homogenous transformation of Endeffector=\n" <<T_E.inverse()<<std::endl <<std::endl;

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
        Eigen::MatrixXd J_ur(6,6);
        J_ur << ez_00.cross(r_0E), ez_01.cross(r_1E), ez_02.cross(r_2E),
                ez_03.cross(r_3E), ez_04.cross(r_4E), ez_05.cross(r_5E),
                ez_00, ez_01, ez_02, ez_03, ez_04, ez_05;
        std::cout <<"Jacobian matrix of UR=\n"<<J_ur<<std::endl;


        /***** Endeffector pose ******/

        /*
        endeff_rotation = T_E.block(0,0,3,3); //(row, column)
        //std::cout<<"Rotation matrix (0)_R_E=\n"<<endeff_rotation<<std::endl;
        double alpha = atan2(-endeff_rotation(1,2),endeff_rotation(2,2));
        double gamma = atan2(-endeff_rotation(0,1),endeff_rotation(0,0));
        double beta = atan2(endeff_rotation(0,2),((endeff_rotation(0,0)*cos(gamma))-(endeff_rotation(0,1)*sin(gamma))));

        //double gamma = atan(endeff_rotation(1,0)/endeff_rotation(0,0));
        //double beta = atan(-endeff_rotation(2,0)/((endeff_rotation(0,0)*cos(gamma)+endeff_rotation(1,0)*sin(gamma))));
        //double alpha = atan((endeff_rotation(0,2)*sin(gamma)-endeff_rotation(1,2)*cos(gamma))/(endeff_rotation(1,1)*cos(gamma)-endeff_rotation(0,1)*sin(gamma)));

        endeff_new << T_E(0,3),  T_E(1,3), T_E(2,3),
                        alpha,
                        beta, 
                        gamma;
        std::cout<<"Endeffector pose is\n"<<endeff_new<<std::endl;

        */

        /***** Uncomment for calculating compliant pose *****
        delta_x(J_ur);
        */

        /*
        Eigen::VectorXd q_inv(6);
        for(int i = 0; i<5; i++){
                q_inv = J_ur.inverse()*T_E.col(3);
                std::cout<<"Calculation"<<i<<"\n"<<q_inv<<std::endl;
        }
        */

        /*
        Eigen::VectorXd joint_vector;
        joint_vector << theta1, theta2, theta3, theta4, theta5, theta6;
        endeff_new = J_ur*joint_vector;
        std::cout <<"Current Endeffector pose is \n"<< endeff_new <<std::endl;#
        */

}


int main(int argc, char **argv){
  ros::init(argc, argv, "get_jacobian_node");
  ros::NodeHandle nh;

  ros::Subscriber joint_angles;

  joint_angles = nh.subscribe("/robot1_ns/joint_states",10, &CalculateJacobians::callbackJointAngles, &obj);
  
  ros::spin();
  return 0;
}