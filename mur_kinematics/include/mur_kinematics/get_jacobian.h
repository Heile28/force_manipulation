/*
 * File: get_jacobian.h
 * Author: Heiko Lenz
 * 
 * Created on 21. August 2020
 * 
 * Header file providing method calculating geometric jacobian matrix
 * 
*/

#include <mur_force_controller/move_mir_compliant.h>

#ifndef GET_JACOBIAN_H
#define GET_JACOBIAN_H

namespace calculate_jacobian{
    /// \brief class to calculate the jacobian matrix through DH-transformations
    
    class GetJacobian : public move_compliant::MurBase //inherits class method listening joint angles
    {
        private:
            ros::NodeHandle nh_;
            ros::ServiceClient endeffector_pose_client;
            ros::Subscriber get_joint_angles_;

            //DH parameters
            const double d1 = 0.089159;
            const double a2 = 0.42500; //-
            const double a3 = 0.39225; //-
            const double d4 = 0.10915;
            const double d5 = 0.09465;
            const double d6 = 0.0823; //caution: set the right distance between wrist_3_link_ur5 and ee_link_ur5 !!!

            const double PI = M_PI;
            const double alpha1 = PI/2;
            const double alpha2 = 0.0;
            const double alpha3 = 0.0;
            const double alpha4 = PI/2;
            const double alpha5 = -PI/2;
            const double alpha6 = 0.0;

            //double theta1, theta2, theta3, theta4, theta5, theta6;
        
        protected:
            double w; //Manipulation measure

        public:
            //standard constructor
            GetJacobian();

            //destructor
            ~GetJacobian();

            //methods
            /**
            * @brief callback function of joints data
            * 
            */
            /*
            virtual void callbackJointAngles(const sensor_msgs::JointState& joint_msg);
            */


            /**
             * @brief calculates the geometric jacobian matrix based on DH-convention
             * 
             * @return Eigen::MatrixXd 
             */
            Eigen::MatrixXd urJacobian();


            /**
             * @brief manipulation measure based on manipulation ellipsoid method
             * 
             */
            void manipulationMeasure();

    };

}

#endif /* GET_JACOBIAN_H */