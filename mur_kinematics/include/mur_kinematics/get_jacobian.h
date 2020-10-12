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
#include <mur_force_controller/mur_base.h>
#include <ur_kinematics/ur_kin.h>

#ifndef GET_JACOBIAN_H
#define GET_JACOBIAN_H

namespace calculate_jacobian{
    /// \brief class to calculate the jacobian matrix through DH-transformations
    
    class GetJacobian
    {
        private:
            MurBase base_;
            ros::NodeHandle nh_;
            ros::ServiceClient endeffector_pose_client;
            ros::Subscriber get_joint_angles_;

            double* theta = new double[6];
            double* T = new double[16];
            double* T1 = new double[16];
            double* T2 = new double[16];
            double* T3 = new double[16];
            double* T4 = new double[16];
            double* T5 = new double[16];
            double* T6 = new double[16];
            const std::string names[6]={"robot1_tf/shoulder_pan_joint", "robot1_tf/shoulder_lift_joint", "robot1_tf/elbow_joint",
                             "robot1_tf/wrist_1_joint", "robot1_tf/wrist_2_joint", "robot1_tf/wrist_3_joint"};
            
            //DH parameters
            /*
            const double d1 = 0.089159;
            const double a2 = 0.42500; //-
            const double a3 = 0.39225; //-
            const double d4 = 0.10915;
            const double d5 = 0.09465;
            const double d6 = 0.0823; //caution: set the right distance between wrist_3_link_ur5 and ee_link_ur5 !!!

            const double PI = M_PI;
            const double alpha1 = PI/2; //+
            const double alpha2 = 0.0;
            const double alpha3 = 0.0;
            const double alpha4 = PI/2; //+
            const double alpha5 = -PI/2; //-
            const double alpha6 = 0.0;

            std::vector<double> theta_;
            */
        
        protected:
            double w; //Manipulation measure

            /**
            * @brief callback function of joints data
            * 
            */
            void callbackJointAngles(sensor_msgs::JointState joint_msg_);

            int mapIndizes(std::string name_);

        public:
            //standard constructor
            GetJacobian();

            //destructor
            ~GetJacobian();

            //methods
            /**
             * @brief calculates the geometric jacobian matrix based on DH-convention
            */
            void forwardKinematics();


            /**
             * @brief calculates the geometric jacobian matrix based on DH-convention
             * 
             * @return Eigen::MatrixXd 
             */
            Eigen::MatrixXd urJacobian(Eigen::MatrixXd T_E, Eigen::MatrixXd T1_, Eigen::MatrixXd T2_, 
                            Eigen::MatrixXd T3_, Eigen::MatrixXd T4_, Eigen::MatrixXd T5_);


            /**
             * @brief manipulation measure based on manipulation ellipsoid method
             * 
             */
            void manipulationMeasure(Eigen::MatrixXd J_ur_);

            /**
             * @brief calculate torque by specified wrench and jacobian matrix
             * 
             * @param J_ur_ Jacobian matrix
             * @param target_wrench_ specified wrench vector
             * 
             * @return Eigen::MatrixXd
             */ 
            Eigen::VectorXd getTorque(Eigen::MatrixXd J_ur_, Eigen::VectorXd target_wrench_);


    };

}

#endif /* GET_JACOBIAN_H */