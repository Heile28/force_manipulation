/*
 * File: move_ur_compliant.h
 * Author: Heiko Lenz
 * 
 * Created on 25. September 2020
 * 
 * Header file providing methods driving the UR in conjunction 
 * to MiR movement
 * 
*/

//ROS
#include <ros/ros.h>

//ROS msgs
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>

//other
#include <tf/transform_broadcaster.h>
#include <mur_force_controller/mur_base.h>


#ifndef MOVE_UR_COMPLIANT_H
#define MOVE_UR_COMPLIANT_H

/// \brief Class of ur_controller driving the UR in conjunction to MiR yaw change
namespace move_ur_compliant{

    class MoveUR{
        private:
            ros::NodeHandle nh_;
            MurBase base_;
            ros::Publisher pub_pose_;
            ros::Subscriber rotation_angle_;
            ros::ServiceClient endeffector_pose_client_;
            double q0_;
            std::vector<double> joint_theta_;

            ros::Time init_time_, current_time_, last_time_;

            std::vector<double> current_local_pose_, initial_local_pose_;
            geometry_msgs::PoseStamped initial_global_pose_, desired_local_pose_;
            double PI = M_PI;

            tf::StampedTransform transform_;

            //forward kinematics
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

        protected:
        
            /**
             * @brief callbacks current wrench
             * 
             * @param angle_msg 
             */
            void angleCallback(std_msgs::Float64 angle_msg);

        public:

            /**
             * @brief Constructs a new Move U R object
             * 
             */
            MoveUR();

            /**
             * @brief Destroys the all objects
             * 
             */
            ~MoveUR();

            /**
             * @brief request initial local pose in ~/base_link_ur5
             * 
             */
            void lookupInitialLocalPosition();

            /**
             * @brief request initial global pose in ~/base_link
             * 
             */
            void lookupInitialGlobalPosition();

            /**
             * @brief calls for local position of endeffector (inside ~/base_link_ur5)
             * 
             * @return std::vector<double> current_local_pose
             */
            virtual std::vector<double> callCurrentLocalPose();

            /**
             * @brief moves endeffector into initial pose
             * 
             */
            void moveInitialPose();

            /**
             * @brief moves endeffector by specified rot_angle
             * 
             * @param rot_angle_ 
             */
            void rotateAngle(double rot_angle_);

            /**
             * @brief calculates endeffector pose based on current joint angles 
             * 
             */
            void forwardKinematics();

    };

}

#endif /* MOVE_UR_COMPLIANT_H */