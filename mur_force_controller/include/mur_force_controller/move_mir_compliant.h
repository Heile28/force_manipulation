/*
 * File: move_mir_compliant.h
 * Author: Heiko Lenz
 * 
 * Created on 20. August 2020
 * 
 * Header file providing methods driving the MiR plattform in conjunction 
 * to cartesian compliance controller
 * 
*/

//ROS
#include <ros/ros.h>

//ROS msgs
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>

//other
#include <eigen3/Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <mur_force_controller/mur_base.h>


#ifndef MOVE_MIR_COMPLIANT_H
#define MOVE_MIR_COMPLIANT_H

namespace move_compliant{
    
    class MoveMir
    {

    private:
        ros::NodeHandle nh_;
        MurBase base_;
        ros::Publisher pub_simple_, pub_pose_, pub_angle_;
        ros::ServiceClient endeffector_pose_client_;
        ros::Time init_time_, current_time_, last_time_;
        double force_theta_;
        double theta_global_, angle_; //angle of MiR rotation
        double theta0_global_, theta0_local_;

        std::vector<double> initial_pose_, current_pose_, displacement_pose_;
        std::vector<double> initial_global_pose_, initial_local_pose_;
        std::vector<double> current_global_pose_, current_local_pose_, current_map_pose_;
        std::vector<double> last_pose_;
        double delta_x_, delta_y_; 
        int func_case_;

        //MiR variables
        double dt_, vth_;
        double delta_th_; //for MiR
        geometry_msgs::Twist tw_msg_;
        std_msgs::Float64 rotation_angle_;
        double PI = M_PI;

        //rotate in Force direction
        ros::Subscriber sub_force_;
        tf::StampedTransform transform_;
        tf::Vector3 tf_force_at_base_, tf_force_at_sensor_;
        geometry_msgs::Vector3 force_, force_at_base_;

    protected:
        /**
        * @brief Callbacks current wrench 
        * 
        */
        void wrenchCallback(geometry_msgs::WrenchStamped wrench_msg);

    public:
        //standard constructor
        MoveMir();

        //methods
        /**
         * @brief request initial global pose in ~/base_link
         * 
         */
        void lookupInitialGlobalPosition();

        /**
         * @brief request initial pose in ~/base_link_ur5
         * 
         */
        void lookupInitialLocalPosition();
        
        /**
         * @brief calls for global position of endeffector (inside ~/base_link)
         * 
         * @return std::vector<double> current_pose
         */
        virtual std::vector<double> callCurrentGlobalPose();

        /**
         * @brief calls for current local position of endeffector (inside ~/base_link_ur5)
         * 
         * @return std::vector<double> current_pose
         */
        virtual std::vector<double> callCurrentLocalPose();

        virtual std::vector<double> callCurrentWorldPose();
        
        /**
        * @brief Calculates displacement based on previous callbacks of initial and current poses
        * to calculate the rotation speed
        * 
        */
        void displacementPose();

        /**
        * @brief Transforms UR5 endeffector pose into mir_base frame
        * 
        */
        //void transferIntoMir(Eigen::VectorXd displ_);

        /**
        * @brief diff_drive controller for the MiR NEUES
        * 
        * @param x_length length of displacement vector "DELTA{x}"
        * 
        */
        void moveGoal();

        /**
         * @brief connects /robot1_ns/arm_cartesian_compliance_controller/target_pose
         * 
         * @param x_d desired pose which to send to cartesian_compliance_controller
         * @param theta_ angle between MiR-x-axis and force direction
         * 
         */
        void nullspace(double theta_);

        void rotateToForceDirection();

        void rotateToPoseDirection();

        void poseUpdater();

        void moveStraight();
    };
}

#endif /* MOVE_MIR_COMPLIANT_H */