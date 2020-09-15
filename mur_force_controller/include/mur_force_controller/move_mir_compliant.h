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
#include <tf/transform_broadcaster.h>

//other
#include <eigen3/Eigen/Dense>
#include <mur_force_controller/mur_base.h>


#ifndef MOVE_MIR_COMPLIANT_H
#define MOVE_MIR_COMPLIANT_H

namespace move_compliant{
    
    class MoveMir //: public MurBase
    {

    private:
        ros::NodeHandle nh_;
        MurBase base_;
        ros::Publisher pub_simple_;
        ros::ServiceClient endeffector_pose_client_;
        ros::Time init_time_, current_time_, last_time_;
        std::vector<double> current_pose_, current_pose_old_, displacement_pose_;
        std::vector<double> initial_pose_;

        //MiR variables
        double dt_, vth_;
        double delta_th_; //for MiR
        geometry_msgs::Twist tw_msg_;

    public:
        //standard constructor
        MoveMir();

        //methods
        void lookupInitialPosition();
        
        /**
         * @brief 
         * 
         * @param pose_msg 
         */
        virtual void callbackCurrentPose();
        
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
        void transferIntoMir(Eigen::VectorXd displ_);

        /**
        * @brief diff_drive controller for the MiR NEUES
        * 
        */
        void moveGoal();
    };
}

#endif /* MOVE_MIR_COMPLIANT_H */