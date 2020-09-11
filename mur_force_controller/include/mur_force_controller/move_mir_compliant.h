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
#include <sensor_msgs/JointState.h>
#include <mur_robot_msgs/PoseMessage.h>
#include <mur_robot_msgs/PoseRequest.h>

//other
#include <eigen3/Eigen/Dense>
<<<<<<< HEAD
#include <mur_force_controller/mur_base.h>
=======
#include <boost/shared_ptr.hpp>
>>>>>>> d0cfbdd073d435640bd6d40c8463e73396a3eb84


#ifndef MOVE_MIR_COMPLIANT_H
#define MOVE_MIR_COMPLIANT_H

namespace move_compliant{
    
<<<<<<< HEAD
=======
    
    /// \brief base class ready to get inherited
    class MurBase
    {
    private:
        ros::NodeHandle nh_;

    public:
        //standard constructor
        MurBase();
        
        //destructor
        ~MurBase();

        //methods
        /**
        * @brief callback function of joints data
        * 
        */
        virtual void callbackJointAngles(const sensor_msgs::JointState::ConstPtr& joint_msg_);

        ros::Subscriber joint_angles_;
        double theta1_, theta2_, theta3_, theta4_, theta5_, theta6_;

    };
    

>>>>>>> d0cfbdd073d435640bd6d40c8463e73396a3eb84
    class MoveMir : public MurBase
    {

    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_pose_;
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
        
        virtual void callbackCurrentPose(mur_robot_msgs::PoseMessage pose_msg);
        
        /**
        * @brief Calculates displacement based on previous callbacks of initial and current poses
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