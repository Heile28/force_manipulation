/*
 * File: mur_base.h
 * Author: Heiko Lenz
 * 
 * Created on 11. September 2020
 * 
 * Header file providing methods to get inherited by subclasses
 * 
*/

//ROS
#include <ros/ros.h>

//ROS messages
#include <sensor_msgs/JointState.h>

#ifndef MUR_BASE_H
#define MUR_BASE_H

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
        void callbackJointAngles(sensor_msgs::JointState joint_msg);

        ros::Subscriber joint_angles_;
        double theta1_, theta2_, theta3_, theta4_, theta5_, theta6_;

        std::vector<double> getAngles();
    };

    #endif /* MUR_BASE_H */