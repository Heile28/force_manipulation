/*
 * File: mur_base.h
 * Author: Heiko Lenz
 * 
 * Created on 11. September 2020
 * 
 * Source file providing methods driving the MiR plattform in conjunction 
 * to cartesian compliance controller
 * 
*/

#include <mur_force_controller/mur_base.h>

//constructor
MurBase::MurBase()
{
    this->nh_ = ros::NodeHandle("mur_base");
    this->joint_angles_ = nh_.subscribe("/robot1_ns/joint_states", 10, &MurBase::callbackJointAngles, this);
    /*
    this->theta1_ = 0.0; 
    this->theta2_ = 0.0;
    this->theta3_ = 0.0;
    this->theta4_ = 0.0;
    this->theta5_ = 0.0;
    this->theta6_ = 0.0;
    */
}

//destructor
MurBase::~MurBase()
{
    printf("Angles have been deleted\n");
}

void MurBase::callbackJointAngles(sensor_msgs::JointState joint_msg_)
{
    theta1_ = joint_msg_.position[0];
    theta2_ = joint_msg_.position[1];
    theta3_ = joint_msg_.position[2];
    theta4_ = joint_msg_.position[3];
    theta5_ = joint_msg_.position[4];
    theta6_ = joint_msg_.position[5];
/*
    std::cout<<"Winkel:"<<std::endl;
    std::cout<<"["<<theta1_<<","<<theta2_<<","<<theta3_<<","<<theta4_<<","<<theta5_<<","<<theta6_<<"]"<<std::endl;
    */
}

std::vector<double> MurBase::getAngles()
{
    std::vector<double> theta_;
    theta_.clear();
    theta_.push_back(theta1_);
    theta_.push_back(theta2_);
    theta_.push_back(theta3_);
    theta_.push_back(theta4_);
    theta_.push_back(theta5_);
    theta_.push_back(theta6_);
    return theta_;
}