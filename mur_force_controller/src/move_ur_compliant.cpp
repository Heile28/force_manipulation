/*
 * File: move_ur_compliant.cpp
 * Author: Heiko Lenz
 * 
 * Created on 25. September 2020
 * 
 * Source file providing methods driving the UR in conjunction 
 * to MiR movement
 * 
*/

#include <mur_force_controller/move_ur_compliant.h>


using namespace move_compliant;

MoveUR::MoveUR()
{
    this->nh_=ros::NodeHandle("move_ur_compliant");
    this->rotation_angle_ = nh_.subscribe("/move_compliant/rotation_angle", 100, &MoveUR::angleCallback, this);
    this->pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot1_ns/arm_cartesian_compliance_controller/target_pose", 100);
}

void MoveUR::angleCallback(std_msgs::Float64 angle_)
{
    theta_ = angle_.data;
}

void MoveUR::lookupInitialLocalPosition(){

    ros::service::waitForService("/mur_base/listen_frames/request_endeffector/pose");
    this->endeffector_pose_client_ = nh_.serviceClient<mur_robot_msgs::PoseRequest>("/mur_base/listen_frames/request_endeffector/pose");

    mur_robot_msgs::PoseRequest pose_msg_;
    pose_msg_.request.request = true;
    pose_msg_.request.source_frame = "robot1_tf/base_link_ur5";
    pose_msg_.request.target_frame = "robot1_tf/ee_link_ur5";

    if(endeffector_pose_client_.call(pose_msg_))
    {
        std::cout<<"Inside service call"<<std::endl;

        //endeffector_pose_client_.call(pose_msg_);
        initial_local_pose_.clear();
        initial_local_pose_.push_back(pose_msg_.response.position.x);
        initial_local_pose_.push_back(pose_msg_.response.position.y);
        initial_local_pose_.push_back(pose_msg_.response.position.z);
        initial_local_pose_.push_back(pose_msg_.response.orientation.x);
        initial_local_pose_.push_back(pose_msg_.response.orientation.y);
        initial_local_pose_.push_back(pose_msg_.response.orientation.z);
        initial_local_pose_.push_back(pose_msg_.response.orientation.w);
        std::cout<<"Initial local pose is: "<<initial_local_pose_[0]<<", "<<initial_local_pose_[1]<<", "<<initial_local_pose_[2]<<", "
                    <<initial_local_pose_[3]<<", "<<initial_local_pose_[4]<<", "<<initial_local_pose_[5]<<", "<<initial_local_pose_[6]<<std::endl;
        
    }
    else
    {
        ROS_ERROR("Service call failed!");
    }
    last_time_ = init_time_;
}

std::vector<double> MoveUR::callCurrentLocalPose()
{
    const std::string source_frame = "robot1_tf/base_link_ur5";
    const std::string target_frame = "robot1_tf/ee_link_ur5";

    current_local_pose_.clear();
    current_local_pose_ = base_.getCurrentPose(source_frame, target_frame);

    // std::cout<<"Current local pose is "<<current_local_pose_[0]<<", "<<current_local_pose_[1]<<", "<<current_local_pose_[2]
    //                 <<", "<<current_local_pose_[3]<<", "<<current_local_pose_[4]<<", "<<current_local_pose_[5]<<std::endl;

    last_time_ = ros::Time::now();

    return current_local_pose_;
}

void MoveUR::nullspace(double theta_)
{
    /***** Publish desired pose to cartesian_compliance_controller *****/
    geometry_msgs::PoseStamped x_d;
    x_d.header.frame_id = "robot1_tf/base_link_ur5";
        
    x_d.pose.position.x = atan(theta_)/current_local_pose_[1]; //-dsad;
    x_d.pose.position.y = atan(theta_)*current_local_pose_[0];
    x_d.pose.position.z = current_local_pose_[2];

    x_d.pose.orientation.x = 0.0;
    x_d.pose.orientation.y = 0.0;
    x_d.pose.orientation.z = 0.0;
    x_d.pose.orientation.w = 0.0;

    std::cout<<"Desired pose: "<<x_d.pose.position.x<<", "<<x_d.pose.position.y<<", "<<x_d.pose.position.z<<std::endl;

    pub_pose_.publish(x_d);
    
}