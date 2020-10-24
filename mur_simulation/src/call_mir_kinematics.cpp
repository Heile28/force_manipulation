/*
 * File: call_mir_kinematics.cpp
 * Author: Heiko Lenz
 * 
 * Created on 10. Oktober 2020
 * 
 * Source file providing a topic publishing current global mir pose 
 * 
*/

#include <ros/ros.h>

#include <mur_force_controller/mur_base.h>
#include <mur_force_controller/gazebo_ft_publisher.h>
#include <mur_force_controller/move_ur_compliant.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"mir_kinematics");
    ROS_INFO("Endeffector publishes data.");
    ros::NodeHandle nh;
    ros::Publisher pub_pose_ = nh.advertise<geometry_msgs::Pose>(nh.getNamespace() + "/get_mir_kinematics/current_pose", 100);

    MurBase base_;
    geometry_msgs::Pose pose;
    std::vector<double> current_pose;
    std::vector<double> old_pose;
    current_pose = {0,0,0,0,0,0};
    old_pose = {0,0,0,0,0,0};
    
    std::string source_frame = "map";
    std::string target_frame = "robot1_tf/base_link";

    while(ros::ok())
    {
        /***** Call current global pose *****/
        current_pose.clear();
        current_pose = base_.getCurrentPose(source_frame, target_frame);

        pose.position.x = current_pose.at(0);
        pose.position.y = current_pose.at(1);
        pose.position.z = current_pose.at(2);

        pose.orientation.x = current_pose.at(6);
        pose.orientation.y = current_pose.at(7);
        pose.orientation.z = current_pose.at(8);
        pose.orientation.w = current_pose.at(9);

        ROS_INFO_STREAM("Aktuelle Lage: "<<pose.position.x<<", "<<pose.position.y<<", "<<pose.position.z);

        /***** Publish pose *****/
        pub_pose_.publish(pose);

        old_pose.clear();
        old_pose = current_pose;
    }

}