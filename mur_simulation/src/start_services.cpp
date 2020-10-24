#include <ros/ros.h>

#include <mur_force_controller/mur_base.h>
#include <mur_force_controller/gazebo_ft_publisher.h>
#include <mur_force_controller/move_ur_compliant.h>



int main(int argc, char** argv)
{
    ros::init(argc,argv,"services");
    ROS_INFO("Services are activated.");
    ros::NodeHandle nh;

    MurBase begin;
    begin.startServiceServer();

    gazebo_ft_publisher::WrenchPublisher wrench_publisher;

    ros::spin();
}