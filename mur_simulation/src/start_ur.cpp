#include <ros/ros.h>
#include <mur_force_controller/move_ur_compliant.h>



int main(int argc, char** argv)
{
    ros::init(argc,argv,"universal_robot");
    ROS_INFO("universal robot is active.");
    ros::NodeHandle nh;

    move_compliant::MoveUR obj2;

    ros::spin();
}