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

    /*
    while(ros::ok())
    {
        wrench_publisher.gravitation_compensation();
        ros::spinOnce();
    }
    */

    
    
    /*
    //Optional service client
    ros::ServiceClient endeffector_pose_client_;
    endeffector_pose_client_ = nh.serviceClient<mur_robot_msgs::PoseRequest>("/mur_base/listen_frames/request_endeffector/pose");

    mur_robot_msgs::PoseRequest pose_msg_;
    pose_msg_.request.request = true;
    pose_msg_.request.source_frame = "robot1_tf/base_link_ur5";
    pose_msg_.request.target_frame = "robot1_tf/ee_link_ur5";

    //ros::service::waitForService("/mur_base/listen_frames/request_endeffector/pose", ros::Duration(2));
    
    if(endeffector_pose_client_.call(pose_msg_))
    {
        ROS_INFO_STREAM("Pose is "<<pose_msg_.response.position.x<<", "<<pose_msg_.response.position.y<<", "<<pose_msg_.response.position.z);
    }
    else
    {
        ROS_ERROR("Service call failed!");
    }
    */
    ros::spin();
}