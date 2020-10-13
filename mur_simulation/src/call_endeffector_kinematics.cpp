#include <ros/ros.h>

#include <mur_force_controller/mur_base.h>
#include <mur_force_controller/gazebo_ft_publisher.h>
#include <mur_force_controller/move_ur_compliant.h>

#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"endeffector_kinematics");
    ROS_INFO("Endeffector publishes data.");
    ros::NodeHandle nh;
    ros::Publisher pub_pose_ = nh.advertise<geometry_msgs::Pose>("/get_ee_kinematics/current_pose", 100);
    
    ros::Publisher pub_twist_ = nh.advertise<geometry_msgs::Twist>("/get_ee_kinematics/current_twist", 100);

    MurBase base_;
    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;
    std::vector<double> current_pose;
    std::vector<double> old_pose;
    current_pose = {0,0,0,0,0,0};
    old_pose = {0,0,0,0,0,0};
    
    std::string source_frame = "robot1_tf/base_link_ur5";
    std::string target_frame = "robot1_tf/ee_link_ur5";

    while(ros::ok())
    {
        /***** Call current pose *****/
        current_pose.clear();
        current_pose = base_.getCurrentPose(source_frame, target_frame);

        pose.position.x = current_pose.at(0);
        pose.position.y = current_pose.at(1);
        pose.position.z = current_pose.at(2);

        pose.orientation.x = current_pose.at(6);
        pose.orientation.y = current_pose.at(7);
        pose.orientation.z = current_pose.at(8);
        pose.orientation.w = current_pose.at(9);

        /***** Set current twist *****/
        twist.linear.x = (current_pose.at(0) - old_pose.at(0))/0.01;
        twist.linear.y = (current_pose.at(1) - old_pose.at(1))/0.01;
        twist.linear.z = (current_pose.at(2) - old_pose.at(2))/0.01;

        twist.angular.x = (current_pose.at(3) - old_pose.at(3))/0.01;
        twist.angular.y = (current_pose.at(4) - old_pose.at(4))/0.01;
        twist.angular.z = (current_pose.at(5) - old_pose.at(5))/0.01;

        /***** Publish pose *****/
        pub_pose_.publish(pose);

        /***** Publish velocity *****/
        pub_twist_.publish(twist);

        old_pose.clear();
        old_pose = current_pose;
    }

}