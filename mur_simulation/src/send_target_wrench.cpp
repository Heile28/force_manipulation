/*
 * File: main.cpp
 * Author: Heiko Lenz
 * 
 * Created on 03. October 2020
 * 
 * Send target wrench to compliance controller
 * 
*/

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

class SendTarget
{
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_wrench = nh_.advertise<geometry_msgs::WrenchStamped>("/robot1_ns/arm_cartesian_compliance_controller/target_wrench",20);
        ros::Subscriber sub_wrench = nh_.subscribe("/apply_force/target_wrench", 100, &SendTarget::wrenchCallback, this);
        geometry_msgs::WrenchStamped pub_wrench_msg;

    public:

        void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg)
        {
            pub_wrench_msg.wrench.force.x = wrench_msg->wrench.force.x;
            pub_wrench_msg.wrench.force.y = wrench_msg->wrench.force.y;
            pub_wrench_msg.wrench.force.z = wrench_msg->wrench.force.z;

            pub_wrench_msg.wrench.torque.x = wrench_msg->wrench.torque.x;
            pub_wrench_msg.wrench.torque.y = wrench_msg->wrench.torque.y;
            pub_wrench_msg.wrench.torque.z = wrench_msg->wrench.torque.z;

            sendTargetWrench();
        }

        void sendTargetWrench()
        {
            /*** Sends wrench to ~/arm_cartesian_compliance_controller/target_wrench ***/

            ROS_INFO("TARGET WRENCH sent");

            pub_wrench_msg.header.stamp = ros::Time::now();
            pub_wrench.publish(pub_wrench_msg);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc,argv,"send_target_wrench");
    ros::NodeHandle nh_;

    ROS_INFO("Ready to send new target to compliance controller.");

    SendTarget obj;
    ros::spin();
}