/*
 * File: main.cpp
 * Author: Heiko Lenz
 * 
 * Created on 21. August 2020
 * 
 * Applying force profiles at wrist3_link
 * 
*/

#include <ros/ros.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/WrenchStamped.h>

void apply_constant_force(ros::NodeHandle nh_)
{
    ros::service::waitForService("/gazebo/apply_body_wrench"); //link: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_msgs/srv/ApplyBodyWrench.srv
    ros::ServiceClient force_client = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    gazebo_msgs::ApplyBodyWrench srv1; //namespace + servicename
    ros::Duration duration_seconds(5.0); //specify duration of force attack in seconds
    
    srv1.request.body_name = "robot1::robot1_tf/wrist_3_link_ur5"; //robot1::robot1_tf/base_footprint
    srv1.request.reference_frame = "robot1_tf/wrist_3_link_ur5";
    srv1.request.start_time = ros::Time().now();
    srv1.request.duration = duration_seconds;
    srv1.request.reference_point.x = 0.0;
    srv1.request.reference_point.y = 0.0; //0.0823;
    srv1.request.reference_point.z = 0.0;
    srv1.request.wrench.force.x = 0.0;
    srv1.request.wrench.force.y = -40.0;
    srv1.request.wrench.force.z = 0.0;
    srv1.request.wrench.torque.x = 0.0;
    srv1.request.wrench.torque.y = 0.0;
    srv1.request.wrench.torque.z = 0.0;
    force_client.call(srv1);

    if(srv1.response.success == true){
        ROS_INFO("Force applied for %lf seconds", duration_seconds.toSec());
    }
    ROS_INFO("Force attack has finished!");
        
}

void profiled_attack()
{

}


/**** METHOD publishes  WRENCH referenced to ~/wrist_3_link ****/
void directly_to_ee(ros::NodeHandle nh_)
{
    ros::Publisher pub_wrench = nh_.advertise<geometry_msgs::WrenchStamped>("/robot1_ns/arm_cartesian_compliance_controller/target_wrench",100);
    geometry_msgs::WrenchStamped pub_wrench_msg;
    pub_wrench_msg.header.frame_id = "robot1_tf/wrist_3_link_ur5"; //wrist_3_link_ur5
    pub_wrench_msg.header.stamp = ros::Time::now();
    pub_wrench_msg.wrench.force.x = 0.0;
    pub_wrench_msg.wrench.force.z = 0.0;
    pub_wrench_msg.wrench.torque.x = 0.0;
    pub_wrench_msg.wrench.torque.y = 0.0;
    pub_wrench_msg.wrench.torque.z = 0.0;

    double current_time = ros::Time::now().toSec();
    double time1 = 2.0;
    double time2 = 6.0;
    double time3 = 8.0;
    ros::Rate r(20.0); //0.05 seconds publishing rate

    ROS_INFO("Force attack for 45 seconds in Simulation (=8 seconds in real).");
    std::cout<<"Current time: "<<current_time<<std::endl;

    while(current_time <= time1){
        pub_wrench_msg.wrench.force.z = 7.5*pow(current_time, 2);
        //std::cout<<"Force in y: "<<pub_wrench_msg.wrench.force.y<<std::endl;
        pub_wrench.publish(pub_wrench_msg);
        current_time = current_time + 0.05;
        //std::cout<<"Current time: "<<current_time<<std::endl;
        r.sleep();
    }

    while(current_time <= time2){
        pub_wrench_msg.wrench.force.z = 30.0;
        std::cout<<"Force in y: "<<pub_wrench_msg.wrench.force.y<<std::endl;
        pub_wrench.publish(pub_wrench_msg);
        current_time = current_time + 0.05;
        std::cout<<"Current time: "<<current_time<<std::endl;
        r.sleep();
    }

    while(current_time <= time3){
        pub_wrench_msg.wrench.force.z = 7.5*pow(current_time-8.0, 2);
        //std::cout<<"Force in y: "<<pub_wrench_msg.wrench.force.y<<std::endl;
        pub_wrench.publish(pub_wrench_msg);
        current_time = current_time + 0.05;
        //std::cout<<"Current time: "<<current_time<<std::endl;
        r.sleep();
    }
    std::cout<<"Publishing finished!"<<std::endl;

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"apply_force");
    ros::NodeHandle nh_;
    apply_constant_force(nh_);
    //directly_to_ee(nh_);

    

    ros::spin();
}