/*
 * File: main.cpp
 * Author: Heiko Lenz
 * 
 * Created on 21. August 2020
 * 
 * Main file executing actions driving MiR in conjunction with UR5
 * 
*/

#include <mur_force_controller/move_mir_compliant.h>
#include <mur_force_controller/move_ur_compliant.h>
#include <mur_kinematics/get_jacobian.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_box.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"Drive_MiR");
    std::cout<<"Ready to drive MiR by force"<<std::endl;
    //ros::NodeHandle nh;
    
    /***** Test manipulation measure w *****/
    //calculate_jacobian::GetJacobian get_jacobian;
    

    /***** Lookup initial angle *****/
    // MurBase base;
    // std::vector<double> static_pose;
    // std::string source_frame = "robot1_tf/base_link";
    // std::string target_frame = "robot1_tf/base_link_ur5";
    // static_pose = base.getCurrentPose(source_frame, target_frame);
    // double theta0 = atan2(static_pose[1],static_pose[0]);
    // ROS_INFO_STREAM("Static angle is "<<theta0);

    /***** start moving MiR robot *****/
    move_mir_compliant::MoveMir obj1;
    
    /*
    ros::Publisher pub_simple_ = nh.advertise<geometry_msgs::Twist>("/robot1_ns/mobile_base_controller/cmd_vel", 100);
    geometry_msgs::Twist tw_msg_;
    tw_msg_.linear.x = 0.0;
    tw_msg_.linear.y = 0.0;
    tw_msg_.linear.z = 0.0;
    tw_msg_.angular.x = 0.0;
    tw_msg_.angular.y = 0.0;
    tw_msg_.angular.z = 0.0;

    pub_simple_.publish(tw_msg_);
    ros::Rate r(2.0);
    r.sleep();

    tw_msg_.angular.z = 2.0;
    pub_simple_.publish(tw_msg_);
    */

    
    while(ros::ok()){
        //obj1.controlMethod1();
        obj1.controlMethod3();

        //get_jacobian.forwardKinematics();
        ros::spinOnce();
    }

    //ros::spin();
    return 0;
}