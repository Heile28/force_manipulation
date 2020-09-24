/*
 * File: main.cpp
 * Author: Heiko Lenz
 * 
 * Created on 21. August 2020
 * 
 * Main file executing all actions driving the MuR robot
 * 
*/

#include <mur_force_controller/move_mir_compliant.h>
#include <mur_kinematics/get_jacobian.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"Drive_MiR");
    std::cout<<"Im in main file now"<<std::endl;
    ros::NodeHandle nh;
    
    // calculate_jacobian::GetJacobian get_jacobian;
    // get_jacobian.urJacobian();

    /***** start moving MiR robot *****/
    move_compliant::MoveMir obj1;

    while(ros::ok()){
        obj1.rotateToPoseDirection();
        ros::spinOnce();
    }
    

    /*
    while (ros::ok())
    {
        obj1.rotateToForceDirection();
        ros::spinOnce();
    }
    */

    ros::spin();
}