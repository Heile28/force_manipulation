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
#include <mur_force_controller/gazebo_ft_publisher.h>
#include <mur_kinematics/get_jacobian.h>

int main(int argc, char** argv)
{
    //ros::init(argc,argv,"Drive_MiR");
    std::cout<<"Im in main file"<<std::endl;
    //ros::NodeHandle nh;

    

    //start publishing wrench data to cartesian_admittance_controller
    gazebo_ft_publisher::WrenchPublisher start_wrench;
    
    //start moving MiR robot
    /*move_compliant::MurBase begin;
    move_compliant::MoveMir obj1;
    obj1.lookupInitialPosition();
    */
    calculate_jacobian::GetJacobian get_jacobian;
    get_jacobian.urJacobian();


    ros::spin();
}