/*
 * File: main.cpp
 * Author: Heiko Lenz
 * 
 * Created on 21. August 2020
 * 
 * Main file executing all actions driving the MuR robot
 * 
*/

#include <mur_force_controller/move_compliant.h>
#include <mur_simulation/gazebo_ft_publisher.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"Drive_MiR");  
    ros::NodeHandle nh;

    //start publishing wrench data to cartesian_admittance_controller
    gazebo_ft_publisher::WrenchPublisher startWrench;
    

    //start moving MiR robot
    move_compliant::MurBase begin;
    move_compliant::MoveMir obj1;
    obj1.lookupInitialPosition();

    ros::spin();
}