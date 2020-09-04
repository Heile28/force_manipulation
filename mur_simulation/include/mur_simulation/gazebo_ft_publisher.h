/*
 * File: gazebo_ft_publisher.h
 * Author: Heiko Lenz
 * 
 * Created on 21. August 2020
 * 
 * Header file providing a method sourcing the cartesian_admittance_controller
 * publishes measured WRENCH to admittance controller's reference link named 'end_effector_link'
 * -> as the Gazebo F/T-Sensor-Plugin is mounted on wrist3_link_ur5 the data have to be transformed into ee_link_ur5
 * 
*/

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

#include <eigen3/Eigen/Dense>


#ifndef GAZEBO_FT_PUBLISHER_H
#define GAZEBO_FT_PUBLISHER_H

class WrenchPublisher
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    geometry_msgs::WrenchStamped measured_wrench_;
    
public:
    //standard constructor
    WrenchPublisher();

    //destructor
    ~WrenchPublisher();

    struct Frames{};

    /**
    * @brief Callbacks current wrench and publishes it to cartesian_admittance_controller 
    * 
    */

    void wrenchCallback(geometry_msgs::WrenchStamped::ConstPtr wrench_msg);

    /**
    * @brief transfers current wrench of Gazebo-F/T-sensor-plugin fixed at 'wrist3_link_ur5' into 'ee_link_ur5'
    * 
    */
    void transform_wrench_into_ee();

};

#endif /* GAZEBO_FT_PUBLISHER_H */
