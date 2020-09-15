/*
 * File: mur_base.h
 * Author: Heiko Lenz
 * 
 * Created on 11. September 2020
 * 
 * Header file providing methods to get inherited by subclasses
 * 
*/

//ROS
#include <ros/ros.h>

//tf
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

//ROS messages
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <mur_robot_msgs/PoseRequest.h>
#include <mur_robot_msgs/PoseMessage.h>

//other
#include <cmath>

#ifndef MUR_BASE_H
#define MUR_BASE_H

/// \brief base class ready to get inherited
class MurBase
    {
    private:
        ros::NodeHandle nh_;
        ros::ServiceServer server_pose_;

        tf::TransformListener listener_;
        tf::StampedTransform transform_;
        tf::Vector3 r; //translation vector size 3
        tf::tfVector4 quat_rot; //Vektor der Rotation in Quaternion
        tf::Matrix3x3 R; //Rotation matrix
        geometry_msgs::Vector3 rpy;

        double roll, pitch, yaw; //angles in RAD
        double alpha, beta, gamma; //angles in DEG
        const double PI = M_PI;

    protected:
        /**
        * @brief callback function of joints data
        * 
        */
        void callbackJointAngles(sensor_msgs::JointState joint_msg);

        /**
         * @brief Service server to request for current endeffector pose
         * 
         * @param req activate to true if wanted
         * @param res is able to return cartesian position, orientation (quaternion + rpy)
         * @return true
         * @return false
         */
        bool callbackRequestEndeffector(mur_robot_msgs::PoseRequest::Request& req, mur_robot_msgs::PoseRequest::Response& res);

        /**
         * @brief callback function of joints data
         * 
         * @param source_frame frame origin
         * @param target_frame frame in which to transform
         */
        void getLinkTransformUR5(const std::string source_frame, const std::string target_frame);

        /**
         * @brief Returns an transformation object
         * 
         * @param source_frame_ 
         * @param target_frame_ 
         * @return tf::StampedTransform 
         */
        tf::StampedTransform transform(const std::string source_frame_, const std::string target_frame_);

    public:
        struct Orientation {double x; double y; double z;};
        struct Translation {double x; double y; double z;};
        struct Quaternion {double x; double y; double z; double w;};

        //standard constructor
        MurBase();
        
        //destructor
        ~MurBase();

        //methods

        ros::Subscriber joint_angles_;
        double theta1_, theta2_, theta3_, theta4_, theta5_, theta6_;

        /**
         * @brief Get the Angles object in coherence of a JointAnglesListener()
         * 
         * @return std::vector<double> theta_
         */
        std::vector<double> getAngles();

        /**
         * @brief Get the Current Pose object
         * 
         * @param source_frame_ 
         * @param target_frame_ 
         * @return std::vector<double> pose_
         */
        std::vector<double> getCurrentPose(std::string source_frame_, std::string target_frame_);

    };

    #endif /* MUR_BASE_H */