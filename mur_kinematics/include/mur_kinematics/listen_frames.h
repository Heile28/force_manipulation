/*
 * File: listen_frames.h
 * Author: Heiko Lenz
 * 
 * Created on 07. September 2020
 * 
 * Header file providing methods transforming between frames 
 * 
*/

//ROS
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

//ROS msgs
#include <geometry_msgs/Vector3.h>
#include <mur_robot_msgs/PoseRequest.h>
#include <mur_robot_msgs/PoseMessage.h>

//other
#include <cmath>

#ifndef LISTEN_FRAMES_H
#define LISTEN_FRAMES_H

namespace listen_frames{
    
    #define POSE_ARRAY_SIZE 6

    class ListenFrames
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
        /*
        Orientation orientation;
        Translation translation;
        Quaternion quaternion;
        */
        


    public:
        struct Orientation {double x; double y; double z;};
        struct Translation {double x; double y; double z;};
        struct Quaternion {double x; double y; double z; double w;};
        
        //standard constructor
        ListenFrames();
        
        //destructor
        ~ListenFrames();

        //methods
        /**
         * @brief callback function of joints data
         * 
         * @param source_frame frame origin
         * @param target_frame frame in which to transform
         */
        void getLinkTransformUR5(const std::string source_frame, const std::string target_frame);

        /**
         * @brief Service server to request for current endeffector pose
         * 
         * @param req activate to true if wanted
         * @param res is able to return cartesian position, orientation (quaternion + rpy)
         * @return true 
         * @return false 
         */
        bool callbackRequestEndeffector(mur_robot_msgs::PoseRequest::Request& req, mur_robot_msgs::PoseRequest::Response& res);
    };
}
#endif /* LISTEN_FRAMES_H */