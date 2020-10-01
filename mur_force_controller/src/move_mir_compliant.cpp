/*
 * File: move_mir_compliant.cpp
 * Author: Heiko Lenz
 * 
 * Created on 20. August 2020
 * 
 * Source file providing methods driving the MiR plattform in conjunction 
 * to cartesian compliance controller
 * 
*/

#include <mur_force_controller/move_mir_compliant.h>

using namespace move_compliant;

//constructor
MoveMir::MoveMir()
{
    this->nh_=ros::NodeHandle("move_mir_compliant");
    this->lookupInitialGlobalPosition();
    this->lookupInitialWorldPosition();
    this->lookupInitialLocalPosition();
    this->pub_simple_ = nh_.advertise<geometry_msgs::Twist>("/robot1_ns/mobile_base_controller/cmd_vel", 100);
    this->pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot1_ns/arm_cartesian_compliance_controller/target_pose", 100);
    this->pub_angle_ = nh_.advertise<std_msgs::Float64>("/move_mir_compliant/rotation_angle", 100);
    this->sub_force_ = nh_.subscribe("/robot1_ns/arm_cartesian_compliance_controller/ft_sensor_wrench", 100, &MoveMir::wrenchCallback, this);
    
    //this->scan_pub_=this->nh_.advertise<sensor_msgs::PointCloud>("scan",10);
    init_time_ = ros::Time(0);
    this->rotation_angle_.data = 0.0;
    this->theta_global_ = 0.0;
    this->rot_angle_old = 0.0;
    this->activate_ = 0;
}
void MoveMir::lookupInitialWorldPosition(){
    ros::service::waitForService("/mur_base/listen_frames/request_endeffector/pose");
    this->endeffector_pose_client_ = nh_.serviceClient<mur_robot_msgs::PoseRequest>("/mur_base/listen_frames/request_endeffector/pose");

    mur_robot_msgs::PoseRequest pose_msg_;
    pose_msg_.request.request = true;
    pose_msg_.request.source_frame = "map";
    pose_msg_.request.target_frame = "robot1_tf/ee_link_ur5";

    //ros::service::waitForService("/mur_base/listen_frames/request_endeffector/pose", ros::Duration(2));
    
    if(endeffector_pose_client_.call(pose_msg_))
    {
        initial_world_pose_.clear();
        initial_world_pose_.push_back(pose_msg_.response.position.x);
        initial_world_pose_.push_back(pose_msg_.response.position.y);
        initial_world_pose_.push_back(pose_msg_.response.position.z);
        initial_world_pose_.push_back(pose_msg_.response.rpy_orientation.x);
        initial_world_pose_.push_back(pose_msg_.response.rpy_orientation.y);
        initial_world_pose_.push_back(pose_msg_.response.rpy_orientation.z);
        std::cout<<"Initial world pose is: "<<initial_world_pose_[0]<<", "<<initial_world_pose_[1]<<", "<<initial_world_pose_[2]<<", "
                    <<initial_world_pose_[3]<<", "<<initial_world_pose_[4]<<", "<<initial_world_pose_[5]<<std::endl;
        //give vector initial values which are changeable
        //delta_x_ = initial_global_pose_[0];
        //delta_y_ = initial_global_pose_[1];
    }
    else
    {
        ROS_ERROR("Service call failed!");
    }
    //theta0_world_ = atan2(initial_world_pose_[1],initial_world_pose_[0]);
    //std::cout<<"Initial theta0: "<<theta0_world_<<std::endl;
    last_time_ = init_time_;
}

void MoveMir::lookupInitialGlobalPosition(){

        ros::service::waitForService("/mur_base/listen_frames/request_endeffector/pose");
        this->endeffector_pose_client_ = nh_.serviceClient<mur_robot_msgs::PoseRequest>("/mur_base/listen_frames/request_endeffector/pose");

        mur_robot_msgs::PoseRequest pose_msg_;
        pose_msg_.request.request = true;
        pose_msg_.request.source_frame = "robot1_tf/base_link";
        pose_msg_.request.target_frame = "robot1_tf/ee_link_ur5";

        //ros::service::waitForService("/mur_base/listen_frames/request_endeffector/pose", ros::Duration(2));
        
        if(endeffector_pose_client_.call(pose_msg_))
        {
            initial_global_pose_.clear();
            initial_global_pose_.push_back(pose_msg_.response.position.x);
            initial_global_pose_.push_back(pose_msg_.response.position.y);
            initial_global_pose_.push_back(pose_msg_.response.position.z);
            initial_global_pose_.push_back(pose_msg_.response.orientation.x);
            initial_global_pose_.push_back(pose_msg_.response.orientation.y);
            initial_global_pose_.push_back(pose_msg_.response.orientation.z);
            initial_global_pose_.push_back(pose_msg_.response.orientation.w);
            std::cout<<"Initial global pose is: "<<initial_global_pose_[0]<<", "<<initial_global_pose_[1]<<", "<<initial_global_pose_[2]<<", "
                        <<initial_global_pose_[3]<<", "<<initial_global_pose_[4]<<", "<<initial_global_pose_[5]<<", "<<initial_global_pose_[6]<<std::endl;
            //give vector initial values which are changeable
            //delta_x_ = initial_global_pose_[0];
            //delta_y_ = initial_global_pose_[1];
        }
        else
        {
            ROS_ERROR("Service call failed!");
        }
        initial_pose_.clear();
        initial_pose_ = initial_global_pose_;
        theta0_global_ = atan2(initial_global_pose_[1],initial_global_pose_[0]);
        std::cout<<"Initial theta0: "<<theta0_global_<<std::endl;
        last_time_ = init_time_;
        
}

void MoveMir::lookupInitialLocalPosition(){

        ros::service::waitForService("/mur_base/listen_frames/request_endeffector/pose");
        this->endeffector_pose_client_ = nh_.serviceClient<mur_robot_msgs::PoseRequest>("/mur_base/listen_frames/request_endeffector/pose");

        mur_robot_msgs::PoseRequest pose_msg_;
        pose_msg_.request.request = true;
        pose_msg_.request.source_frame = "robot1_tf/base_link_ur5";
        pose_msg_.request.target_frame = "robot1_tf/ee_link_ur5";

        if(endeffector_pose_client_.call(pose_msg_))
        {
            std::cout<<"Inside service call"<<std::endl;

            //endeffector_pose_client_.call(pose_msg_);
            initial_local_pose_.clear();
            initial_local_pose_.push_back(pose_msg_.response.position.x);
            initial_local_pose_.push_back(pose_msg_.response.position.y);
            initial_local_pose_.push_back(pose_msg_.response.position.z);
            initial_local_pose_.push_back(pose_msg_.response.orientation.x);
            initial_local_pose_.push_back(pose_msg_.response.orientation.y);
            initial_local_pose_.push_back(pose_msg_.response.orientation.z);
            initial_local_pose_.push_back(pose_msg_.response.orientation.w);
            std::cout<<"Initial local pose is: "<<initial_local_pose_[0]<<", "<<initial_local_pose_[1]<<", "<<initial_local_pose_[2]<<", "
                        <<initial_local_pose_[3]<<", "<<initial_local_pose_[4]<<", "<<initial_local_pose_[5]<<", "<<initial_local_pose_[6]<<std::endl;
            
            
        }
        else
        {
            ROS_ERROR("Service call failed!");
        }
        theta0_local_ = atan2(initial_local_pose_[1],initial_local_pose_[0]);

        initial_pose_.clear();
        initial_pose_ = initial_local_pose_;
        last_time_ = init_time_;
}

std::vector<double> MoveMir::callCurrentGlobalPose()
{
    std::string source_frame = "robot1_tf/base_link";
    std::string target_frame = "robot1_tf/ee_link_ur5";

    current_global_pose_.clear();
    current_global_pose_ = base_.getCurrentPose(source_frame, target_frame);

    //std::cout<<"Current global pose is "<<current_global_pose_[0]<<", "<<current_global_pose_[1]<<", "<<current_global_pose_[2]
    //               <<", "<<current_global_pose_[3]<<", "<<current_global_pose_[4]<<", "<<current_global_pose_[5]<<std::endl;
    
    // current_time_ = ros::Time::now();
    // dt_ = (current_time_-last_time_).toSec();

    current_pose_.clear();
    current_pose_ = current_global_pose_;
    //last_time_ = ros::Time::now();

    theta_global_ = atan2(current_global_pose_[1],current_global_pose_[0]);

    // /**** Create a unit vector directed into x-axis ****/ 
    // mir_direction_.setX(current_global_pose_[0]);
    // mir_direction_.setY(current_global_pose_[1]);
    // mir_direction_.setZ(current_global_pose_[2]);
    // mir_direction_.normalize();
    // mir_direction_.setY(0.0);
    // mir_direction_.setZ(0.0);

    return current_global_pose_;
}

std::vector<double> MoveMir::callCurrentLocalPose()
{
    const std::string source_frame = "robot1_tf/base_link_ur5";
    const std::string target_frame = "robot1_tf/ee_link_ur5";

    current_local_pose_.clear();
    current_local_pose_ = base_.getCurrentPose(source_frame, target_frame);

    // std::cout<<"Current pose is "<<current_local_pose_[0]<<", "<<current_local_pose_[1]<<", "<<current_local_pose_[2]
    //                 <<", "<<current_local_pose_[3]<<", "<<current_local_pose_[4]<<", "<<current_local_pose_[5]<<std::endl;
    

    // current_time_ = ros::Time::now();
    // dt_ = (current_time_-last_time_).toSec();
    //std::cout<<"Current dt: "<<dt_<<std::endl;

    current_pose_.clear();
    current_pose_ = current_local_pose_;
    //last_time_ = ros::Time::now();

    return current_local_pose_;
}

std::vector<double> MoveMir::callCurrentWorldPose()
{
    current_map_pose_.clear();
    current_mir_map_pose_.clear();
    
    /**** EE in world frame ****/
    const std::string source_frame = "map";
    std::string target_frame = "robot1_tf/ee_link_ur5";
    current_map_pose_ = base_.getCurrentPose(source_frame, target_frame);
    
    /**** MiR in world frame ****/
    target_frame = "robot1_tf/base_link";
    current_mir_map_pose_ = base_.getCurrentPose(source_frame, target_frame);

    theta_world_ = atan2(current_map_pose_[1], current_map_pose_[0]);
    theta_mir_world_ = atan2(current_mir_map_pose_[1], current_mir_map_pose_[0]);

    //last_time_ = ros::Time::now();
    return current_map_pose_, current_mir_map_pose_;
}

void MoveMir::wrenchCallback(geometry_msgs::WrenchStamped wrench_msg_){
    force_.x = wrench_msg_.wrench.force.x;
    force_.y = wrench_msg_.wrench.force.y;
    force_.z = wrench_msg_.wrench.force.z;
    torque_.x = wrench_msg_.wrench.torque.x;
    torque_.y = wrench_msg_.wrench.torque.y;
    torque_.z = wrench_msg_.wrench.torque.z;
    
    //std::cout<<"Wrench is"<<force_.x<<", "<<force_.y<<", "<<force_.z<<std::endl;

    if(abs(force_.x) > 5.0 || abs(force_.y) > 5.0|| abs(force_.z) > 5.0){
        activate_ = 1;
        ROS_INFO("Force attack! -> activate_ = 1");
    }
    //     poseUpdater();
    //     poseUpdater2();
    //std::cout<<"Wrench is"<<force_.x<<", "<<force_.y<<", "<<force_.z<<std::endl;

}

void MoveMir::moveGoal()
{
    do{
        double x_length = sqrt(pow((displacement_pose_.at(0)),2)+pow((displacement_pose_.at(1)),2) );
        tw_msg_.linear.x = x_length*cos(delta_th_)/dt_;
        tw_msg_.angular.z = vth_;
        pub_simple_.publish(tw_msg_);
        last_pose_ = current_pose_;
        func_case_ = 1;
        callCurrentLocalPose();
    }while (current_local_pose_==last_pose_);
    
}

void MoveMir::nullspace(double theta_)
{
    /***** Rotate MiR platform *****/
    double v = ( sqrt(pow(current_mir_map_pose_[0]-current_map_pose_[0],2) +
            pow(current_mir_map_pose_[1]-current_map_pose_[1],2)) )/dt_;
    tw_msg_.linear.x = 0.0;
    tw_msg_.linear.y = 0.0;
    tw_msg_.linear.z = 0.0;
    tw_msg_.angular.x = 0.0;
    tw_msg_.angular.y = 0.0;
    //std::cout<<"MiR angle "<<angle_<<std::endl;
    //tw_msg_.angular.z = angle_/0.01;
    tw_msg_.angular.z = theta_; ///0.01;

    pub_simple_.publish(tw_msg_);

    /***** Publish angle to compliance controller *****/
    pub_angle_.publish(rotation_angle_);
}



// void MoveMir::publishVelocityCommand()
// {
//     geometry_msgs::Twist msg_vel;
//     msg_vel.linear.x=this->control_.v;
//     msg_vel.angular.z=this->control_.omega;
//     this->pub_cmd_vel.publish(msg_vel);
//}


void MoveMir::rotateToForceDirection()
{
        /***** Request current global pose *****/
        callCurrentGlobalPose();

        /***** Transform wrench vector from ft_sensor_ref_link into ~/base_link *****/
        transform_ = base_.transform("robot1_tf/wrist_3_link_ur5", "robot1_tf/base_link");
        
        tf::vector3MsgToTF(force_,tf_force_at_sensor_);
        tf_force_at_base_ = transform_.getBasis().inverse() * tf_force_at_sensor_;
        tf::vector3TFToMsg(tf_force_at_base_, force_at_base_);

        /***** Calculate vector orientation in x-y plane *****/
        force_at_base_.z = 0;
        double rot_angle2  = atan2(force_at_base_.y, force_at_base_.x);
        rot_angle2 = normalize_angle(rot_angle2);
        ROS_INFO_STREAM("Orientation of angle: "<<rot_angle2);
        
        rotateToPoseDirection(rot_angle2);
   
}

void MoveMir::poseUpdater()
{
    callCurrentGlobalPose();
    callCurrentLocalPose();
    current_time_ = ros::Time::now();
    dt_ = (current_time_-last_time_).toSec();
    last_time_ = ros::Time::now();
    
    //rotation_angle_.data = theta0_local_;

    /***IF CASES FOR INSIDE UR5 ***/

    
    //case for second frame quadrant
    if(theta_global_<0.0)
    {
        
        if(theta_global_ < theta0_global_)
        {
            rotation_angle_.data = theta_global_+theta0_global_;
            ROS_INFO_STREAM("Case 1.1: angle is"<<rotation_angle_.data);
            //pub_angle_.publish(rotation_angle_);
            if(abs(rotation_angle_.data)>0.017453293) //minimally 1 degree
                nullspace(rotation_angle_.data);
        }
        else
        {
            rotation_angle_.data = theta0_global_-theta_global_;
            ROS_INFO_STREAM("Case 1.2: angle is"<<rotation_angle_.data);
            //pub_angle_.publish(rotation_angle_);
            if(abs(rotation_angle_.data)>0.017453293) //minimally 1 degree
                nullspace(rotation_angle_.data);
        }

    }
    
    //case for first frame quadrant
    if(theta_global_ >0.0){
        rotation_angle_.data = theta0_global_ + theta_global_; //angle assigning desired pose x_d
        ROS_INFO_STREAM("Case 2: angle is"<<rotation_angle_.data);
        //pub_angle_.publish(rotation_angle_);
        if(abs(rotation_angle_.data)>0.017453293) //minimally 1 degree
            nullspace(rotation_angle_.data);
    }
    else
    {
        rotation_angle_.data = 0.0;
        //pub_angle_.publish(rotation_angle_);
        nullspace(rotation_angle_.data);
    }

}

void MoveMir::poseUpdater2()
{
    callCurrentGlobalPose();
    callCurrentWorldPose();

    //double theta_star = atan2(current_map_pose_[1]-current_mir_map_pose_[1],current_map_pose_[0]-current_mir_map_pose_[0]);
    double theta_star = atan2(current_map_pose_[1]-current_mir_map_pose_[1], current_map_pose_[0]-current_mir_map_pose_[0]);
    //std::cout<<"theta_star: "<<theta_star<<std::endl;

    double rot_angle1 = theta_global_ - theta_star;
    rot_angle1 = normalize_angle(rot_angle1);
    //std::cout<<"rot_angle: "<<rot_angle<<std::endl;

    //rotation_angle_.data = rot_angle-rot_angle_old-theta0_local_; //for ur5 rotation
    rotation_angle_.data = rot_angle1+theta0_local_; //for ur5 rotation
    //std::cout<<"Rotation angle "<<rotation_angle_.data<<std::endl;
    ROS_INFO_STREAM("Theta global: "<<theta_global_<<" and theta initial: "<<theta0_global_);
    
    if(abs(abs(theta_global_) - abs(theta0_global_)) > 0.1 && activate_ == 1) //1
    {
        rotateToPoseDirection(rot_angle1);
        rotateToForceDirection();
        activate_ = 0;
    }
    if(abs(abs(theta_global_) - abs(theta0_global_)) < 0.1 && activate_ == 1)
        moveStraight();
    else
    {
        activate_=0;
    }
    
    
    /*
    ros::Rate r(3.0);
    //PI in 3 seconds -> PI/3 rad/s rotation speed -> 1/3 -> 3Hz
    double current_time = ros::Time(0).toSec();
    double time = 3/ PI * abs(rotation_angle_.data);

    while(abs(abs(theta_global_) - abs(theta0_global_)) > 0.01)
    //if(abs(force_.x) > 5.0 || abs(force_.y) > 5.0|| abs(force_.z) > 5.0)
    {
        
        ROS_INFO_STREAM("time: "<<time);
        //double current_time = ros::Time::now().toSec();
        current_time = ros::Time(0).toSec();
        ROS_INFO_STREAM("current time: "<<current_time);
        while(current_time <= time)
        {
            ROS_INFO("Inside moving");
            
            //pub_angle_.publish(rotation_angle_);
            
            nullspace(rotation_angle_.data/time);
            start_ur_.moveInitialPose();    

            //rot_angle_old = rot_angle;
            callCurrentGlobalPose();
            current_time = current_time + 1/3;
            ROS_INFO_STREAM("current time: "<<current_time);
            ROS_INFO_STREAM("time: "<<time);
            r.sleep();
        }
        //start_ur_.moveInitialPose();
    
    }
    ROS_INFO("No force attack.");
    */
}

void MoveMir::poseUpdater3()
{
    ros::Duration(1.0).sleep();
    //if(abs(force_.x) > 5.0 || abs(force_.y) > 5.0|| abs(force_.z) > 5.0)
    //{
        
    callCurrentLocalPose();
    double theta = atan2(current_local_pose_[1]-initial_local_pose_[1], current_local_pose_[0]-initial_local_pose_[0]);
    
    double time = 2;

    //ros::Rate r(10.0);
    //double current_time = ros::Time::now().toSec();
    //timeline is 2 seconds -> 20 steps

    //while (current_time <= time)
    //{
    //    if(theta == 0)
    //        break;
        
        if(theta != 0)
        {
            nullspace(theta/20); //divided in 20 steps
            rotation_angle_.data = theta/20;
            pub_angle_.publish(rotation_angle_);

            //current_time = current_time + 0.01;
            ROS_INFO_STREAM("Theta is "<<theta);
            //callCurrentLocalPose();
            //theta = atan2(current_local_pose_[1]-initial_local_pose_[1], current_local_pose_[0]-initial_local_pose_[0]);
        }
        
    }
    //}


void MoveMir::rotateToPoseDirection(double rot_angle)
{
    ROS_INFO("Test: Inside updater4");
    //double theta_current = 0.0;
    ros::Rate r(3.0);
    //PI in 3 seconds -> PI/3 rad/s rotation speed -> 1/3 -> 3Hz
    double current_time = ros::Time(0).toSec();
    double time = 3/ PI * abs(rot_angle);
    x_dot_ = rot_angle/time * 0.44521/2; //445.21 distance
    ROS_INFO_STREAM("MiR time: "<<time);
    ROS_INFO_STREAM("MiR has to execute rotation angle: "<<rot_angle);
    while(current_time < time)
    {
        /***** execute nullspace movement *****/   
        rotation_angle_.data = rot_angle/time;
        nullspace(rotation_angle_.data);
        
        current_time = current_time + time;
        //ROS_INFO_STREAM("Current time is "<<current_time);
        r.sleep();
    }
}

void MoveMir::moveStraight()
{
    tw_msg_.linear.x = x_dot_;//( sqrt(pow(current_map_pose_[0]-current_global_pose_[0] - initial_local_pose_[0],2) + 
                          //  pow(current_map_pose_[1]-current_global_pose_[1] - initial_local_pose_[1],2)) )/dt_; //Sicherheitsabstand einhalten
    tw_msg_.linear.y = 0.0;
    tw_msg_.linear.z = 0.0;
    tw_msg_.angular.x = 0.0;
    tw_msg_.angular.y = 0.0;
    tw_msg_.angular.z = 0.0;

    pub_simple_.publish(tw_msg_);

}

double MoveMir::normalize_angle(double angle)
{
    const double result = fmod((angle + M_PI), 2.0*M_PI);
    if(result <= 0.0) return result + M_PI;
    return result - M_PI;
}
