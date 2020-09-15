/*
 * File: move_mir_compliant.h
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
    this->pub_simple_ = nh_.advertise<geometry_msgs::Twist>("/robot1_ns/mobile_base_controller/cmd_vel", 100);
    this->endeffector_pose_client_ = nh_.serviceClient<mur_robot_msgs::PoseRequest>("listen_frames/request_endeffector/pose");
    //this->scan_pub_=this->nh_.advertise<sensor_msgs::PointCloud>("scan",10);
    init_time_ = ros::Time(0);
    for(unsigned i=0; i<6; i++)
        initial_pose_.push_back(0.0);
}

void MoveMir::lookupInitialPosition(){

        ros::service::waitForService("mur_base/listen_frames/request_endeffector/pose");

        mur_robot_msgs::PoseRequest pose_msg_;
        pose_msg_.request.request = true;
        pose_msg_.request.source_frame = "robot1_tf/base_link";
        pose_msg_.request.target_frame = "robot1_tf/ee_link_ur5";
        
        try{
            endeffector_pose_client_.call(pose_msg_);
            initial_pose_.clear();
            initial_pose_.push_back(pose_msg_.response.position.x);
            initial_pose_.push_back(pose_msg_.response.position.y);
            initial_pose_.push_back(pose_msg_.response.position.z);
            initial_pose_.push_back(pose_msg_.response.rpy_orientation.x);
            initial_pose_.push_back(pose_msg_.response.rpy_orientation.y);
            initial_pose_.push_back(pose_msg_.response.rpy_orientation.z);
            std::cout<<"Initial pose is: "<<initial_pose_[0]<<", "<<initial_pose_[1]<<", "<<initial_pose_[2]<<", "
                        <<initial_pose_[3]<<", "<<initial_pose_[4]<<", "<<initial_pose_[5]<<std::endl;
            //give vector initial values which are changeable
        }
        catch(ros::Exception &ex)
        {
            ROS_ERROR("Error occured %s", ex.what());
        }
        last_time_ = init_time_;
}

void MoveMir::callbackCurrentPose()
{
    std::string source_frame = "robot1_tf/base_link";
    std::string target_frame = "robot1_tf/ee_link_ur5";

    current_pose_.clear();
    current_pose_ = base_.getCurrentPose(source_frame, target_frame);

    std::cout<<"Current pose is "<<current_pose_[0]<<", "<<current_pose_[1]<<", "<<current_pose_[2]<<", "<<current_pose_[3]<<", "<<current_pose_[4]
    <<", "<<current_pose_[5]<<std::endl;

    current_time_ = ros::Time::now();
    dt_ = (current_time_-last_time_).toSec();
    last_time_ = ros::Time::now();

    displacementPose();
}

void MoveMir::displacementPose(){
    displacement_pose_.clear();
    for(int i=0; i<6; i++)
        displacement_pose_.push_back(current_pose_[i] - initial_pose_[i]);        

    std::cout<<"Displacement is ["<<displacement_pose_[0]<<", "<<displacement_pose_[1]<<", "<<displacement_pose_[2]<<", "<<displacement_pose_[3]<<", "<<displacement_pose_[4]
    <<", "<<displacement_pose_[5]<<"]"<<std::endl;

    vth_ = atan(displacement_pose_.at(0)/displacement_pose_.at(1))/dt_;

    moveGoal();

}

void MoveMir::transferIntoMir(Eigen::VectorXd displ_){
    /* transform into (KS)MiR using following matrix*/
    Eigen::MatrixXd T_(4,4);
    T_ << -0.99988004, -0.01548866, 0, 0.244311,
        0.01548866, -0.99988004, 0, 0.140242,
        0, 0, 1, 0.450477,
        0, 0, 0, 1;
    Eigen::VectorXd x_mir_;
    x_mir_ = T_.inverse()*displ_;
    std::cout<<"x_mir: "<<x_mir_<<std::endl;

    //calculate rotation using for nullspace movement
    //Bedingung: gazebo_force_torque_publisher muss laufen
    delta_th_ = atan(x_mir_(0)/x_mir_(1));
    std::cout<<"Delta th: \n"<<delta_th_<<std::endl;
    
    vth_ = delta_th_/dt_;

    //speed of rotation is fundamental -> we want to rotate as fast as possible due to endeffector's changing position
    
    moveGoal();
}

void MoveMir::moveGoal(){
    tw_msg_.linear.x = displacement_pose_.at(0)/dt_;
    tw_msg_.angular.z = vth_;
    pub_simple_.publish(tw_msg_);
}