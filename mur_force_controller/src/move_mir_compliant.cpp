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
MurBase::MurBase()
{
    this->nh_ = ros::NodeHandle("mur_base");
    this->joint_angles_ = nh_.subscribe("/robot1_ns/joint_states", 10, &MurBase::callbackJointAngles, this);
    /*
    this->theta1_ = 0.0; 
    this->theta2_ = 0.0;
    this->theta3_ = 0.0;
    this->theta4_ = 0.0;
    this->theta5_ = 0.0;
    this->theta6_ = 0.0;
    */
}

//destructor
MurBase::~MurBase()
{
    printf("Angles have been deleted\n");
}

void MurBase::callbackJointAngles(const sensor_msgs::JointState::ConstPtr& joint_msg_)
{
    this->theta1_ = joint_msg_->position[0];
    this->theta2_ = joint_msg_->position[1];
    this->theta3_ = joint_msg_->position[2];
    this->theta4_ = joint_msg_->position[3];
    this->theta5_ = joint_msg_->position[4];
    this->theta6_ = joint_msg_->position[5];
    
    std::cout<<"Winkel:"<<std::endl;
    std::cout<<"["<<theta1_<<","<<theta2_<<","<<theta3_<<","<<theta4_<<","<<theta5_<<","<<theta6_<<"]"<<std::endl;
    
}

//constructor
MoveMir::MoveMir()
{
    this->nh_=ros::NodeHandle("move_mir_compliant");
    this->sub_pose_ = nh_.subscribe("/cartesian/endeffector_pose/data", 100, &MoveMir::callbackCurrentPose, this); //from listen_frames_node
    this->pub_simple_ = nh_.advertise<geometry_msgs::Twist>("/robot1_ns/mobile_base_controller/cmd_vel", 100);
    this->endeffector_pose_client_ = nh_.serviceClient<mur_robot_msgs::PoseRequest>("listen_frames/request_endeffector/pose");
    //this->scan_pub_=this->nh_.advertise<sensor_msgs::PointCloud>("scan",10);
    init_time_ = ros::Time(0);
    for(unsigned i=0; i<6; i++)
        initial_pose_.push_back(0.0);
}

void MoveMir::lookupInitialPosition(){

        ros::service::waitForService("listen_frames/request_endeffector/pose");

        mur_robot_msgs::PoseRequest pose_msg_;
        pose_msg_.request.request = true;
        
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

void MoveMir::callbackCurrentPose(mur_robot_msgs::PoseMessage current_pose_msg_)
{
    current_pose_.clear();
    current_pose_.push_back(current_pose_msg_.position.x);
    current_pose_.push_back(current_pose_msg_.position.y);
    current_pose_.push_back(current_pose_msg_.position.z);
    current_pose_.push_back(current_pose_msg_.rpy_orientation.x);
    current_pose_.push_back(current_pose_msg_.rpy_orientation.y);
    current_pose_.push_back(current_pose_msg_.rpy_orientation.z);

    std::cout<<"Current pose is "<<current_pose_[0]<<", "<<current_pose_[1]<<", "<<current_pose_[2]<<", "<<current_pose_[3]<<", "<<current_pose_[4]
    <<", "<<current_pose_[5]<<std::endl;

    current_time_ = ros::Time::now();
    dt_ = (current_time_-last_time_).toSec();
    last_time_ = ros::Time::now();

    displacementPose();
}

void MoveMir::displacementPose(){
    displacement_pose_.clear();
        for(int i=0; i<6; i++){
            displacement_pose_.push_back(current_pose_[i] - initial_pose_[i]);
        }
        
        for(unsigned i = 0; i<6; i++){
            displacement_pose_[i] = (int)(displacement_pose_[0]*100+0.5)/100.0;
        }
        std::cout<<"Displacement is ["<<displacement_pose_[0]<<", "<<displacement_pose_[1]<<", "<<displacement_pose_[2]<<", "<<displacement_pose_[3]<<", "<<displacement_pose_[4]
        <<", "<<displacement_pose_[5]<<"]"<<std::endl;

        Eigen::VectorXd displ_(4);
        displ_ << displacement_pose_[0], displacement_pose_[1], displacement_pose_[2], 1;
        transferIntoMir(displ_);
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
    tw_msg_.linear.x = 0.0;
    tw_msg_.linear.y = 0.0;
    tw_msg_.angular.z = vth_;
    pub_simple_.publish(tw_msg_);
}