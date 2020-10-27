/*
 * File: move_ur_compliant.cpp
 * Author: Heiko Lenz
 * 
 * Created on 25. September 2020
 * 
 * Source file providing methods driving the UR in conjunction 
 * to MiR movement
 * 
*/

#include <mur_force_controller/move_ur_compliant.h>
#include <eigen3/Eigen/Dense>
#include <ur_kinematics/ur_kin.h>


using namespace move_ur_compliant;

MoveUR::MoveUR()
{
    //this->nh_=nh_;
    this->rotation_angle_ = nh_.subscribe("/move_mir_compliant/rotation_angle", 100, &MoveUR::angleCallback, this);
    this->pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot1_ns/arm_cartesian_compliance_controller/target_pose", 100);
    this->lookupInitialLocalPosition();
    this->current_local_pose_ = initial_local_pose_;
    this->lookupInitialGlobalPosition();
    this->q0_ = 0.0;
}

MoveUR::~MoveUR(){}

void MoveUR::angleCallback(std_msgs::Float64 angle_)
{
    q0_ = angle_.data;

    /**** Methods proceeding subscribed angle ****/
    //nullspace(theta_);
    //moveInitialPose();

    if(abs(q0_) > 0.03)
        rotateAngle(q0_);
    else
        moveInitialPose();
}

void MoveUR::lookupInitialLocalPosition(){

    ros::service::waitForService("/mur_base/listen_frames/request_endeffector/pose");
    this->endeffector_pose_client_ = nh_.serviceClient<mur_robot_msgs::PoseRequest>("/mur_base/listen_frames/request_endeffector/pose");

    mur_robot_msgs::PoseRequest pose_msg_;
    pose_msg_.request.request = true;
    pose_msg_.request.source_frame = "robot1_tf/base_link_ur5";
    pose_msg_.request.target_frame = "robot1_tf/ee_link_ur5";

    if(endeffector_pose_client_.call(pose_msg_))
    {
        std::cout<<"Inside service call"<<std::endl;
        //RPY ANGLES CHOSEN !!!

        //endeffector_pose_client_.call(pose_msg_);
        initial_local_pose_.clear();
        initial_local_pose_.push_back(pose_msg_.response.position.x);
        initial_local_pose_.push_back(pose_msg_.response.position.y);
        initial_local_pose_.push_back(pose_msg_.response.position.z);
        initial_local_pose_.push_back(pose_msg_.response.rpy_orientation.x);
        initial_local_pose_.push_back(pose_msg_.response.rpy_orientation.y);
        initial_local_pose_.push_back(pose_msg_.response.rpy_orientation.z);
        initial_local_pose_.push_back(pose_msg_.response.orientation.x);
        initial_local_pose_.push_back(pose_msg_.response.orientation.y);
        initial_local_pose_.push_back(pose_msg_.response.orientation.z);
        initial_local_pose_.push_back(pose_msg_.response.orientation.w);
        std::cout<<"Initial local pose is: "<<initial_local_pose_[0]<<", "<<initial_local_pose_[1]<<", "<<initial_local_pose_[2]<<", "
                    <<initial_local_pose_[6]<<", "<<initial_local_pose_[7]<<", "<<initial_local_pose_[8]<<", "<<initial_local_pose_[9]<<std::endl;
        
    }
    else
    {
        ROS_ERROR("Service call failed!");
    }
    last_time_ = init_time_;
}

void MoveUR::lookupInitialGlobalPosition(){

        ros::service::waitForService("/mur_base/listen_frames/request_endeffector/pose");
        this->endeffector_pose_client_ = nh_.serviceClient<mur_robot_msgs::PoseRequest>("/mur_base/listen_frames/request_endeffector/pose");

        mur_robot_msgs::PoseRequest pose_msg_;
        pose_msg_.request.request = true;
        pose_msg_.request.source_frame = "robot1_tf/base_link";
        pose_msg_.request.target_frame = "robot1_tf/ee_link_ur5";

        //ros::service::waitForService("/mur_base/listen_frames/request_endeffector/pose", ros::Duration(2));
        
        if(endeffector_pose_client_.call(pose_msg_))
        {
            initial_global_pose_.header.frame_id = "robot1_tf/base_link";
            initial_global_pose_.pose.position.x = pose_msg_.response.position.x;
            initial_global_pose_.pose.position.y = pose_msg_.response.position.y;
            initial_global_pose_.pose.position.z = pose_msg_.response.position.z;
            initial_global_pose_.pose.orientation.x = pose_msg_.response.orientation.x;
            initial_global_pose_.pose.orientation.y = pose_msg_.response.orientation.y;
            initial_global_pose_.pose.orientation.z = pose_msg_.response.orientation.z;
            initial_global_pose_.pose.orientation.w = pose_msg_.response.orientation.w;
            
            //std::cout<<"Initial global position is: "<<initial_global_pose_.pose.position.x<<", "<<initial_global_pose_.pose.position.y<<", "<<
            //            initial_global_pose_.pose.position.z<<std::endl;
        }
        else
        {
            ROS_ERROR("Service call failed!");
        }
}

std::vector<double> MoveUR::callCurrentLocalPose()
{
    const std::string source_frame = "robot1_tf/base_link_ur5";
    const std::string target_frame = "robot1_tf/ee_link_ur5";

    current_local_pose_.clear();
    current_local_pose_ = base_.getCurrentPose(source_frame, target_frame);

    // std::cout<<"Current local pose is "<<current_local_pose_[0]<<", "<<current_local_pose_[1]<<", "<<current_local_pose_[2]
    //                 <<", "<<current_local_pose_[3]<<", "<<current_local_pose_[4]<<", "<<current_local_pose_[5]<<std::endl;

    last_time_ = ros::Time::now();

    return current_local_pose_;
}

void MoveUR::moveInitialPose()
{

    transform_ = base_.transform("robot1_tf/base_link", "robot1_tf/ee_link_ur5");

    geometry_msgs::PoseStamped x_d;
    x_d.header.frame_id = "robot1_tf/base_link_ur5";
    x_d.header.stamp = ros::Time::now();

    tf::Transform _to_initial, _from_initial;
    tf::poseMsgToTF(initial_global_pose_.pose,_from_initial);
    _to_initial = transform_*_from_initial;
    tf::poseTFToMsg(_to_initial,x_d.pose);
    
    x_d.pose.position.x = initial_local_pose_[0]; //-dsad;
    x_d.pose.position.y = initial_local_pose_[1];
    x_d.pose.position.z = initial_local_pose_[2];

    x_d.pose.orientation.x = initial_local_pose_[6];
    x_d.pose.orientation.y = initial_local_pose_[7];
    x_d.pose.orientation.z = initial_local_pose_[8];
    x_d.pose.orientation.w = initial_local_pose_[9];

    //std::cout<<"Desired pose: "<<x_d.pose.position.x<<", "<<x_d.pose.position.y<<", "<<x_d.pose.position.z<<std::endl;

    /**** Publish to compliance controller input *****/
    pub_pose_.publish(x_d);
}

void MoveUR::rotateAngle(double rot_angle_)
{
    //lookupInitialLocalPosition();

    /**** Query current joint configuration ****/
    joint_theta_.clear();
    joint_theta_ = base_.getAngles();
    //ROS_INFO_STREAM("Current theta: "<<joint_theta_[0]);

    //transform_ = base_.transform("robot1_tf/base_link", "robot1_tf/ee_link_ur5");

    /**** Change first angle to rotation angle ****/
    if(rot_angle_ > 0)
        joint_theta_[0] = joint_theta_[0] + abs(rot_angle_);
    else
        joint_theta_[0] = joint_theta_[0] - abs(rot_angle_);
    
    ROS_INFO_STREAM("New theta: "<<joint_theta_[0]);

    /**** Calculate new pose rotated by rot_angle_ ****/
    forwardKinematics();

    geometry_msgs::PoseStamped x_d;
    x_d.header.frame_id = "robot1_tf/base_link_ur5";
    x_d.header.stamp = ros::Time::now();
    //x_d.pose.position.x = T[3];
    if(abs(T[11]) > 0.8)
        x_d.pose.position.x = 0.75;
    else
        x_d.pose.position.x = T[3];
    x_d.pose.position.y = T[7];
    x_d.pose.position.z = T[11];

    x_d.pose.orientation.x = initial_local_pose_[6]; //quat.x; //current_local_pose_[6]; //
    x_d.pose.orientation.y = initial_local_pose_[7]; //quat.y; //current_local_pose_[7]; //
    x_d.pose.orientation.z = initial_local_pose_[8]; //quat.z; //current_local_pose_[8]; //
    x_d.pose.orientation.w = initial_local_pose_[9]; //quat.w; //current_local_pose_[9]; //

    //ROS_INFO_STREAM("Desired pose: "<<x_d.pose.position.x<<", "<<x_d.pose.position.y<<", "<<x_d.pose.position.z<<
    //", "<<x_d.pose.orientation.x<<", "<<x_d.pose.orientation.y<<", "<<x_d.pose.orientation.z<<", "<<x_d.pose.orientation.w);

    /**** Publish to compliance controller input *****/
    pub_pose_.publish(x_d);
}

void MoveUR::forwardKinematics(){
    for(int i =0; i<joint_theta_.size(); i++)
        theta[i] = joint_theta_.at(i);
    ur_kinematics::forward(theta, T);

    /*
    for(int i=0; i<6;i++)
        ROS_INFO_STREAM(theta[i]);
        

    for(int i=0;i<4;i++) {
        for(int j=i*4;j<(i+1)*4;j++)
            printf("%1.3f ", T[j]);
        printf("\n");
    }
    printf("\n");
    */

}

