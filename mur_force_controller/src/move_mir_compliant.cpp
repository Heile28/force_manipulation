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
    this->pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot1_ns/arm_cartesian_compliance_controller/target_pose", 100);
    this->sub_force_ = nh_.subscribe("/robot1_ns/arm_cartesian_compliance_controller/ft_sensor_wrench", 100, &MoveMir::wrenchCallback, this);
    
    //this->scan_pub_=this->nh_.advertise<sensor_msgs::PointCloud>("scan",10);
    init_time_ = ros::Time(0);
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
            initial_global_pose_.push_back(pose_msg_.response.rpy_orientation.x);
            initial_global_pose_.push_back(pose_msg_.response.rpy_orientation.y);
            initial_global_pose_.push_back(pose_msg_.response.rpy_orientation.z);
            //std::cout<<"Initial global pose is: "<<initial_global_pose_[0]<<", "<<initial_global_pose_[1]<<", "<<initial_global_pose_[2]<<", "
            //            <<initial_global_pose_[3]<<", "<<initial_global_pose_[4]<<", "<<initial_global_pose_[5]<<std::endl;
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
    //                <<", "<<current_global_pose_[3]<<", "<<current_global_pose_[4]<<", "<<current_global_pose_[5]<<std::endl;
    
    // current_time_ = ros::Time::now();
    // dt_ = (current_time_-last_time_).toSec();

    current_pose_.clear();
    current_pose_ = current_global_pose_;
    //last_time_ = ros::Time::now();

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
    last_time_ = ros::Time::now();

    return current_local_pose_;
}

std::vector<double> MoveMir::callCurrentWorldPose()
{
    const std::string source_frame = "map";
    const std::string target_frame = "robot1_tf/base_link";

    current_map_pose_ = base_.getCurrentPose(source_frame, target_frame);

    current_map_pose_.clear();
    last_time_ = ros::Time::now();

    return current_map_pose_;
}

void MoveMir::wrenchCallback(geometry_msgs::WrenchStamped wrench_msg_){
    force_.x = wrench_msg_.wrench.force.x;
    force_.y = wrench_msg_.wrench.force.y;
    force_.z = wrench_msg_.wrench.force.z;
    poseUpdater();
    //std::cout<<"Wrench is"<<force_.x<<", "<<force_.y<<", "<<force_.z<<std::endl;

}

void MoveMir::displacementPose(){
    displacement_pose_.clear();
    for(int i=0; i<6; i++)
        displacement_pose_.push_back(current_pose_[i] - initial_pose_[i]);        

    std::cout<<"Displacement is ["<<displacement_pose_[0]<<", "<<displacement_pose_[1]<<", "
    <<displacement_pose_[2]<<", "<<displacement_pose_[3]<<", "<<displacement_pose_[4]<<", "<<displacement_pose_[5]<<"]"<<std::endl;

    vth_ = atan(displacement_pose_.at(0)/displacement_pose_.at(1))/dt_;

    moveGoal();
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
    tw_msg_.linear.x = 0.0;
    tw_msg_.linear.y = 0.0;
    tw_msg_.linear.z = 0.0;
    tw_msg_.angular.x = 0.0;
    tw_msg_.angular.y = 0.0;
    tw_msg_.angular.z = atan2(current_global_pose_[1] - current_map_pose_[1], current_global_pose_[0] - current_map_pose_[1])/dt_;
    
    /***** Publish desired pose to cartesian_compliance_controller *****/
    geometry_msgs::PoseStamped x_d;
    x_d.header.frame_id = "robot1_tf/base_link_ur5";
        
    // x_d.pose.position.x = atan(theta_)/current_pose_[1]; //-dsad;
    // x_d.pose.position.y = atan(theta_)*current_pose_[0];
    // x_d.pose.position.z = current_pose_[2];

    // x_d.pose.orientation.x = current_pose_[6];
    // x_d.pose.orientation.x = current_pose_[7];
    // x_d.pose.orientation.x = current_pose_[8];
    // x_d.pose.orientation.x = current_pose_[9];

    x_d.pose.orientation.x = 0.0;
    x_d.pose.orientation.y = 0.0;
    x_d.pose.orientation.z = 0.0;
    x_d.pose.orientation.w = 0.0;

    std::cout<<"Desired pose: "<<x_d.pose.position.x<<", "<<x_d.pose.position.y<<", "<<x_d.pose.position.z<<std::endl;

    pub_pose_.publish(x_d);
    pub_simple_.publish(tw_msg_);
    
}

void MoveMir::rotateToForceDirection()
{
    do{
        // /***** Request current global pose *****/
        // callCurrentGlobalPose();

        /***** Transform wrench vector from ft_sensor_ref_link into ~/base_link *****/
        transform_ = base_.transform("robot1_tf/wrist_3_link_ur5", "robot1_tf/base_link");

        //tf::Transform( transform_.getRotation(),tf::Vector3(0.0,0.0,0.0) );

        tf::vector3MsgToTF(force_,tf_force_at_sensor_);
        tf_force_at_base_ = transform_.getBasis() * tf_force_at_sensor_;
        tf::vector3TFToMsg(tf_force_at_base_, force_at_base_);

        ROS_INFO_STREAM("Force vector in base is: \n"<<force_at_base_.x<<", "<<force_at_base_.y<<", "<<force_at_base_.z);

        if(force_at_base_.x != 0.0){
            theta_ = atan(force_at_base_.y/force_at_base_.x);
            ROS_INFO_STREAM("Theta is "<<theta_);
        }
        else
        {
            theta_ = 0.0;
            ROS_INFO("No force attack!!!");
            break;
        }
        //move manipulator and platform inversely
        nullspace(theta_);
        
        std::cout<<"Theta is "<<theta_<<std::endl;
        
    }while (theta_ != 0); //Vorteil: erst nach Schleifendurchlauf prüfen
    
}

void MoveMir::rotateToPoseDirection()
{
    lookupInitialGlobalPosition();
    ROS_INFO_STREAM("Force is"<<force_.x<<", "<<force_.y<<", "<<force_.z);

    while(abs(force_.x) > 5.0 || abs(force_.y) > 5.0|| abs(force_.z) > 5.0)
    {       
        ros::Duration(5.0).sleep();
        double theta0, theta1;
        // if(current_global_pose_[0] != 0.0)
        // {
        //     //theta0 = atan(initial_global_pose_[1]/initial_global_pose_[0]);
        //     theta0 = atan2(initial_global_pose_[1],initial_global_pose_[0]);
        //     ROS_INFO_STREAM("theta0 is "<<theta0);
        //     //theta1 = atan(current_pose_[0]/current_pose_[1]);
        //     theta1 = atan2(current_pose_[0],current_pose_[1]);
        //     ROS_INFO_STREAM("theta1 is "<<theta1);
        //     theta_ = (PI/2)-theta0-theta1;
        //     ROS_INFO_STREAM("theta is "<<theta_);

        // }
        // else
        // {
        //     theta_ = 0.0;
        //     ROS_INFO("No force attack!!!");
        // }
        
        nullspace(theta_);
    }
}

void MoveMir::poseUpdater()
{
    callCurrentGlobalPose();
    callCurrentLocalPose();
    current_time_ = ros::Time::now();
    dt_ = (current_time_-last_time_).toSec();
    last_time_ = ros::Time::now();
}

void MoveMir::moveStraight()
{
    tw_msg_.linear.x = ( sqrt(pow(current_map_pose_[0]-current_global_pose_[0] - initial_local_pose_[0],2) + 
                            pow(current_map_pose_[1]-current_global_pose_[1] - initial_local_pose_[1],2)) )/dt_; //Sicherheitsabstand einhalten
    tw_msg_.linear.y = 0.0;
    tw_msg_.linear.z = 0.0;
    tw_msg_.angular.x = 0.0;
    tw_msg_.angular.y = 0.0;
    tw_msg_.angular.z = 0.0;
    
    geometry_msgs::PoseStamped x_d;
    x_d.pose.position.x = current_local_pose_[0]-(tw_msg_.linear.x * dt_);
    x_d.pose.position.y = current_local_pose_[1];
    x_d.pose.position.z = current_local_pose_[2];

    x_d.pose.orientation.x = 0.0; //MUSS EVTL NOCH FESTGELEGT WERDEN
    x_d.pose.orientation.y = 0.0;
    x_d.pose.orientation.z = 0.0;
    x_d.pose.orientation.w = 0.0;

    pub_pose_.publish(x_d);
    pub_simple_.publish(tw_msg_);
    
}



/*
void MoveMir::transferIntoMir(Eigen::VectorXd displ_){
    // transform into (KS)MiR using following matrix
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
    delta_th_ = atan(x_mir_(1)/x_mir_(0));
    std::cout<<"Delta th: \n"<<delta_th_<<std::endl;
    
    vth_ = delta_th_/dt_;

    //speed of rotation is fundamental -> we want to rotate as fast as possible due to endeffector's changing position
    
    moveGoal();
}
*/