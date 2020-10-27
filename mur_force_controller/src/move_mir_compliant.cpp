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
    this->lookupInitialMiRPosition();
    this->pub_angle_ = nh_.advertise<std_msgs::Float64>("/move_mir_compliant/rotation_angle", 100);
    this->pub_simple_ = nh_.advertise<geometry_msgs::Twist>("/robot1_ns/mobile_base_controller/cmd_vel", 100);
    this->pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot1_ns/arm_cartesian_compliance_controller/target_pose", 100);
    this->sub_force_ = nh_.subscribe("/robot1_ns/arm_cartesian_compliance_controller/ft_sensor_wrench", 100, &MoveMir::wrenchCallback, this);
    
    init_time_ = ros::Time(0);
    this->rotation_angle_.data = 0.0;
    this->theta_global_ = 0.0;
    this->activate_force_ = 0; //CHANGES TO 1 WHEN FT_SENSOR ACTIVE Force changes!!!
    this->activate_rotation1_ = 0; //NOT READY FIRST ROTATION
    this->activate_rotation2_ = 1; //READY FOR FIRST ROTATION
    this->x_dot_ = 0.2331114;
    this->old_mir_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
}

//destructor
MoveMir::~MoveMir(){}

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
        //std::cout<<"Initial world pose is: "<<initial_world_pose_[0]<<", "<<initial_world_pose_[1]<<", "<<initial_world_pose_[2]<<", "
        //            <<initial_world_pose_[3]<<", "<<initial_world_pose_[4]<<", "<<initial_world_pose_[5]<<std::endl;
        
    }
    else
    {
        ROS_ERROR("Service call failed!");
    }
    //theta0_world_ = atan2(initial_world_pose_[1],initial_world_pose_[0]);
    //std::cout<<"Initial theta0: "<<theta0_world_<<std::endl;
    last_time_ = init_time_;
}

void MoveMir::lookupInitialMiRPosition(){
    ros::service::waitForService("/mur_base/listen_frames/request_endeffector/pose");
    this->endeffector_pose_client_ = nh_.serviceClient<mur_robot_msgs::PoseRequest>("/mur_base/listen_frames/request_endeffector/pose");

    mur_robot_msgs::PoseRequest pose_msg_;
    pose_msg_.request.request = true;
    pose_msg_.request.source_frame = "map";
    pose_msg_.request.target_frame = "robot1_tf/base_link";

    //ros::service::waitForService("/mur_base/listen_frames/request_endeffector/pose", ros::Duration(2));
    
    if(endeffector_pose_client_.call(pose_msg_))
    {
        initial_mir_pose_.clear();
        initial_mir_pose_.push_back(pose_msg_.response.position.x);
        initial_mir_pose_.push_back(pose_msg_.response.position.y);
        initial_mir_pose_.push_back(pose_msg_.response.position.z);
        initial_mir_pose_.push_back(pose_msg_.response.rpy_orientation.x);
        initial_mir_pose_.push_back(pose_msg_.response.rpy_orientation.y);
        initial_mir_pose_.push_back(pose_msg_.response.rpy_orientation.z);
        initial_mir_pose_.push_back(pose_msg_.response.orientation.x);
        initial_mir_pose_.push_back(pose_msg_.response.orientation.y);
        initial_mir_pose_.push_back(pose_msg_.response.orientation.z);
        initial_mir_pose_.push_back(pose_msg_.response.orientation.w);
        
    }
    else
    {
        ROS_ERROR("Service call failed!");
    }
    last_time_ = init_time_;
    old_mir_pose = initial_mir_pose_;
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
            //std::cout<<"Initial global pose is: "<<initial_global_pose_[0]<<", "<<initial_global_pose_[1]<<", "<<initial_global_pose_[2]<<", "
            //            <<initial_global_pose_[3]<<", "<<initial_global_pose_[4]<<", "<<initial_global_pose_[5]<<", "<<initial_global_pose_[6]<<std::endl;
        }
        else
        {
            ROS_ERROR("Service call failed!");
        }
        initial_pose_.clear();
        initial_pose_ = initial_global_pose_;
        theta0_global_ = atan2(initial_global_pose_[1],initial_global_pose_[0]);
        //std::cout<<"Initial theta0: "<<theta0_global_<<std::endl;
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
            //std::cout<<"Inside service call"<<std::endl;

            //endeffector_pose_client_.call(pose_msg_);
            initial_local_pose_.clear();
            initial_local_pose_.push_back(pose_msg_.response.position.x);
            initial_local_pose_.push_back(pose_msg_.response.position.y);
            initial_local_pose_.push_back(pose_msg_.response.position.z);
            initial_local_pose_.push_back(pose_msg_.response.orientation.x);
            initial_local_pose_.push_back(pose_msg_.response.orientation.y);
            initial_local_pose_.push_back(pose_msg_.response.orientation.z);
            initial_local_pose_.push_back(pose_msg_.response.orientation.w);
            //std::cout<<"Initial local pose is: "<<initial_local_pose_[0]<<", "<<initial_local_pose_[1]<<", "<<initial_local_pose_[2]<<", "
            //            <<initial_local_pose_[3]<<", "<<initial_local_pose_[4]<<", "<<initial_local_pose_[5]<<", "<<initial_local_pose_[6]<<std::endl;
            
            
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

    return current_global_pose_;
}

std::vector<double> MoveMir::callCurrentLocalPose()
{
    const std::string source_frame = "robot1_tf/base_link_ur5";
    const std::string target_frame = "robot1_tf/ee_link_ur5";

    current_local_pose_.clear();
    current_local_pose_ = base_.getCurrentPose(source_frame, target_frame);

    //std::cout<<"Current pose is "<<current_local_pose_[0]<<", "<<current_local_pose_[1]<<", "<<current_local_pose_[2]
    //                 <<", "<<current_local_pose_[3]<<", "<<current_local_pose_[4]<<", "<<current_local_pose_[5]<<std::endl;
    

    // current_time_ = ros::Time::now();
    // dt_ = (current_time_-last_time_).toSec();

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
    //ROS_INFO_STREAM("Current EE in map: "<<current_map_pose_[0]<<", "<<current_map_pose_[1]<<", "<<current_map_pose_[2]);
    
    /**** MiR in world frame ****/
    target_frame = "robot1_tf/base_link";
    current_mir_map_pose_ = base_.getCurrentPose(source_frame, target_frame);
    //ROS_INFO_STREAM("Current MiR in map: "<<current_mir_map_pose_[0]<<", "<<current_mir_map_pose_[1]<<", "<<current_mir_map_pose_[2]);
    

    theta_world_ = atan2(current_map_pose_[1], current_map_pose_[0]);
    theta_mir_world_ = current_mir_map_pose_[5]; //YAW
    //std::cout<<"theta_world_ DEG: "<<theta_world_*180/PI<<std::endl;
    //std::cout<<"theta_mir_world_ DEG: "<<theta_mir_world_*180/PI<<std::endl;

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
    
    //std::cout<<"Wrench is \n"<<force_.x<<", "<<force_.y<<", "<<force_.z<<std::endl;

    if(abs(force_.x) > 1.0 || abs(force_.y) > 1.0|| abs(force_.z) > 1.0){
        this->activate_force_ = 1;
        //ROS_INFO("Force attack! -> activate_ = 1");
    }
    else
        this->activate_force_ = 0;

}

void MoveMir::poseUpdater(double force_angle)
{
    ROS_INFO_STREAM("Force angle is "<<force_angle);
    if(abs(force_angle) < 0.03)
    {
        activate_rotation2_ = 0;
    }
    if(abs(force_angle) >= 0.03) //if greater 2 DEG begin rotation
        activate_rotation2_ = 1;
}
        
void MoveMir::relativeAngleUpdater()
{
    double theta_star = atan2(current_map_pose_[1]-current_mir_map_pose_[1], current_map_pose_[0]-current_mir_map_pose_[0]);
    std::cout<<"theta_star: "<<theta_star<<std::endl;

    double rot_angle1 = theta_star - theta_mir_world_;
    rot_angle1 = normalize_angle(rot_angle1);
    //std::cout<<"rot_angle1 normalized: "<<rot_angle1<<std::endl;
    

    /***** Store as rotation angle for ur5 controller *****/
    rotation_angle_.data = rot_angle1;
    
    /***** Rotate when pose angle is greater 1 degree *****/
    if(activate_force_ == 1 && activate_rotation1_ == 1 && abs(rot_angle1) > 0.03)
    {
        //ROS_INFO_STREAM("Rotation angle in DEG: "<<rot_angle1*180/PI);
        rotateToPoseDirection(rot_angle1);
        activate_rotation1_ = 0;    //READY FOR SECOND ROTATION
    }

}

void MoveMir::rotateToForceDirection()
{
        ROS_INFO("Now is time for rotation in force direction!");
        double force_angle;
    
        /**** Rate considerates synchronisation between
         * rotateToPoseDirection() and poseUpdater() ****/
        ros::Rate r(1.7);
        
        /***** Rotate MiR in direction *****/
        while(nh_.ok() && activate_rotation2_)
        {
            // Query angle of force attack
            force_angle = getCurrentForceAngle();
            
            //send the rotation command
            rotateToPoseDirection(force_angle);
            r.sleep();
            
            //interchange with manipulator
            relativeAngleUpdater();
            pub_angle_.publish(rotation_angle_);
            
            //get current pose
            poseUpdater(force_angle);
            
            ros::spinOnce();
        }
}

double MoveMir::getCurrentForceAngle()
{
    /***** Request current global + world poses *****/
    callCurrentGlobalPose();
    callCurrentWorldPose();

    /***** Transform wrench vector from ft_sensor_ref_link into ~/base_link *****/
    transform_ = base_.transform("robot1_tf/wrist_3_link_ur5", "robot1_tf/base_link");
    
    tf::vector3MsgToTF(force_,tf_force_at_sensor_);
    tf_force_at_base_ = transform_.getBasis().inverse() * tf_force_at_sensor_;
    tf::vector3TFToMsg(tf_force_at_base_, force_at_base_);
    //ROS_INFO_STREAM("Force at base: "<< force_at_base_.x<<", "<< force_at_base_.y<<", "<< force_at_base_.z);

    /***** Set force direction *****/
    this->isPositiveForce_ = ((int)force_at_base_.x != 0) ? 1 : 0; //if greater 0 set boolean 1

    /***** Calculate vector orientation in x-y plane *****/
    force_at_base_.z = 0;
    double force_angle  = atan2((int)force_at_base_.y, (int)force_at_base_.x);
    //ROS_INFO_STREAM("Rotation angle is "<<rot_angle);
    force_angle = normalize_angle(force_angle);
    if(abs(force_angle) == PI)
        force_angle = 0;
    ROS_INFO_STREAM("Normalized Yaw angle of force: "<<force_angle);

    return force_angle;
}

void MoveMir::controlMethod1()
{
    /***** Query current normalized force angle *****/
    double theta_norm = getCurrentForceAngle();

    //rotateToPoseDirection(force_angle);
    //std::vector<double> old_mir_pose = initial_mir_pose_;

    // tw_msg_.linear.x = 0.0;
    tw_msg_.linear.y = 0.0;
    tw_msg_.linear.z = 0.0;
    tw_msg_.angular.x = 0.0;
    tw_msg_.angular.y = 0.0;

    /**** Query current MiR pose ****/
    callCurrentWorldPose();

    /**** Query current EE pose in local frame ****/
    callCurrentLocalPose();
    
    /***** Send move command to manipulator + platform when force angle greater 2 degrees *****/
    if(abs(theta_norm) >= 0.03) // && activate_force_ == 1)
    {
            /**** Set velocities when force not parallel ****/
            tw_msg_.angular.z = M_PI/3*theta_norm;
            tw_msg_.linear.x = (isPositiveForce_==1) ? -x_dot_ : x_dot_;
            /*
            std::cout<<"Current MiR pose:\n";
            for(int i=0; i<current_mir_map_pose_.size();i++)
                std::cout<<current_mir_map_pose_[i]<<", ";
            std::cout<<"\n";

            std::cout<<"Old MiR pose:\n";
            for(int i=0; i<old_mir_pose.size();i++)
                std::cout<<old_mir_pose[i]<<", ";
            std::cout<<"\n";
            */
            
            /**** Update displacement ****/
            //delta_pose.clear();
            //for(int i = 0; i<current_mir_map_pose_.size(); i++)
            //    this->delta_pose[i] = current_mir_map_pose_[i]- old_mir_pose[i];
            

            // ROS_INFO_STREAM("delta_pose size is"<< delta_pose.size());
            // for(int i=0; i<delta_pose.size();i++)
            //     std::cout<<delta_pose[i]<<", ";
            // std::cout<<"\n";

            /*
            this->new_pose.header.stamp = ros::Time::now();
            this->new_pose.header.frame_id = "robot1_tf/base_link_ur5";
            this->new_pose.pose.position.x = current_local_pose_[0] - (current_mir_map_pose_[0]- old_mir_pose[0]);
            this->new_pose.pose.position.y = current_local_pose_[1] - (current_mir_map_pose_[1]- old_mir_pose[1]);
            this->new_pose.pose.position.z = current_local_pose_[2] - (current_mir_map_pose_[2]- old_mir_pose[2]);

            this->new_pose.pose.orientation.x = initial_local_pose_[3];
            this->new_pose.pose.orientation.y = initial_local_pose_[4];
            this->new_pose.pose.orientation.z = initial_local_pose_[5];
            this->new_pose.pose.orientation.w = initial_local_pose_[6];
            */
            this->new_pose.header.stamp = ros::Time::now();
            this->new_pose.header.frame_id = "robot1_tf/base_link_ur5";
            this->new_pose.pose.position.x = current_local_pose_[0] - (current_mir_map_pose_[0]- old_mir_pose[0]);
            this->new_pose.pose.position.y = current_local_pose_[1] - (current_mir_map_pose_[1]- old_mir_pose[1]);
            this->new_pose.pose.position.z = current_local_pose_[2] - (current_mir_map_pose_[2]- old_mir_pose[2]);

            this->new_pose.pose.orientation.x = current_local_pose_[6] - (current_mir_map_pose_[6]- old_mir_pose[6]);
            this->new_pose.pose.orientation.y = current_local_pose_[7] - (current_mir_map_pose_[7]- old_mir_pose[7]);
            this->new_pose.pose.orientation.z = current_local_pose_[8] - (current_mir_map_pose_[8]- old_mir_pose[8]);
            this->new_pose.pose.orientation.w = current_local_pose_[9] - (current_mir_map_pose_[9]- old_mir_pose[9]);
            old_mir_pose.clear();

            /**** Send commands ****/
            pub_simple_.publish(tw_msg_);
            pub_pose_.publish(new_pose);

            old_mir_pose = current_mir_map_pose_;

    }

}

void MoveMir::controlMethod2()
{
    //ros::Duration(1.0).sleep(); //sleep until manipulator has moved

    /***** Rotate MiR towards endeffector pose *****/
    relativeAngleUpdater();

    /***** Query current normalized force angle *****/
    double theta_norm = getCurrentForceAngle();

    /**** Decide about translation or rotation ****/
    poseUpdater(theta_norm);

    if(activate_force_ == 1 && activate_rotation2_ == 1)
        rotateToForceDirection();
    if(activate_force_ == 1 && activate_rotation2_ == 0)
        moveStraight();
    else
        activate_force_= 0;

}

void MoveMir::controlMethod3()
{
    //ros::Duration(1.0).sleep(); //sleep until manipulator has moved
    
    /***** Query current normalized force angle *****/
    double theta_norm = getCurrentForceAngle();

    /**** Decide about translation or rotation ****/
    poseUpdater(theta_norm);

    if(activate_force_ == 1 && activate_rotation2_ == 1)
        rotateToForceDirection();

    //if(activate_force_ == 1 && activate_rotation1_ == 0 && activate_rotation2_ == 0)
    if(activate_force_ == 1 && activate_rotation2_ == 0)
        moveStraight();
}

void MoveMir::rotateToPoseDirection(double rot_angle)
{
    ros::Rate r(0.5);
    /***** Rotate MiR platform *****/
    tw_msg_.linear.x = 0.0;
    tw_msg_.linear.y = 0.0;
    tw_msg_.linear.z = 0.0;
    tw_msg_.angular.x = 0.0;
    tw_msg_.angular.y = 0.0;
    tw_msg_.angular.z = 0.0;

    pub_simple_.publish(tw_msg_);
    
    /***** Specify angle *****/
    tw_msg_.angular.z = M_PI/3*rot_angle;
    //ros::Rate r2(8.0);
    //r2.sleep(); //sleep for 0.125 seconds
    do
    {
        pub_simple_.publish(tw_msg_);
        /***** Publish previously specified angle to ur5 controller *****/
        pub_angle_.publish(rotation_angle_);
        rot_angle = getCurrentForceAngle();
        tw_msg_.angular.z = rot_angle;
        r.sleep();
    } while (rot_angle > 0.03);
}

void MoveMir::moveStraight()
{
    //information: the velocity doesn't depend on the force amount
    //is calculated as following and always equals 0.2331114
    //double time = 3/PI * rot_angle;
    //x_dot_ = force_angle/time * 0.44521/2;

    ros::Rate r(0.5);
    ROS_INFO("Inside straight movement");
    tw_msg_.linear.x = 0.0;
    tw_msg_.linear.y = 0.0;
    tw_msg_.linear.z = 0.0;
    tw_msg_.angular.x = 0.0;
    tw_msg_.angular.y = 0.0;
    tw_msg_.angular.z = 0.0;
    pub_simple_.publish(tw_msg_);

    do{
        tw_msg_.linear.x = (isPositiveForce_==1) ? -x_dot_ : x_dot_;
        pub_simple_.publish(tw_msg_);
        //poseUpdater();
        ros::spinOnce();
        r.sleep();
    } while(activate_force_ == 1 && activate_rotation2_ == 0);
    
}

double MoveMir::normalize_angle(double angle)
{
    const double result = fmod((angle + M_PI), 2*M_PI)-M_PI;

    if(result > M_PI/2)
        return result - M_PI;
    if(result < M_PI/2)
        return result + M_PI;
    else
        return 0.0;

}
