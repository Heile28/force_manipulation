/*
 * File: mur_base.h
 * Author: Heiko Lenz
 * 
 * Created on 11. September 2020
 * 
 * Source file providing methods driving the MiR plattform in conjunction 
 * to cartesian compliance controller
 * 
*/

#include <mur_force_controller/mur_base.h>

MurBase::Orientation orientation;
MurBase::Translation translation;
MurBase::Quaternion quaternion;

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

int MurBase::mapIndizes(std::string name)
{
    for(int i=0;i<6;i++)
    {
        if (names[i]==name)
        return i;
    }    
}

void MurBase::callbackJointAngles(sensor_msgs::JointState joint_msg_)
{
    double theta_[6];
    std::string name;
    for(unsigned i = 0; i<6; i++){
        name=joint_msg_.name[i];
        theta_[mapIndizes(name)] = joint_msg_.position[i];
    }
    
    theta1_ = theta_[0];
    theta2_ = theta_[1];
    theta3_ = theta_[2];
    theta4_ = theta_[3];
    theta5_ = theta_[4];
    theta6_ = theta_[5];
    
    /*
    std::cout<<"Winkel:"<<std::endl;
    std::cout<<"["<<theta1_<<","<<theta2_<<","<<theta3_<<","<<theta4_<<","<<theta5_<<","<<theta6_<<"]"<<std::endl;
    */
    
    
}

void MurBase::startServiceServer()
{
  this->server_pose_ = nh_.advertiseService("listen_frames/request_endeffector/pose", &MurBase::callbackRequestEndeffector, this);
}

std::vector<double> MurBase::getAngles()
{
    std::vector<double> theta_;
    theta_.clear();
    theta_.push_back(theta1_);
    theta_.push_back(theta2_);
    theta_.push_back(theta3_);
    theta_.push_back(theta4_);
    theta_.push_back(theta5_);
    theta_.push_back(theta6_);
    return theta_;
}

std::vector<double> MurBase::getCurrentPose(std::string source_frame_, std::string target_frame_)
{
    std::vector<double> pose_;
    //get struct lists filled
    getLinkTransformUR5(source_frame_, target_frame_);

    pose_.push_back(translation.x);
    pose_.push_back(translation.y);
    pose_.push_back(translation.z);
    pose_.push_back(orientation.x);
    pose_.push_back(orientation.y);
    pose_.push_back(orientation.z);

    pose_.push_back(quaternion.x);
    pose_.push_back(quaternion.y);
    pose_.push_back(quaternion.z);
    pose_.push_back(quaternion.w);
    return pose_;
}

void MurBase::getLinkTransformUR5(std::string source_frame, std::string target_frame)
{
  /***** Migrate transformation into object *****/
  //ROS_INFO("Translation from %s into %s", source_frame.c_str(), target_frame.c_str());
  transform_ = transform(source_frame, target_frame);
  
  /***** Get orientation'in quaternions *****/
  r = transform_.getOrigin();
  quat_rot = {transform_.getRotation().getX(), transform_.getRotation().getY(), transform_.getRotation().getZ(), transform_.getRotation().getW()};
  
  /***** Transfer rotation into RPY and write RAD angles into variables *****/
  R = transform_.getBasis();
  tf::Matrix3x3(R).getRPY(roll, pitch, yaw);
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;

  /***** Transfer angles into DEG *****/
  /*
  alpha = 180.0*roll/PI;
  beta = 180.0*pitch/PI;
  gamma = 180.0*yaw/PI;
  */
  
  /***** Store data of orientation, translation and quaternion *****/
  //orientation = {roll, pitch, yaw};
  orientation.x = roll; orientation.y = pitch; orientation.z = yaw;
  //translation = {r[0], r[1], r[2]};
  translation.x = r[0]; translation.y = r[1]; translation.z = r[2];
  //quaternion = {quat_rot.x(), quat_rot.y(), quat_rot.z(), quat_rot.w()};
  quaternion.x = quat_rot.x(); quaternion.y = quat_rot.y(); quaternion.z = quat_rot.z(); quaternion.w = quat_rot.w();
}

bool MurBase::callbackRequestEndeffector(mur_robot_msgs::PoseRequest::Request& req, mur_robot_msgs::PoseRequest::Response& res)
{
  std::cout<<"Service Request Current EndeffectorPose has called"<<std::endl;
  bool request = req.request;

  getLinkTransformUR5((std::string)req.source_frame, (std::string)req.target_frame);

  if(request == true){
    /***** Call *****/
    res.success = true;
    res.position.x = translation.x;
    res.position.y = translation.y;
    res.position.z = translation.z;

    res.orientation.x = quaternion.x;
    res.orientation.y = quaternion.y;
    res.orientation.z = quaternion.z;
    res.orientation.w = quaternion.w;

    res.rpy_orientation.x = orientation.x;
    res.rpy_orientation.y = orientation.y;
    res.rpy_orientation.z = orientation.z;

    return true;
  }
  else
  {
    res.success = false;
    ROS_WARN("Failed to request endeffector pose.");
  }
  return true;
}

tf::StampedTransform MurBase::transform(const std::string source_frame_, const std::string target_frame_)
{
    ros::Time now = ros::Time(0);
    tf::TransformListener list;
    tf::StampedTransform trans;

    try{
        list.waitForTransform(source_frame_, target_frame_, now, ros::Duration(2.0)); 
        list.lookupTransform(source_frame_, target_frame_, now, trans);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.5).sleep();
    }

    return trans;
}