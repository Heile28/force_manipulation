/***********************************************************
 ***Use tf to get access to frame transformations***********
 ***link: http://wiki.ros.org/tf/Overview/Transformations***
 ***code for rosrun tf tf_echo [reference_frame] [target_frame]****
 **********************************************************/

#include <mur_kinematics/listen_frames.h>

using namespace listen_frames;
    
ListenFrames::Orientation orientation;
ListenFrames::Translation translation;
ListenFrames::Quaternion quaternion;

ListenFrames::ListenFrames()
{
  this->nh_=nh_;
  this->server_pose_ = nh_.advertiseService("listen_frames/request_endeffector/pose", &ListenFrames::callbackRequestEndeffector, this);
}

bool ListenFrames::callbackRequestEndeffector(mur_robot_msgs::PoseRequest::Request& req, mur_robot_msgs::PoseRequest::Response& res)
{

  std::cout<<"Service Request Current EndeffectorPose has called"<<std::endl;
  bool request = req.request;

  if(request == true){
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

void ListenFrames::getLinkTransformUR5(const std::string source_frame, const std::string target_frame)
{
  
  ros::Time now = ros::Time::now();
  
  try{
    listener_.waitForTransform(target_frame, source_frame, now, ros::Duration(1.0));
    listener_.lookupTransform(target_frame, source_frame,
                          now, transform_);
  }
  catch(tf::TransformException ex)
  {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
  }
  
  ROS_INFO("Translation from %s into %s", target_frame.c_str(), source_frame.c_str());
  r = transform_.getOrigin();
  /*
  std::cout <<"Translationsvektor******"<<std::endl;
  std::cout <<"["<< r[0] <<", ";
  std::cout << r[1] <<", ";
  std::cout << r[2] <<"]^T" <<std::endl;
  */

  quat_rot = {transform_.getRotation().getX(), transform_.getRotation().getY(), transform_.getRotation().getZ(), transform_.getRotation().getW()};
    
  //transfer Rotation into RPY and write angles into variables
  R = transform_.getBasis();
  tf::Matrix3x3(R).getRPY(roll, pitch, yaw);
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;
  /*
  std::cout <<std::endl;
  std::cout <<"transformation into RPY xyz [rad] yields: "<<std::endl;
  std::cout <<"["<<roll<<", "<<pitch<<", "<<yaw <<"]" <<std::endl;
  */
    
  //into degree
  alpha = 180.0*roll/PI;
  beta = 180.0*pitch/PI;
  gamma = 180.0*yaw/PI;
  /*
  std::cout <<"The transformation into RPY xyz [DEG] yields: "<<std::endl;
  std::cout <<"["<<alpha<<", "<<beta<<", "<<gamma <<"]" <<std::endl;
  */

  orientation = {roll, pitch, yaw}; //define variables
  translation = {r[0], r[1], r[2]};
  quaternion = {quat_rot[0], quat_rot[1], quat_rot[2], quat_rot[3]};
}