#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <mur_robot_msgs/EndeffPose.h>
#include <mur_robot_msgs/PoseRequest.h>
#include <mur_robot_msgs/PoseMessage.h>

/****************************************************************
****This code provides the endeffector pose transferred from ****
****"robot1_tf/base_link_ur5" into "robot1_tf/wrist_3_link_ur5" *
*****************************************************************/ 

geometry_msgs::Pose target_pose;
mur_robot_msgs::PoseMessage current_pose;

void callbackCurrentPose(const mur_robot_msgs::PoseMessage& msg){
  std::cout<<"Subscribe endeffector pose directed from /robot1_tf/base_link_ur5"<<std::endl;
  current_pose.position.x = msg.position.x;
  current_pose.position.y = msg.position.y;
  current_pose.position.z = msg.position.z;
  current_pose.orientation.x = msg.orientation.x;
  current_pose.orientation.y = msg.orientation.y;
  current_pose.orientation.z = msg.orientation.z;
  current_pose.orientation.w = msg.orientation.w;
  current_pose.rpy_orientation.x = msg.rpy_orientation.x;
  current_pose.rpy_orientation.y = msg.rpy_orientation.y;
  current_pose.rpy_orientation.z = msg.rpy_orientation.z;
}

void callbackTargetPose(const geometry_msgs::Pose& target_msg){
  std::cout<<"Subscribe endeffector target"<<std::endl;
  target_pose.position.x = target_msg.position.x;
  target_pose.position.y = target_msg.position.y;
  target_pose.position.z = target_msg.position.z;
  target_pose.orientation.x = target_msg.orientation.x;
  target_pose.orientation.y = target_msg.orientation.y;
  target_pose.orientation.z = target_msg.orientation.z;
  target_pose.orientation.w = target_msg.orientation.w;
}

bool callbackRequestEndeffector(mur_robot_msgs::PoseRequest::Request& req, mur_robot_msgs::PoseRequest::Response& res){
  
  std::cout<<"Service Request Current EndeffectorPose has called"<<std::endl;
  bool request = req.request;

  if(request == true){
    res.success = true;
    res.position.x = current_pose.position.x;
    res.position.y = current_pose.position.y;
    res.position.z = current_pose.position.z;

    res.orientation.x = current_pose.orientation.x;
    res.orientation.y = current_pose.orientation.y;
    res.orientation.z = current_pose.orientation.z;
    res.orientation.w = current_pose.orientation.w;

    res.rpy_orientation.x = current_pose.rpy_orientation.x;
    res.rpy_orientation.y = current_pose.rpy_orientation.y;
    res.rpy_orientation.z = current_pose.rpy_orientation.z;
    
    //ROS_INFO("Endeffector Pose: [%lf, %lf, %lf, %lf, %lf, %lf, %lf]", res.position.x, res.position.y, res.position.z, 
    //res.orientation.x, res.orientation.y, res.orientation.z, res.orientation.w);
    return true;
  }
  else
  {
    res.success = false;
    ROS_WARN("Failed to request endeffector pose");
  }
  return true;
}

/**** Following service provides Current Endeffector pose*/
bool callbackRequestTarget(mur_robot_msgs::EndeffPose::Request& req1, 
                                mur_robot_msgs::EndeffPose::Response& res1)
{
  std::cout<<"Service Request Target EndeffectorPose has called"<<std::endl;
  bool request = req1.request;

  if(request == true){
    res1.success = true;
    res1.position.x = current_pose.position.x;
    res1.position.y = current_pose.position.y;
    res1.position.z = current_pose.position.z;

    res1.orientation.x = current_pose.orientation.x;
    res1.orientation.y = current_pose.orientation.y;
    res1.orientation.z = current_pose.orientation.z;
    res1.orientation.w = current_pose.orientation.w;
    
    ROS_INFO("Endeffector Pose: [%lf, %lf, %lf, %lf, %lf, %lf, %lf]", res1.position.x, res1.position.y, res1.position.z, 
    res1.orientation.x, res1.orientation.y, res1.orientation.z, res1.orientation.w);
    return true;
  }
  else
  {
    res1.success = false;
    ROS_WARN("Failed to request endeffector pose");
  }
  return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "get_endeffector_pose_server");
  ros::NodeHandle nh;
  ros::Rate rate(10.0);
  
  //try
  //{
    //ROS_INFO("Wait for /force/endeffector_pose/vector-> Is listen_btw_frames_node active?");
    //ros::topic::waitForMessage<geometry_msgs::Pose>("/force/endeffector_pose/vector",ros::Duration(5.0));
    //ROS_INFO("Connection to /force/endeffector_pose/vector succeeded.");
    //ROS_INFO("Wait for /force/target_goal/cartesian_vector-> Is control_force_node active?");
    //ros::topic::waitForMessage<geometry_msgs::Pose>("/force/target_goal/cartesian_vector",ros::Duration(5.0));
    //ROS_INFO("Connection to /force/target_goal/cartesian_vector succeeded.");
  //}
  //catch(std::exception e)
  //{
  //  ROS_ERROR("Failed to connect to /force/endeffector_pose/vector or /force/endeffector_pose/vector! Suggestion: launch 'control_force_node'.");
  //  ros::shutdown();
 // }

  ros::Subscriber sub_pose = nh.subscribe("/cartesian/endeffector_pose/data", 100, callbackCurrentPose); //from listen_frames_node

  //ros::Subscriber sub_target_pose = nh.subscribe("/force/target_goal/cartesian_vector", 10, callbackTargetPose); //from control_force.cpp

  ros::ServiceServer server_pose = nh.advertiseService("/request_endeffector/pose", callbackRequestEndeffector); //client in control_force_node + calculate_jacobian_node 

  //ros::ServiceServer server_target_pose = nh.advertiseService("/request_endeffector/target_pose", callbackRequestTarget); //client in movegroup_interface.cpp

  ros::spin();
}