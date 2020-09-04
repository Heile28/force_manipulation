/***********************************************************
 ***Use tf to get access to frame transformations***********
 ***link: http://wiki.ros.org/tf/Overview/Transformations***
 ***code for rosrun tf tf_echo [reference_frame] [target_frame]****
 **********************************************************/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <cmath>
//#include <mur_robot_msgs/EndeffPose.h>
//#include <my_mur_msgs/EndeffPose.h>
//#include <my_mur_msgs/PoseMessage.h>
#include <mur_robot_msgs/PoseMessage.h>

struct Orientation {double x; double y; double z;};
struct Translation {double x; double y; double z;};
struct Quaternion {double x; double y; double z; double w;};

#define POSE_ARRAY_SIZE 6

class ListenFrames{
  
  private: 
    ros::NodeHandle nh_;
    tf::TransformListener listener;
    tf::TransformListener listener2;
    tf::Vector3 r, r2; //translation vector size 3
    tf::tfVector4 quat_rot; //Vektor der Rotation in Quaternion
    tf::Matrix3x3 R, R2, R3; //3x3 Rotationsmatrix
    tf::Quaternion quat;
    double roll, pitch, yaw; //angles in RAD
    double alpha, beta, gamma; //angles in DEG
    const int ROW = 3; //array size
    const int COLUMN = 3; //array size
    geometry_msgs::Vector3 rpy;
    const double PI = M_PI;

  public:
    double pose_endeffector[POSE_ARRAY_SIZE]; //pose-vector
    double pp;
    Orientation orientation;
    Translation translation;
    Quaternion quaternion;
    
    //ros::Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false);
    
  ListenFrames() {}
  

  void getLinkTransformUR5(ros::Publisher pub_endeffector_pose_, const std::string target_frame2){
    double alpha, beta, gamma;
    double alpha1, beta1, gamma1;
    double R_kard1[ROW][COLUMN], R_kard2[ROW][COLUMN], R_rpy1[ROW][COLUMN], R_rpy2[ROW][COLUMN];
    //ros::Rate rate(10.0);

    std::string target_frame = "robot1_tf/base_footprint"; //robot1_tf/ee_link_ur5 robot1_tf/base_footprint
    const std::string source_frame = "robot1_tf/ee_link_ur5"; //robot1_tf/base_footprint robot1_tf/base_link_ur5 robot1_tf/ee_link_ur5

    //robot1_tf/base_link_ur5 robot1_tf/ee_link_ur5 robot1_tf/base_footprint

    tf::StampedTransform transform;
    tf::StampedTransform transform2;
    ros::Time now = ros::Time::now();
    
    try{
      listener.waitForTransform(target_frame, source_frame, now, ros::Duration(1.0)); //listener.waitForTransform("robot1_tf/ee_link_ur5", "robot1_tf/base_link_ur5",now,ros::Duration(3.0))
      listener2.waitForTransform(target_frame2, source_frame, now, ros::Duration(1.0));
      listener.lookupTransform(target_frame, source_frame,
                            now, transform); //We want the transform from frame 'robot0_tf/base_link_ur5' to frame 'robot0_tf/wrist_3_link_ur5'.
                                //The time at which we want to transform. Providing ros::Time(0) will just get us the latest available transform
                                //The object in which we store the resulting transform
                                //ros::Duration(...) is time at which to transform
        //ein anderer Weg zur Verzoegerung um 5s:
        //ros::Time past = ros::Time::now() - ros::Duration(5.0);
        //listener.waitForTransform("robot1_tf/wrist_3_link_ur5", "robot1_tf/base_link_ur5",past,ros::Duration(3.0));
        //listener.lookupTransform("robot1_tf/wrist_3_link_ur5", "robot1_tf/base_link_ur5",
        //                       past, transform);
      listener2.lookupTransform(target_frame2, source_frame,
                            now, transform2);

    }
    catch(std::exception ex)
    {
      ROS_WARN("Exception in lookup!");
    }
    
  /*
    //get and inherit transformation vector
    std::cout <<"***********1st Transformation ************************"<<std::endl;
    ROS_INFO("Translation from %s into %s", target_frame.c_str(),source_frame.c_str());
    r = transform.getOrigin();
    std::cout <<"Translationsvektor******"<<std::endl;
    std::cout <<"["<< r[0] <<", ";
    std::cout << r[1] <<", ";
    std::cout << r[2] <<"]^T" <<std::endl;
      */
      /*
      //Auslesen einzelner Werte
      r[0] = transform.getOrigin().getX();
      r[1] = transform.getOrigin().getY();
      r[2] = transform.getOrigin().getZ();
      std::cout <<"Neue Berechnung" <<std::endl;
      std::cout <<"["<< r[0] <<", ";
      std::cout << r[1] <<", ";
      std::cout << r[2] <<"]^T" <<std::endl;
      
      geometry_msgs::Twist vel_msg;
      vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                      transform.getOrigin().x());
      vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                    pow(transform.getOrigin().y(), 2));
                                    //power of 2
                                    //transform.getOrigin() 
                                    //-> returns the origin vector translation from frame /turtle1 to frame /turtle2
                                    //
      turtle_vel.publish(vel_msg);
  */
      
      //Transformationsmatrix
  /*
    R = transform.getBasis(); //yields KARDAN rotation matrix
    double T[][4] = { {R[0][0], R[0][1], R[0][2], r[0]},
                        {R[1][0], R[1][1], R[1][2], r[1]},
                        {R[2][0], R[2][1], R[2][2], r[2]},
                        {0, 0, 0, 1} }; //matrix['Zeilenzahl']['Spaltenzahl']
    //print matrix
    std::cout <<"Transformationsmatrix*******"<<std::endl;
    std::cout <<"["<<std::endl;
    for (int i=0; i<4; i++){
        std::cout<<T[i][0]<<", "<<T[i][1]<<", "<<T[i][2]<<", "
        <<T[i][3]<<std::endl;
    }
    std::cout << "]"<<std::endl;

    //get Rotation in Quaternion
    std::cout <<"Rotation in quaternions[x,y,z,w]**********" <<std::endl;
    std::cout <<"[" << transform.getRotation().getX() <<", "<< transform.getRotation().getY()<<", "
    << transform.getRotation().getZ()<<", "<< transform.getRotation().getW()<<"]"<<std::endl;

    quat_rot = {transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW()};
      
    //transfer Rotation into RPY and write angles into variables
    tf::Matrix3x3(R).getRPY(roll, pitch, yaw);
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;
    std::cout <<std::endl;
    std::cout <<"transformation into RPY xyz [rad] yields: "<<std::endl;
    std::cout <<"["<<roll<<", "<<pitch<<", "<<yaw <<"]" <<std::endl;
      
    //into degree
    alpha = 180.0*roll/PI;
    beta = 180.0*pitch/PI;
    gamma = 180.0*yaw/PI;
    std::cout <<"The transformation into RPY xyz [DEG] yields: "<<std::endl;
    std::cout <<"["<<alpha<<", "<<beta<<", "<<gamma <<"]" <<std::endl;
    getPose(pub_endeffector_pose_);

    std::cout <<"***********2nd Transformation ************************"<<std::endl;
*/
    ROS_INFO("Translation from %s into %s", target_frame2.c_str(),source_frame.c_str());
    r2 = transform2.getOrigin();
    std::cout <<"Translationsvektor******"<<std::endl;
    std::cout <<"["<< r2[0] <<", ";
    std::cout << r2[1] <<", ";
    std::cout << r2[2] <<"]^T" <<std::endl;

      //Transformationsmatrix
    R2 = transform2.getBasis(); //yields KARDAN rotation matrix
    double T2[][4] = { {R2[0][0], R2[0][1], R2[0][2], r2[0]},
                        {R2[1][0], R2[1][1], R2[1][2], r2[1]},
                        {R2[2][0], R2[2][1], R2[2][2], r2[2]},
                        {0, 0, 0, 1} }; //matrix['Zeilenzahl']['Spaltenzahl']
    //print matrix
    std::cout <<"Transformationsmatrix*******"<<std::endl;
    std::cout <<"["<<std::endl;
    for (int i=0; i<4; i++){
        std::cout<<T2[i][0]<<", "<<T2[i][1]<<", "<<T2[i][2]<<", "
        <<T2[i][3]<<std::endl;
    }
    std::cout << "]"<<std::endl;

    /******* get Rotation in Quaternion *******/
    //std::cout <<"Rotation in quaternions[x,y,z,w]**********" <<std::endl;
    //std::cout <<"[" << transform.getRotation().getX() <<", "<< transform.getRotation().getY()<<", "
    //<< transform.getRotation().getZ()<<", "<< transform.getRotation().getW()<<"]"<<std::endl;

    quat_rot = {transform2.getRotation().getX(), transform2.getRotation().getY(), transform2.getRotation().getZ(), transform2.getRotation().getW()};
      
    //transfer Rotation into RPY and write angles into variables
    tf::Matrix3x3(R2).getRPY(roll, pitch, yaw);

    /*
    //Get another Rotationmatrix
    R_YPR = transform2.getBasis().getEulerZYX();
    std::cout<<"Rotation matrix as euler angles around ZYX \n"<<"["<<R_YPR<<std::endl;
    */
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;
    std::cout <<std::endl;
    std::cout <<"transformation into RPY xyz [rad] yields: "<<std::endl;
    std::cout <<"["<<roll<<", "<<pitch<<", "<<yaw <<"]" <<std::endl;
      
    //into degree
    alpha = 180.0*roll/PI;
    beta = 180.0*pitch/PI;
    gamma = 180.0*yaw/PI;
    std::cout <<"The transformation into RPY xyz [DEG] yields: "<<std::endl;
    std::cout <<"["<<alpha<<", "<<beta<<", "<<gamma <<"]" <<std::endl;

    /*
    Eigen::Matrix3d endeff_rotation;
    endeff_rotation << T2[0][0], T2[0][1], T2[0][2],
                      T2[1][0], T2[1][1], T2[1][2],
                      T2[2][0], T2[2][1], T2[2][2];
    std::cout<<"New calculated R = \n"<<endeff_rotation<<std::endl;

    double alpha2 = atan2(-endeff_rotation(1,2),endeff_rotation(2,2));
    double gamma2 = atan2(-endeff_rotation(0,1),endeff_rotation(0,0));
    double beta2 = atan2(endeff_rotation(0,2),((endeff_rotation(0,0)*cos(gamma))-(endeff_rotation(0,1)*sin(gamma))));
    std::cout <<"Orientation in [RAD] yields: "<<std::endl;
    std::cout <<"["<<alpha2<<", "<<beta2<<", "<<gamma2 <<"]" <<std::endl;
    */
    getPose(pub_endeffector_pose_, alpha, beta, gamma);

      /*
      //Manual Calculation of KARDAN convention*************************************
      //back into RAD
      alpha = roll;
      beta = pitch;
      gamma = yaw;
      //Calculate Rotationmatrix of RPY
      double R_x[ROW][COLUMN] = { {1,0,0}, 
                          {0, cos(alpha), (-sin(alpha))}, 
                          {0, sin(alpha), cos(alpha)} };
      double R_y[ROW][COLUMN] = { {cos(beta), 0, sin(beta)},
                          {0, 1, 0},
                          {(-sin(beta)), 0, cos(beta)} };
      double R_z[ROW][COLUMN] = { {cos(gamma), (-sin(gamma)), 0},
                          {sin(gamma), cos(gamma), 0},
                          {0, 0, 1} };
      
      // Initializing elements of matrices to 0.
      for(int i = 0; i < ROW; ++i)
          for(int j = 0; j < COLUMN; ++j)
          {
              R_kard1[i][j]=0;
              R_kard2[i][j]=0;
              R_rpy1[i][j]=0;
              R_rpy2[i][j]=0;
          }
      
      //Berechnung der KARDAN*******************************
      for(int i = 0; i < ROW; ++i)
          for(int j = 0; j < COLUMN; ++j)
              for(int k = 0; k < COLUMN; ++k)
              {
                  R_kard1[i][j] += R_x[i][k] * R_y[k][j];
              }
      //jetzt mit dritter Matrix multiplizieren
      for(int i = 0; i < ROW; ++i)
          for(int j = 0; j < COLUMN; ++j)
              for(int k = 0; k < COLUMN; ++k)
              {
                  R_kard2[i][j] += R_kard1[i][k] * R_z[k][j];
              }
      //auslesen
      std::cout <<"Calculated Rotationmatrix again*******"<<std::endl;
      for (int i=0; i<ROW; i++){
          std::cout<<R_kard2[i][0]<<", "<<R_kard2[i][1]<<", "<<R_kard2[i][2]<<std::endl;
      }
      std::cout << "]"<<std::endl;
      //*************************************************************************************
      */
      

      //Analytische Jacobimatrix berechnen


      
    //}
  }

  void getPose(ros::Publisher pub_endeffector_pose_, double alpha, double beta, double gamma){
    orientation = {roll, pitch, yaw}; //define variables
    translation = {r2[0], r2[1], r2[2]};
    quaternion = {quat_rot[0], quat_rot[1], quat_rot[2], quat_rot[3]};
    double pose1[] = {translation.x, translation.y, translation.z, orientation.x, orientation.y, orientation.z};
    for (unsigned n=0; n<POSE_ARRAY_SIZE; ++n){
      pose_endeffector[n] = pose1[n]; //yields pose of endeffector from base_link_ur5
    }

    mur_robot_msgs::PoseMessage pose_msg;
    pose_msg.position.x = translation.x;
    pose_msg.position.y = translation.y;
    pose_msg.position.z = translation.z;

    pose_msg.orientation.x = quaternion.x;
    pose_msg.orientation.y = quaternion.y;
    pose_msg.orientation.z = quaternion.z;
    pose_msg.orientation.w = quaternion.w;

    pose_msg.rpy_orientation.x = alpha;
    pose_msg.rpy_orientation.y = beta;
    pose_msg.rpy_orientation.z = gamma;
    pub_endeffector_pose_.publish(pose_msg);
    
  }

  void jacobianUR5(){
    //calculates analytical jacobian matrix
    //see on: https://github.com/matchRos/MuR-205/blob/master/mur_robot/src/jacobian_publisher.py

  }

};

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle nh;
  ros::Rate rate(10.0);

  std::string target_frame = "robot1_tf/base_link_ur5";
  ListenFrames obj;

  ros::Publisher pub_endeffector_pose1 = nh.advertise<mur_robot_msgs::PoseMessage>("/cartesian/endeffector_pose/data",100);
  ros::Publisher pub_endeffector_pose = nh.advertise<mur_robot_msgs::PoseMessage>("/cartesian/endeffector_pose/data",100);

  while (nh.ok()){
    obj.getLinkTransformUR5(pub_endeffector_pose1, target_frame);
    rate.sleep();
  }

  return 0;
}