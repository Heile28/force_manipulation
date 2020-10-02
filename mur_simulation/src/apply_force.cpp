/*
 * File: main.cpp
 * Author: Heiko Lenz
 * 
 * Created on 21. August 2020
 * 
 * Applying force profiles at wrist3_link
 * 
*/

#include <ros/ros.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/WrenchStamped.h>

#include <mur_force_controller/mur_base.h>
#include <tf/transform_broadcaster.h>

class ApplyForce
{
    private:
        MurBase base_;
        tf::StampedTransform transform_;
        tf::Vector3 tf_force_apply_, tf_force_at_base_;
        geometry_msgs::Vector3 force_, force_at_base_;
        ros::NodeHandle nh_;
        ros::Publisher pub_wrench = nh_.advertise<geometry_msgs::WrenchStamped>("/robot1_ns/arm_cartesian_compliance_controller/target_wrench",20);

    public:
        void apply_constant_force()
        {
            ros::Rate r(20.0); //0.05 seconds publishing rate
            
            ros::service::waitForService("/gazebo/apply_body_wrench"); //link: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_msgs/srv/ApplyBodyWrench.srv
            ros::ServiceClient force_client = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
            gazebo_msgs::ApplyBodyWrench srv1; //namespace + servicename
            ros::Duration duration_seconds(0.05); //specify duration of force attack in seconds
            
            srv1.request.body_name = "robot1::robot1_tf/wrist_3_link_ur5"; //robot1::robot1_tf/base_footprint
            srv1.request.reference_frame = "robot1_tf/wrist_3_link_ur5";
            srv1.request.start_time = ros::Time().now();
            srv1.request.duration = duration_seconds;
            srv1.request.reference_point.x = 0.0;
            srv1.request.reference_point.y = 0.0; //0.0823;
            srv1.request.reference_point.z = 0.0;
            /**** forces specified in global coordinates ****/

            /***** Transform wrench vector from ft_sensor_ref_link into ~/base_link *****/
            //transform_ = base_.transform("robot1_tf/ee_link_ur5", "robot1_tf/base_link_ur5");
            transform_ = base_.transform("robot1_tf/wrist_3_link_ur5", "robot1_tf/base_link_ur5");

            //tf::Transform( transform_.getRotation(),tf::Vector3(0.0,0.0,0.0) );

            tf::vector3MsgToTF(force_,tf_force_apply_);
            tf_force_at_base_ = transform_.getBasis().inverse() * tf_force_apply_;
            tf::vector3TFToMsg(tf_force_at_base_, force_at_base_);

            srv1.request.wrench.force.x = force_at_base_.x; 
            srv1.request.wrench.force.y = force_at_base_.y; 
            srv1.request.wrench.force.z = force_at_base_.z; 
            srv1.request.wrench.torque.x = 0.0;
            srv1.request.wrench.torque.y = 0.0;
            srv1.request.wrench.torque.z = 0.0;
            force_client.call(srv1);

            /*
            if(srv1.response.success == true){
                ROS_INFO("Force applied");
            }
            */
            //ros::Duration(5.0).sleep();
            //ROS_INFO("Force attack has finished!");
                
        }

        void profiled_attack()
        {

        }


        /**** METHOD publishes WRENCH referenced to ~/wrist_3_link ****/
        void apply_force_profile1()
        {
            ros::Rate r(20.0);
            double force[3]={0.0,0.0,0.0};
            geometry_msgs::WrenchStamped pub_wrench_msg;
            //pub_wrench_msg.header.frame_id = "robot1_tf/wrist_3_link_ur5"; //wrist_3_link_ur5

            /**** Force profile ****/
            double current_time = ros::Time::now().toSec();
            double time1 = 2.0;
            double time2 = 6.0;
            double time3 = 8.0;
            

            ROS_INFO("Force attack 8 seconds in real.");
            std::cout<<"Current time: "<<current_time<<std::endl;

            pub_wrench_msg.wrench.force.x = 0.0;
            pub_wrench_msg.wrench.force.y = 0.0;
            pub_wrench_msg.wrench.force.z = 0.0;
            pub_wrench_msg.wrench.torque.x = 0.0;
            pub_wrench_msg.wrench.torque.y = 0.0;
            pub_wrench_msg.wrench.torque.z = 0.0;

            while(current_time <= time1){
                pub_wrench_msg.wrench.force.y = 7.5*pow(current_time, 2);
                //std::cout<<"Force in z: "<<pub_wrench_msg.wrench.force.y<<std::endl
                pub_wrench_msg.header.stamp = ros::Time::now();
                force_.x = pub_wrench_msg.wrench.force.x;
                force_.y = pub_wrench_msg.wrench.force.y;
                force_.z = pub_wrench_msg.wrench.force.z;
                apply_constant_force();
                pub_wrench.publish(pub_wrench_msg);
                current_time = current_time + 0.05;
                //std::cout<<"Current time: "<<current_time<<std::endl;
                r.sleep();
            }

            while(current_time <= time2){
                pub_wrench_msg.wrench.force.y = 30.0;
                pub_wrench_msg.header.stamp = ros::Time::now();
                //std::cout<<"Force in z: "<<pub_wrench_msg.wrench.force.y<<std::endl;
                force_.x = pub_wrench_msg.wrench.force.x;
                force_.y = pub_wrench_msg.wrench.force.y;
                force_.z = pub_wrench_msg.wrench.force.z;
                apply_constant_force();
                pub_wrench.publish(pub_wrench_msg);
                current_time = current_time + 0.05;
                //std::cout<<"Current time: "<<current_time<<std::endl;
                r.sleep();
            }

            while(current_time <= time3){
                pub_wrench_msg.wrench.force.y = 7.5*pow(current_time-8.0, 2);
                pub_wrench_msg.header.stamp = ros::Time::now();
                //std::cout<<"Force in z: "<<pub_wrench_msg.wrench.force.y<<std::endl;
                force_.x = pub_wrench_msg.wrench.force.x;
                force_.y = pub_wrench_msg.wrench.force.y;
                force_.z = pub_wrench_msg.wrench.force.z;
                apply_constant_force();
                pub_wrench.publish(pub_wrench_msg);
                current_time = current_time + 0.05;
                //std::cout<<"Current time: "<<current_time<<std::endl;
                r.sleep();
            }
            /*** Reset ****/
            pub_wrench_msg.wrench.force.x = 0.0;
            pub_wrench_msg.wrench.force.y = 0.0;
            pub_wrench_msg.wrench.force.z = 0.0;
            pub_wrench_msg.wrench.torque.x = 0.0;
            pub_wrench_msg.wrench.torque.y = 0.0;
            pub_wrench_msg.wrench.torque.z = 0.0;
            pub_wrench.publish(pub_wrench_msg);
            
            std::cout<<"Force attack finished!"<<std::endl;

        }

        void apply_force_profile2()
        {
            /**** Constant force ****/
            ros::Rate r(20.0);
            
            
            geometry_msgs::WrenchStamped pub_wrench_msg;
            pub_wrench_msg.header.frame_id = "robot1_tf/base_link_ur5"; //wrist_3_link_ur5
            
            double current_time = ros::Time::now().toSec();
            std::cout<<"Current time "<<current_time<<std::endl;
            double time1 = 5.0;

            while(current_time <= time1){
                pub_wrench_msg.header.stamp = ros::Time::now();

                //Specify force in ee_link related to ~/base_link_ur5
                pub_wrench_msg.wrench.force.x = 0.0;
                pub_wrench_msg.wrench.force.y = 80.0;
                pub_wrench_msg.wrench.force.z = 30.0;
                pub_wrench_msg.wrench.torque.x = 0.0;
                pub_wrench_msg.wrench.torque.y = 0.0;
                pub_wrench_msg.wrench.torque.z = 0.0;

                force_.x = pub_wrench_msg.wrench.force.x;
                force_.y = pub_wrench_msg.wrench.force.y;
                force_.z = pub_wrench_msg.wrench.force.z;

                apply_constant_force();
                
                current_time = current_time + 0.05;
                std::cout<<"Current time "<<current_time<<std::endl;
                r.sleep();
                pub_wrench.publish(pub_wrench_msg);
            }
            current_time = ros::Time(0).toSec();
            ROS_INFO_STREAM("Current time 2 "<<current_time);
            double time2 = 2;
            while(current_time <= time2)
            {
                pub_wrench_msg.wrench.force.x = 0.0;
                pub_wrench_msg.wrench.force.y = 0.0;
                pub_wrench_msg.wrench.force.z = 0.0;
                pub_wrench_msg.wrench.torque.x = 0.0;
                pub_wrench_msg.wrench.torque.y = 0.0;
                pub_wrench_msg.wrench.torque.z = 0.0;
                pub_wrench.publish(pub_wrench_msg);
                current_time +=time2;
                r.sleep();
            }
            
            std::cout<<"Force attack finished!"<<std::endl;
            
            

        }

        void reset()
        {
            /*** Reset ****/
            double time = 2.0;
            double current_time = ros::Time(0).toSec();

            ros::Rate r(1.5);

            geometry_msgs::WrenchStamped pub_wrench_msg;
            pub_wrench_msg.wrench.force.x = 0.0;
            pub_wrench_msg.wrench.force.y = 0.0;
            pub_wrench_msg.wrench.force.z = 0.0;
            pub_wrench_msg.wrench.torque.x = 0.0;
            pub_wrench_msg.wrench.torque.y = 0.0;
            pub_wrench_msg.wrench.torque.z = 0.0;
            pub_wrench_msg.header.stamp = ros::Time::now();
            
            while(current_time <= time){
                pub_wrench_msg.header.stamp = ros::Time::now();
                force_.y =pub_wrench_msg.wrench.force.y;
                //apply_constant_force();
                pub_wrench.publish(pub_wrench_msg);
                r.sleep();
                current_time = current_time + 0.666;
            }

        }



};





int main(int argc, char** argv)
{
    ros::init(argc,argv,"apply_force");
    ros::NodeHandle nh_;

    ApplyForce obj;
    obj.apply_force_profile2();
    
    ros::spin();
}