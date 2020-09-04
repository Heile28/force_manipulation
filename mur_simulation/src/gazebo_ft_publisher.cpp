#include <mur_simulation/gazebo_ft_publisher.h>

using namespace gazebo_ft_publisher;

WrenchPublisher::WrenchPublisher()
{
    this->nh_=nh_;
    //this->nh_=ros::NodeHandle("publish_sensor_wrench");
    this->pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/robot1_ns/arm_cartesian_compliance_controller/ft_sensor_wrench",100);
    this->sub_= nh_.subscribe("/robot1_ns/ee_force_torque_sensor", 100, &WrenchPublisher::wrenchCallback, this);
}

void WrenchPublisher::wrenchCallback(geometry_msgs::WrenchStamped::ConstPtr wrench_msg_){
    
    force_.header.frame_id = wrench_msg_->header.frame_id;
    force_.header.seq = wrench_msg_->header.seq;
    force_.header.stamp = wrench_msg_->header.stamp;
    force_.vector.x = wrench_msg_->wrench.force.x;
    force_.vector.y = wrench_msg_->wrench.force.y;
    force_.vector.z = wrench_msg_->wrench.force.z;
    
    torque_.header.frame_id = force_.header.frame_id;
    torque_.header.seq = force_.header.seq;
    torque_.header.stamp = force_.header.stamp;
    torque_.vector.x = wrench_msg_->wrench.torque.x;
    torque_.vector.y = wrench_msg_->wrench.torque.y;
    torque_.vector.z = wrench_msg_->wrench.torque.z;

    transform_wrench_into_ee();
}



tf::StampedTransform WrenchPublisher::tf_listener(std::string &source_frame, std::string &target_frame)
{
    //reference: http://wiki.ros.org/tf2/Tutorials/Using%20stamped%20datatypes%20with%20tf2::MessageFilter
    ros::Time now = ros::Time(0); //start time when calling method (when a loop runs through it inside, the time is set as the very starting time)

    try{
        listener_.waitForTransform(source_frame, target_frame, now, ros::Duration(1.0));
        listener_.lookupTransform(source_frame, target_frame, now, transform_);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    /***************************
     * *****query translation******
     * *****************************/
    //p = transform.getOrigin();
    
    
    /***************************
     * *****query rotation******
     * *****************************/
    /*
    R = transform.getBasis(); //yields Quaternion rotation matrix ???
    t.clear();
    t.push_back(R[0][0]); t.push_back(R[0][1]); t.push_back(R[0][2]); t.push_back(p[0]);
    t.push_back(R[1][0]); t.push_back(R[1][1]); t.push_back(R[1][2]); t.push_back(p[1]);
    t.push_back(R[2][0]); t.push_back(R[2][1]); t.push_back(R[2][2]); t.push_back(p[2]);
    t.push_back(0.0); t.push_back(0.0); t.push_back(0.0); t.push_back(1.0);
    */
    /*
    T << R[0][0], R[0][1], R[0][2], r[0],
        R[1][0], R[1][1], R[1][2], r[1],
        R[2][0], R[2][1], R[2][2], r[2],
        0, 0, 0, 1;
        */
    return transform_;
}

void WrenchPublisher::transform_wrench_into_ee(){

    std::cout<<"Transformationsmatrix zwischen robot1_tf/wrist_3_link_ur5 und robot1_tf/ee_link_ur5: "<<std::endl;
    std::string source_frame = "robot1_tf/wrist_3_link_ur5";
    std::string target_frame = "robot1_tf/ee_link_ur5";
    
    transform = tf_listener(source_frame, target_frame);
 
    //store wrench values
    listener_.transformVector(target_frame, force_, wrench_force_);
    listener_.transformVector(target_frame, force_, wrench_torque_);
    
    wrench_.wrench.force = wrench_force_.vector;
    wrench_.wrench.torque = wrench_torque_.vector;

    pub_.publish(wrench_);
    
    /*
    //send force vector to tf
    tf::vector3StampedMsgToTF(force_,force_at_wrist3);

    force_at_ee = transform * force_at_wrist3;

    tf::vector3TFToMsg(force_at_ee, wrench_force_);
    */
}