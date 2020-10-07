#include <mur_force_controller/gazebo_ft_publisher.h>

using namespace gazebo_ft_publisher;

WrenchPublisher::WrenchPublisher()
{
    this->nh_=nh_;
    //this->nh_=ros::NodeHandle("publish_sensor_wrench");
    this->pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/robot1_ns/arm_cartesian_compliance_controller/ft_sensor_wrench",100);
    this->sub_= nh_.subscribe("/robot1_ns/ee_force_torque_sensor", 100, &WrenchPublisher::wrenchCallback, this);
}

//destructor
WrenchPublisher::~WrenchPublisher()
{
    printf("No forces are gonna be published to admittance controller \n");
}

void WrenchPublisher::transform_wrench_into_ee(){
    ros::Time now = ros::Time(0); //starting time when calling method (when a loop runs through it inside, the time is set as the very starting time)
    //std::cout<<"Transformationsmatrix zwischen robot1_tf/wrist_3_link_ur5 und robot1_tf/ee_link_ur5: "<<std::endl;
    std::string source_frame = "robot1_tf/wrist_3_link_ur5";
    std::string target_frame = "robot1_tf/ee_link_ur5";
    
    try{
        listener_.waitForTransform(source_frame, target_frame, now, ros::Duration(5.0));
        listener_.lookupTransform(source_frame, target_frame, now, transform_);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    
    // tf::Vector3 tf_force, tf_force_at_base;
    // tf::Vector3 tf_torque, tf_torque_at_base;
    // //force transformation
    // tf::vector3MsgToTF(force_.vector,tf_force);
    // tf_force_at_base = transform_.getBasis().inverse() * tf_force;
    // tf::vector3TFToMsg(tf_force_at_base, wrench_.wrench.force);

    
    // tf::vector3MsgToTF(torque_.vector,tf_torque);
    // tf_torque_at_base = transform_.getBasis().inverse() * tf_torque;
    // tf::vector3TFToMsg(tf_torque_at_base, wrench_.wrench.torque);
    // ROS_INFO_STREAM("Force at ee is: "<<wrench_.wrench.force.x<<", "<<wrench_.wrench.force.y<<", "<<wrench_.wrench.force.z);

    /*****force/torque transformation ******/
    listener_.transformVector(target_frame, force_, wrench_force_);
    listener_.transformVector(target_frame, torque_,wrench_torque_);
    ROS_INFO_STREAM("Force at ee is: "<<wrench_force_.vector.x<<", "<<wrench_force_.vector.y<<", "<<wrench_force_.vector.z);
    
    /***** store wrench values *****/
    wrench_.wrench.force = wrench_force_.vector;
    wrench_.wrench.torque = wrench_torque_.vector;

    // std::cout<<"Wrench is \n";
    // std::cout<<wrench_.wrench.force.x<<std::endl;
    // std::cout<<wrench_.wrench.force.y<<std::endl;
    // std::cout<<wrench_.wrench.force.z<<std::endl;
    // std::cout<<wrench_.wrench.torque.x<<std::endl;
    // std::cout<<wrench_.wrench.torque.y<<std::endl;
    // std::cout<<wrench_.wrench.torque.z<<std::endl;

    /***** Gravitational compensation *****/
    // now = ros::Time(0);
    // tf::StampedTransform transform2_;
    // geometry_msgs::Vector3Stamped gravitation, force_compensated;
    // gravitation.vector.x = 0.0;
    // gravitation.vector.y = 0.0;
    // gravitation.vector.z = 9.81;
    // gravitation.header.frame_id = "/robot1_tf/base_link";
    // gravitation.header.stamp = now;

    // try{
    //     listener_.waitForTransform("robot1_tf/base_link", "robot1_tf/wrist_3_link_ur5", now, ros::Duration(1.0));
    //     listener_.lookupTransform("robot1_tf/base_link", "robot1_tf/wrist_3_link_ur5", now, transform2_);
    // }
    // catch(tf::TransformException ex)
    // {
    //     ROS_ERROR("%s",ex.what());
    //     ros::Duration(1.0).sleep();
    // }
    // listener_.transformVector("robot1_tf/wrist_3_link_ur5",gravitation, force_compensated);
    
    
    // std::cout<<"Compensated force is \n";
    // std::cout<<force_.vector.x - force_compensated.vector.x<<std::endl;
    // std::cout<<force_.vector.y - force_compensated.vector.y<<std::endl;
    // std::cout<<force_.vector.z - force_compensated.vector.z<<std::endl;

    
    /***** Finally publish wrench data to cartesian admittance controller *****/
    pub_.publish(wrench_);

}

void WrenchPublisher::wrenchCallback(geometry_msgs::WrenchStamped wrench_msg_){
    
    force_.header.frame_id = wrench_msg_.header.frame_id;
    force_.header.seq = wrench_msg_.header.seq;
    force_.header.stamp = wrench_msg_.header.stamp;
    force_.vector.x = wrench_msg_.wrench.force.x;
    force_.vector.y = wrench_msg_.wrench.force.y;
    force_.vector.z = wrench_msg_.wrench.force.z;
    
    torque_.header.frame_id = force_.header.frame_id;
    torque_.header.seq = force_.header.seq;
    torque_.header.stamp = force_.header.stamp;
    torque_.vector.x = wrench_msg_.wrench.torque.x;
    torque_.vector.y = wrench_msg_.wrench.torque.y;
    torque_.vector.z = wrench_msg_.wrench.torque.z;

    /***send directly to topic ***/
    gravitation_compensation();
    //send_directly_to_topic();
    //ros::Duration(0.5).sleep();

}

void WrenchPublisher::gravitation_compensation()
{
    Eigen::VectorXd target_wrench(6);
    Eigen::VectorXd gazebo_wrench(6);
    Eigen::VectorXd gravitation(3); //gravitation based on ~/base_link
    Eigen::VectorXd dead_wrench(6); //wrench by gravitation
    Eigen::VectorXd wrench_compensated(6);
    Eigen::Vector3d dead_force;
    Eigen::Vector3d dead_torque;
    Eigen::Matrix3d Rotation; //rotation matrix
    Eigen::Vector3d through;
    Eigen::Vector3d lever; //distance between wrist_3 pose and ee pose
    double mass = 0.01; //wrist 3 mass
    

    gazebo_wrench << force_.vector.x, force_.vector.y, force_.vector.z,  torque_.vector.x, torque_.vector.y, torque_.vector.z;
    
    lever << 0.0, 0.0823, 0.0; 
    
    gravitation << 0.0, 0.0, 9.81;

    std::string source_frame = "robot1_tf/base_link";
    std::string target_frame = "robot1_tf/wrist_3_link_ur5";
    
    ros::Time now = ros::Time(0);
    try{
        listener_.waitForTransform(source_frame, target_frame, now, ros::Duration(5.0));
        listener_.lookupTransform(source_frame, target_frame, now, transform_ft_);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    R = transform_ft_.getBasis(); //.inverse();
    
    Rotation << R[0][0], R[0][1], R[0][2],
                R[1][0], R[1][1], R[1][2],
                R[2][0], R[2][1], R[2][2];
    //ROS_INFO_STREAM("Rotation matrix: \n"<<Rotation);

    /**** Calculate dead load in wrist 3 ****/
    
    dead_force = Rotation.transpose()*mass*gravitation;
    through = Rotation.transpose()*lever;

    dead_torque = through.cross(dead_force);

    dead_wrench << dead_force(0), dead_force(1), dead_force(2), dead_torque(0), dead_torque(1), dead_torque(2);

    wrench_compensated = gazebo_wrench + dead_wrench;

    //ROS_INFO_STREAM("Compensated wrench is: \n"<<wrench_compensated);

    wrench_.wrench.force.x = wrench_compensated(0);
    wrench_.wrench.force.y = wrench_compensated(1);
    wrench_.wrench.force.z = wrench_compensated(2);

    wrench_.wrench.torque.x = wrench_compensated(3);
    wrench_.wrench.torque.y = wrench_compensated(4);
    wrench_.wrench.torque.z = wrench_compensated(5);

    pub_.publish(wrench_);

}

void WrenchPublisher::send_directly_to_topic()
{
    //wrench_.header.frame_id = "robot1_tf/wrist_3_link_ur5";
    wrench_.wrench.force = force_.vector;
    wrench_.wrench.torque = torque_.vector;

    pub_.publish(wrench_);
}

