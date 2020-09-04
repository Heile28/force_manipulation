#include <mur_simulation/gazebo_ft_publisher.h>

struct Frames {
    std::string frame0 = "robot1_tf/wrist_3_link_ur5";
    std::string frame0 = "robot1_tf/ee_link_ur5";

    std::string& operator[](int n) 
    {
        // the idea, get the pointer of the first element
        // and treat it as an array
        return (&frame0)[n];
    }
};

WrenchPublisher::WrenchPublisher()
{
    this->nh_=ros::NodeHandle("~");
    this->pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/robot1_ns/arm_cartesian_compliance_controller/ft_sensor_wrench",100);
    this->sub_= nh_.subscribe("/robot1_ns/ee_force_torque_sensor", 100, &WrenchPublisher::wrenchCallback, this);
}

void WrenchPublisher::wrenchCallback(geometry_msgs::WrenchStamped::ConstPtr wrench_msg_){
    measured_wrench_.header.frame_id = wrench_msg_->header.frame_id;
    measured_wrench_.header.seq = wrench_msg_->header.seq;
    measured_wrench_.header.stamp = wrench_msg_->header.stamp;

    measured_wrench_.wrench.force.x = wrench_msg_->wrench.force.x;
    measured_wrench_.wrench.force.y = wrench_msg_->wrench.force.y;
    measured_wrench_.wrench.force.z = wrench_msg_->wrench.force.z;
    measured_wrench_.wrench.torque.x = wrench_msg_->wrench.torque.x;
    measured_wrench_.wrench.torque.y = wrench_msg_->wrench.torque.y;
    measured_wrench_.wrench.torque.z = wrench_msg_->wrench.torque.z;

    
}

std::vector<double> tf_listener(std::string &source_frame, std::string &target_frame)
{
    //reference: http://wiki.ros.org/tf2/Tutorials/Using%20stamped%20datatypes%20with%20tf2::MessageFilter
    std::vector<double> t;
    tf::Vector3 p;
    tf::Matrix3x3 R;

    tf::StampedTransform transform;
    tf::TransformListener listener;
    ros::Time now = ros::Time(0); //start time when calling method (when a loop runs through it inside, the time is set as the very starting time)

    try{
        listener.waitForTransform(source_frame, target_frame, now, ros::Duration(1.0));
        listener.lookupTransform(source_frame, target_frame, now, transform); 
    }
    catch(tf2::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    /***************************
     * *****query translation******
     * *****************************/
    p = transform.getOrigin();
    
    
    /***************************
     * *****query rotation******
     * *****************************/
    R = transform.getBasis(); //yields Quaternion rotation matrix ???
    t.clear();
    t.push_back(R[0][0]); t.push_back(R[0][1]); t.push_back(R[0][2]); t.push_back(p[0]);
    t.push_back(R[1][0]); t.push_back(R[1][1]); t.push_back(R[1][2]); t.push_back(p[1]);
    t.push_back(R[2][0]); t.push_back(R[2][1]); t.push_back(R[2][2]); t.push_back(p[2]);
    t.push_back(0.0); t.push_back(0.0); t.push_back(0.0); t.push_back(1.0);

    /*
    T << R[0][0], R[0][1], R[0][2], r[0],
        R[1][0], R[1][1], R[1][2], r[1],
        R[2][0], R[2][1], R[2][2], r[2],
        0, 0, 0, 1;
        */
    return t;
}

void tansform_wrench_into_ee(){
    std::vector<double> transform_vector;
    Frames frames;
    std::cout<<"Transformationsmatrix zwischen "<<frames[0]<<" und "<<frames[1]<<": "<<std::endl;
    transform_vector = tf_listener(frames[0], frames[1]);
    //print out vector values
    for(int m=0; m<transform_vector.size(); m++)
        std::cout<<transform_vector.at(m)<<std::endl;

    tf::Transform trafo();
    trafo
    
    
}