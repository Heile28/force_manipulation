/************************************************
 * Berechnung der geometrischen Jacobimatrix ****
 * aus den homogenen Transformationsmatrizen ****
 * und Auslenkung des Endeffektors           ****
 * *********************************************/

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2/transform_datatypes.h>
//#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <my_mur_msgs/PoseRequest.h>
#include <my_mur_msgs/PoseMessage.h>
#include <geometry_msgs/WrenchStamped.h> //for robot1_ns/ee_force_torque_sensor

#include <math.h>

struct Transform {
    Eigen::Matrix4d T1;
    Eigen::Matrix4d T2; 
    Eigen::Matrix4d T3; 
    Eigen::Matrix4d T4; 
    Eigen::Matrix4d T5; 
    Eigen::Matrix4d T6;

    Eigen::Matrix4d& operator[](int n) 
    {
        // the idea, get the pointer of the first element
        // and treat it as an array
        return (&T1)[n];
    }
    
};

struct TransformVector{
    std::vector<double> t1;
    std::vector<double> t2;
    std::vector<double> t3;
    std::vector<double> t4;
    std::vector<double> t5;
    std::vector<double> t6;
    
    std::vector<double>& operator[](int n) 
    {
        // the idea, get the pointer of the first element
        // and treat it as an array
        return (&t1)[n];
    }
};

struct Frames {
    std::string frame0 = "robot1_tf/base_link_ur5";
    std::string frame1 = "robot1_tf/shoulder_link_ur5";
    std::string frame2 = "robot1_tf/upper_arm_link_ur5";
    std::string frame3 = "robot1_tf/forearm_link_ur5";
    std::string frame4 = "robot1_tf/wrist_1_link_ur5";
    std::string frame5 = "robot1_tf/wrist_2_link_ur5";
    std::string frame6 = "robot1_tf/wrist_3_link_ur5";

    std::string& operator[](int n) 
    {
        // the idea, get the pointer of the first element
        // and treat it as an array
        return (&frame0)[n];
    }
};

class CalculateJacobians{
    private:
    ros::NodeHandle nh_;
    ros::ServiceClient endeffector_pose_client;
    //ros::Subscriber sub_pose_, sub_force_;
    double theta1, theta2, theta3, theta4, theta5, theta6;
    //stiffness parameters
    double null[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double weigh_factor = 0.05; //weighting factor

    //DH parameters
    const double d1 = 0.089159;
    const double a2 = 0.42500; //-
    const double a3 = 0.39225; //-
    const double d4 = 0.10915; 
    const double d5 = 0.09465;
    const double d6 = 0.0823; //caution: set the right distance between wrist_3_link_ur5 and ee_link_ur5 !!!

    const double PI = M_PI;
    const double alpha1 = PI/2;
    const double alpha2 = 0.0;
    const double alpha3 = 0.0;
    const double alpha4 = PI/2;
    const double alpha5 = -PI/2;
    const double alpha6 = 0.0;
    const double d[6] = {d1, 0.0, 0.0, d4, d5, d6};
    const double a[6] = {0.0, a2, a3, 0.0, 0.0, 0.0};
    std::vector<double> wrench; //{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> initial_pose;
    std::vector<double> current_pose; //{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> stiffness_list;

    public:

    CalculateJacobians(){}

    void callbackJointAngles(const sensor_msgs::JointState& joint_msg){
        //sub_force_ = nh_.subscribe("/robot1_ns/ee_force_torque_sensor", 100, &CalculateJacobians::callbackWrenchState, this); //from gazebo ft_sensor plugin
        //sub_pose_ = nh_.subscribe("/cartesian/endeffector_pose/data", 100, &CalculateJacobians::callbackCurrentPose,  this); //from listen_frames_node
        theta1 = joint_msg.position[0];
        theta2 = joint_msg.position[1];
        theta3 = joint_msg.position[2];
        theta4 = joint_msg.position[3];
        theta5 = joint_msg.position[4];
        theta6 = joint_msg.position[5];
        //std::cout<<"Inside callback!"<<std::endl;
        
        urJacobian();
        //analytical_jacobian();
        
    }

    void callbackCurrentPose(const my_mur_msgs::PoseMessage& pose_msg){
        current_pose.clear();
        //std::cout<<"Subscribe endeffector pose directed from /robot1_tf/base_link_ur5"<<std::endl;
        current_pose.push_back(pose_msg.position.x);
        current_pose.push_back(pose_msg.position.y);
        current_pose.push_back(pose_msg.position.z);
        current_pose.push_back(pose_msg.rpy_orientation.x);
        current_pose.push_back(pose_msg.rpy_orientation.y);
        current_pose.push_back(pose_msg.rpy_orientation.z);
        
        /*
        current_pose[0] = pose_msg.position.x;
        current_pose[1] = pose_msg.position.y;
        current_pose[2] = pose_msg.position.z;
        current_pose[3] = pose_msg.rpy_orientation.x;
        current_pose[4] = pose_msg.rpy_orientation.y;

        current_pose[5] = pose_msg.rpy_orientation.z;
        std::cout<<current_pose[1]<<"\n"<<std::endl;
        */
        
    }

    
    std::vector<double> tf_listener(std::string &source_frame, std::string &target_frame)
    {
        //reference: http://wiki.ros.org/tf2/Tutorials/Using%20stamped%20datatypes%20with%20tf2::MessageFilter
        std::vector<double> t;
        tf::Vector3 p;
        tf::Matrix3x3 R;
        /*
        tf2::Vector3 p;
        tf2::Matrix3x3 R;
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf2_listener(tf_buffer);
        geometry_msgs::TransformStamped transform;
        */
        tf::StampedTransform transform;
        tf::TransformListener listener;
        ros::Time now = ros::Time(0);

        try{
            
            //transform = tf_buffer.lookupTransform(source_frame, target_frame, now, ros::Duration(1.0));
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
        //ROS_INFO("Translation from %s into %s", target_frame.c_str(), source_frame.c_str());
        //p = transform.transform.translation();
        p = transform.getOrigin();
        /* //print Translationsvektor
        std::cout <<"Translationsvektor******"<<std::endl;
        std::cout <<"["<< r[0] <<", ";
        std::cout << r[1] <<", ";
        std::cout << r[2] <<"]^T" <<std::endl;
        */
        
        
        /***************************
         * *****query rotation******
         * *****************************/
        /*
        tf2::Quaternion quat;
        quat.setRPY(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
        transform.transform.rotation.x = quat.x();
        transform.transform.rotation.y = quat.y();
        transform.transform.rotation.z = quat.z();
        transform.transform.rotation.q = quat.w();
        */
        
        R = transform.getBasis(); //yields Quaternion rotation matrix
        t.clear();
        t.push_back(R[0][0]); t.push_back(R[0][1]); t.push_back(R[0][2]); t.push_back(p[0]);
        t.push_back(R[1][0]); t.push_back(R[1][1]); t.push_back(R[1][2]); t.push_back(p[1]);
        t.push_back(R[2][0]); t.push_back(R[2][1]); t.push_back(R[2][2]); t.push_back(p[2]);
        t.push_back(0.0); t.push_back(0.0); t.push_back(0.0); t.push_back(1.0);

        tf::Quaternion q;
        std::cout<<q.x()<<", "<<q.y()<<", "<<q.z()<<", "<<q.w()<<std::endl;
        

        /*
        T << R[0][0], R[0][1], R[0][2], r[0],
            R[1][0], R[1][1], R[1][2], r[1],
            R[2][0], R[2][1], R[2][2], r[2],
            0, 0, 0, 1;
            */
        return t;
    }

    void test(){
        TransformVector transform_vector;
        Frames frames;
        std::cout<<"Transformationsmatrix zwischen "<<frames[5]<<" und "<<frames[6]<<": "<<std::endl;
        transform_vector[0] = tf_listener(frames[5], frames[6]);
        for(int m=0; m<transform_vector[0].size(); m++)
            std::cout<<transform_vector[0].at(m)<<std::endl;

        std::cout<<"Transformationsmatrix zwischen "<<frames[2]<<" und "<<frames[3]<<": "<<std::endl;
        transform_vector[1] = tf_listener(frames[2], frames[3]);
        for(int m=0; m<transform_vector[1].size(); m++)
            std::cout<<transform_vector[1].at(m)<<std::endl;
    }
    
    void analytical_jacobian(){
        //analytische Jacobimatrix
        double x_e, y_e, z_e;
        x_e = -(d5*(sin(theta1)*cos(theta2+theta3+theta4)-cos(theta1)*sin(theta2+theta3+theta4)))/2.0 + 
            (d5*(sin(theta1)*cos(theta2+theta3+theta4)+cos(theta1)*sin(theta2+theta3+theta4)))/2.0 +
            d4*sin(theta1) - (d6*(cos(theta1)*cos(theta2+theta3+theta4)-sin(theta1)*sin(theta2+theta3+theta4))*sin(theta5))/2.0 -
            (d6*(cos(theta1)*cos(theta2+theta3+theta4)+sin(theta1)*sin(theta2+theta3+theta4))*sin(theta5))/2.0 + 
            a2*cos(theta1)*cos(theta2) + d6*cos(theta5)*sin(theta1) +
            a3*cos(theta1)*cos(theta2)*cos(theta3) - a3*cos(theta1)*sin(theta2)*sin(theta3);

        y_e = -(d5*((cos(theta1)*cos(theta2+theta3+theta4))-(sin(theta1)*sin(theta2+theta3+theta4))))/2.0 + 
            (d5*(cos(theta1)*cos(theta2+theta3+theta4)+sin(theta1)*sin(theta2+theta3+theta4)))/2.0 -
            d4*cos(theta1) - (d6*(sin(theta1)*cos(theta2+theta3+theta4) + cos(theta1)*sin(theta2+theta3+theta4))*sin(theta5))/2.0 -
            (d6*(sin(theta1)*cos(theta2+theta3+theta4)-cos(theta1)*sin(theta2+theta3+theta4))*sin(theta5))/2.0 - 
            d6*cos(theta1)*cos(theta5) + a2*cos(theta2)*sin(theta1) + 
            a3*cos(theta2)*cos(theta3)*sin(theta1) - a3*sin(theta1)*sin(theta2)*sin(theta3);

        z_e = d1 + (d6*(cos(theta2+theta3+theta4)*cos(theta5) - sin(theta2+theta3+theta4)*sin(theta5)))/2.0 + a3*(sin(theta2)*cos(theta3) +
            cos(theta2)*sin(theta3)) + a2*sin(theta2) - (d6*(cos(theta2+theta3+theta4)*cos(theta5) + 
            sin(theta2+theta3+theta4)*sin(theta5)))/2.0 - d5*cos(theta2+theta3+theta4);
        std::cout<<"Endeffector is at position: "<<x_e<<", "<<y_e<<", "<<z_e<<std::endl;

        Eigen::MatrixXd J_anal(3,6);
        //derive by theta1
        J_anal(0,0) = -0.5*d6*(cos(theta1)*sin(theta4+theta3+theta2)-sin(theta1)*cos(theta4+theta3+theta2))*sin(theta5)-
                0.5*d6*(-cos(theta1)*sin(theta4+theta3+theta2)-sin(theta1)*cos(theta4+theta3+theta2))*sin(theta5)+
                d6*cos(theta1)*cos(theta5)-0.5*d5*(sin(theta1)*sin(theta4+theta3+theta2)+cos(theta1)*cos(theta4+theta3+theta2))+
                0.5*d5*(cos(theta1)*cos(theta4+theta3+theta2)-sin(theta1)*sin(theta4+theta3+theta2))+a3*sin(theta1)*sin(theta2)*sin(theta3)-
                a3*sin(theta1)*cos(theta2)*cos(theta3)-a2*sin(theta1)*cos(theta2)+d4*cos(theta1);
        //derive by theta2
        J_anal(0,1) = -0.5*d6*(sin(theta1)*cos(theta4+theta3+theta2)-cos(theta1)*sin(theta4+theta3+theta2))*sin(theta5)-
                0.5*d6*(-cos(theta1)*sin(theta4+theta3+theta2)-sin(theta1)*cos(theta4+theta3+theta2))*sin(theta5)+
                0.5*d5*(cos(theta1)*cos(theta4+theta3+theta2)-sin(theta1)*sin(theta4+theta3+theta2))-
                0.5*d5*(-sin(theta1)*sin(theta4+theta3+theta2)-cos(theta1)*cos(theta4+theta3+theta2))-
                a3*cos(theta1)*cos(theta2)*sin(theta3)-a3*cos(theta1)*sin(theta2)*cos(theta3)-a2*cos(theta1)*sin(theta2);
        //derive by theta3
        J_anal(0,2) = -0.5*d6*(sin(theta1)*cos(theta4+theta3+theta2)-cos(theta1)*sin(theta4+theta3+theta2))*sin(theta5)-
                0.5*d6*(-cos(theta1)*sin(theta4+theta3+theta2)-sin(theta1)*cos(theta4+theta3+theta2))*sin(theta5)+
                0.5*d5*(cos(theta1)*cos(theta4+theta3+theta2)-sin(theta1)*sin(theta4+theta3+theta2))-
                0.5*d5*(-sin(theta1)*sin(theta4+theta3+theta2)-cos(theta1)*cos(theta4+theta3+theta2))-
                a3*cos(theta1)*cos(theta2)*sin(theta3)-a3*cos(theta1)*sin(theta2)*cos(theta3);
        //derive by theta4
        J_anal(0,3) = -0.5*d6*(sin(theta1)*cos(theta4+theta3+theta2)-cos(theta1)*sin(theta4+theta3+theta2))*sin(theta5)-
                0.5*d6*(-cos(theta1)*sin(theta4+theta3+theta2)-sin(theta1)*cos(theta4+theta3+theta2))*sin(theta5)+
                0.5*d5*(cos(theta1)*cos(theta4+theta3+theta2)-sin(theta1)*sin(theta4+theta3+theta2))-
                0.5*d5*(-sin(theta1)*sin(theta4+theta3+theta2)-cos(theta1)*cos(theta4+theta3+theta2));
        //derive by theta5
        J_anal(0,4) = -d6*sin(theta1)*sin(theta5)-0.5*d6*(sin(theta1)*sin(theta4+theta3+theta2)+cos(theta1)*cos(theta4+theta3+theta2))*cos(theta5)-
                0.5*d6*(cos(theta1)*cos(theta4+theta3+theta2)-sin(theta1)*sin(theta4+theta3+theta2))*cos(theta5);
        //derive by theta6
        J_anal(0,5) = 0;

        J_anal(1,0) = -0.5*d6*(sin(theta1)*sin(theta4+theta3+theta2)+cos(theta1)*cos(theta4+theta3+theta2))*sin(theta5)-
                0.5*d6*(cos(theta1)*cos(theta4+theta3+theta2)-sin(theta1)*sin(theta4+theta3+theta2))*sin(theta5)+
                d6*sin(theta1)*cos(theta5)-0.5*((d5*(cos(theta1)*cos(theta4+theta3+theta2))-sin(theta1)*sin(theta4+theta3+theta2))/theta1)+
                0.5*d5*(cos(theta1)*sin(theta4+theta3+theta2)-sin(theta1)*cos(theta4+theta3+theta2))-a3*cos(theta1)*sin(theta2)*sin(theta3)+
                a3*cos(theta1)*cos(theta2)*cos(theta3)+a2*cos(theta1)*cos(theta2)+d4*sin(theta1);
        J_anal(1,1) = -0.5*d6*(cos(theta1)*cos(theta4+theta3+theta2)-sin(theta1)*sin(theta4+theta3+theta2))*sin(theta5)-
                0.5*d6*(-sin(theta1)*sin(theta4+theta3+theta2)-cos(theta1)*cos(theta4+theta3+theta2))*sin(theta5)-
                0.5*(d5*(cos(theta1)*cos(theta4+theta3+theta2)-sin(theta1)*sin(theta4+theta3+theta2))/theta2)+
                0.5*d5*(sin(theta1)*cos(theta4+theta3+theta2)-cos(theta1)*sin(theta4+theta3+theta2))-
                a3*sin(theta1)*cos(theta2)*sin(theta3)-a3*sin(theta1)*sin(theta2)*cos(theta3)-a2*sin(theta1)*sin(theta2);
        J_anal(1,2) = -0.5*d6*(cos(theta1)*cos(theta4+theta3+theta2)-sin(theta1)*sin(theta4+theta3+theta2))*sin(theta5)-
                0.5*d6*(-sin(theta1)*sin(theta4+theta3+theta2)-cos(theta1)*cos(theta4+theta3+theta2))*sin(theta5)-
                0.5*(d5*(cos(theta1)*cos(theta4+theta3+theta2)-sin(theta1)*sin(theta4+theta3+theta2))/theta3)+
                0.5*d5*(sin(theta1)*cos(theta4+theta3+theta2)-cos(theta1)*sin(theta4+theta3+theta2))-
                a3*sin(theta1)*cos(theta2)*sin(theta3)-a3*sin(theta1)*sin(theta2)*cos(theta3);
        J_anal(1,3) = -0.5*d6*(cos(theta1)*cos(theta4+theta3+theta2)-sin(theta1)*sin(theta4+theta3+theta2))*sin(theta5)-
                0.5*d6*(-sin(theta1)*sin(theta4+theta3+theta2)-cos(theta1)*cos(theta4+theta3+theta2))*sin(theta5)-
                0.5*(d5*(cos(theta1)*cos(theta4+theta3+theta2)-sin(theta1)*sin(theta4+theta3+theta2))/theta4)+	
                0.5*d5*(sin(theta1)*cos(theta4+theta3+theta2)-cos(theta1)*sin(theta4+theta3+theta2));
        J_anal(1,4) = d6*cos(theta1)*sin(theta5)-0.5*d6*(cos(theta1)*sin(theta4+theta3+theta2)+
                sin(theta1)*cos(theta4+theta3+theta2))*cos(theta5)-0.5*d6*(sin(theta1)*cos(theta4+theta3+theta2)-
                cos(theta1)*sin(theta4+theta3+theta2))*cos(theta5);
        J_anal(1,5) = 0;

        J_anal(2,0) = 0;
        J_anal(2,1) = -0.5*d6*(cos(theta4+theta3+theta2)*sin(theta5)-sin(theta4+theta3+theta2)*cos(theta5))+
                0.5*d6*(-cos(theta4+theta3+theta2)*sin(theta5)-sin(theta4+theta3+theta2)*cos(theta5))+
                d5*sin(theta4+theta3+theta2)+a3*(cos(theta2)*cos(theta3)-sin(theta2)*sin(theta3))+a2*cos(theta2);
        J_anal(2,2) = -0.5*d6*(cos(theta4+theta3+theta2)*sin(theta5)-sin(theta4+theta3+theta2)*cos(theta5))+
                0.5*d6*(-cos(theta4+theta3+theta2)*sin(theta5)-sin(theta4+theta3+theta2)*cos(theta5))+
                d5*sin(theta4+theta3+theta2)+a3*(cos(theta2)*cos(theta3)-sin(theta2)*sin(theta3));
        J_anal(2,3) = -0.5*d6*(cos(theta4+theta3+theta2)*sin(theta5)-sin(theta4+theta3+theta2)*cos(theta5))+
                0.5*d6*(-cos(theta4+theta3+theta2)*sin(theta5)-sin(theta4+theta3+theta2)*cos(theta5))+d5*sin(theta4+theta3+theta2);
        J_anal(2,4) = 0.5*d6*(-cos(theta4+theta3+theta2)*sin(theta5)-sin(theta4+theta3+theta2)*cos(theta5))-
                0.5*d6*(sin(theta4+theta3+theta2)*cos(theta5)-cos(theta4+theta3+theta2)*sin(theta5));
        J_anal(2,5) = 0;

        std::cout<<"Analytical Jacobian J_v is:\n"<<J_anal<<std::endl;

        /***** Manipulation ellipsoid *****/
        /*
        //eigenvalues and eigenvectors
        Eigen::EigenSolver<Eigen::MatrixXd> es;

        Eigen::Matrix3d JJ;
        JJ = J_anal*J_anal.transpose();
        
        es.compute(J_anal*J_anal.transpose(), true);
        //std::cout<<"The eigenvalues of Jacobian are: \n"<<es.eigenvalues().transpose()<<std::endl;
        //std::cout<<"\n";
        //std::cout<<"The eigenvectors are: \n"<<es.eigenvectors()<<std::endl;
        //std::cout<<"**********************"<<std::endl;
        
        //Manipulation measure
        double w;
        w = sqrt(JJ.determinant());
        std::cout<<"Manipulation measure is: "<<w<<std::endl;

        */


    }

    void urJacobian_from_tf(){
        /***** Initialise variables *****/

        Eigen::Matrix4d T_E;

        Eigen::Vector3d ez_00, ez_01, ez_02, ez_03, ez_04, ez_05;
        Eigen::Vector3d r_0E, r_1E, r_2E, r_3E, r_4E, r_5E;
        const Eigen::Vector4d r(0.0, 0.0, 0.0, 1.0);

        /***** Receive transformation matrices from tf *****/
        Transform transform;
        TransformVector transform_vector;
        Frames frames;
        
        for(int i=0; i<6; i++){
            transform_vector[i] = tf_listener(frames[i], frames[i+1]);
            transform[i] << transform_vector[i].at(0), transform_vector[i].at(1), transform_vector[i].at(2), transform_vector[i].at(3),
                            transform_vector[i].at(4), transform_vector[i].at(5), transform_vector[i].at(6), transform_vector[i].at(7),
                            transform_vector[i].at(8), transform_vector[i].at(9), transform_vector[i].at(10), transform_vector[i].at(11),
                            transform_vector[i].at(12), transform_vector[i].at(13), transform_vector[i].at(14), transform_vector[i].at(15);
            //std::cout<<"Transformation "<<i+1<<" is \n"<<transform[i]<<std::endl;
        }

        T_E = transform.T1*transform.T2*transform.T3*transform.T4*transform.T5*transform.T6;
        std::cout << "Transformation matrix of Endeffector=\n" <<T_E<<std::endl;
        std::cout <<"Possible Endeffector transformation=\n" <<transform.T6<<std::endl;

        std::vector<double> ee;
        std::string f1 = "robot1_tf/base_link_ur5";
        std::string f2 = "robot1_tf/ee_link_ur5";
        ee = tf_listener(f1, f2);

        std::cout<<"Endeffector is at: "<<std::endl;
        for(int m=0; m<ee.size(); m++)
            std::cout<<ee.at(m)<<std::endl;

        /***** spin axis (z) relative to ur5_base *****/
        /*** Calculation analogous to Robotik Skript1 (SS19, p.304)
        */
        ez_00 << 0.0, 0.0, 1.0;
        ez_01 = transform.T1.block(0,0,3,3)*ez_00;
        ez_02 = (transform.T1*transform.T2).block(0,0,3,3)*ez_00;
        ez_03 = (transform.T1*transform.T2*transform.T3).block(0,0,3,3)*ez_00;
        ez_04 = (transform.T1*transform.T2*transform.T3*transform.T4).block(0,0,3,3)*ez_00;
        ez_05 = (transform.T1*transform.T2*transform.T3*transform.T4*transform.T5).block(0,0,3,3)*ez_00;

        /***** homogenous directional vectors *****/
        r_0E = (T_E * r - r).head(3);
        r_1E = (T_E * r - transform.T1*r).head(3);
        r_2E = (T_E * r - (transform.T1*transform.T2)*r).head(3);
        r_3E = (T_E * r - (transform.T1*transform.T2*transform.T3)*r).head(3);
        r_4E = (T_E * r - (transform.T1*transform.T2*transform.T3*transform.T4)*r).head(3);
        r_5E = (T_E * r - (transform.T1*transform.T2*transform.T3*transform.T4*transform.T5)*r).head(3);

        /***** Jacobian matrix *****/
        Eigen::MatrixXd J_ur(6,6);
        J_ur << ez_00.cross(r_0E), ez_01.cross(r_1E), ez_02.cross(r_2E),
                ez_03.cross(r_3E), ez_04.cross(r_4E), ez_05.cross(r_5E),
                ez_00, ez_01, ez_02, ez_03, ez_04, ez_05;
        //std::cout <<"Jacobian matrix of UR=\n"<<J_ur<<std::endl;

        std::cout<<"********************************************"<<std::endl;
        


    }


    void urJacobian(){
        /***** Initialise variables *****/
        Eigen::MatrixXd T1(4,4), T2(4,4), T3(4,4), T4(4,4), T5(4,4), T6(4,4);
        //-> Using Eigen:MatrixXd <double, XxX> see: http://eigen.tuxfamily.org/dox/GettingStarted.html 
        //(2) wie in Skript S.51 verfahren
        Eigen::MatrixXd T_E(4,4);
        //Eigen::VectorXd endeff_new(6);
        //Eigen::Matrix3d endeff_rotation;

        Eigen::Vector3d ez_00, ez_01, ez_02, ez_03, ez_04, ez_05;
        Eigen::Vector3d r_0E, r_1E, r_2E, r_3E, r_4E, r_5E;
        Eigen::Vector4d r(0.0, 0.0, 0.0, 1.0);


        /***** Calculation of Transformation matrices *****/

        /*** OLD **
        T1 << cos(theta1), -sin(theta1), 0, a[0],
            cos(alpha1)*sin(theta1), cos(alpha1)*cos(theta1), -sin(alpha1), -sin(alpha1)*d[0],
            sin(alpha1)*sin(theta1), sin(alpha1)*cos(theta1), cos(alpha1), cos(alpha1)*d[0],
            0, 0, 0, 1;

        T2 << cos(theta2), -sin(theta2), 0, a[1],
            cos(alpha2)*sin(theta2), cos(alpha2)*cos(theta2), -sin(alpha2), -sin(alpha2)*d[1],
            sin(alpha2)*sin(theta2), sin(alpha2)*cos(theta2), cos(alpha2), cos(alpha2)*d[1],
            0, 0, 0, 1;

        T3 << cos(theta3), -sin(theta3), 0, a[2],
            cos(alpha3)*sin(theta3), cos(alpha3)*cos(theta3), -sin(alpha3), -sin(alpha3)*d[2],
            sin(alpha3)*sin(theta3), sin(alpha3)*cos(theta3), cos(alpha3), cos(alpha3)*d[2],
            0, 0, 0, 1;
        
        T4 << cos(theta4), -sin(theta4), 0, a[3],
            cos(alpha4)*sin(theta4), cos(alpha4)*cos(theta4), -sin(alpha4), -sin(alpha4)*d[3],
            sin(alpha4)*sin(theta4), sin(alpha4)*cos(theta4), cos(alpha4), cos(alpha2)*d[3],
            0, 0, 0, 1;

        T5 << cos(theta5), -sin(theta5), 0, a[4],
            cos(alpha5)*sin(theta5), cos(alpha5)*cos(theta5), -sin(alpha5), -sin(alpha5)*d[4],
            sin(alpha5)*sin(theta5), sin(alpha5)*cos(theta5), cos(alpha5), cos(alpha2)*d[4],
            0, 0, 0, 1;

        T6 << cos(theta6), -sin(theta6), 0, a[5],
            cos(alpha6)*sin(theta6), cos(alpha6)*cos(theta6), -sin(alpha6), -sin(alpha6)*d[5],
            sin(alpha6)*sin(theta6), sin(alpha6)*cos(theta6), cos(alpha6), cos(alpha6)*d[5],
            0, 0, 0, 1;
        */

        /*
        T1 <<   cos(theta1), -sin(theta1)*cos(alpha1), sin(theta1)*sin(alpha1), a[0]*cos(theta1),
                sin(theta1), cos(theta1)*cos(alpha1), -cos(theta1)*sin(alpha1), a[0]*sin(theta1),
                0, sin(alpha1), cos(alpha1), d[0],
                0, 0, 0, 1;
        T2 <<   cos(theta2), -sin(theta2)*cos(alpha2), sin(theta2)*sin(alpha2), a[1]*cos(theta2),
                sin(theta2), cos(theta2)*cos(alpha2), -cos(theta2)*sin(alpha2), a[1]*sin(theta2),
                0, sin(alpha2), cos(alpha2), d[1],
                0, 0, 0, 1;
        T3 <<   cos(theta3), -sin(theta3)*cos(alpha3), sin(theta3)*sin(alpha3), a[2]*cos(theta3),
                sin(theta3), cos(theta3)*cos(alpha3), -cos(theta3)*sin(alpha3), a[2]*sin(theta3),
                0, sin(alpha3), cos(alpha3), d[2],
                0, 0, 0, 1;
        T4 <<   cos(theta4), -sin(theta4)*cos(alpha4), sin(theta4)*sin(alpha4), a[3]*cos(theta4),
                sin(theta4), cos(theta4)*cos(alpha4), -cos(theta4)*sin(alpha4), a[3]*sin(theta4),
                0, sin(alpha4), cos(alpha4), d[3],
                0, 0, 0, 1;
        T5 <<   cos(theta5), -sin(theta5)*cos(alpha5), sin(theta5)*sin(alpha5), a[4]*cos(theta5),
                sin(theta5), cos(theta5)*cos(alpha5), -cos(theta5)*sin(alpha5), a[4]*sin(theta5),
                0, sin(alpha5), cos(alpha5), d[4],
                0, 0, 0, 1;
        T6 <<   cos(theta6), -sin(theta6)*cos(alpha6), sin(theta6)*sin(alpha6), a[5]*cos(theta6),
                sin(theta6), cos(theta6)*cos(alpha6), -cos(theta6)*sin(alpha6), a[5]*sin(theta6),
                0, sin(alpha6), cos(alpha6), d[5],
                0, 0, 0, 1;
        std::cout <<"(0)_T_1 =\n "<< T1 <<std::endl;
        std::cout <<"(1)_T_2 =\n "<< T2 <<std::endl;
        std::cout <<"(2)_T_3 =\n "<< T3 <<std::endl;
        std::cout <<"(3)_T_4 =\n "<< T4 <<std::endl;
        std::cout <<"(4)_T_5 =\n "<< T5 <<std::endl;
        std::cout <<"(5)_T_E =\n "<< T6 <<std::endl;

        */

        //Simplificatrion
        T1 << cos(theta1), 0, sin(theta1), 0,
                sin(theta1), 0, -cos(theta1), 0,
                0, 1, 0, d1,
                0, 0, 0, 1;
        T2 << cos(theta2), -sin(theta2), 0, a2*cos(theta2),
                sin(theta2), cos(theta2), 0, a2*sin(theta2),
                0, 0, 1, 0,
                0, 0, 0, 1;
        T3 << cos(theta3), -sin(theta3), 0, a3*cos(theta3),
                sin(theta3), cos(theta3), 0, a3*sin(theta3),
                0, 0, 1, 0,
                0, 0, 0, 1;
        T4 << cos(theta4), 0, sin(theta4), 0,
                sin(theta4), 0, -cos(theta4), 0,
                0, 1, 0, d4,
                0, 0, 0, 1;
        T5 << cos(theta5), 0, -sin(theta5), 0,
                sin(theta5), 0, cos(theta5), 0,
                0, -1, 0, d5,
                0, 0, 0, 1;
        T6 << cos(theta6), -sin(theta6), 0, 0,
                sin(theta6), cos(theta6), 0, 0,
                0, 0, 1, d6,
                0, 0, 0, 1; 
        /*
        std::cout <<"(0)_T_1 =\n "<< T1 <<std::endl;
        std::cout <<"(1)_T_2 =\n "<< T2 <<std::endl;
        std::cout <<"(2)_T_3 =\n "<< T3 <<std::endl;
        std::cout <<"(3)_T_4 =\n "<< T4 <<std::endl;
        std::cout <<"(4)_T_5 =\n "<< T5 <<std::endl;
        std::cout <<"(5)_T_E =\n "<< T6 <<std::endl;
        */
        //std::cout << "Homogenous transformation of Endeffector=\n" <<T1*T2*T3*T4*T5*T6 <<std::endl <<std::endl;

        //Endeffector transformation
        T_E = T1*T2*T3*T4*T5*T6;
        std::cout << "Homogenous Transformation matrix of Endeffector=\n" <<T_E<<std::endl;
        //std::cout << "******************************"<<std::endl;

        /***** spin axis (z) relative to ur5_base *****/
        ez_00 << 0.0, 0.0, 1.0;
        ez_01 = T1.block(0,0,3,3)*ez_00;
        ez_02 = (T1*T2).block(0,0,3,3)*ez_00;
        ez_03 = (T1*T2*T3).block(0,0,3,3)*ez_00;
        ez_04 = (T1*T2*T3*T4).block(0,0,3,3)*ez_00;
        ez_05 = (T1*T2*T3*T4*T5).block(0,0,3,3)*ez_00;
        /*
        std::cout<<"ez_00:\n"<<ez_00<<std::endl;
        std::cout<<"ez_01:\n"<<ez_01<<std::endl;
        std::cout<<"ez_02:\n"<<ez_02<<std::endl;
        std::cout<<"ez_03:\n"<<ez_03<<std::endl;
        std::cout<<"ez_04:\n"<<ez_04<<std::endl;
        std::cout<<"ez_05:\n"<<ez_05<<std::endl;
        */
        /***** homogenous directional vectors *****/
        r_0E = (T_E * r - r).head(3);
        r_1E = (T_E * r - T1*r).head(3);
        r_2E = (T_E * r - (T1*T2)*r).head(3);
        r_3E = (T_E * r - (T1*T2*T3)*r).head(3);
        r_4E = (T_E * r - (T1*T2*T3*T4)*r).head(3);
        r_5E = (T_E * r - (T1*T2*T3*T4*T5)*r).head(3);

        /***** Jacobian matrix *****/
        Eigen::MatrixXd J_ur(6,6);
        J_ur << ez_00.cross(r_0E), ez_01.cross(r_1E), ez_02.cross(r_2E),
                ez_03.cross(r_3E), ez_04.cross(r_4E), ez_05.cross(r_5E),
                ez_00, ez_01, ez_02, ez_03, ez_04, ez_05;
        std::cout <<"Jacobian matrix of UR=\n"<<J_ur<<std::endl;

        
        /***** Manipulation ellipsoid *****/
        /*
        //eigenvalues and eigenvectors
        Eigen::EigenSolver<Eigen::MatrixXd> es;
        
        Eigen::MatrixXd J_v(3,6);
        J_v = J_ur.block(0,0,3,6);

        //J_v = J_ur.block<3,6>(3,0) = m; //sets the last three rows to zero
        //std::cout<<"J_v: \n"<<J_v<<std::endl;

        Eigen::Matrix3d JJ;
        JJ = J_v*J_v.transpose();
        
        
        es.compute(J_v*J_v.transpose(), true);
        Eigen::VectorXd eigen_values = es.eigenvalues().real(); //just the real part
        Eigen::MatrixXd eigen_vectors = es.eigenvectors().real(); //just the real part
        std::cout<<"The eigenvalues of Jacobian are: \n"<<eigen_values<<std::endl;
        std::cout<<"\n";
        //std::cout<<"The eigenvectors are: \n"<<eigen_vectors<<std::endl;
        //std::cout<<"**********************"<<std::endl;
        
        
        
        //Manipulation measure
        double w;
        w = sqrt(JJ.determinant());
        //std::cout<<"Manipulation measure is: "<<w<<std::endl;

        /***** Endeffector pose ******/
        /*
        endeff_rotation = T_E.block(0,0,3,3); //(row, column)
        //std::cout<<"Rotation matrix (0)_R_E=\n"<<endeff_rotation<<std::endl;
        double alpha = atan2(-endeff_rotation(1,2),endeff_rotation(2,2));
        double gamma = atan2(-endeff_rotation(0,1),endeff_rotation(0,0));
        double beta = atan2(endeff_rotation(0,2),((endeff_rotation(0,0)*cos(gamma))-(endeff_rotation(0,1)*sin(gamma))));

        //double gamma = atan(endeff_rotation(1,0)/endeff_rotation(0,0));
        //double beta = atan(-endeff_rotation(2,0)/((endeff_rotation(0,0)*cos(gamma)+endeff_rotation(1,0)*sin(gamma))));
        //double alpha = atan((endeff_rotation(0,2)*sin(gamma)-endeff_rotation(1,2)*cos(gamma))/(endeff_rotation(1,1)*cos(gamma)-endeff_rotation(0,1)*sin(gamma)));

        endeff_new << T_E(0,3),  T_E(1,3), T_E(2,3),
                        alpha,
                        beta, 
                        gamma;
        std::cout<<"Endeffector pose is\n"<<endeff_new<<std::endl;

        */

        /***** Uncomment for calculating compliant pose *****
        delta_x(J_ur);
        */

        /*
        Eigen::VectorXd q_inv(6);
        for(int i = 0; i<5; i++){
                q_inv = J_ur.inverse()*T_E.col(3);
                std::cout<<"Calculation"<<i<<"\n"<<q_inv<<std::endl;
        }
        */

        /*
        Eigen::VectorXd joint_vector;
        joint_vector << theta1, theta2, theta3, theta4, theta5, theta6;
        endeff_new = J_ur*joint_vector;
        std::cout <<"Current Endeffector pose is \n"<< endeff_new <<std::endl;#
        */

    }

    void delta_x(Eigen::MatrixXd J_ur){
        Eigen::VectorXd x_0(6), deltaX(6), x_d(6);
        Eigen::VectorXd x_current(6);
        Eigen::MatrixXd K(6,6);
        K.setZero();

        std::cout<<"Test whether J_ur was inherited: \n"<<J_ur<<std::endl;

        for(unsigned i=0; i<6; ++i){
                K(i,i) = stiffness_list[i];
        }
        std::cout<<"Still here!"<<std::endl;

        x_current << current_pose[0], current_pose[1], current_pose[2], current_pose[3], current_pose[4], current_pose[5];
        std::cout<<"Current pose \n"<<x_current<<std::endl;

        Eigen::VectorXd F(6);
        F << wrench[0], wrench[1], wrench[2], wrench[3], wrench[4], wrench[5];
        std::cout<<"Current wrench \n"<<F<<std::endl;
        x_0 << initial_pose[0], initial_pose[1], initial_pose[2], initial_pose[3], initial_pose[4], initial_pose[5];

        std::cout<<"Initial pose \n"<<x_0<<std::endl;
        //Calculate direction of force attack to get define desired position x_d
        // 1 N -> means a displacement of 0.05 m
        x_d = F*weigh_factor + x_0; //6x1
        std::cout<<"desired pose is \n"<<x_d<<std::endl;
        
        deltaX = J_ur*K.inverse()*J_ur.transpose()*(F-K*(x_current-x_d));
        std::cout<<"Displacement x=\n"<<deltaX<<std::endl;
    }
    

    void lookupInitialPosition(){
        //get endeffector pose from pose_server.cpp
        ros::service::waitForService("/request_endeffector/pose");

        endeffector_pose_client = nh_.serviceClient<my_mur_msgs::PoseRequest>("/request_endeffector/pose"); //from pose_server.cpp
        my_mur_msgs::PoseRequest pose_msg;
        pose_msg.request.request = true;

        
        try{
            endeffector_pose_client.call(pose_msg);
            initial_pose.push_back(pose_msg.response.position.x);
            initial_pose.push_back(pose_msg.response.position.y);
            initial_pose.push_back(pose_msg.response.position.z);
            initial_pose.push_back(pose_msg.response.rpy_orientation.x);
            initial_pose.push_back(pose_msg.response.rpy_orientation.y);
            initial_pose.push_back(pose_msg.response.rpy_orientation.z);
            std::cout<<"Initial pose is: "<<initial_pose[0]<<", "<<initial_pose[1]<<", "<<initial_pose[2]<<", "
                <<initial_pose[3]<<", "<<initial_pose[4]<<", "<<initial_pose[5]<<std::endl;

        }    
        catch(ros::Exception &ex)
        {
            ROS_ERROR("Error occured %s", ex.what());
        }     
    }

    void lookupStiffness(){ 
        //get param of stiffness
        //std::cout<<"Stiffness from Parameter server: "<<std::endl;
        
        if( !nh_.getParam("/robot1_ns/virtual_constant_parameter/cartesian_stiffness_diagonal", stiffness_list) )
            ROS_ERROR("Failed to get parameter from server.");
    }

    void callbackWrenchState(const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg){
        wrench.clear();
        wrench.push_back(wrench_msg->wrench.force.x);
        wrench.push_back(wrench_msg->wrench.force.y);
        wrench.push_back(wrench_msg->wrench.force.z);
        wrench.push_back(wrench_msg->wrench.torque.x);
        wrench.push_back(wrench_msg->wrench.torque.y);
        wrench.push_back(wrench_msg->wrench.torque.z);
    }

    void endeffectorPose(){
        
    }


};

int main(int argc, char **argv){
  ros::init(argc, argv, "get_jacobian_node");
  ros::NodeHandle nh;

  ros::Subscriber joint_angles;
  ros::Subscriber sub_force;
  ros::Subscriber sub_pose;
  CalculateJacobians obj;
  ros::Rate r(2);

  //obj.lookupStiffness();
  //obj.lookupInitialPosition();
  
  //sub_force = nh.subscribe("/robot1_ns/ee_force_torque_sensor", 100, &CalculateJacobians::callbackWrenchState, &obj); //from gazebo ft_sensor plugin
  //sub_pose = nh.subscribe("/cartesian/endeffector_pose/data", 100, &CalculateJacobians::callbackCurrentPose,  &obj); //from listen_frames_node
  joint_angles = nh.subscribe("/robot1_ns/joint_states",10, &CalculateJacobians::callbackJointAngles, &obj);

  /*
  while (ros::ok()){
    obj.urJacobian_from_tf();
    r.sleep();
  }
  */

  //}
  
  /*{
        
      //rate.sleep();
  }
  */
  //ros::waitForShutdown();
  ros::spin();
  return 0;
}
