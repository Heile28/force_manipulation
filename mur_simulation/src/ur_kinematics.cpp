#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <eigen3/Eigen/Dense>

#include <ur_kinematics/ur_kin.h>
#include <mur_force_controller/mur_base.h>

double* theta = new double[6];
double* T = new double[16];
double* T1 = new double[16];
double* T2 = new double[16];
double* T3 = new double[16];
double* T4 = new double[16];
double* T5 = new double[16];
double* T6 = new double[16];
const std::string names[6]={"robot1_tf/shoulder_pan_joint", "robot1_tf/shoulder_lift_joint", "robot1_tf/elbow_joint",
                             "robot1_tf/wrist_1_joint", "robot1_tf/wrist_2_joint", "robot1_tf/wrist_3_joint"};

void forwardKinematics();
void inverseKinematics();
void manipulationMeasure();
Eigen::MatrixXd urJacobian();

int mapIndizes(std::string name)
{
    for(int i=0;i<6;i++)
    {
        if (names[i]==name)
        return i;
    }    
}

void callbackJointAngles(const sensor_msgs::JointState& joint_msg){
    //sub_force_ = nh_.subscribe("/robot1_ns/ee_force_torque_sensor", 100, &CalculateJacobians::callbackWrenchState, this); //from gazebo ft_sensor plugin
    //sub_pose_ = nh_.subscribe("/cartesian/endeffector_pose/data", 100, &CalculateJacobians::callbackCurrentPose,  this); //from listen_frames_node
    std::string name;
    for(unsigned i = 0; i<6; i++){
        name=joint_msg.name[i];
        theta[mapIndizes(name)] = joint_msg.position[i];
    }
    //std::cout<<"Inside callback!"<<std::endl;

    std::cout<<"Winkel:"<<std::endl;
    std::cout<<"["<<theta[0]<<","<<theta[1]<<","<<theta[2]<<","<<theta[3]<<","<<theta[4]<<","<<theta[5]<<"]"<<std::endl;
    forwardKinematics();
}

void forwardKinematics(){
    ur_kinematics::forward(theta, T);

    for(int i=0; i<6;i++)
        ROS_INFO_STREAM(theta[i]);

    for(int i=0;i<4;i++) {
        for(int j=i*4;j<(i+1)*4;j++)
            printf("%1.3f ", T[j]);
        printf("\n");
    }
    printf("\n");

    /*
    ur_kinematics::forward_all(theta, T1, T2, T3, T4, T5, T6);  
    for(int i=0;i<4;i++) {
        for(int j=i*4;j<(i+1)*4;j++)
            printf("%1.3f ", T1[j]);
        printf("\n");

    }
    printf("\n");
    for(int i=0;i<4;i++) {
        for(int j=i*4;j<(i+1)*4;j++)
            printf("%1.3f ", T2[j]);
        printf("\n");
    }
    printf("\n");
    for(int i=0;i<4;i++) {
        for(int j=i*4;j<(i+1)*4;j++)
            printf("%1.3f ", T3[j]);
        printf("\n");
    }
    printf("\n");
    for(int i=0;i<4;i++) {
        for(int j=i*4;j<(i+1)*4;j++)
            printf("%1.3f ", T4[j]);
        printf("\n");
    }
    printf("\n");
    for(int i=0;i<4;i++) {
        for(int j=i*4;j<(i+1)*4;j++)
            printf("%1.3f ", T5[j]);
        printf("\n");
    }
    printf("\n");
    for(int i=0;i<4;i++) {
        for(int j=i*4;j<(i+1)*4;j++)
            printf("%1.3f ", T6[j]);
        printf("\n");
    }
    printf("\n");
    */

    Eigen::MatrixXd _T1(4,4), _T2(4,4), _T3(4,4), _T4(4,4), _T5(4,4), _T6(4,4);
    int i = 0;
    
    for (int k = 0; k<4; k++){
        for (int l = 0; l<4; l++){
            _T1(k,l) = T1[i];
            _T2(k,l) = T2[i];
            _T3(k,l) = T3[i];
            _T4(k,l) = T4[i];
            _T5(k,l) = T5[i];
            _T6(k,l) = T6[i];
            i++;
        }
    }
    /*
    std::cout<<"_T3: \n"<<_T3<<std::endl;
    std::cout<<"_T4: \n"<<_T4<<std::endl;
    std::cout<<"_T5: \n"<<_T5<<std::endl;
    std::cout<<"_T6: \n"<<_T6<<std::endl;
    */
    /**** Calculate relative transformation matrices ****/
    /*
    Eigen::MatrixXd T1_(4,4), T2_(4,4), T3_(4,4), T4_(4,4), T5_(4,4), T6_(4,4);
    T1_ = _T1;
    //T2_ = _T2.inverse()*_T2;
    T2_ = (_T1.inverse())*_T2;
    T2_ = T2_.inverse();
    std::cout<<"T2_: \n"<<T2_<<std::endl;
    T3_ = _T2.inverse()*_T3;
    T4_ = _T3.inverse()*_T4;
    T5_ = _T4.inverse()*_T5;
    T6_ = _T5.inverse()*_T6;

    /**** Calculate endeffector pose ****/
    /*
    Eigen::MatrixXd T_E(4,4);

    
    //with inverses
    T_E = T1_*T2_*T3_*T4_*T5_*T6_;
    std::cout<<"Endeffector pose is: \n"<<T_E<<std::endl;
    
    */
    /*
    T_E = _T1*_T2*_T3*_T4*_T5*_T6;
    std::cout<<"Endeffector pose is: \n"<<T_E<<std::endl;
    */
    
    /*
    std::cout<<"_T1 pose is: \n"<<_T1<<std::endl;
    std::cout<<"_T2 pose is: \n"<<_T2<<std::endl;
    std::cout<<"_T3 pose is: \n"<<_T3<<std::endl;
    std::cout<<"_T4 pose is: \n"<<_T4<<std::endl;
    std::cout<<"_T5 pose is: \n"<<_T5<<std::endl;
    
    std::cout<<"_T6 pose is: \n"<<_T6<<std::endl;

    T_E = _T6*_T5*_T4*_T3*_T2*_T1;
    std::cout<<"Neue Endeffector pose is: \n"<<T_E<<std::endl;
    */

    /*
    //execute inverse kinematics
    inverseKinematics();
    */

    //manipulationMeasure();
}

void inverseKinematics()
{
    //double* q_sols = new double[8][6];
    double q_sols[8*6];
    int num_sols;
    num_sols = ur_kinematics::inverse(T, q_sols);
    ROS_INFO("Calculated angles: ");
    for(int i=0;i<num_sols;i++)
        printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n", 
                q_sols[i*6+0], q_sols[i*6+1], q_sols[i*6+2], q_sols[i*6+3], q_sols[i*6+4], q_sols[i*6+5]);
    // for(int i=0;i<=4;i++)
    // printf("%f ", PI/2.0*i);
    // printf("\n");
}

Eigen::MatrixXd urJacobian()
{
    MurBase base_;
    Eigen::MatrixXd T_E(4,4);
    Eigen::MatrixXd _T1(4,4), _T2(4,4), _T3(4,4), _T4(4,4), _T5(4,4), _T6(4,4);

    Eigen::Vector3d ez_00, ez_01, ez_02, ez_03, ez_04, ez_05;
    Eigen::Vector3d r_0E, r_1E, r_2E, r_3E, r_4E, r_5E;
    Eigen::Vector4d r(0.0, 0.0, 0.0, 1.0);

    /*
    std::vector<double> theta_;
    theta_ = base_.getAngles();
    */

    /***** Calculation of Transformation matrices *****/

    //Endeffector transformation
    T_E << T[0], T[1], T[2], T[3],
            T[4], T[5], T[6], T[7], 
            T[8], T[9], T[10], T[11];
    std::cout << "Homogenous transformation of Endeffector=\n" <<T_E<<std::endl;

    int i = 0;
    for (int k = 0; k<4; k++){
        for (int l = 0; l<4; l++){
            _T1(k,l) = T1[i];
            _T2(k,l) = T2[i];
            _T3(k,l) = T3[i];
            _T4(k,l) = T4[i];
            _T5(k,l) = T5[i];
            _T6(k,l) = T6[i];
            i++;
        }
    }

    /***** spin axis (z) relative to ur5_base *****/
    ez_00 << 0.0, 0.0, 1.0;
    ez_01 = _T1.block(0,0,3,3)*ez_00;
    ez_02 = (_T1*_T2).block(0,0,3,3)*ez_00;
    ez_03 = (_T1*_T2*_T3).block(0,0,3,3)*ez_00;
    ez_04 = (_T1*_T2*_T3*_T4).block(0,0,3,3)*ez_00;
    ez_05 = (_T1*_T2*_T3*_T4*_T5).block(0,0,3,3)*ez_00;

    /***** homogenous directional vectors *****/
    r_0E = (T_E * r - r).head(3);
    r_1E = (T_E * r - _T1*r).head(3);
    r_2E = (T_E * r - (_T1*_T2)*r).head(3);
    r_3E = (T_E * r - (_T1*_T2*_T3)*r).head(3);
    r_4E = (T_E * r - (_T1*_T2*_T3*_T4)*r).head(3);
    r_5E = (T_E * r - (_T1*_T2*_T3*_T4*_T5)*r).head(3);

    /***** Jacobian matrix *****/
    Eigen::MatrixXd J_ur_(6,6);
    J_ur_ << ez_00.cross(r_0E), ez_01.cross(r_1E), ez_02.cross(r_2E),
            ez_03.cross(r_3E), ez_04.cross(r_4E), ez_05.cross(r_5E),
            ez_00, ez_01, ez_02, ez_03, ez_04, ez_05;
    //std::cout <<"Jacobian matrix of UR=\n"<<J_ur_<<std::endl;

    return J_ur_;
}

void manipulationMeasure()
{
    /***** Manipulation ellipsoid *****/
    //eigenvalues and eigenvectors
    Eigen::EigenSolver<Eigen::MatrixXd> es;
    
    Eigen::MatrixXd J_ur(6,6);
    J_ur = urJacobian();

    Eigen::MatrixXd J_v(3,6);
    J_v = J_ur.block(0,0,3,6);

    Eigen::Matrix3d JJ;
    JJ = J_v*J_v.transpose();
    
    es.compute(J_v*J_v.transpose(), true);
    Eigen::VectorXd eigen_values = es.eigenvalues().real(); //just the real part
    Eigen::MatrixXd eigen_vectors = es.eigenvectors().real(); //just the real part
    //std::cout<<"The eigenvalues of Jacobian are: \n"<<eigen_values<<std::endl;
    std::cout<<"\n";
    
    //Manipulation measure
    double w = sqrt(JJ.determinant());
    //std::cout<<"Manipulation measure is: "<<w<<std::endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ur_kinematics_solver");

    // Create a node handle
    ros::NodeHandle nh;


    //Subscribe joint values
    ros::Subscriber joint_angles;
    joint_angles = nh.subscribe("/robot1_ns/joint_states",10 ,callbackJointAngles);

    ros::spin();
}
