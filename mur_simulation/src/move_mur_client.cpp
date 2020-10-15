/*
 * File: move_mur_client.cpp
 * Author: Heiko Lenz
 * 
 * Created on 5. April 2020
 * 
 * Action lib driving the MuR-robot via joint_velocity_controller 
 * 
*/

#include <ros/ros.h>

#include <geometry_msgs/Twist.h> //für topic /robot1_ns/mobile_base_controller/cmd_vel
#include <actionlib/client/action_client.h>
#include <actionlib/client/client_helpers.h>
#include <control_msgs/FollowJointTrajectoryAction.h> //für topic /robot1_ns/arm_controller/follow_joint_trajectory/action_topics
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <controller_manager_msgs/SwitchController.h>

#include <string>
#include <map>

#include <tf/transform_datatypes.h>


/* Action file: PlanTrajectoryAction
# Goal
geometry_msgs/Pose target_pose # Target pose that the trajectory should be planned to
int8 movement_type_id # This id is defined by an enumeration that specifies if the movement should be PTP or Cartesian
---
# Result
bool succeeded # Boolean flag if the operation with a specified goal was completed
int32 robot_id # Id that defines which robot sent the result
int32 result_id # Result id that tells the sender what went wrong if something went wrong
---
#Feedback
int32 progress_percentage # Progress percentage from 0 to 100
int32 progress_id # id that describes the current status of the action
*/

typedef actionlib::ClientGoalHandle<control_msgs::FollowJointTrajectoryAction> GoalHandle;
  //typedef um nicht immer die Typdefinition wiederholen zu muessen

class MoveRobotClient {
    protected:
    ros::NodeHandle _nh;
    actionlib::ActionClient<control_msgs::FollowJointTrajectoryAction> _ac;
    ros::ServiceClient robot_switch_client, query_active_controller;
    bool sw = false; //switch initially false as arm_velocity_traj_controller is active
    
    public:
    std::map<int, GoalHandle*> _goalHandles; //um mehrere goals zu senden
    
    
    //Ros waits for ActionServer to connect client to it; it takes some millisecond to connect
    //-> so it may happen that first few goals have been sent but not been processed
    //-> that's why asynchronous spinner has introduced to check more than once whether a connection is
    //active which parallely carry information to send

    void switchController(){

        robot_switch_client = _nh.serviceClient<controller_manager_msgs::SwitchController>("/robot1_ns/controller_manager/switch_controller");
        controller_manager_msgs::SwitchController switch_msg;

        if(sw == false){
            switch_msg.request.start_controllers.clear();
            switch_msg.request.stop_controllers.clear();

            switch_msg.request.stop_controllers = {"arm_cartesian_compliance_controller"};
            switch_msg.request.start_controllers = {"arm_velocity_traj_controller"};
            switch_msg.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
            robot_switch_client.waitForExistence(ros::Duration(5.0));
            ROS_INFO("arm_velocity_traj_controller is active!");
            sw = true; //switch to true
        }
        else{
            switch_msg.request.start_controllers.clear();
            switch_msg.request.stop_controllers.clear();

            switch_msg.request.stop_controllers = {"arm_velocity_traj_controller"};
            switch_msg.request.start_controllers = {"arm_cartesian_compliance_controller"};
            switch_msg.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
            robot_switch_client.waitForExistence(ros::Duration(5.0));
            ROS_INFO("arm_cartesian_compliance_controller is active!");
            sw = false; //switch to false
        }
        
        
        if(robot_switch_client.call(switch_msg)){
            ROS_INFO("Controller has switched.");
        }
        else{
            ROS_WARN_STREAM("Failed to switch controller!");  
        }
    }

    
    MoveRobotClient():
        _ac("/robot1_ns/arm_velocity_traj_controller/follow_joint_trajectory") //specify topic or controller to which to connect
    {
        ROS_INFO("Wait for the action server to start...");
        _ac.waitForActionServerToStart();
        ROS_INFO("Server is now up. We can send goals");
    }

    void onTransition(
        const GoalHandle gh)
    {
        int index = 0;
        std::map<int, GoalHandle*>::iterator it = _goalHandles.begin();
        while (it != _goalHandles.end())
        {
            if (*(it->second) == gh) {
                index = it->first;
                break;
            }
            it++;
        }

        ROS_INFO("%d --- Transition callback", index);
        actionlib::CommState commState = gh.getCommState();

        ROS_INFO("%d: Comm state: %s", index, commState.toString().c_str());

        if (commState.toString() == "ACTIVE") {
            ROS_INFO("%d: Goal just went active", index);
        }
        else if (commState.toString() == "DONE") {
            ROS_INFO("%d: Goal is DONE", index);
            actionlib::TerminalState state = gh.getTerminalState();
            ROS_INFO("Client terminal state: %s", state.toString().c_str());
            control_msgs::FollowJointTrajectoryResultConstPtr result = gh.getResult();
        }
    }

    void onFeedback(const GoalHandle gh,
        const control_msgs::FollowJointTrajectoryFeedbackConstPtr &feedback)
    {
        //ROS_INFO("Current joint_state: %lf", feedback->actual(double));
    }

    GoalHandle sendGoal(std::string frame){
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.header.stamp = ros::Time::now();
        goal.trajectory.header.frame_id = frame;
        goal.trajectory.joint_names.push_back("robot1_tf/shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("robot1_tf/shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("robot1_tf/elbow_joint");
        goal.trajectory.joint_names.push_back("robot1_tf/wrist_1_joint");
        goal.trajectory.joint_names.push_back("robot1_tf/wrist_2_joint");
        goal.trajectory.joint_names.push_back("robot1_tf/wrist_3_joint");

        //due to two waypoints we resize this goal trajectory
        
        
        
        // int ind = 0;
        // goal.trajectory.points[ind].positions.resize(6);
        // goal.trajectory.points[ind].positions[0] = 0.0;
        // goal.trajectory.points[ind].positions[1] = 0.0;
        // goal.trajectory.points[ind].positions[2] = 0.0;
        // goal.trajectory.points[ind].positions[3] = 0.0;
        // goal.trajectory.points[ind].positions[4] = 0.0;
        // goal.trajectory.points[ind].positions[5] = 0.0;
        
        // goal.trajectory.points[ind].velocities.resize(6);
        // for (size_t j = 0; j<6; ++j)
        // {
        //     goal.trajectory.points[ind].velocities[j] = 0.0;
        //     goal.trajectory.points[ind].accelerations[j] = 0.0;       
        // }
        // //specify time: trajectory point reached 1s after starting along the trajectory
        // /goal.trajectory.points[ind].time_from_start = ros::Duration(0.0);

        //trajectory point 2 in rad
        int ind = 1;
        goal.trajectory.points.resize(2);
        goal.trajectory.points[ind].positions.resize(6);
            
        goal.trajectory.points[ind].positions[0] = 0.0;//0.0;
        goal.trajectory.points[ind].positions[1] = -1.95476;//-2.18;
        goal.trajectory.points[ind].positions[2] = -1.13446;//-1.00;
        goal.trajectory.points[ind].positions[3] = -1.63188;//-1.51;
        goal.trajectory.points[ind].positions[4] = 1.57;
        goal.trajectory.points[ind].positions[5] = 0.0;
        

        goal.trajectory.points[ind].velocities.resize(6);
        goal.trajectory.points[ind].accelerations.resize(6);
        for (size_t j = 0; j<6; ++j)
        {
            goal.trajectory.points[ind].velocities[j] = 0.0;
            goal.trajectory.points[ind].accelerations[j] = 0.0;
        }
        goal.trajectory.points[ind].time_from_start = ros::Duration(5.0);

        //we are done; return the goal
        GoalHandle goal_handle = _ac.sendGoal(goal,
            boost::bind(&MoveRobotClient::onTransition, this, _1),
            boost::bind(&MoveRobotClient::onFeedback, this, _1, _2));
        ROS_INFO("Goal has been sent.");
        
        return goal_handle;
        

    }
};

void tf_test(){
    //see API http://docs.ros.org/kinetic/api/tf/html/c++/classtf_1_1Vector3.html#details
    tfScalar t;
    tf::Vector3 result;
    //Interpolation of two vectors
    result = tf::lerp(tf::Vector3(2.0,5.0,1.0),tf::Vector3(3.0,4.0,2.0),t);
    ROS_INFO_STREAM("TF scalar is "<<result.getX()<<", "<<result.getY()<<","<<result.getZ());

    //normalize a vector -> return vector with magnitude to 1 -> length is set to 1
    result = result.normalize();
    ROS_INFO_STREAM("Normalized "<<result.getX()<<", "<<result.getY()<<","<<result.getZ());
}

int main(int argc, char** argv){
    //init the ROS node
    ros::init(argc, argv, "robot_action");

    ros::AsyncSpinner spinner(4); //number of threads
    spinner.start(); //has to be started before client starts, otherwise it would hang forever waiting for server
    //a thread of execution is the smallest sequence of programmed instructions that 
    //can be managed independently by a scheduler 
    //is as part of the operating system
    //It's an application that can do multiple things at once. 
    //For example, if you're tying a document in Word, there's a thread responding to your keyboard, 
    //there's a thread that's checking your spelling, there's one that's checking your grammar, there may be another thread saving a backup of your document in case the program crashes.
    //multithreading is an application that uses more than one thread internally to accomplish its goal.
    
    MoveRobotClient client; //create object client that belongs to class
    client.switchController();

    GoalHandle goal_handle1 = client.sendGoal("robot1_tf/base_footprint"); //oder doch lokales frame? map
    client._goalHandles[1] = &goal_handle1;

    //ros::Duration(1.5).sleep();
    //goal_handle1.cancel();

    ros::Duration(8.0).sleep();
    client.switchController();

    ros::waitForShutdown();
}
