# force_manipulation ![alt text](https://img.shields.io/github/issues/Heile28/force_manipulation) ![alt text](https://img.shields.io/github/forks/Heile28/force_manipulation)




This repository can be used to implement a controlling mechanism for driving the mobile robot MuR205 consisting of UR5 manipulator and MiR plattform by external forces.
It is intended for gazebo simulation inside ROS.

## Prerequisites
The following instructions assume that a Catkin workspace has been created. Browse in 
```bash
cd $HOME/catkin_ws/src
```

### Clone the following repositories
* [force_manipulation](https://github.com/Heile28/force_manipulation.git)
* [universal_robot](https://github.com/ros-industrial/universal_robot)
* [MiR200_Sim](https://github.com/matchRos/MiR200_Sim)
* [MiR200-with-UR5](https://github.com/Heile28/MiR200-with-UR5)
* [gazebo_ros_link_attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher)
* [cartesian_controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers)
```bash
git clone https://github.com/Heile28/force_manipulation.git
git clone -b $melodic-devel https://github.com/ros-industrial/universal_robot.git
git clone https://github.com/matchRos/MiR200_Sim.git
git clone https://github.com/Heile28/MiR200-with-UR5.git
git clone -b $melodic-devel https://github.com/pal-robotics/gazebo_ros_link_attacher
git clone https://github.com/matchRos/MiR200_Sim.git
```
into your catkin workspace.

### Dependencies
```bash
cd ~/MiR200_Sim/mir_driver/nodes/
chmod +x rep117_filter.py
cd $HOME/catkin_ws

# Install 
sudo apt-get install ros-melodic-moveit
sudo apt-get install ros-melodic-moveit-visual-tools
sudo apt install ros-melodic-cob-gazebo-ros-control
sudo apt-get install ros-melodic-plotjuggler
sudo apt-get install ros-melodic-joint-trajectory-controller
sudo apt-get install ros-melodic-dwa-local-planner


# checking dependencies
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep
sudo rosdep init (or sudo rm /etc/ros/rosdep/sources.list.d/20-default.list)
sudo rosdep update or sudo apt update + ~ upgrade
sudo rosdep install --from-paths ./ -i -y --rosdistro melodic

catkin_make

# activate this workspace via ~/.bashrc
source $HOME/catkin_ws/devel/setup.bash
```
# Getting Started
## Preamble
### Main Control Loop
The mobile robot is driven by two separated systems of MiR and UR5. As the movement - in observance of non-holomic constraints - is controlled by the well-known diff_drive_controller, the UR5 reacts compliant to any force attack. The main controller connects both systems by interchanging relative motions. The current version reduces the linear velocity to 0.233 m/s in advance whereas the compliance controller absorbs the force intensity.
![Image of Main_Control](https://github.com/Heile28/force_manipulation/blob/master/etc/main_control.png)

### FDCC-Solver
The Cartesian Compliance controller is a merging of impedance, admittance and force control visibly as following:
![Image of FDCC_Control](https://github.com/Heile28/force_manipulation/blob/master/etc/FDCC_model.PNG)

Information about the implemented control loop used for the Forward Dynamics Solver provides [this paper](https://arxiv.org/pdf/1908.06252.pdf).

## Configurations
### Configure the cartesian compliance controller
#### Write a yaml.-file with the following entries saved as 'cartesian_compliance_controller.yaml'
```bash
arm_cartesian_compliance_controller:
  type: "position_controllers/CartesianComplianceController"
  end_effector_link: "$(arg tf_prefix)ee_link_ur5"    # All links below must come before this link
  robot_base_link: "$(arg tf_prefix)base_link_ur5"
  ft_sensor_ref_link: "$(arg tf_prefix)wrist_3_link_ur5"
  compliance_ref_link: "$(arg tf_prefix)ee_link_ur5"
  target_frame_topic: "target_pose" #target_frame
  joints:
    - $(arg tf_prefix)shoulder_pan_joint
    - $(arg tf_prefix)shoulder_lift_joint
    - $(arg tf_prefix)elbow_joint
    - $(arg tf_prefix)wrist_1_joint
    - $(arg tf_prefix)wrist_2_joint
    - $(arg tf_prefix)wrist_3_joint

  stiffness:
      trans_x: 200
      trans_y: 200
      trans_z: 200
      rot_x: 150
      rot_y: 150
      rot_z: 150

  pd_gains:
      trans_x: {p: 0.05}
      trans_y: {p: 0.05}
      trans_z: {p: 0.05}
      rot_x: {p: 0.01}
      rot_y: {p: 0.01}
      rot_z: {p: 0.01}
```
and store it in ~/MiR200-with-UR5/mir_ur5_description/config
#### Get this configuration launched as ROS controller with Gazebo
* Browse into MiR200-with-UR5/mir_ur5/launch/mir_ur5.launch
and include cartesian controller as argument
```bash
<arg name="active_controllers" default="arm_cartesian_compliance_controller"/>
```
* and load controller configuration from YAML file by
```bash
<rosparam file="$(find mir_ur5_description)/config/cartesian_compliance_controller.yaml" command="load" subst_value="true"/>
```
* Spawn the controller (besides joint_state_controller) by
```bash
<!-- spawn controller manager with all active controllers (args are all namespaces from inside controller_configurations files-->
<node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen" ns="/$(arg robot_namespace)"
            args="$(arg active_controllers)">
            
<!-- Robot state publisher that publishes the current state of the robot to tf -->
      <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen"/>
 </node>
```
### Add Gazebo Force Torque Sensor
#### Create a configuration file for the F/T-plugin saved as 'force_sensor.gazebo.xacro'
```bash
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/wiki/xacro"> 
  <xacro:macro name="sensor_force_torque" params="joint_name topic_name">
    <gazebo reference="${joint_name}">
     <provideFeedback>true</provideFeedback>
     <!-- <turnGravityOff>false</turnGravityOff> -->
     <visualize>true</visualize>
    </gazebo>
    <gazebo>
     <plugin name="ft_sensor" filename="libgazebo_ros_ft_sensor.so">
       <alwaysOn>true</alwaysOn>
       <updateRate>100.0</updateRate>
       <topicName>${topic_name}</topicName>
       <jointName>${joint_name}</jointName>
       <!--<gaussianNoise>0.0</gaussianNoise>-->
     </plugin>
    </gazebo>
  </xacro:macro>
</robot>
```
* and store it in ~/MiR200-with-UR5/mir_ur5_desription/urdf/include

* Browse into MiR200-with-UR5/mir_ur5_desription/urdf/mir_ur5.urdf.xacro and add
```bash
<!--add force_torque sensor-->
<xacro:include filename="$(find mir_ur5_description)/urdf/include/force_sensor.gazebo.xacro" />
<xacro:sensor_force_torque joint_name="$(arg tf_prefix)wrist_3_joint" topic_name="ee_force_torque_sensor"/>
```
### Configure UR Kinematics Package
The [universal_robot](https://github.com/ros-industrial/universal_robot) -package provides the repo namely "[ur_kinematics](https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_kinematics)". 
* Adapt its file [ur_kin.cpp](https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_kinematics/src/ur_kin.cpp) by uncommenting line 16-26:
```bash
/*
    #define UR10_PARAMS
    #ifdef UR10_PARAMS
    const double d1 =  0.1273;
    const double a2 = -0.612;
    const double a3 = -0.5723;
    const double d4 =  0.163941;
    const double d5 =  0.1157;
    const double d6 =  0.0922;
    #endif
    
    #define UR5_PARAMS
    #ifdef UR5_PARAMS
*/
```
* and line 34-44:
```bash
/*
    #endif
    
    #define UR3_PARAMS
    #ifdef UR3_PARAMS
    const double d1 =  0.1519;
    const double a2 = -0.24365;
    const double a3 = -0.21325;
    const double d4 =  0.11235;
    const double d5 =  0.08535;
    const double d6 =  0.0819;
    #endif
*/
```
* Again execute
```bash
catkin_make
```
## Start simulation
```bash
roslaunch mir_ur5 system_sim.launch
roslaunch mir_simulation activate_force_manipulation.launch
```
and start force_controllers
```bash
rosrun mur_simulation start_mir_node
rosun mur_simulation start_ur_node
```
### Simulate an force attack
```bash
rosrun mur_simulation apply_force_node
```
### Feature
```bash
sudo apt-get install ros-melodic-rqt-joint-trajectory-controller
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```
