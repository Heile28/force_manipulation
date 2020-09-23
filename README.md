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
sudo apt install ros-melodic-cob-gazebo-ros-control
sudo apt-get install ros-melodic-moveit-visual-tools

# checking dependencies
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep
sudo rosdep init
sudo rosdep update
sudo rosdep install --from-paths ./ -i -y --rosdistro melodic

catkin_make

# activate this workspace
source $HOME/catkin_ws/devel/setup.bash
```
## FDCC-Solver
The Cartesian Compliance controller is a merging of impedance, admittance and force control visibly as following:
![Image of FDCC_Control](https://github.com/Heile28/force_manipulation/blob/master/etc/FDCC_model.PNG)

Information about the implemented control loop used for the Forward Dynamics Solver provides [this paper](https://arxiv.org/pdf/1908.06252.pdf).

### Configuration
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
