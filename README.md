# force_manipulation
https://img.shields.io/github/issues/Heile28/force_manipulation.git
https://img.shields.io/github/forks/Heile28/force_manipulation.git

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
