#!/usr/bin/env bash

cd $HOME/catkin_ws/src
git clone https://github.com/Heile28/force_manipulation.git
git clone -b $melodic-devel https://github.com/ros-industrial/universal_robot.git
git clone https://github.com/matchRos/MiR200_Sim.git
git clone https://github.com/Heile28/MiR200-with-UR5.git
git clone -b $melodic-devel https://github.com/pal-robotics/gazebo_ros_link_attacher

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

# activate this workspace
source $HOME/catkin_ws/devel/setup.bash
