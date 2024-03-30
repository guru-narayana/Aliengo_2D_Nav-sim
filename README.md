# Unitree Aliengo Navigation Simulation
[![Repository Status](https://img.shields.io/badge/Repository%20Status-Maintained-dark%20green.svg)](https://github.com/guru-narayana/Aliengo_Nav-sim)
[![Author](https://img.shields.io/badge/Author-Nara%20Guru%20Narayanaswamy-blue)](https://www.linkedin.com/in/nara-guru-narayanaswamy-658a811b0/)
[![Latest Release](https://img.shields.io/badge/Latest%20Release-27%20July%202022-yellow.svg)](https://github.com/guru-narayana/Aliengo_Nav-sim/releases/tag/v3.0)
# Introduction
Here are the ROS simulation packages for Unitree AlienGo Quadruped, You can simulate the robot in Gazebo, so you can perform high-level control (Using the Twist message) and the low-level control using the (control the torque, position and angular velocity) of the robot joints.You can also perform navigation using the ROS navigaiton stack using the configuration files built into the following packages.
# Publication
If you use this work in an academic context, please cite the following publication as relevant:

N. G. Narayanaswamy and F. Kanehiro, "Vision-Based Software System for Indoor Quadrupedal Locomotion: Integrated with SLAM, Foothold Planning, and Multimodal Gait," 2024 IEEE/SICE International Symposium on System Integration (SII)
```
@INPROCEEDINGS{10417432,
  author={Narayanaswamy, Nara Guru and Kanehiro, Fumio},
  booktitle={2024 IEEE/SICE International Symposium on System Integration (SII)}, 
  title={Vision-Based Software System for Indoor Quadrupedal Locomotion: Integrated with SLAM, Foothold Planning, and Multimodal Gait}, 
  year={2024},
  volume={},
  number={},
  pages={1330-1335},
  keywords={Simultaneous localization and mapping;Trajectory planning;Computer architecture;Software systems;Stability analysis;Planning;Quadrupedal robots},
  doi={10.1109/SII58957.2024.10417432}}
```
## Packages:
modified unitree_ros-master packages :  `robots/aliengo_description `, `unitree_controller `,`unitree_gazebo `,`unitree_legged_control `,`unitree_legged_msgs `

Realsense_gazebo: `realsense2_description `,`realsense_gazebo_plugin`

gait and navigation related: `aliengo_gait_controller `, `aliengo_2dnav`

Robot data control related: `keyboard_control `, `depthimage_to_laserscan`

SLAM:  `rtab_map_testing_pkg `
# Dependencies
* [ROS](https://www.ros.org/)
* [Gazebo8](http://gazebosim.org/)
* [RTAB_Map](https://github.com/introlab/rtabmap_ros)
* [m-explore](https://github.com/hrnr/m-explore)
* [Navigation](https://github.com/ros-planning/navigation)
* [geometry2](https://github.com/ros/geometry2)
# Build
<!-- If you would like to fully compile the `unitree_ros`, please run the following command to install relative packages. -->

For ROS Melodic:
```
sudo apt-get install ros-melodic-controller-interface  ros-melodic-gazebo-ros-control ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller
```
For ROS Kinetic:
```
sudo apt-get install ros-kinetic-controller-manager ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-joint-state-controller ros-kinetic-effort-controllers ros-kinetic-velocity-controllers ros-kinetic-position-controllers ros-kinetic-robot-controllers ros-kinetic-robot-state-publisher ros-kinetic-gazebo8-ros ros-kinetic-gazebo8-ros-control ros-kinetic-gazebo8-ros-pkgs ros-kinetic-gazebo8-ros-dev
```

Then you can use catkin_make to build:
```
cd ~/catkin_ws
catkin_make
```
Similarly build the remaining dependencies 
# Detail of Packages

## unitree_gazebo:
You can launch the Gazebo simulation of the aliiengo robot with the following command:
```
roslaunch unitree_gazebo aliengo_gazebo.launch
```

## aliengo_gait_controller
After launching the aliengo gazebo simulation, you can start to control the robot:
```
roslaunch aliengo_gait_controller  aliengo_gait.launch
```

And you can use the keyboard to control the robot after launching:
```
rosrun keyboard_control teleop_keyboard.py
```
Use the keyboard keys w,a,s,d to increase the velocity in that perticular direction and use x on the keyboard to stop the robot motion 
## rtab_map_testing_pkg
To launch the RTAB Map localisation node use the command 
```
roslaunch rtab_map_testing_pkg rtab_map.launch 
```
## aliengo_2dnav
To use the navigation on aliengo simulation run the following command
```
roslaunch aliengo_2dnav navigation_aliengo.launch
```
This will open the rviz GUI window which you can use to input the goal point for navigation

Make sure you closed the teleop_keyboard node before starting the navigation
It is also possible to create a map of environment without manually controlling the robot using the m-explore package to use it run the follwing (don't run it together with navigation_aliengo as this launch file launches navigation by itself)
```
roslaunch aliengo_2dnav explore_aliengo.launch 
```
# Final Result
## Gait control
![aliengo gait](https://github.com/guru-narayana/Aliengo_Nav-sim/blob/main/data/aliengo_gait.gif)
## Navigation
![aliengo navigation](https://github.com/guru-narayana/Aliengo_Nav-sim/blob/main/data/aliengo_navigation.gif)
## Exploration
![aliengo explore](https://github.com/guru-narayana/Aliengo_Nav-sim/blob/main/data/aliengo_explore.gif)
