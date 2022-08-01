# Unitree Aliengo Navigation Simulation
[![Repository Status](https://img.shields.io/badge/Repository%20Status-Maintained-dark%20green.svg)](https://github.com/guru-narayana/Aliengo_Nav-sim)
[![Author](https://img.shields.io/badge/Author-Nara%20Guru%20Narayanaswamy-blue)](https://www.linkedin.com/in/nara-guru-narayanaswamy-658a811b0/)
[![Latest Release](https://img.shields.io/badge/Latest%20Release-27%20July%202022-yellow.svg)](https://github.com/guru-narayana/Aliengo_Nav-sim/releases/tag/v3.0)
# Introduction
Here are the ROS simulation packages for Unitree AlienGo Quadruped, You can simulate the robot in Gazebo, so you can perform high-level control (Using the Twist message) and the low-level control using the (control the torque, position and angular velocity) of the robot joints.You can also perform navigation using the ROS navigaiton stack using the configuration files built into the following packages.

## Packages:
modified unitree_ros-master packages :  `robots/aliengo_description `, `unitree_controller `,`unitree_gazebo `,`unitree_legged_control `,`unitree_legged_msgs `

Realsense_gazebo: `realsense2_description `,`realsense_gazebo_plugin`

gait and navigation related: `aliengo_gait_controller `, `aliengo_2dnav`

Robot data control related: `keyboard_control `, `depthimage_to_laserscan`

# Dependencies
* [ROS](https://www.ros.org/)
* [Gazebo8](http://gazebosim.org/)
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
## aliengo_2dnav
To use the navigation on aliengo simulation run the following command
```
roslaunch aliengo_2dnav navigation_aliengo.launch
```
This will open the rviz GUI window which you can use to input the goal point for navigation

Make sure you closed the teleop_keyboard node before starting the navigation

# Final Result
## Gait control
![aliengo gait](https://github.com/guru-narayana/Aliengo_Nav-sim/blob/main/data/aliengo_gait.gif)
## Navigation
![aliengo navigation](https://github.com/guru-narayana/Aliengo_Nav-sim/blob/main/data/aliengo_navigation.gif)
