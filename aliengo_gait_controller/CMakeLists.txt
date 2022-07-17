cmake_minimum_required(VERSION 3.0.2)
project(aliengo_gait_controller)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  unitree_legged_msgs
  
)


catkin_package(
  CATKIN_DEPENDS 
  unitree_legged_msgs 
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(Aliengo_gait_Control src/gait.cpp src/trajectory_generator.cpp src/Kinematics.cpp)
add_dependencies(Aliengo_gait_Control unitree_legged_msgs_gencpp)

target_link_libraries(Aliengo_gait_Control ${catkin_LIBRARIES})
