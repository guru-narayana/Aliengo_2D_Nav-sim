cmake_minimum_required(VERSION 2.8.3)
project(aliengo_description)

find_package(catkin REQUIRED COMPONENTS
    genmsg
    roscpp
    std_msgs
    tf
)

catkin_package(
    CATKIN_DEPENDS
)

include_directories(
    # include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)
add_executable(odometry src/odometry.cpp)
add_dependencies(odometry nav_msgs_generate_messages_cpp)
target_link_libraries(odometry   ${catkin_LIBRARIES})