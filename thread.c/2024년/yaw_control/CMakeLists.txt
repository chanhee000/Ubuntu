cmake_minimum_required(VERSION 3.0.2)
project(yaw_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs

)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES yaw_control
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_yaw_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(yaw_control_node_node src/yaw_control_node.cpp)
add_dependencies(yaw_control_node_node ${yaw_control_node_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(yaw_control_node_node ${catkin_LIBRARIES})
