cmake_minimum_required(VERSION 2.8.3)
project(xela_server)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  grid_map_ros
  grid_map_msgs
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  SensPoint.msg
  xSensorData.msg
  xServerMsg.msg
)

add_service_files(
  FILES
  TactileRecord.srv
  XelaSensorX.srv
  XelaSensorY.srv
  XelaSensorZ.srv
  XelaSensorXY.srv
  XelaSensorXYZ.srv
  XelaSensorStream.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs

)
catkin_package(
  CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    geometry_msgs
    message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  grid_map_ploter_xela
  src/grid_map_ploter.cpp
)

target_link_libraries(
  grid_map_ploter_xela
  ${catkin_LIBRARIES}
)

## This makes sure message headers of this package are generated before being used.
add_dependencies(grid_map_ploter_xela xela_server_generate_messages_cpp)


add_executable(
  uskin_messages_converter
  src/uskin_messages_converter.cpp
)

target_link_libraries(
  uskin_messages_converter
  ${catkin_LIBRARIES}
)

## This makes sure message headers of this package are generated before being used.
add_dependencies(uskin_messages_converter xela_server_generate_messages_cpp)


#############
## Install ##
#############

install(PROGRAMS
  scripts/xela_server
  scripts/xela_service
  launch/service.launch
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
