cmake_minimum_required(VERSION 2.8.3)
project(oreo_base)
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation  
  tf
  nav_msgs
)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   EncoderPair.msg
   VelocityPair.msg
 )

## Generate services in the 'srv' folder
# add_service_files(DIRECTORY srv
#   FILES
#   service1.srv
#   service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
   sensor_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
#v_msgs# INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  SYSTEM
  ${EXTERNAL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(hello_world
#   src/${PROJECT_NAME}/hello_world.cpp
# )

## Declare a cpp executable
# add_executable(hello_world_node src/hello_world_node.cpp)
#add_executable(position_array src/position_array.cpp)
#add_executable(bay_leg_track src/bayesian_leg_track.cpp src/nd.cpp)
add_executable(keyboard_controller src/keyboard_controller.cpp)
#add_executable(motion_controller src/motion_controller.cpp)
add_executable(motor_controller src/motor_controller.cpp)
#add_executable(motor_controller_turtle_teleop src/motor_controller_turtle_teleop.cpp)
#add_executable(odometer src/odometer.cpp)
#add_executable(obstacle_avoidance src/obstacle_avoidance.cpp src/nd.cpp)
add_executable(laser_latency_test src/laser_latency_test.cpp)
add_executable(odometer_odom src/odometer_odom.cpp)
#add_executable(social_force src/social_force.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(position_array oreo_base_generate_messages_cpp)
#add_dependencies(bay_leg_track oreo_base_generate_messages_cpp)
add_dependencies(keyboard_controller oreo_base_generate_messages_cpp)
add_dependencies(motor_controller oreo_base_generate_messages_cpp)
#add_dependencies(motion_controller oreo_base_generate_messages_cpp)
#add_dependencies(motor_controller oreo_base_generate_messages_cpp)
#add_dependencies(odometer oreo_base_generate_messages_cpp)
add_dependencies(laser_latency_test laser_latency_test_generate_messages_cpp)
add_dependencies(odometer_odom oreo_base_generate_messages_cpp)
#add_executable(odometer oreo_base_generate_messages_cpp)
#add_dependencies(social_force oreo_base_generate_messages_cpp)
#add_dependencies(obstacle_avoidance oreo_base_generate_messages_cpp)
## Specify libraries to link a library or executable target against
# target_link_libraries(hello_world_node
#   ${catkin_LIBRARIES}
# )
#target_link_libraries(position_array ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#target_link_libraries(bay_leg_track ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(keyboard_controller curses)
target_link_libraries(keyboard_controller ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#target_link_libraries(motion_controller ${catkin_LIBRARIES})
target_link_libraries(motor_controller ${catkin_LIBRARIES})
#target_link_libraries(motor_controller_turtle_teleop ${catkin_LIBRARIES})
#######target_link_libraries(odometer ${catkin_LIBRARIES})
#target_link_libraries(obstacle_avoidance ${catkin_LIBRARIES} m ${OpenCV_LIBRARIES})
target_link_libraries(laser_latency_test ${catkin_LIBRARIES})
target_link_libraries(odometer_odom ${catkin_LIBRARIES})
#target_link_libraries(social_force ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS hello_world hello_world_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hello_world.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
