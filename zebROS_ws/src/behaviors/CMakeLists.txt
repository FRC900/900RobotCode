cmake_minimum_required(VERSION 3.5.1)
project(behaviors)

include ("../cmake_modules/CMakeOpt.cmake")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  actionlib
  actionlib_msgs
  behavior_actions
  controllers_2019_msgs
  controllers_2020_msgs
  frc_msgs
  field_obj
  nav_msgs
  message_generation
  message_filters
  roscpp
  rospy
  std_msgs
  std_srvs
  talon_state_msgs
  teraranger_array
  tf2_ros
  path_follower
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
#add_service_files(
# FILES
#   Service2.srv
# )

## Generate actions in the 'action' folder
#add_action_files(
#DIRECTORY
#action
#FILES
#Path.action
#Intake.action
#Place.action
#Climb.action
#Elevator.action
#Align.action
#)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   nav_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

# ACTIONLIB STUFF

#find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
#generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
   INCLUDE_DIRS include
   #LIBRARIES behaviors
   CATKIN_DEPENDS actionlib_msgs actionlib behavior_actions path_follower nav_msgs roscpp rospy std_msgs teraranger_array tf2_ros
   DEPENDS 
)

add_executable(auto_node src/auto_node.cpp)
target_link_libraries(auto_node ${catkin_LIBRARIES})
add_dependencies(auto_node ${behaviors_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 2020 actionlib servers

add_executable(indexer_server src/indexer_server.cpp)
target_link_libraries(indexer_server ${catkin_LIBRARIES})
add_dependencies(indexer_server ${behaviors_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(eject_server src/eject_actionlib_server.cpp)
target_link_libraries(eject_server ${catkin_LIBRARIES})
add_dependencies(eject_server ${behaviors_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 2019 actionlib servers

add_executable(intake_cargo_server src/intake_cargo_server.cpp)
target_link_libraries(intake_cargo_server ${catkin_LIBRARIES})
add_dependencies(intake_cargo_server ${behaviors_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(elevator_server src/elevator_server.cpp)
target_link_libraries(elevator_server ${catkin_LIBRARIES})
add_dependencies(elevator_server ${behaviors_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#
add_executable(intake_hatch_panel_server src/intake_hatch_panel_server.cpp)
target_link_libraries(intake_hatch_panel_server ${catkin_LIBRARIES})
add_dependencies(intake_hatch_panel_server ${behaviors_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#
add_executable(climber_server src/climber_server.cpp)
target_link_libraries(climber_server ${catkin_LIBRARIES})
add_dependencies(climber_server ${behaviors_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#
add_executable(outtake_cargo_server src/outtake_cargo_server.cpp)
target_link_libraries(outtake_cargo_server ${catkin_LIBRARIES})
add_dependencies(outtake_cargo_server ${behaviors_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#
add_executable(outtake_hatch_panel_server src/outtake_hatch_panel_server.cpp)
target_link_libraries(outtake_hatch_panel_server ${catkin_LIBRARIES})
add_dependencies(outtake_hatch_panel_server ${behaviors_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(intake_powercell_server src/intake_powercell_server.cpp)
target_link_libraries(intake_powercell_server ${catkin_LIBRARIES})
add_dependencies(intake_powercell_server ${behaviors_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



add_executable(test_actionlib src/test_actionlib.cpp)
target_link_libraries(test_actionlib ${catkin_LIBRARIES})
add_dependencies(test_actionlib ${behaviors_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_executable(dummy src/dummy.cpp)
#target_link_libraries(
#    dummy
#    ${catkin_LIBRARIES}
#)
#add_dependencies(
#	dummy
#    ${behaviors_EXPORTED_TARGETS}
#	${catkin_EXPORTED_TARGETS}
#)
###########
## Build ##
###########

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/behaviors.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(publish_pid_cmd_vel_node src/publish_pid_cmd_vel.cpp)
add_dependencies(publish_pid_cmd_vel_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_pid_cmd_vel_node
  ${catkin_LIBRARIES}
)

add_executable(state_setpoint_publisher_node src/state_setpoint_publisher.cpp)
add_dependencies(state_setpoint_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(state_setpoint_publisher_node
  ${catkin_LIBRARIES}
)

add_executable(align_with_terabee_node src/align_with_terabee.cpp)
add_dependencies(align_with_terabee_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(align_with_terabee_node
  ${catkin_LIBRARIES}
)

add_executable(align_with_camera src/align_with_camera.cpp)
add_dependencies(align_with_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(align_with_camera
  ${catkin_LIBRARIES}
)

add_executable(align_hatch_server src/align_hatch_panels.cpp)
add_dependencies(align_hatch_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(align_hatch_server
  ${catkin_LIBRARIES}
)

add_executable(shooter_actionlib_server src/shooter_actionlib_server.cpp)
add_dependencies(shooter_actionlib_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(shooter_actionlib_server
  ${catkin_LIBRARIES}
)
#add_executable(base_align_server src/base_align_server.cpp)
#add_dependencies(base_align_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
#target_link_libraries(base_align_server
#  ${catkin_LIBRARIES}
#)
# add_executable(align_cargo_cargoship_server src/align_cargo_cargoship.cpp)
# add_dependencies(align_cargo_cargoship_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
# target_link_libraries(align_cargo_cargoship_server
#   ${catkin_LIBRARIES}
# )
# 
# add_executable(align_cargo_rocketship src/align_cargo_rocketship.cpp)
# add_dependencies(align_cargo_rocketship ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
# target_link_libraries(align_cargo_rocketship
#   ${catkin_LIBRARIES}
# )
#add_executable(base_align_server src/base_align_server.cpp)
#add_dependencies(base_align_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
#target_link_libraries(base_align_server
#  ${catkin_LIBRARIES}
#)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above

## Specify libraries to link a library or executable target against

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
install(TARGETS intake_cargo_server outtake_cargo_server elevator_server publish_pid_cmd_vel_node state_setpoint_publisher_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
	config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	PATTERN *~ EXCLUDE
	PATTERN *.sw? EXCLUDE
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_behaviors.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
