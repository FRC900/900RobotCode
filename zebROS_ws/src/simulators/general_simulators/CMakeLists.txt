cmake_minimum_required(VERSION 3.0.2)
project(general_simulators)

unset(CMAKE_CXX_STANDARD CACHE)
set(CMAKE_CXX_STANDARD 20)
include ("../../cmake_modules/CMakeOpt.cmake")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  ctre_interfaces
  ddynamic_reconfigure
  geometry_msgs
  message_generation
  pluginlib
  roscpp
  rospy
  simulator_interface
  std_srvs
  talon_state_msgs
)

# Can't depend on ros_control_boilerplate because that depends on eigen_conversions which breaks wpilib math

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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
  SwerveDebug.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
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
  talon_state_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES general_simulators
#  CATKIN_DEPENDS actionlib ctre_interfaces pluginlib roscpp rospy simulator_interface talon_state_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set (ALLWPILIB ../../../../allwpilib)
set (WPI_SYSROOT $ENV{HOME}/wpilib/2025/roborio/arm-frc2024-linux-gnueabi)
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${WPI_SYSROOT}/include
	${WPI_SYSROOT}/include/navx
)

# Used to find HALInitializer from wpilib internal repo
include_directories(${ALLWPILIB}/hal/src/main/native/athena)
include_directories(${ALLWPILIB}/hal/src/main/native/include)

# Used to find main frc/*.h, frc2/*.h, hal/*.h and wpi/*.h
include_directories(${ALLWPILIB}/wpilibc/src/main/native/include)
include_directories(${ALLWPILIB}/wpimath/src/main/native/include)
include_directories(${ALLWPILIB}/wpiutil/src/main/native/include)
include_directories(${ALLWPILIB}/wpimath/src/main/native/thirdparty/gcem/include)
include_directories(${ALLWPILIB}/wpiutil/src/main/native/thirdparty/json/include)
include_directories(${ALLWPILIB}/wpiutil/src/main/native/thirdparty/fmtlib/include)
include_directories(${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/include)

include_directories(${ALLWPILIB}/wpimath/src/main/native/thirdparty/eigen/include/)
include_directories(${ALLWPILIB}/wpiutil/src/main/native/thirdparty/nanopb/include/)
include_directories(${ALLWPILIB}/wpimath/src/generated/main/native/cpp/)

# include_directories(/home/ubuntu/900RobotCode/zebROS_ws/devel/include)
# include_directories(../../ros_control_boilerplate/include)

include_directories(${WPI_SYSROOT}/include/wpilib)

add_library(wpilib_simulation_utils STATIC
  ${ALLWPILIB}/wpimath/src/main/native/cpp/system/plant/struct/DCMotorStruct.cpp
  ${ALLWPILIB}/wpilibc/src/main/native/cpp/simulation/FlywheelSim.cpp
  ${ALLWPILIB}/wpilibc/src/main/native/cpp/simulation/SingleJointedArmSim.cpp
)

add_dependencies(wpilib_simulation_utils
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## Declare a C++ library
add_library(general_simulators
  # ${ALLWPILIB}/hal/src/main/native/athena/HALInitializer.cpp
  src/dummy_wpilib_hw.cpp
  ${ALLWPILIB}/hal/src/main/native/sim/HALInitializer.cpp
  ${ALLWPILIB}/wpilibc/src/main/native/cpp/RobotController.cpp
  src/general_simulators.cpp
)

target_link_libraries(general_simulators
  wpilib_simulation_utils
  ${catkin_LIBRARIES}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/general_simulators.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/general_simulators_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS general_simulators
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_general_simulators.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
