cmake_minimum_required(VERSION 3.20)
project(robot_characterization)

unset(CMAKE_CXX_STANDARD CACHE)
set(CMAKE_CXX_STANDARD 20)
include ("../cmake_modules/CMakeOpt.cmake")
set (ALLWPILIB ../../../allwpilib)
execute_process(COMMAND python3 ${ALLWPILIB}/ntcore/generate_topics.py ${ALLWPILIB}/ntcore RESULT_VARIABLE generateResult)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  talon_state_msgs
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
#generate_messages(
#DEPENDENCIES
#std_msgs  # Or other packages containing msgs
#)

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
#  INCLUDE_DIRS include
#  LIBRARIES robot_characterization
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set (WPI_SYSROOT $ENV{HOME}/wpilib/2022/roborio/arm-frc2022-linux-gnueabi)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${WPI_SYSROOT}/include/wpilib
  ${WPI_SYSROOT}/include
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/robot_characterization.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

# Add network table source code
include_directories(${ALLWPILIB}/ntcore/src/main/native/cpp)
include_directories(${ALLWPILIB}/ntcore/src/main/native/include)
include_directories(${ALLWPILIB}/ntcore/generated/main/native/include)
include_directories(${ALLWPILIB}/wpinet/src/main/native/include)
include_directories(${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/include)
include_directories(${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src)
include_directories(${ALLWPILIB}/wpinet/src/main/native/thirdparty/tcpsockets/include)
include_directories(${ALLWPILIB}/wpiutil/src/main/native/include)
include_directories(${ALLWPILIB}/wpiutil/src/main/native/thirdparty/fmtlib/include)
include_directories(${ALLWPILIB}/wpiutil/src/main/native/thirdparty/json/include)
include_directories(${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/include)
include_directories(${ALLWPILIB}/wpiutil/src/main/native/thirdparty/mpack/include)
include_directories(${ALLWPILIB}/wpiutil/src/main/native/thirdparty/sigslot/include)
file(GLOB WPILIBSRC
    ${ALLWPILIB}/ntcore/src/main/native/cpp/*.cpp
    ${ALLWPILIB}/ntcore/generated/main/native/cpp/*.cpp
    ${ALLWPILIB}/ntcore/src/main/native/cpp/net/*.cpp
    ${ALLWPILIB}/ntcore/src/main/native/cpp/net3/*.cpp
    ${ALLWPILIB}/ntcore/src/main/native/cpp/networktables/*.cpp
    ${ALLWPILIB}/ntcore/src/main/native/cpp/tables/*.cpp
	${ALLWPILIB}/wpinet/src/main/native/cpp/*.cpp
	${ALLWPILIB}/wpinet/src/main/native/cpp/uv/*.cpp
	${ALLWPILIB}/wpinet/src/main/native/linux/*.cpp
	${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/*.cpp)

set (WPILIBSRC
	${WPILIBSRC}
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/async.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/core.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/epoll.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/fs.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/fsevents.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/getaddrinfo.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/linux-core.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/linux-inotify.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/linux-syscalls.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/loop.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/loop-watcher.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/pipe.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/posix-hrtime.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/process.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/proctitle.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/poll.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/signal.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/stream.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/tcp.cpp
	${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/thread.cpp
	${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/tty.cpp
    ${ALLWPILIB}/wpinet/src/main/native/thirdparty/libuv/src/unix/udp.cpp

	${ALLWPILIB}/wpiutil/src/main/native/cpp/fs.cpp
	${ALLWPILIB}/wpiutil/src/main/native/cpp/leb128.cpp
	${ALLWPILIB}/wpiutil/src/main/native/cpp/raw_istream.cpp
	${ALLWPILIB}/wpiutil/src/main/native/cpp/timestamp.cpp
	${ALLWPILIB}/wpiutil/src/main/native/cpp/Base64.cpp
	${ALLWPILIB}/wpiutil/src/main/native/cpp/DataLog.cpp
	${ALLWPILIB}/wpiutil/src/main/native/cpp/future.cpp
	${ALLWPILIB}/wpiutil/src/main/native/cpp/Logger.cpp
	${ALLWPILIB}/wpiutil/src/main/native/cpp/SafeThread.cpp
	${ALLWPILIB}/wpiutil/src/main/native/cpp/sha1.cpp
	${ALLWPILIB}/wpiutil/src/main/native/cpp/Synchronization.cpp

	${ALLWPILIB}/wpiutil/src/main/native/thirdparty/fmtlib/src/format.cpp
	${ALLWPILIB}/wpiutil/src/main/native/thirdparty/json/cpp/json.cpp
    ${ALLWPILIB}/wpiutil/src/main/native/thirdparty/json/cpp/json_parser.cpp
	${ALLWPILIB}/wpiutil/src/main/native/thirdparty/json/cpp/json_pointer.cpp
	${ALLWPILIB}/wpiutil/src/main/native/thirdparty/json/cpp/json_serializer.cpp
	${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/raw_ostream.cpp
	${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/raw_os_ostream.cpp
	${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/ErrorHandling.cpp
	${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/MemAlloc.cpp
	${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/SmallVector.cpp
    ${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/StringExtras.cpp
	${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/StringMap.cpp
	${ALLWPILIB}/wpiutil/src/main/native/thirdparty/mpack/src/mpack.cpp
)

add_library(wpilib_net STATIC ${WPILIBSRC})
add_dependencies(wpilib_net ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(robot_characterization 
	src/robot_characterization.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(robot_characterization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(robot_characterization
  ${catkin_LIBRARIES}
  wpilib_net
)

add_executable(robot_characterization_singlemotor 
	src/robot_characterization_singlemotor.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(robot_characterization_singlemotor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(robot_characterization_singlemotor
  ${catkin_LIBRARIES}
  wpilib_net
)

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

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS robot_characterization
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_characterization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
