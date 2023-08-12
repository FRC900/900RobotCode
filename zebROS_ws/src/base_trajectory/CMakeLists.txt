cmake_minimum_required(VERSION 3.5)
project(base_trajectory LANGUAGES CXX CUDA)

include ("../cmake_modules/CMakeOpt.cmake")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  angles
  base_trajectory_msgs
  cmake_modules
  costmap_2d
  ddynamic_reconfigure
  joint_trajectory_controller
  nav_msgs
  roscpp
  spline_util
  std_msgs
  field_obj
)

list(FILTER catkin_LIBRARIES EXCLUDE REGEX ".*libjoint_trajectory_controller.so.*")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#Append Gperftools_DIR 
LIST(APPEND CMAKE_MODULE_PATH "/home/ubuntu/2023RobotCode/zebROS_ws/src/cmake_modules")
find_package(Gperftools REQUIRED)

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
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp ddynamic_reconfigure nav_msgs
#  DEPENDS system_lib
)
find_package(CUDA REQUIRED)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

find_library(nlopt_LIBRARY nlopt REQUIRED)
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
   	src/base_trajectory.cpp
	src/cuda_utils.cu
   	src/kinematic_constraints.cpp
	src/obstacle_calculator.cpp
	src/obstacle_cost_impl_cpu.cpp
	src/obstacle_gradient_impl.cpp
	src/obstacle_gradient_impl_cuda.cu
	src/obstacle_gradient_impl_cpu.cpp
	src/obstacle_gradient_impl_threaded.cpp
   	src/opt_params.cpp
   	src/opt_params_list.cpp
   	src/matlab_printer.cpp
	src/sample_trajectory_impl.cpp
	src/sample_trajectory_impl_cpu.cpp
	)

#add_compile_options("$<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_61>")
add_compile_options("$<$<COMPILE_LANGUAGE:CUDA>:-gencode=arch=compute_61,code=sm_61>")
add_compile_options("$<$<COMPILE_LANGUAGE:CUDA>:-gencode=arch=compute_62,code=sm_62>")
add_compile_options("$<$<COMPILE_LANGUAGE:CUDA>:-gencode=arch=compute_72,code=sm_72>")
add_compile_options("$<$<COMPILE_LANGUAGE:CUDA>:-gencode=arch=compute_86,code=sm_86>")
add_compile_options("$<$<COMPILE_LANGUAGE:CUDA>:-gencode=arch=compute_87,code=sm_87>")
add_compile_options("$<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-std=gnu++11>")
add_compile_options("$<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-O3>")
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${GPERFTOOLS_LIBRARIES}
  ${CUDA_nvToolsExt_LIBRARY}
  ${nlopt_LIBRARY}
)

if (0)
add_executable(test_obstacle_cost
	src/cuda_utils.cu
	src/obstacle_cost.cpp
	src/obstacle_cost_impl_cpu.cpp
	src/obstacle_cost_impl_cuda.cu
   	src/test_obstacle_cost.cpp
   	)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(test_obstacle_cost ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(test_obstacle_cost
  ${catkin_LIBRARIES}
  ${GPERFTOOLS_LIBRARIES}
  ${CUDA_nvToolsExt_LIBRARY}
)
add_executable(test_obstacle_gradient
	src/cuda_utils.cu
	src/obstacle_gradient.cpp
	src/obstacle_gradient_impl.cpp
	src/obstacle_gradient_impl_cpu.cpp
	src/obstacle_gradient_impl_cuda.cu
	src/obstacle_gradient_impl_threaded.cpp
   	src/test_obstacle_gradient.cpp
   	)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(test_obstacle_gradient ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(test_obstacle_gradient
  ${catkin_LIBRARIES}
  ${GPERFTOOLS_LIBRARIES}
  ${CUDA_nvToolsExt_LIBRARY}
)

add_executable(test_sample_trajectory
	src/cuda_spline_utils.cu
	src/cuda_utils.cu
	src/sample_trajectory.cpp
	src/sample_trajectory_impl.cpp
	src/sample_trajectory_impl_cpu.cpp
	src/sample_trajectory_impl_cuda.cu
	src/sample_trajectory_impl_hybrid.cu
	src/sample_trajectory_impl_threaded.cpp
   	src/test_sample_trajectory.cpp
   	)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(test_sample_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(test_sample_trajectory
  ${catkin_LIBRARIES}
  ${GPERFTOOLS_LIBRARIES}
  ${CUDA_nvToolsExt_LIBRARY}
)
endif()

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
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

