cmake_minimum_required(VERSION 3.18)
project(deeptag_ros)
enable_language(CUDA)

include("../cmake_modules/CMakeOpt.cmake")
add_compile_options(-Wno-deprecated-declarations)
add_compile_options(-Wno-unused-parameter)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast -DNDEBUG -Wno-deprecated-declarations")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  apriltag_msgs
  cv_bridge
  ddynamic_reconfigure
  field_obj
  geometry_msgs
  image_geometry
  image_transport
  nodelet
  roscpp
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
# For finding FindTensorRT.cmake
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# Use the correct version of CUDA
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)

# We require CUDA, OpenCV, and TensorRT
find_package(TensorRT REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_library(LIBNVTOOLSEXT nvToolsExt PATHS ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})

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
# generate_messages(
#   DEPENDENCIES
#   apriltag_ros_msgs#   geometry_msgs#   sensor_msgs#   std_msgs
# )

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
#  LIBRARIES deeptag_ros
#  CATKIN_DEPENDS apriltag_ros_msgs cv_bridge geometry_msgs image_geometry image_transport opencv roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TensorRT_INCLUDE_DIRS}
)

add_library(deeptag STATIC
    src/aruco_dict.cpp
    src/confidence_filter.cu
    src/cuda_event_timing.cpp
    src/cuda_ssd_preprocess.cu
    src/cuda_image_tile.cu
    src/cuda_utils.cpp
    src/decoder_engine.cpp
    src/decoder_preprocess.cu
    src/decoder_softmax.cu
    src/deeptag.cpp
    src/deeptag_impls.cpp
    src/detection_engine.cpp
    src/engine.cpp
    src/grid_prior.cu
    src/gpu_image_wrapper.cpp
    src/marker_dict.cpp
    src/pose_estimator.cpp
    src/roi_generator.cpp
    src/softmax.cu
    src/ssd_grid_prior.cu
    src/ssd_tag_keypoint.cu
    src/stag_decoder.cpp
    src/stag_detector.cpp
    src/stage1_grid.cu
    src/stage1_grid_group.cu
    src/stage1_ssd_group.cu
    src/stage2_corners.cu
    src/stage2_keypoint.cu
    src/stage2_keypoint_group.cu
    src/stage2_keypoint_trust.cu
    src/suppress_and_average_keypoints.cu
    src/unit_arucotag.cpp
    src/unit_chessboard_tag.cpp
    src/unit_tag_template.cpp
    src/warp_perspective_points.cpp
)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(tensorrt_api_cpp ${tensorrt_api_cpp_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_include_directories(deeptag PUBLIC ${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${TensorRT_INCLUDE_DIRS})
target_link_libraries(deeptag PUBLIC ${OpenCV_LIBS} ${CUDA_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${TensorRT_LIBRARIES} ${LIBNVTOOLSEXT})
set_target_properties(deeptag PROPERTIES CUDA_ARCHITECTURES "86")
set_property(TARGET deeptag PROPERTY CUDA_SEPARABLE_COMPILATION ON)

add_executable(run_inference_benchmark src/main.cpp)
target_link_libraries(run_inference_benchmark PUBLIC deeptag)
set_target_properties(run_inference_benchmark PROPERTIES CUDA_ARCHITECTURES "86")
set_property(TARGET run_inference_benchmark PROPERTY CUDA_SEPARABLE_COMPILATION ON)

add_executable(decoder_test src/decoder_test.cpp)
target_link_libraries(decoder_test PUBLIC deeptag)
set_target_properties(decoder_test PROPERTIES CUDA_ARCHITECTURES "86")
set_property(TARGET decoder_test PROPERTY CUDA_SEPARABLE_COMPILATION ON)
# add_executable(softmax_test src/softmax_test.cpp)
# target_link_libraries(softmax_test PUBLIC deeptag)
# set_target_properties(softmax_test PROPERTIES CUDA_ARCHITECTURES "86")

# add_executable(fine_grid_test src/fine_grid_test.cpp)
# target_link_libraries(fine_grid_test PUBLIC deeptag)
# set_target_properties(fine_grid_test PROPERTIES CUDA_ARCHITECTURES "86")

## Declare a C++ executable
add_executable(deeptag_node
	src/deeptag_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(deeptag_node
  deeptag
	${catkin_LIBRARIES}
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(deeptag_node
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

add_library(deeptag_nodelet
	src/deeptag_nodelet.cpp
)

target_link_libraries(deeptag_nodelet
  deeptag
	${catkin_LIBRARIES}
)

add_dependencies(deeptag_nodelet
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/deeptag_ros.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/deeptag_ros_node.cpp)

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
install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_deeptag_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
