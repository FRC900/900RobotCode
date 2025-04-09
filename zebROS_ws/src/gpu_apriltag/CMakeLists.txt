cmake_minimum_required(VERSION 3.21)
project(gpu_apriltag)
enable_language(CUDA)

include("../cmake_modules/CMakeOpt.cmake")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast -DNDEBUG -Wno-deprecated-declarations")
add_compile_options(-Wno-deprecated-declarations)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  apriltag_msgs
  cv_bridge
  ddynamic_reconfigure
  field_obj
  image_geometry
  image_transport
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  gpu_apriltag_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr")

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# Use the correct version of CUDA
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
find_package(TensorRT REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_library(LIBNVTOOLSEXT nvToolsExt PATHS ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
# find_package(glog REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)
find_package(GTest REQUIRED)
find_package(absl REQUIRED)
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
#   sensor_msgs#   std_msgs
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
#  LIBRARIES gpu_apriltag
#  CATKIN_DEPENDS cv_bridge ddynamic_reconfigure image_geometry image_transport nodelet roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

# add_definitions(-DTHRUST_IGNORE_CUB_VERSION_CHECK)
set (FRC971 ../../../971-Robot-Code)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ../../../cccl/cub
  ../../../cccl/thrust
  ../../../cccl/libcudacxx/include
  ${CUDA_INCLUDE_DIRS}
  ${TensorRT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${FRC971}
  ${FRC971}/third_party/apriltag
  ${glog_INCLUDE_DIRS}
  ${absl_INCLUDE_DIRS}
)

set_source_files_properties(${FRC971}/frc971/orin/apriltag.cc PROPERTIES LANGUAGE CUDA)
set_source_files_properties(${FRC971}/frc971/orin/apriltag_detect.cc PROPERTIES LANGUAGE CUDA)
set_source_files_properties(${FRC971}/frc971/orin/cuda.cc PROPERTIES LANGUAGE CUDA)
set_source_files_properties(${FRC971}/frc971/orin/labeling_allegretti_2019_BKE.cc PROPERTIES LANGUAGE CUDA)
set_source_files_properties(${FRC971}/frc971/orin/line_fit_filter.cc PROPERTIES LANGUAGE CUDA)
set_source_files_properties(${FRC971}/frc971/orin/threshold.cc PROPERTIES LANGUAGE CUDA)
## Declare a C++ library
add_library(${PROJECT_NAME}
  src/gpu_apriltag.cu
  src/gpu_apriltag_impl.cu
  src/gpu_apriltag_nodelet.cpp
  src/stubs.cpp
  ${FRC971}/frc971/orin/apriltag.cc
  ${FRC971}/frc971/orin/apriltag_detect.cc
  ${FRC971}/frc971/orin/aruco_dict.cpp
  ${FRC971}/frc971/orin/confidence_filter.cu
  ${FRC971}/frc971/orin/cuda.cc
  ${FRC971}/frc971/orin/cuda_event_timing.cc
  ${FRC971}/frc971/orin/cuda_utils.cc
  ${FRC971}/frc971/orin/labeling_allegretti_2019_BKE.cc
  ${FRC971}/frc971/orin/line_fit_filter.cc
  ${FRC971}/frc971/orin/threshold.cc
  ${FRC971}/frc971/orin/engine.cpp
  ${FRC971}/frc971/orin/decoder_engine.cpp
  ${FRC971}/frc971/orin/decoder_preprocess.cu
  ${FRC971}/frc971/orin/decoder_softmax.cu
  ${FRC971}/frc971/orin/grid_prior.cu
  ${FRC971}/frc971/orin/marker_dict.cpp
  ${FRC971}/frc971/orin/stag_decoder.cpp
  ${FRC971}/frc971/orin/stage2_corners.cu
  ${FRC971}/frc971/orin/stage2_keypoint_group.cu
  ${FRC971}/frc971/orin/stage2_keypoint_group_trust.cpp
  ${FRC971}/frc971/orin/stage2_keypoint_trust.cu
  ${FRC971}/frc971/orin/stage2_keypoint.cu
  ${FRC971}/frc971/orin/suppress_and_average_keypoints.cu
  ${FRC971}/frc971/orin/unit_arucotag.cpp
  ${FRC971}/frc971/orin/unit_chessboard_tag.cpp
  ${FRC971}/frc971/orin/unit_tag_template.cpp
  ${FRC971}/frc971/orin/warp_perspective_points.cpp
  ${FRC971}/third_party/apriltag/apriltag.c
  ${FRC971}/third_party/apriltag/apriltag_pose.c
  ${FRC971}/third_party/apriltag/apriltag_quad_thresh.c
  ${FRC971}/third_party/apriltag/tag36h11.c
  ${FRC971}/third_party/apriltag/common/g2d.c
  ${FRC971}/third_party/apriltag/common/homography.c
  ${FRC971}/third_party/apriltag/common/image_u8.c
  ${FRC971}/third_party/apriltag/common/image_u8x3.c
  ${FRC971}/third_party/apriltag/common/matd.c
  ${FRC971}/third_party/apriltag/common/pnm.c
  ${FRC971}/third_party/apriltag/common/time_util.c
  ${FRC971}/third_party/apriltag/common/svd22.c
  ${FRC971}/third_party/apriltag/common/workerpool.c
  ${FRC971}/third_party/apriltag/common/zmaxheap.c
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES} 
  ${TensorRT_LIBRARIES}
  ${LIBNVTOOLSEXT}
  ${glog_LIBRARIES}
  # absl::flags_internal
  /usr/local/lib/libabsl_flags_internal.a
  absl::flags
  absl::log
)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  # Xavier and Orin ... Xavier can probably be disabled before too long
  # ... unless perf is good enough to use it for a camera platform?
  set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_ARCHITECTURES "72;87")
else()
  # RTX 30xx. Use 89 for RTX40xx cards if needed
  set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_ARCHITECTURES "86")
endif()

set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_SEPARABLE_COMPILATION ON)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/gpu_apriltag_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

if(0)
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_test src/gpu_apriltag_test.cu)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_test
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
set_target_properties(${PROJECT_NAME}_test PROPERTIES CUDA_ARCHITECTURES "86;87")
set_property(TARGET ${PROJECT_NAME}_test PROPERTY CUDA_SEPARABLE_COMPILATION ON)
endif()

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gpu_apriltag.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
