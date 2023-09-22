# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

cmake_minimum_required(VERSION 3.20)
project(cuda_apriltag_ros LANGUAGES CXX)
include("../cmake_modules/CMakeOpt.cmake")
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

add_message_files(
  FILES
  AprilTagDetection.msg
  AprilTagDetectionArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CUDA_MIN_VERSION "10.2")

execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message( STATUS "Architecture: ${ARCHITECTURE}" )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    image_transport
    message_runtime
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
)
# Eigen
#find_package(Eigen3 REQUIRED)
#find_package(Threads REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})

# CUDA
find_package(CUDA ${CUDA_MIN_VERSION} REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_TOOLKIT_ROOT_DIR}/lib64)

# OpenCV
#find_package( OpenCV REQUIRED )
#include_directories( ${OpenCV_INCLUDE_DIRS} )

# CUAprilTags
include_directories(cuapriltags)
add_library(cuapriltags STATIC IMPORTED)
if( ${ARCHITECTURE} STREQUAL "x86_64" )
  set_property(TARGET cuapriltags PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/cuapriltags/lib_x86_64_cuda_11_8/libcuapriltags.a)
elseif( ${ARCHITECTURE} STREQUAL "aarch64" )
  set_property(TARGET cuapriltags PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/cuapriltags/lib_aarch64_jetpack51/libcuapriltags.a)
endif()

# Catkin
include_directories(${catkin_INCLUDE_DIRS})
#add_library(cuda_continuous_detector src/cuda_continuous_detector.cpp)


#MESSAGE("CWD : ${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(cuda_continuous_detector src/cuda_continuous_detector.cpp)

add_dependencies(cuda_continuous_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cuda_continuous_detector
  ${catkin_LIBRARIES}
  #${CUDA_nvToolsExt_LIBRARY}
  #${OpenCV_LIBS}
  cuapriltags
  ${CUDA_LIBRARIES}
)
MESSAGE("INSTALL : ${CMAKE_DESTINATION}")

install(TARGETS cuda_continuous_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
