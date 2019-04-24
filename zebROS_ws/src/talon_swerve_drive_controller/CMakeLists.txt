cmake_minimum_required(VERSION 2.8.3)
project(talon_swerve_drive_controller)

include ("../cmake_modules/CMakeOpt.cmake")

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    controller_interface
    std_msgs
	std_srvs
    nav_msgs
    geometry_msgs
    trajectory_msgs
    realtime_tools
    talon_interface
    talon_controllers
    tf
    urdf
    message_generation
)

find_package(Eigen3 REQUIRED)

#add_message_files (
#FILES
#)

add_service_files (
  FILES
  MotionProfile.srv
  SetXY.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
  talon_swerve_drive_controller
)

catkin_package(
  CATKIN_DEPENDS
  controller_interface
  INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}
  DEPENDS
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} 
	src/swerve_drive_controller.cpp 
	src/odometry.cpp 
	src/speed_limiter.cpp
	src/900Math.cpp
	src/SwerveMath.cpp
	src/Swerve.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  talon_swerve_drive_controller_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(zero_wheel_service src/zero_wheel_angle.cpp)
target_link_libraries(zero_wheel_service ${catkin_LIBRARIES})
add_dependencies(zero_wheel_service
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

add_executable(dump_offsets_service src/dump_wheel_offsets.cpp)
target_link_libraries(dump_offsets_service ${catkin_LIBRARIES})
add_dependencies(dump_offsets_service
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES ${PROJECT_NAME}_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

