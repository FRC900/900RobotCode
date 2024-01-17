cmake_minimum_required(VERSION 3.20)
project(talon_swerve_drive_controller)

include ("../cmake_modules/CMakeOpt.cmake")

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    controller_interface
    ctre_interfaces
    geometry_msgs
    nav_msgs
    std_msgs
	  std_srvs
    periodic_interval_counter
    realtime_tools
	  roscpp
    talon_controllers
	  talon_state_msgs
    talon_swerve_drive_controller_msgs
    tf
    trajectory_msgs
    urdf
    periodic_interval_counter
)

find_package(Eigen3 REQUIRED)

#add_message_files (
#FILES
#)

#add_service_files (
  #FILES
  #SetXY.srv
#)

#generate_messages(
  #DEPENDENCIES
  #std_msgs
  #geometry_msgs
  #trajectory_msgs
  #talon_swerve_drive_controller
#)

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
  src/swerve_drive_state_controller.cpp
	#src/odometry.cpp 
	#src/speed_limiter.cpp
	src/900Math.cpp
	src/SwerveMath.cpp
	src/Swerve.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
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

# Need to find a better way of testing "is this an x86 Linux build"
if (NOT (("${ARM_PREFIX}" STREQUAL "arm-frc2020-linux-gnueabi") OR
	     ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "arm-linux-gnueabihf") OR
		 ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "aarch64-linux-gnu") )) 
	add_executable(test_filtering src/test_filtering.cpp src/Swerve.cpp src/SwerveMath.cpp src/900Math.cpp)
	target_link_libraries(test_filtering ${catkin_LIBRARIES})
	add_dependencies(test_filtering
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS}
	)

  add_executable(swervebot test/swervebot.cpp src/Swerve.cpp src/SwerveMath.cpp src/900Math.cpp)
	target_link_libraries(swervebot ${catkin_LIBRARIES})
	add_dependencies(swervebot
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS}
	)

  add_executable(swervebot_from_bag test/swervebot_from_bag.cpp src/Swerve.cpp src/SwerveMath.cpp src/900Math.cpp)
  target_link_libraries(swervebot_from_bag ${catkin_LIBRARIES})
  add_dependencies(swervebot_from_bag
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
endif()
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES ${PROJECT_NAME}_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
