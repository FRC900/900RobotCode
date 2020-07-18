cmake_minimum_required(VERSION 2.8.3)
project(talon_controllers)

include ("../cmake_modules/CMakeOpt.cmake")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  message_generation
  realtime_tools
  roscpp
  talon_interface
  talon_controller_msgs
  pluginlib
  hardware_interface
)

generate_dynamic_reconfigure_options(cfg/TalonConfig.cfg)

add_service_files(FILES PidfSlot.srv)
generate_messages(DEPENDENCIES talon_controllers)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    controller_interface
	dynamic_reconfigure
	message_runtime
    realtime_tools
	talon_interface
  INCLUDE_DIRS include
  #LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/talon_controller.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above. This is needed to force
## talon controller messages to be built before the
## talon controller code tries to use them
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(orchestra_controller src/orchestra_controller.cpp)
target_link_libraries(orchestra_controller ${catkin_LIBRARIES})
add_dependencies(orchestra_controller
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN *~ EXCLUDE
  PATTERN *.sw? EXCLUDE
  )

install(TARGETS ${PROJECT_NAME} orchestra_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
    talon_controllers_plugins.xml
    orchestra_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
