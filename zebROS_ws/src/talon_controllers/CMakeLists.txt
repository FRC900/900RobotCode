cmake_minimum_required(VERSION 3.20)
project(talon_controllers)

include ("../cmake_modules/CMakeOpt.cmake")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  ctre_interfaces
  dynamic_reconfigure
  ddynamic_reconfigure
  ddr_updater
  message_generation
  pluginlib
  realtime_tools
  roscpp
  talon_controller_msgs
)

generate_dynamic_reconfigure_options(cfg/TalonConfig.cfg)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    controller_interface
	  ctre_interfaces
    ddr_updater
	  ddynamic_reconfigure
	  message_runtime
    # realtime_tools
  INCLUDE_DIRS include
  LIBRARIES talon_controller_interface
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(talon_controller_interface STATIC
  src/talon_controller_interface.cpp
  src/talonfxpro_controller_interface.cpp
)
add_dependencies(talon_controller_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(talon_controller_interface
  INTERFACE
  -Wl,--whole-archive ${catkin_LIBRARIES} -Wl,--no-whole-archive # Hack needed to make sure all symbols from .a files are pulled into resulting .o
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/talon_controller.cpp
  src/talonfxpro_controller.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above. This is needed to force
## talon controller messages to be built before the
## talon controller code tries to use them
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  talon_controller_interface
  ${catkin_LIBRARIES}
)

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

install(TARGETS talon_controllers talon_controller_interface orchestra_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
    talon_controllers_plugins.xml
    talonfxpro_controllers_plugins.xml
    orchestra_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
