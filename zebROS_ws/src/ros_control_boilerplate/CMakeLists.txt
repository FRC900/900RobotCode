cmake_minimum_required(VERSION 3.0.2)
project(ros_control_boilerplate)
if(NOT CMAKE_BUILD_TYPE AND DEFINED CMAKE_TOOLCHAIN_FILE)  # Cross-build for Rio
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

include ("../cmake_modules/CMakeOpt.cmake")

find_package(catkin REQUIRED COMPONENTS
  as726x_interface
  cmake_modules
  controller_manager
  control_msgs
  frc_msgs
  frc_interfaces
  hardware_interface
  message_generation
  realtime_tools
  remote_joint_interface
  roscpp
  rosparam_shortcuts
  sensor_msgs
  std_msgs
  talon_interface
  trajectory_msgs
  urdf
)

add_service_files (
  FILES
  set_limit_switch.srv
  LineBreakSensors.srv
  DSError.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  ros_control_boilerplate
)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
	as726x_interface
	control_msgs
	frc_msgs
	frc_interfaces
	hardware_interface
	roscpp
	rosparam_shortcuts
	sensor_msgs
	std_msgs
	talon_interface
	trajectory_msgs
	message_runtime
  LIBRARIES
)

###########
## Build ##
###########

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

add_library(frc_robot_interface STATIC
	src/AS726x.cpp
	src/frc_robot_interface.cpp
	src/generic_hw_control_loop.cpp
	src/ros_iterative_robot.cpp
)
target_link_libraries(frc_robot_interface 
	${catkin_LIBRARIES}
)

set (ALLWPILIB ../../../allwpilib)

# Don't build sim code for the Rio or Jetson -
# saves time and space for something which will
# never be run
if (NOT (("${ARM_PREFIX}" STREQUAL "arm-frc2020-linux-gnueabi") OR
	     ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "arm-linux-gnueabihf") OR
		 ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "aarch64-linux-gnu") )) 
    # Sim version of main control
    # executable.  For now this just prints debugging
    # info for testing code which doens't actually hit
    # RoboRIO or CAN hardware. Eventually expand into
    # controlling some sort of sim?

	add_library(frc_sim_wpilib STATIC
		${ALLWPILIB}/hal/src/main/native/cpp/cpp/fpga_clock.cpp
		${ALLWPILIB}/hal/src/main/native/cpp/handles/HandlesInternal.cpp
		${ALLWPILIB}/hal/src/main/native/cpp/Main.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Accelerometer.cpp
		${ALLWPILIB}/hal/src/main/native/sim/AddressableLED.cpp
		${ALLWPILIB}/hal/src/main/native/sim/AnalogAccumulator.cpp
		${ALLWPILIB}/hal/src/main/native/sim/AnalogGyro.cpp
		${ALLWPILIB}/hal/src/main/native/sim/AnalogInput.cpp
		${ALLWPILIB}/hal/src/main/native/sim/AnalogInternal.cpp
		${ALLWPILIB}/hal/src/main/native/sim/AnalogOutput.cpp
		${ALLWPILIB}/hal/src/main/native/sim/AnalogTrigger.cpp
		${ALLWPILIB}/hal/src/main/native/sim/CANAPI.cpp
		${ALLWPILIB}/hal/src/main/native/sim/CAN.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Compressor.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Constants.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Counter.cpp
		${ALLWPILIB}/hal/src/main/native/sim/DigitalInternal.cpp
		${ALLWPILIB}/hal/src/main/native/sim/DIO.cpp
		${ALLWPILIB}/hal/src/main/native/sim/DriverStation.cpp
		${ALLWPILIB}/hal/src/main/native/sim/DutyCycle.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Encoder.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Extensions.cpp
		${ALLWPILIB}/hal/src/main/native/sim/HAL.cpp
		${ALLWPILIB}/hal/src/main/native/sim/HALInitializer.cpp
		${ALLWPILIB}/hal/src/main/native/sim/I2C.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Interrupts.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/AccelerometerData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/AddressableLEDData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/AnalogGyroData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/AnalogInData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/AnalogOutData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/AnalogTriggerData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/CanDataInternal.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/DigitalPWMData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/DIOData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/DriverStationData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/DutyCycleData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/EncoderData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/I2CData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/PCMData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/PDPData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/PWMData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/RelayData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/RoboRioData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/SimDeviceData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/SPIAccelerometerData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/SPIData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/MockHooks.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Notifier.cpp
		${ALLWPILIB}/hal/src/main/native/sim/PDP.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Ports.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Power.cpp
		${ALLWPILIB}/hal/src/main/native/sim/PWM.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Relay.cpp
		${ALLWPILIB}/hal/src/main/native/sim/SerialPort.cpp
		${ALLWPILIB}/hal/src/main/native/sim/SimDevice.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Solenoid.cpp
		${ALLWPILIB}/hal/src/main/native/sim/SPI.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Threads.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/AnalogInput.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/DigitalInput.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/DigitalOutput.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/DriverStation.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/ErrorBase.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Error.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/frc2/Timer.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/GenericHID.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/InterruptableSensorBase.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/IterativeRobotBase.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Joystick.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/RobotController.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/RobotState.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/SensorUtil.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Timer.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Utility.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Watchdog.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/Error.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/ErrorHandling.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/Hashing.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/ManagedStatic.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/NativeFormatting.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/Path.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/raw_ostream.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/SmallPtrSet.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/SmallVector.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/StringMap.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/StringRef.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/Twine.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/timestamp.cpp
		${ALLWPILIB}/wpiutil/src/main/native/unix/Demangle.cpp
		${ALLWPILIB}/wpiutil/src/main/native/unix/StackTrace.cpp
		src/dummy_wpilib_sendable.cpp
	)
	add_executable(frcrobot_sim_main 
		src/frcrobot_sim_main.cpp
		src/frcrobot_sim_interface.cpp
		src/dummy_wpilib_common.cpp
	)

	target_link_libraries(frcrobot_sim_main
		${catkin_LIBRARIES}
		frc_robot_interface
		frc_sim_wpilib
	)

    # Forces messages to be generated before
    # the target which uses them
	add_dependencies(frcrobot_sim_main
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS}
	)

	install(TARGETS
	  frcrobot_sim_main
	  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
endif()

add_library(convert STATIC
	src/as726x_convert.cpp
	src/cancoder_convert.cpp
	src/canifier_convert.cpp
	src/talon_convert.cpp
)
# Main control executable
set (FRCROBOT_HW_MAIN_SRCS 
	src/error_queue.cpp
	src/frcrobot_hw_main.cpp
	src/frcrobot_hw_interface.cpp
)

set (WPI_SYSROOT $ENV{HOME}/wpilib/2020/roborio/arm-frc2020-linux-gnueabi)
include_directories(
	${catkin_INCLUDE_DIRS}
	${WPI_SYSROOT}/include/wpilib
	${WPI_SYSROOT}/include
	${WPI_SYSROOT}/include/navx
)

# Base regardless of OS type
set (WPI_BASE_LIB_DIR ${WPI_SYSROOT}/lib/wpilib/linux)

# Rio-only code
if ("${ARM_PREFIX}" STREQUAL "arm-frc2020-linux-gnueabi")
	set (CTRE_LIB_DIR ${WPI_SYSROOT}/lib/ctre/linux/athena/static)

	# Link against wpi user libraries for CAN talon and navX-MXP code and
	# wpilib for HAL, driver station, and so on
	set (WPI_SHARED_LIB_DIR ${WPI_BASE_LIB_DIR}/athena/shared)
	set (PLATFORM_SPECIFIC_LIBS
		${WPI_SHARED_LIB_DIR}/libntcore.so
		${WPI_SHARED_LIB_DIR}/libwpilibc.so
		${WPI_SHARED_LIB_DIR}/libwpiHal.so
		${WPI_SHARED_LIB_DIR}/libwpiutil.so
		${WPI_SHARED_LIB_DIR}/libvisa.so
		${WPI_SHARED_LIB_DIR}/libFRC_NetworkCommunication.so.20.0.0
		${WPI_SHARED_LIB_DIR}/libNiFpga.so.19.0.0
		${WPI_SHARED_LIB_DIR}/libNiFpgaLv.so.19.0.0
		${WPI_SHARED_LIB_DIR}/libRoboRIO_FRC_ChipObject.so.20.0.0
		${WPI_SHARED_LIB_DIR}/libNiRioSrv.so.19.0.0
		${WPI_SHARED_LIB_DIR}/libniriodevenum.so.19.0.0
		${WPI_SHARED_LIB_DIR}/libniriosession.so.18.0.0
		${WPI_SHARED_LIB_DIR}/libni_emb.so.12.0.0
		${WPI_SHARED_LIB_DIR}/libnirio_emb_can.so.16.0.0
		${WPI_SHARED_LIB_DIR}/libni_rtlog.so.2.8.0
		${WPI_SYSROOT}/lib/navx/linux/athena/static/libnavx_frc.a
	)
elseif ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "arm-linux-gnueabihf") # Jetson TK1
	set (CTRE_LIB_DIR ${WPI_SYSROOT}/lib/ctre/linux/raspbian/static)    # Who knows if this will work
	find_library(LIB_GPIOD libgpiod.a)
elseif ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "aarch64-linux-gnu") # Jetson TX1/TX2/Xavier AGX/Xavier NX
	set (CTRE_LIB_DIR ${WPI_SYSROOT}/lib/ctre/linux/jetsontx/static)
	find_library(LIB_GPIOD libgpiod.a)
else() # x86-64, hopefully. First person to try and build native on a Mac gets kicked off the team
	set (CTRE_LIB_DIR ${WPI_SYSROOT}/lib/ctre/linux/x86-64/static)
endif()
  
# All platforms need these, they just live
# in different dirs for different architectures
set (CTRE_USER_LIBS
	${CTRE_LIB_DIR}/libCTRE_Phoenix.a
	${CTRE_LIB_DIR}/libCTRE_PhoenixCCI.a
	${CTRE_LIB_DIR}/libCTRE_PhoenixCore.a
	${CTRE_LIB_DIR}/libCTRE_PhoenixDiagnostics.a
)

# Used to find HALInitializer from wpilib internal repo
include_directories(${ALLWPILIB}/hal/src/main/native/athena)

if (NOT "${ARM_PREFIX}" STREQUAL "arm-frc2020-linux-gnueabi")
    # Add socketcan library to everything but the Rio
    # This adds code to use linux CAN sockets to talk to
    # the CAN bus for the Jetson
    # Also add all of the wpilib code needed to talk to PCM and PDP
	set (CTRE_USER_LIBS 
		${CTRE_USER_LIBS}
		${CTRE_LIB_DIR}/libCTRE_PhoenixCanutils.a
		${CTRE_LIB_DIR}/libCTRE_PhoenixPlatform_socketcan.a
	)

	set (FRCROBOT_HW_MAIN_SRCS ${FRCROBOT_HW_MAIN_SRCS} 
		src/dummy_wpilib_common.cpp
		src/dummy_wpilib_hw.cpp
		${ALLWPILIB}/hal/src/main/native/athena/CANAPI.cpp
		${ALLWPILIB}/hal/src/main/native/athena/Compressor.cpp
		${ALLWPILIB}/hal/src/main/native/athena/HALInitializer.cpp
		${ALLWPILIB}/hal/src/main/native/athena/PCMInternal.cpp
		${ALLWPILIB}/hal/src/main/native/athena/PDP.cpp
		${ALLWPILIB}/hal/src/main/native/athena/Solenoid.cpp
		${ALLWPILIB}/hal/src/main/native/athena/ctre/CtreCanNode.cpp
		${ALLWPILIB}/hal/src/main/native/athena/ctre/PCM.cpp
		${ALLWPILIB}/hal/src/main/native/cpp/handles/HandlesInternal.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Joystick.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/raw_ostream.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/Error.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/ErrorHandling.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/Hashing.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/ManagedStatic.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/NativeFormatting.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/Path.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/SmallVector.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/StringRef.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/Twine.cpp
	)

	if ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "aarch64-linux-gnu") # Jetson TX1/TX2/Xaver AGX/Xavier NX
		set (FRCROBOT_HW_MAIN_SRCS ${FRCROBOT_HW_MAIN_SRCS}
			src/nano_nx_dio_hal.cpp
			src/nano_nx_i2c_hal.cpp
			src/nano_nx_spi_hal.cpp
			src/dummy_wpilib_sendable.cpp
			${ALLWPILIB}/wpilibc/src/main/native/cpp/DigitalInput.cpp
			${ALLWPILIB}/wpilibc/src/main/native/cpp/DigitalOutput.cpp
			${ALLWPILIB}/wpilibc/src/main/native/cpp/ErrorBase.cpp
			${ALLWPILIB}/wpilibc/src/main/native/cpp/Error.cpp
			${ALLWPILIB}/wpilibc/src/main/native/cpp/frc2/Timer.cpp
			${ALLWPILIB}/wpilibc/src/main/native/cpp/I2C.cpp
			${ALLWPILIB}/wpilibc/src/main/native/cpp/RobotController.cpp
			${ALLWPILIB}/wpilibc/src/main/native/cpp/SensorUtil.cpp
			${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/StringMap.cpp
			${ALLWPILIB}/wpiutil/src/main/native/cpp/timestamp.cpp
			${ALLWPILIB}/wpiutil/src/main/native/unix/Demangle.cpp
			${ALLWPILIB}/wpiutil/src/main/native/unix/StackTrace.cpp
		)
	else() # anything we don't want to support GPIO access on - x86, older Jetson, etc
		set (FRCROBOT_HW_MAIN_SRCS ${FRCROBOT_HW_MAIN_SRCS}
			src/dummy_wpilib_hw_x86.cpp
			src/dummy_wpilib_i2c.cpp
		)
	endif()

    # Don't build sim code for Jetson TK or TX architectures
	# Also won't be built for the Rio because of the outer if-block
	# surrounds all of this code
	if (NOT (("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "arm-linux-gnueabihf") OR
		      "${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "aarch64-linux-gnu"))  
		add_executable(frcrobot_phoenixsim_main
			src/frcrobot_phoenixsim_interface.cpp
			src/frcrobot_phoenixsim_main.cpp
			src/frcrobot_hw_interface.cpp
			src/dummy_wpilib_phoenixsim.cpp
			src/error_queue.cpp
			src/dummy_wpilib_common.cpp
			src/dummy_wpilib_i2c.cpp
			src/dummy_wpilib_phoenixsim.cpp
		)

	    # Everything but Platform_socketcan.a
		target_link_libraries(frcrobot_phoenixsim_main
			${catkin_LIBRARIES}
			${CTRE_LIB_DIR}/libCTRE_Phoenix.a
			${CTRE_LIB_DIR}/libCTRE_PhoenixCanutils.a
			${CTRE_LIB_DIR}/libCTRE_PhoenixCCI.a
			${CTRE_LIB_DIR}/libCTRE_PhoenixCore.a
			${CTRE_LIB_DIR}/libCTRE_PhoenixDiagnostics.a
			${CTRE_LIB_DIR}/../shared/libCTRE_PhoenixPlatform_sim.so
			frc_robot_interface
			frc_sim_wpilib
			convert
		)
		## Add cmake target dependencies of the executable
		## same as for the library above. This is needed to force
		## talon controller messages to be built before the
		## talon controller code tries to use them
		add_dependencies(frcrobot_phoenixsim_main
			${${PROJECT_NAME}_EXPORTED_TARGETS}
			${catkin_EXPORTED_TARGETS}
		)

		# Install executable
		install(TARGETS
		  frcrobot_phoenixsim_main
		  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)
	endif()
endif()

add_executable(frcrobot_hw_main ${FRCROBOT_HW_MAIN_SRCS})

# Link against wpi user libraries for CAN talon and navX-MXP code and
# wpilib for HAL, driver station, and so on
target_link_libraries(frcrobot_hw_main
	${catkin_LIBRARIES}
	${CTRE_USER_LIBS}
	${LIB_GPIOD}
	frc_robot_interface
	convert
	${PLATFORM_SPECIFIC_LIBS}
)

## Add cmake target dependencies of the executable
## same as for the library above. This is needed to force
## talon controller messages to be built before the
## talon controller code tries to use them
add_dependencies(frcrobot_hw_main
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(q_to_rpy src/q_to_rpy.cpp)
target_link_libraries(q_to_rpy
	${catkin_LIBRARIES}
)

add_dependencies(q_to_rpy
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
## Install ------------------------------------------------------------

# Install executables
install(TARGETS
  frcrobot_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (0)
# Test trajectory generator node
add_executable(${PROJECT_NAME}_test_trajectory src/tools/test_trajectory.cpp)
set_target_properties(${PROJECT_NAME}_test_trajectory PROPERTIES OUTPUT_NAME test_trajectory PREFIX "")
target_link_libraries(${PROJECT_NAME}_test_trajectory
  ${catkin_LIBRARIES}
)

## TOOLS ------------------------------------------------------

# Tool for analyzing controller performance
add_executable(${PROJECT_NAME}_controller_to_csv_main 
	src/tools/controller_to_csv_main.cpp
	src/tools/controller_to_csv.cpp)
set_target_properties(${PROJECT_NAME}_controller_to_csv_main PROPERTIES OUTPUT_NAME controller_to_csv_main PREFIX "")
target_link_libraries(${PROJECT_NAME}_controller_to_csv_main
  ${Gflags_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Tool for analyzing controller performance
add_library(csv_to_controller src/tools/csv_to_controller.cpp)
target_link_libraries(csv_to_controller
  ${catkin_LIBRARIES}
)
add_executable(${PROJECT_NAME}_csv_to_controller_main 
	src/tools/csv_to_controller_main.cpp
	src/tools/csv_to_controller.cpp)
set_target_properties(${PROJECT_NAME}_csv_to_controller_main PROPERTIES OUTPUT_NAME csv_to_controller_main PREFIX "")
target_link_libraries(${PROJECT_NAME}_csv_to_controller_main
  ${Gflags_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Tool for controlling a robot from keyboard
add_executable(${PROJECT_NAME}_keyboard_teleop src/tools/keyboard_teleop.cpp)
set_target_properties(${PROJECT_NAME}_keyboard_teleop PROPERTIES OUTPUT_NAME keyboard_teleop PREFIX "")
target_link_libraries(${PROJECT_NAME}_keyboard_teleop
  ${catkin_LIBRARIES}

install(TARGETS
    ${PROJECT_NAME}_test_trajectory
    ${PROJECT_NAME}_controller_to_csv_main
    ${PROJECT_NAME}_csv_to_controller_main
    ${PROJECT_NAME}_keyboard_teleop
    
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
)
endif()

# Install header files
install(DIRECTORY 
	include/${PROJECT_NAME}
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	PATTERN *~ EXCLUDE
	PATTERN *.sw? EXCLUDE
)

install(DIRECTORY
	launch
	config
	urdf
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	PATTERN *~ EXCLUDE
	PATTERN *.sw? EXCLUDE
)
