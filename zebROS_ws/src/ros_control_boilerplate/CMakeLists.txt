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
  spark_max_interface
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
	message_runtime
	roscpp
	rosparam_shortcuts
	sensor_msgs
	spark_max_interface
	std_msgs
	talon_interface
	trajectory_msgs
  LIBRARIES
)

###########
## Build ##
###########

# Various source which is common to both sim and hardware
# interface, regardless of platform being run
add_library(frc_robot_interface STATIC
	src/AS726x.cpp
	src/frc_robot_interface.cpp
	src/generic_hw_control_loop.cpp
	src/ros_iterative_robot.cpp
	src/as726x_convert.cpp
	src/cancoder_convert.cpp
	src/canifier_convert.cpp
	src/rev_convert.cpp
	src/talon_convert.cpp
)

# Main control executable
# Additional files will be appended based on the platform being built
# for - x86 has access to much less real hardware than the Jetson, so
# some files which make sense for one case don't for the other
set (FRCROBOT_HW_MAIN_SRCS 
	src/error_queue.cpp
	src/frcrobot_hw_main.cpp
	src/frcrobot_hw_interface.cpp
)

set (ALLWPILIB ../../../allwpilib)
set (WPI_SYSROOT $ENV{HOME}/wpilib/2022/roborio/arm-frc2022-linux-gnueabi)
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${WPI_SYSROOT}/include
	${WPI_SYSROOT}/include/navx
)

# Base regardless of OS type
set (WPI_BASE_LIB_DIR ${WPI_SYSROOT}/lib/wpilib/linux)

# Rio-only code
if ("${ARM_PREFIX}" STREQUAL "arm-frc2022-linux-gnueabi")
	set (CTRE_LIB_DIR ${WPI_SYSROOT}/lib/ctre/linux/athena/shared)
	set (REV_LIB_DIR ${WPI_SYSROOT}/lib/rev/linux/athena/shared)

	# Link against wpi user libraries for CAN talon and navX-MXP code and
	# wpilib for HAL, driver station, and so on
	set (WPI_SHARED_LIB_DIR ${WPI_BASE_LIB_DIR}/athena/shared)
	set (PLATFORM_SPECIFIC_LIBS
		${WPI_SHARED_LIB_DIR}/libntcore.so
		${WPI_SHARED_LIB_DIR}/libwpilibc.so
		${WPI_SHARED_LIB_DIR}/libwpiHal.so
		${WPI_SHARED_LIB_DIR}/libwpimath.so
		${WPI_SHARED_LIB_DIR}/libwpiutil.so
		${WPI_SHARED_LIB_DIR}/libvisa.so.21.0.0
		${WPI_SHARED_LIB_DIR}/libembcanshim.so
		${WPI_SHARED_LIB_DIR}/libfpgalvshim.so
		${WPI_SHARED_LIB_DIR}/libFRC_NetworkCommunication.so.22.0.0
		${WPI_SHARED_LIB_DIR}/libRoboRIO_FRC_ChipObject.so.22.0.0
		${WPI_SYSROOT}/lib/navx/linux/athena/static/libnavx_frc.a
	)
	set (FRCROBOT_HW_MAIN_SRCS ${FRCROBOT_HW_MAIN_SRCS}
		src/dummy_wpilib_rio.cpp
	)
elseif ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "arm-linux-gnueabihf") # Jetson TK1
	set (CTRE_LIB_DIR ${WPI_SYSROOT}/lib/ctre/linux/raspbian/shared)    # Who knows if this will work
	set (REV_LIB_DIR ${WPI_SYSROOT}/lib/rev/linux/raspbian/shared)
	find_library(LIB_GPIOD libgpiod.a)
elseif ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "aarch64-linux-gnu") # Jetson TX1/TX2/Xavier AGX/Xavier NX
	set (CTRE_LIB_DIR ${WPI_SYSROOT}/lib/ctre/linux/aarch64bionic/shared)
	set (REV_LIB_DIR ${WPI_SYSROOT}/lib/rev/linux/aarch64bionic/shared)
	find_library(LIB_GPIOD libgpiod.a)
else() # x86-64, hopefully. First person to try and build native on a Mac gets kicked off the team
	set (CTRE_LIB_DIR ${WPI_SYSROOT}/lib/ctre/linux/x86-64/shared)
	set (REV_LIB_DIR ${WPI_SYSROOT}/lib/rev/linux/x86-64/shared)
endif()
  
set (REV_USER_LIBS
	${REV_LIB_DIR}/libREVLibDriver.so
	${REV_LIB_DIR}/libREVLib.so
)
# Used to find HALInitializer from wpilib internal repo
include_directories(${ALLWPILIB}/hal/src/main/native/athena)
include_directories(${ALLWPILIB}/hal/src/main/native/include)

# Used to find main frc/*.h, frc2/*.h, hal/*.h and wpi/*.h
include_directories(${ALLWPILIB}/wpilibc/src/main/native/include)
include_directories(${ALLWPILIB}/wpimath/src/main/native/include)
include_directories(${ALLWPILIB}/wpiutil/src/main/native/include)
include_directories(${ALLWPILIB}/wpiutil/src/main/native/fmtlib/include)
include_directories(${WPI_SYSROOT}/include/wpilib)

# Non-rio builds are more complex - we have to grab appropriate
# parts of WPIlib source and include it manually
if (NOT "${ARM_PREFIX}" STREQUAL "arm-frc2022-linux-gnueabi")
    # This is needed for non-RIO hardware platforms
	set (PLATFORM_SPECIFIC_LIBS
		${PLATFORM_SPECIFIC_LIBS}
		#${CTRE_LIB_DIR}/libCTRE_PhoenixPlatform_socketcan.so
	)

	# WPIlib code common to both hardware and sim interfaces.
	# Create it as a library to avoid building all of these twice
	add_library(frc_common_wpilib STATIC
		${ALLWPILIB}/hal/src/main/native/cpp/cpp/fpga_clock.cpp
		${ALLWPILIB}/hal/src/main/native/cpp/ErrorHandling.cpp
		${ALLWPILIB}/hal/src/main/native/cpp/handles/HandlesInternal.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Compressor.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/DoubleSolenoid.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Errors.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Joystick.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/MotorSafety.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/PneumaticsBase.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/PneumaticsControlModule.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/PneumaticHub.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/RobotController.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/SensorUtil.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Solenoid.cpp
		${ALLWPILIB}/wpimath/src/main/native/cpp/geometry/Rotation2d.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/fs.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/ErrorHandling.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/Hashing.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/ManagedStatic.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/raw_ostream.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/SmallPtrSet.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/SmallVector.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/StringMap.cpp
		${ALLWPILIB}/wpiutil/src/main/native/fmtlib/src/format.cpp
	)
    set(FRC_COMMON_WPILIB frc_common_wpilib)

	set (FRCROBOT_HW_MAIN_SRCS ${FRCROBOT_HW_MAIN_SRCS} 
		src/dummy_wpilib_common.cpp
		src/dummy_wpilib_hw.cpp
		${ALLWPILIB}/hal/src/main/native/athena/CANAPI.cpp
		${ALLWPILIB}/hal/src/main/native/athena/CTREPCM.cpp
		${ALLWPILIB}/hal/src/main/native/athena/CTREPDP.cpp
		${ALLWPILIB}/hal/src/main/native/athena/HALInitializer.cpp
		#${ALLWPILIB}/hal/src/main/native/athena/Ports.cpp
		${ALLWPILIB}/hal/src/main/native/athena/REVPDH.cpp
		${ALLWPILIB}/hal/src/main/native/athena/rev/PDHFrames.cpp
		${ALLWPILIB}/hal/src/main/native/athena/REVPH.cpp
		${ALLWPILIB}/hal/src/main/native/athena/rev/PHFrames.cpp
		${ALLWPILIB}/hal/src/main/native/athena/SimDevice.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Tracer.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/SafeThread.cpp
	)

	if ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "aarch64-linux-gnu") # Jetson TX1/TX2/Xaver AGX/Xavier NX
		set (FRCROBOT_HW_MAIN_SRCS ${FRCROBOT_HW_MAIN_SRCS}
			src/nano_nx_dio_hal.cpp
			src/nano_nx_i2c_hal.cpp
			src/nano_nx_spi_hal.cpp
			${ALLWPILIB}/hal/src/main/native/athena/mockdata/SimDeviceData.cpp
			${ALLWPILIB}/hal/src/main/native/athena/mockdata/MockHooks.cpp
			${ALLWPILIB}/wpilibc/src/main/native/cpp/DigitalInput.cpp
			${ALLWPILIB}/wpilibc/src/main/native/cpp/DigitalOutput.cpp
			#${ALLWPILIB}/wpilibc/src/main/native/cpp/Error.cpp
			#${ALLWPILIB}/wpilibc/src/main/native/cpp/frc2/Timer.cpp
			${ALLWPILIB}/wpilibc/src/main/native/cpp/I2C.cpp
			#${ALLWPILIB}/wpilibc/src/main/native/cpp/RobotController.cpp #TODO- duplicate?
			#${ALLWPILIB}/wpilibc/src/main/native/cpp/SensorUtil.cpp
			${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/ConvertUTF.cpp
			${ALLWPILIB}/wpiutil/src/main/native/cpp/llvm/ConvertUTFWrapper.cpp
			${ALLWPILIB}/wpiutil/src/main/native/cpp/timestamp.cpp
			#${ALLWPILIB}/wpiutil/src/main/native/unix/Demangle.cpp
			#${ALLWPILIB}/wpiutil/src/main/native/unix/StackTrace.cpp
		)
	else() # anything we don't want to support GPIO access on - x86, older Jetson, etc
		set (FRCROBOT_HW_MAIN_SRCS ${FRCROBOT_HW_MAIN_SRCS}
			src/dummy_wpilib_hw_x86.cpp
			src/dummy_wpilib_i2c.cpp
		)
	endif()

	add_library(wpilibc src/dummy_library.cpp)
	add_library(wpiHal  src/dummy_library.cpp)
	add_library(wpimath src/dummy_library.cpp)
	add_library(wpiutil src/dummy_library.cpp)
	install(TARGETS
		wpilibc
		wpiHal
		wpimath
		wpiutil
	  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

endif()

# Don't build sim code for the Rio or Jetson -
# saves time and space for something which will
# never be run
if (NOT (("${ARM_PREFIX}" STREQUAL "arm-frc2022-linux-gnueabi") OR
	     ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "arm-linux-gnueabihf") OR
		 ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "aarch64-linux-gnu") )) 
    # Sim version of main control executable.  This hooks into
	# WPIlib sim code, which maintains state for simulated hardware

	add_library(frc_sim_wpilib STATIC
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
		${ALLWPILIB}/hal/src/main/native/sim/Constants.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Counter.cpp
		${ALLWPILIB}/hal/src/main/native/sim/CTREPCM.cpp
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
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/CTREPCMData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/DigitalPWMData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/DIOData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/DriverStationData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/DutyCycleData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/EncoderData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/I2CData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/PowerDistributionData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/PWMData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/RelayData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/REVPHData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/RoboRioData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/SimDeviceData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/SPIAccelerometerData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/mockdata/SPIData.cpp
		${ALLWPILIB}/hal/src/main/native/sim/MockHooks.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Notifier.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Ports.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Power.cpp
		${ALLWPILIB}/hal/src/main/native/sim/PowerDistribution.cpp
		${ALLWPILIB}/hal/src/main/native/sim/PWM.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Relay.cpp
		${ALLWPILIB}/hal/src/main/native/sim/REVPH.cpp
		${ALLWPILIB}/hal/src/main/native/sim/SerialPort.cpp
		${ALLWPILIB}/hal/src/main/native/sim/SimDevice.cpp
		${ALLWPILIB}/hal/src/main/native/sim/SPI.cpp
		${ALLWPILIB}/hal/src/main/native/sim/Threads.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/AnalogInput.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/DigitalInput.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/DigitalOutput.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/DriverStation.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/GenericHID.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/IterativeRobotBase.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/RobotState.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/simulation/CallbackStore.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/simulation/RoboRioSim.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Timer.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Watchdog.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/StringExtras.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/timestamp.cpp
		${ALLWPILIB}/wpiutil/src/main/native/unix/Demangle.cpp
		${ALLWPILIB}/wpiutil/src/main/native/unix/StackTrace.cpp
	)

	add_executable(frcrobot_sim_main 
		src/frcrobot_sim_main.cpp
		src/frcrobot_sim_interface.cpp
		src/dummy_powerdistribution.cpp
		src/dummy_wpilib_common.cpp
	)

	target_link_libraries(frcrobot_sim_main
		frc_robot_interface
		frc_sim_wpilib
		${FRC_COMMON_WPILIB}
		${CTRE_LIB_DIR}/libCTRE_PhoenixSim.so
		${CTRE_LIB_DIR}/libCTRE_PhoenixCCISim.so
		${CTRE_LIB_DIR}/libCTRE_Phoenix_WPISim.so
		${CTRE_LIB_DIR}/libCTRE_SimCANCoder.so
		${CTRE_LIB_DIR}/libCTRE_SimTalonFX.so
		${CTRE_LIB_DIR}/libCTRE_SimTalonSRX.so
		${CTRE_LIB_DIR}/libCTRE_SimVictorSPX.so
		${catkin_LIBRARIES}
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

add_executable(frcrobot_hw_main ${FRCROBOT_HW_MAIN_SRCS})

# Link against wpi user libraries for CAN talon and navX-MXP code and
# wpilib for HAL, driver station, and so on
target_link_libraries(frcrobot_hw_main
	${LIB_GPIOD}
	${REV_USER_LIBS}
	frc_robot_interface
	${FRC_COMMON_WPILIB}
	${CTRE_LIB_DIR}/libCTRE_Phoenix.so
	${CTRE_LIB_DIR}/libCTRE_PhoenixCCI.so
	${CTRE_LIB_DIR}/libCTRE_Phoenix_WPI.so
	${PLATFORM_SPECIFIC_LIBS}
	${catkin_LIBRARIES}
)

## Add cmake target dependencies of the executable
## same as for the library above. This is needed to force
## talon controller messages to be built before the
## talon controller code tries to use them
add_dependencies(frcrobot_hw_main
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Install executables
install(TARGETS
  frcrobot_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
