cmake_minimum_required(VERSION 3.20)
project(ros_control_boilerplate)

unset(CMAKE_CXX_STANDARD CACHE)
set(CMAKE_CXX_STANDARD 20)
include ("../cmake_modules/CMakeOpt.cmake")

# Only add gazebo to list of required packages on x86 and mac? builds
if (NOT (("${ARM_PREFIX}" STREQUAL "arm-frc2025-linux-gnueabi") OR
	     ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "arm-linux-gnueabihf") OR
		 ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "aarch64-linux-gnu") )) 
find_package(catkin REQUIRED COMPONENTS
  angles
  as726x_interface
  cmake_modules
  controller_manager
  control_msgs
  ctre_interfaces
  eigen_conversions
  frc_msgs
  frc_interfaces
  hardware_interface
  message_generation
  periodic_interval_counter
  pigeon2_interface
  remote_joint_interface
  roscpp
  rosparam_shortcuts
  sensor_msgs
  spark_max_interface
  std_msgs
  trajectory_msgs
  urdf
  gazebo_frcrobot_control
  gazebo_dev
  pluginlib
)
ign_import_target(UUID)
pkg_check_modules(TBB tbb)
else()
find_package(catkin REQUIRED COMPONENTS
  angles
  as726x_interface
  cmake_modules
  controller_manager
  control_msgs
  ctre_interfaces
  eigen_conversions
  frc_msgs
  frc_interfaces
  hardware_interface
  message_generation
  periodic_interval_counter
  pigeon2_interface
  remote_joint_interface
  roscpp
  rosparam_shortcuts
  sensor_msgs
  spark_max_interface
  std_msgs
  trajectory_msgs
  urdf
)
endif()

add_service_files (
  FILES
  set_limit_switch.srv
  LineBreakSensors.srv
  DSError.srv
  set_current.srv
)

generate_messages(
  DEPENDENCIES
  ros_control_boilerplate
  std_msgs
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
	ctre_interfaces
	trajectory_msgs
  LIBRARIES
)

###########
## Build ##
###########

# Various source which is common to both sim and hardware
# interface, regardless of platform being run
add_library(frc_robot_interface STATIC
    src/analog_input_device.cpp
	src/analog_input_devices.cpp
	src/AS726x.cpp
	src/canifier_device.cpp
	src/canifier_devices.cpp
	src/can_bus_status_device.cpp
	src/can_bus_status_devices.cpp
    src/ctre_v5_device.cpp
    src/ctre_v6_device.cpp
    src/digital_input_device.cpp
	src/digital_input_devices.cpp
    src/digital_output_device.cpp
	src/digital_output_devices.cpp
	src/double_solenoid_device.cpp
	src/double_solenoid_devices.cpp
	src/generic_hw_control_loop.cpp
	src/get_conversion_factor.cpp
	src/hoot_logger_devices.cpp
	src/latency_compensation_group.cpp
	src/latency_compensation_groups.cpp
    src/pcm_device.cpp
	src/pcm_devices.cpp
    src/pdh_device.cpp
	src/pdh_devices.cpp
    src/pdp_device.cpp
	src/pdp_devices.cpp
    src/ph_device.cpp
	src/ph_devices.cpp
	src/pigeon2_device.cpp
	src/pigeon2_devices.cpp
    src/pwm_device.cpp
	src/pwm_devices.cpp
	src/read_config_utils.cpp
    src/robot_controller_device.cpp
	src/robot_controller_devices.cpp
	src/ros_iterative_robot.cpp
	src/ros_iterative_robot_devices.cpp
	src/ros_math_shared.cpp
	src/ready_device.cpp
	src/ready_devices.cpp
	src/rumble_device.cpp
	src/rumble_devices.cpp
	src/servo_device.cpp
	src/servo_devices.cpp
	src/solenoid_device.cpp
	src/solenoid_devices.cpp
	src/talonfxpro_device.cpp
	src/talon_orchestra_device.cpp
	src/talon_orchestra_devices.cpp
)

add_dependencies(frc_robot_interface
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

# Main control executable
# Additional files will be appended based on the platform being built
# for - x86 has access to much less real hardware than the Jetson, so
# some files which make sense for one case don't for the other
set (FRCROBOT_HW_MAIN_SRCS 
	src/frc_robot_interface_hw.cpp
	src/as726x_hw_device.cpp
	src/cancoder_hw_device.cpp
	src/candle_hw_device.cpp
	# src/ctre_v5_motor_controller_hw_device.cpp
	src/error_queue.cpp
	src/frcrobot_hw_main.cpp
	src/frcrobot_hw_interface.cpp
    src/joystick_hw_device.cpp
	src/match_data_hw_device.cpp
	# src/sparkmax_hw_device.cpp
	src/talonfxpro_hw_devices.cpp
)

set (ALLWPILIB ../../../allwpilib)
set (WPI_SYSROOT $ENV{HOME}/wpilib/2025/roborio/arm-frc2024-linux-gnueabi)
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${WPI_SYSROOT}/include
	${WPI_SYSROOT}/include/navx
)

# Base regardless of OS type
set (WPI_BASE_LIB_DIR ${WPI_SYSROOT}/lib/wpilib/linux)

# Rio-only code
if ("${ARM_PREFIX}" STREQUAL "arm-frc2025-linux-gnueabi")
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
		${WPI_SHARED_LIB_DIR}/libwpinet.so
		${WPI_SHARED_LIB_DIR}/libwpiutil.so
		${WPI_SHARED_LIB_DIR}/libvisa.so.23.3.0
		${WPI_SHARED_LIB_DIR}/libembcanshim.so
		${WPI_SHARED_LIB_DIR}/libfpgalvshim.so
		${WPI_SHARED_LIB_DIR}/libFRC_NetworkCommunication.so.25.0.0
		${WPI_SHARED_LIB_DIR}/libRoboRIO_FRC_ChipObject.so.25.0.0
		# ${WPI_SYSROOT}/lib/navx/linux/athena/static/libnavx_frc.a
	)
	set (FRCROBOT_HW_MAIN_SRCS ${FRCROBOT_HW_MAIN_SRCS}
		src/dummy_wpilib_rio.cpp
	)
elseif ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "arm-linux-gnueabihf") # Jetson TK1
	set (CTRE_LIB_DIR ${WPI_SYSROOT}/lib/ctre/linux/arm32/shared)    # Who knows if this will work
	set (REV_LIB_DIR ${WPI_SYSROOT}/lib/rev/linux/arm32/shared)
	find_library(LIB_GPIOD libgpiod.a)
elseif ("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "aarch64-linux-gnu") # Jetson TX1/TX2/Xavier AGX/Xavier NX
	set (CTRE_LIB_DIR ${WPI_SYSROOT}/lib/ctre/linux/arm64/shared)
	set (REV_LIB_DIR ${WPI_SYSROOT}/lib/rev/linux/arm64/shared)
	find_library(LIB_GPIOD libgpiod.a)
else() # x86-64, hopefully. First person to try and build native on a Mac gets kicked off the team
	set (CTRE_LIB_DIR ${WPI_SYSROOT}/lib/ctre/linux/x86-64/shared)
	set (REV_LIB_DIR ${WPI_SYSROOT}/lib/rev/linux/x86-64/shared)
endif()
  
set (REV_USER_LIBS
	# ${REV_LIB_DIR}/libREVLibDriver.so
	# ${REV_LIB_DIR}/libREVLib.so
)
# Used to find HALInitializer from wpilib internal repo
include_directories(${ALLWPILIB}/hal/src/main/native/athena)
include_directories(${ALLWPILIB}/hal/src/main/native/include)

# Used to find main frc/*.h, frc2/*.h, hal/*.h and wpi/*.h
include_directories(${ALLWPILIB}/wpilibc/src/main/native/include)
include_directories(${ALLWPILIB}/wpimath/src/main/native/include)
include_directories(${ALLWPILIB}/wpiutil/src/main/native/include)
include_directories(${ALLWPILIB}/wpimath/src/main/native/thirdparty/gcem/include)
include_directories(${ALLWPILIB}/wpiutil/src/main/native/thirdparty/json/include)
include_directories(${ALLWPILIB}/wpiutil/src/main/native/thirdparty/fmtlib/include)
include_directories(${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/include)

include_directories(${WPI_SYSROOT}/include/wpilib)

# Non-rio builds are more complex - we have to grab appropriate
# parts of WPIlib source and include it manually
if (NOT "${ARM_PREFIX}" STREQUAL "arm-frc2025-linux-gnueabi")
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
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Notifier.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/PneumaticsBase.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/PneumaticsControlModule.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/PneumaticHub.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/PWM.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/RobotController.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/SensorUtil.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Solenoid.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/event/BooleanEvent.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/event/EventLoop.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/motorcontrol/MotorController.cpp
		${ALLWPILIB}/wpimath/src/main/native/cpp/MathShared.cpp
		${ALLWPILIB}/wpimath/src/main/native/cpp/filter/Debouncer.cpp
		#${ALLWPILIB}/wpimath/src/main/native/cpp/geometry/Rotation2d.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/fs.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/SafeThread.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/StackTraceWrap.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/Synchronization.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/timestamp.cpp
		${ALLWPILIB}/wpiutil/src/main/native/thirdparty/fmtlib/src/format.cpp
        ${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/ConvertUTF.cpp
        ${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/ConvertUTFWrapper.cpp
		${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/Hashing.cpp
		${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/raw_ostream.cpp
		${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/ErrorHandling.cpp
		${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/MemAlloc.cpp
		${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/SmallPtrSet.cpp
		${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/SmallVector.cpp
	    ${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/xxhash.cpp
		${ALLWPILIB}/wpiutil/src/main/native/unix/Demangle.cpp
		${ALLWPILIB}/wpiutil/src/main/native/unix/StackTrace.cpp
	)
    set(FRC_COMMON_WPILIB frc_common_wpilib)
	add_dependencies(frc_common_wpilib
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS}
	)

	set (FRCROBOT_HW_MAIN_SRCS ${FRCROBOT_HW_MAIN_SRCS} 
		src/dummy_wpilib_common.cpp
		src/dummy_wpilib_hw.cpp
		${ALLWPILIB}/hal/src/main/native/athena/CANAPI.cpp
		${ALLWPILIB}/hal/src/main/native/athena/CTREPCM.cpp
		${ALLWPILIB}/hal/src/main/native/athena/CTREPDP.cpp
		${ALLWPILIB}/hal/src/main/native/athena/HALInitializer.cpp
		${ALLWPILIB}/hal/src/main/native/athena/REVPDH.cpp
		${ALLWPILIB}/hal/src/main/native/athena/rev/PDHFrames.cpp
		${ALLWPILIB}/hal/src/main/native/athena/REVPH.cpp
		${ALLWPILIB}/hal/src/main/native/athena/rev/PHFrames.cpp
		${ALLWPILIB}/hal/src/main/native/athena/SimDevice.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Tracer.cpp
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
			${ALLWPILIB}/wpilibc/src/main/native/cpp/I2C.cpp
			${ALLWPILIB}/wpilibc/src/main/native/cpp/Notifier.cpp
			${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/ConvertUTF.cpp
			${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/ConvertUTFWrapper.cpp
		)
	else() # anything we don't want to support GPIO access on - x86, older Jetson, etc
		set (FRCROBOT_HW_MAIN_SRCS ${FRCROBOT_HW_MAIN_SRCS}
			src/dummy_wpilib_hw_x86.cpp
			src/dummy_wpilib_i2c.cpp
		)
	endif()

	add_library(ntcore  src/dummy_library.cpp)
	add_library(wpilibc src/dummy_library.cpp)
	add_library(wpiHal  src/dummy_library.cpp)
	add_library(wpimath src/dummy_library.cpp)
	add_library(wpinet  src/dummy_library.cpp)
	add_library(wpiutil src/dummy_library.cpp)
	install(TARGETS
		ntcore
		wpilibc
		wpiHal
		wpimath
		wpinet
		wpiutil

	  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
	link_directories(../../devel/.private/ros_control_boilerplate/lib)
endif()

# Don't build sim code for the Rio or Jetson -
# saves time and space for something which will
# never be run
if (NOT (("${ARM_PREFIX}" STREQUAL "arm-frc2025-linux-gnueabi") OR
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
        ${ALLWPILIB}/wpilibc/src/main/native/cpp/I2C.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/IterativeRobotBase.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/RobotState.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/simulation/CallbackStore.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/simulation/RoboRioSim.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Timer.cpp
		${ALLWPILIB}/wpilibc/src/main/native/cpp/Watchdog.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/DataLog.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/Logger.cpp
		${ALLWPILIB}/wpiutil/src/main/native/thirdparty/llvm/cpp/llvm/StringExtras.cpp
		${ALLWPILIB}/wpiutil/src/main/native/cpp/timestamp.cpp
		${ALLWPILIB}/wpiutil/src/main/native/unix/Demangle.cpp
		${ALLWPILIB}/wpiutil/src/main/native/unix/StackTrace.cpp
	)
	add_dependencies(frc_sim_wpilib
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS}
	)

	add_executable(frcrobot_sim_main 
		src/frcrobot_sim_main.cpp
		src/frc_robot_interface_sim.cpp
		src/as726x_sim_device.cpp
		src/cancoder_sim_device.cpp
		src/candle_sim_device.cpp
		# src/ctre_v5_motor_controller_sim_device.cpp
		src/frcrobot_sim_interface.cpp
		src/dummy_powerdistribution.cpp
		src/dummy_wpilib_common.cpp
		src/dummy_wpilib_sim.cpp
		src/joystick_sim_device.cpp
		src/match_data_sim_device.cpp
		# src/sim_ctre_v5_motor_controller.cpp
		src/sim_talonfxpro_device.cpp
		# src/sparkmax_sim_device.cpp
		src/talonfxpro_sim_devices.cpp
	)
	# set_source_files_properties(
	# src/ctre_v5_motor_controller_hw_device.cpp
	# PROPERTIES
	# 	COMPILE_OPTIONS "g -O0"
	# )
	# set_source_files_properties(
	# src/ctre_v5_motor_controller_sim_device.cpp
	# PROPERTIES
	# 	COMPILE_OPTIONS "-g -O0"
	# )
	# set_source_files_properties(
	# src/sim_ctre_v5_motor_controller.cpp
	# PROPERTIES
	# 	COMPILE_OPTIONS "-g -O0"
	# )
	target_link_libraries(frcrobot_sim_main
		frc_robot_interface
		frc_sim_wpilib
		${FRC_COMMON_WPILIB}
		${CTRE_LIB_DIR}/libCTRE_PhoenixSim.so
		${CTRE_LIB_DIR}/libCTRE_PhoenixCCISim.so
		${CTRE_LIB_DIR}/libCTRE_PhoenixTools_Sim.so
		${CTRE_LIB_DIR}/libCTRE_Phoenix_WPISim.so
		${CTRE_LIB_DIR}/libCTRE_Phoenix6_WPISim.so
		${CTRE_LIB_DIR}/libCTRE_SimCANCoder.so
        ${CTRE_LIB_DIR}/libCTRE_SimPigeonIMU.so
        ${CTRE_LIB_DIR}/libCTRE_SimProPigeon2.so
		${CTRE_LIB_DIR}/libCTRE_SimProTalonFX.so
		${CTRE_LIB_DIR}/libCTRE_SimTalonSRX.so
		${CTRE_LIB_DIR}/libCTRE_SimVictorSPX.so
		${REV_USER_LIBS}
		${catkin_LIBRARIES}
		${TBB_LIBRARIES}
	)

    # Forces messages to be generated before
    # the target which uses them
	add_dependencies(frcrobot_sim_main
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS}
	)

	add_library(frcrobot_gazebosim
		src/frcrobot_gazebosim_interface.cpp
		src/frc_robot_interface_sim.cpp
		src/as726x_sim_device.cpp
		src/cancoder_sim_device.cpp
		src/candle_sim_device.cpp
		# src/ctre_v5_motor_controller_sim_device.cpp
		src/frcrobot_sim_interface.cpp
		src/dummy_powerdistribution.cpp
		src/dummy_wpilib_common.cpp
		src/dummy_wpilib_sim.cpp
		src/joystick_sim_device.cpp
		src/match_data_sim_device.cpp
		# src/sim_ctre_v5_motor_controller.cpp
		src/sim_talonfxpro_device.cpp
		# src/sparkmax_sim_device.cpp
		src/talonfxpro_sim_devices.cpp
	)

	target_link_libraries(frcrobot_gazebosim
		frc_robot_interface
		frc_sim_wpilib
		${FRC_COMMON_WPILIB}
		${CTRE_LIB_DIR}/libCTRE_PhoenixSim.so
		${CTRE_LIB_DIR}/libCTRE_PhoenixCCISim.so
		${CTRE_LIB_DIR}/libCTRE_PhoenixTools_Sim.so
		${CTRE_LIB_DIR}/libCTRE_Phoenix_WPISim.so
		${CTRE_LIB_DIR}/libCTRE_Phoenix6_WPISim.so
		${CTRE_LIB_DIR}/libCTRE_SimCANCoder.so
        ${CTRE_LIB_DIR}/libCTRE_SimPigeonIMU.so
        ${CTRE_LIB_DIR}/libCTRE_SimProPigeon2.so
		${CTRE_LIB_DIR}/libCTRE_SimProTalonFX.so
		${CTRE_LIB_DIR}/libCTRE_SimTalonSRX.so
		${CTRE_LIB_DIR}/libCTRE_SimVictorSPX.so
		${REV_USER_LIBS}
		${catkin_LIBRARIES}
		${TBB_LIBRARIES}
	)

    # Forces messages to be generated before
    # the target which uses them
	add_dependencies(frcrobot_gazebosim
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS}
	)

	install(TARGETS
	  frcrobot_sim_main
	  frcrobot_gazebosim
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
	${CTRE_LIB_DIR}/libCTRE_Phoenix.so
	${CTRE_LIB_DIR}/libCTRE_PhoenixCCI.so
	${CTRE_LIB_DIR}/libCTRE_PhoenixTools.so
	${CTRE_LIB_DIR}/libCTRE_Phoenix_WPI.so
	${CTRE_LIB_DIR}/libCTRE_Phoenix6_WPI.so
	${PLATFORM_SPECIFIC_LIBS}
	${FRC_COMMON_WPILIB}
	${catkin_LIBRARIES}
	${TBB_LIBRARIES}
	${CTRE_LIB_DIR}/libCTRE_Phoenix.so
	${CTRE_LIB_DIR}/libCTRE_PhoenixCCI.so
	${CTRE_LIB_DIR}/libCTRE_PhoenixTools.so
	${CTRE_LIB_DIR}/libCTRE_Phoenix_WPI.so
	${CTRE_LIB_DIR}/libCTRE_Phoenix6_WPI.so
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
	${TBB_LIBRARIES}
)

add_dependencies(q_to_rpy
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
## Install ------------------------------------------------------------

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

install(FILES frcrobot_gazebosim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)