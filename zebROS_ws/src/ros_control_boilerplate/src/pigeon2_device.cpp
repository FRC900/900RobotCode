#include <thread>
#include <ros/node_handle.h>
#include <angles/angles.h>
#include "tf2/LinearMath/Matrix3x3.h"                         // for Matrix3x3
#include "tf2/LinearMath/Quaternion.h"                        // for Quaternion

#include "ctre/phoenix6/core/CorePigeon2.hpp"

#include "pigeon2_interface/pigeon2_command_interface.h"
#include "remote_joint_interface/remote_joint_interface.h"
#include "ros_control_boilerplate/pigeon2_device.h"
#include "ros_control_boilerplate/tracer.h"

Pigeon2Device::Pigeon2Device(const std::string &name_space,
                             const int joint_index,
                             const std::string &joint_name,
                             const int can_id,
                             const std::string &can_bus,
                             const std::string &frame_id,
                             const bool local_hardware,
                             const bool local_update,
                             const double read_hz_)
    : CTREV6Device("Pigeon2", joint_name, can_id)
    , frame_id_{frame_id}
    , local_hardware_{local_hardware}
    , local_update_{local_update}
    , pigeon2_{nullptr}
    , state_{std::make_unique<hardware_interface::pigeon2::Pigeon2HWState>(can_id)}
    , command_{std::make_unique<hardware_interface::pigeon2::Pigeon2HWCommand>()}
    , read_thread_state_{nullptr}
    , read_state_mutex_{nullptr}
    , read_thread_{nullptr}
{
    // TODO : old code had a check for duplicate use of DIO channels? Needed?
    ROS_INFO_STREAM_NAMED("frc_robot_interface",
                        "Loading joint " << joint_index << "=" << joint_name <<
                        (local_update_ ? " local" : " remote") << " update, " <<
                        (local_hardware_ ? "local" : "remote") << " hardware" <<
                        " as Pigeon2 " << can_id << " on bus " << can_bus);

    if (local_hardware_)
    {
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " : " << __LINE__ << " id = " << can_id << " can_bus = " << can_bus);
        pigeon2_ = std::make_unique<ctre::phoenix6::hardware::core::CorePigeon2>(can_id, can_bus);
        setParentDevice(pigeon2_.get());
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " : " << __LINE__);
        read_thread_state_ = std::make_unique<hardware_interface::pigeon2::Pigeon2HWState>(can_id);
        read_state_mutex_ = std::make_unique<std::mutex>();
        read_thread_ = std::make_unique<std::thread>(&Pigeon2Device::read_thread, this,
                                                     std::make_unique<Tracer>("pigeon2_read_" + joint_name + " " + name_space),
                                                     read_hz_);
    }
}

Pigeon2Device::~Pigeon2Device(void)
{
    if (read_thread_ && read_thread_->joinable())
    {
        read_thread_->join();
    }
}

void Pigeon2Device::registerInterfaces(hardware_interface::pigeon2::Pigeon2StateInterface &state_interface,
                                       hardware_interface::pigeon2::Pigeon2CommandInterface &command_interface,
                                       hardware_interface::pigeon2::RemotePigeon2StateInterface &remote_state_interface,
                                       hardware_interface::ImuSensorInterface &imu_interface,
                                       hardware_interface::RemoteImuSensorInterface &imu_remote_interface)
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for Pigeon2 : " << getName() << " at CAN id " << getId());

    hardware_interface::pigeon2::Pigeon2StateHandle state_handle(getName(), state_.get());
    state_interface.registerHandle(state_handle);

    hardware_interface::pigeon2::Pigeon2CommandHandle command_handle(state_handle, command_.get());
    command_interface.registerHandle(command_handle);

    if (!local_update_)
    {
        hardware_interface::pigeon2::Pigeon2WritableStateHandle remote_handle(getName(), state_.get());
        remote_state_interface.registerHandle(remote_handle);
    }

	// Create state interface for the given IMU and point it to the 
    // data stored in the corresponding class imu arrays
    // Those arrays are updated in ::read() with the values
    // read from hardware
    // With this, standard ROS code expecting an ImuSensor object
    // can work with the data provided by the Pigeon2
	hardware_interface::ImuSensorHandle::Data imu_data;
	imu_data.name = getName();
	imu_data.frame_id = frame_id_;
	for (size_t j = 0; j < 3; j++)
	{
		imu_orientation_[j] = 0;
		imu_angular_velocity_[j] = 0;
		imu_linear_acceleration_[j] = 0;
    }
    imu_orientation_[3] = 1;
	imu_data.orientation = &imu_orientation_[0];
	imu_data.orientation_covariance = &imu_orientation_covariance_[0];
	imu_data.angular_velocity = &imu_angular_velocity_[0];
	imu_data.angular_velocity_covariance = &imu_angular_velocity_covariance_[0];
	imu_data.linear_acceleration = &imu_linear_acceleration_[0];
	imu_data.linear_acceleration_covariance = &imu_linear_acceleration_covariance_[0];

	hardware_interface::ImuSensorHandle imuh(imu_data);
	imu_interface.registerHandle(imuh);

	if (!local_update_)
	{
		hardware_interface::ImuWritableSensorHandle ish(imu_data);
		imu_remote_interface.registerHandle(ish);
	}
}

// Create a subscriber reading simulated yaw values
void Pigeon2Device::simInit(ros::NodeHandle nh, size_t joint_index)
{
    std::stringstream s;
    s << "imu_" << joint_index << "_in";
    sim_sub_ = nh.subscribe<nav_msgs::Odometry>(s.str(), 1, &Pigeon2Device::imuOdomCallback, this);
}

void Pigeon2Device::imuOdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
	double roll, pitch, yaw;
	const geometry_msgs::Quaternion &q = msg->pose.pose.orientation;
	tf2::Quaternion tf_q(
		q.x,
		q.y,
		q.z,
		q.w);
	tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
	sim_yaw_.store(yaw);
}

void Pigeon2Device::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (local_update_)
    {
        // Fill in the full pigeon2 state using the last
        // data read from the pigeon2 read thread for this hardware
        {
            std::unique_lock<std::mutex> l(*read_state_mutex_, std::try_to_lock);
            if (!l.owns_lock())
            {
                return;
            }
            state_->setVersionMajor(read_thread_state_->getVersionMajor());
            state_->setVersionMinor(read_thread_state_->getVersionMinor());
            state_->setVersionBugfix(read_thread_state_->getVersionBugfix());
            state_->setVersionBuild(read_thread_state_->getVersionBuild());

            state_->setYaw(read_thread_state_->getYaw());
            state_->setPitch(read_thread_state_->getPitch());
            state_->setRoll(read_thread_state_->getRoll());

            state_->setQuatW(read_thread_state_->getQuatW());
            state_->setQuatX(read_thread_state_->getQuatX());
            state_->setQuatY(read_thread_state_->getQuatY());
            state_->setQuatZ(read_thread_state_->getQuatZ());

            state_->setGravityVectorX(read_thread_state_->getGravityVectorX());
            state_->setGravityVectorY(read_thread_state_->getGravityVectorY());
            state_->setGravityVectorZ(read_thread_state_->getGravityVectorZ());

            state_->setTemperature(read_thread_state_->getTemperature());
            state_->setNoMotionCount(read_thread_state_->getNoMotionCount());
            state_->setUptime(read_thread_state_->getUptime());

            state_->setAccumGyroX(read_thread_state_->getAccumGyroX());
            state_->setAccumGyroY(read_thread_state_->getAccumGyroY());
            state_->setAccumGyroZ(read_thread_state_->getAccumGyroZ());

            state_->setAngularVelocityX(read_thread_state_->getAngularVelocityX());
            state_->setAngularVelocityY(read_thread_state_->getAngularVelocityY());
            state_->setAngularVelocityZ(read_thread_state_->getAngularVelocityZ());

            state_->setAccelerationX(read_thread_state_->getAccelerationX());
            state_->setAccelerationY(read_thread_state_->getAccelerationY());
            state_->setAccelerationZ(read_thread_state_->getAccelerationZ());

            state_->setSupplyVoltage(read_thread_state_->getSupplyVoltage());

            state_->setMagneticFieldX(read_thread_state_->getMagneticFieldX());
            state_->setMagneticFieldY(read_thread_state_->getMagneticFieldY());
            state_->setMagneticFieldZ(read_thread_state_->getMagneticFieldZ());

            state_->setRawMagneticFieldX(read_thread_state_->getRawMagneticFieldX());
            state_->setRawMagneticFieldY(read_thread_state_->getRawMagneticFieldY());
            state_->setRawMagneticFieldZ(read_thread_state_->getRawMagneticFieldZ());

            state_->setFaultHardware(read_thread_state_->getFaultHardware());
            state_->setFaultUndervoltage(read_thread_state_->getFaultUndervoltage());
            state_->setFaultBootDuringEnable(read_thread_state_->getFaultBootDuringEnable());
            state_->setFaultUnlicensedFeatureInUse(read_thread_state_->getFaultUnlicensedFeatureInUse());
            state_->setFaultBootupAccelerometer(read_thread_state_->getFaultBootupAccelerometer());
            state_->setFaultBootupGyroscope(read_thread_state_->getFaultBootupGyroscope());
            state_->setFaultBootupMagnetometer(read_thread_state_->getFaultBootupMagnetometer());
            state_->setFaultBootIntoMotion(read_thread_state_->getFaultBootIntoMotion());
            state_->setFaultDataAcquiredLate(read_thread_state_->getFaultDataAcquiredLate());
            state_->setFaultLoopTimeSlow(read_thread_state_->getFaultLoopTimeSlow());
            state_->setFaultSaturatedMagnetometer(read_thread_state_->getFaultSaturatedMagnetometer());
            state_->setFaultSaturatedAccelometer(read_thread_state_->getFaultSaturatedAccelometer());
            state_->setFaultSaturatedGyroscope(read_thread_state_->getFaultSaturatedGyroscope());

            state_->setStickyFaultHardware(read_thread_state_->getStickyFaultHardware());
            state_->setStickyFaultUndervoltage(read_thread_state_->getStickyFaultUndervoltage());
            state_->setStickyFaultBootDuringEnable(read_thread_state_->getStickyFaultBootDuringEnable());
            state_->setStickyFaultUnlicensedFeatureInUse(read_thread_state_->getStickyFaultUnlicensedFeatureInUse());
            state_->setStickyFaultBootupAccelerometer(read_thread_state_->getStickyFaultBootupAccelerometer());
            state_->setStickyFaultBootupGyroscope(read_thread_state_->getStickyFaultBootupGyroscope());
            state_->setStickyFaultBootupMagnetometer(read_thread_state_->getStickyFaultBootupMagnetometer());
            state_->setStickyFaultBootIntoMotion(read_thread_state_->getStickyFaultBootIntoMotion());
            state_->setStickyFaultDataAcquiredLate(read_thread_state_->getStickyFaultDataAcquiredLate());
            state_->setStickyFaultLoopTimeSlow(read_thread_state_->getStickyFaultLoopTimeSlow());
            state_->setStickyFaultSaturatedMagnetometer(read_thread_state_->getStickyFaultSaturatedMagnetometer());
            state_->setStickyFaultSaturatedAccelometer(read_thread_state_->getStickyFaultSaturatedAccelometer());
            state_->setStickyFaultSaturatedGyroscope(read_thread_state_->getStickyFaultSaturatedGyroscope());
        }

        // also, fill in the standard ROS imu state info
        // for this.  We really only care about orientation,
        // but do a best effort for the rest

        imu_orientation_[3] = state_->getQuatW();
        imu_orientation_[0] = state_->getQuatX();
        imu_orientation_[1] = state_->getQuatY();
        imu_orientation_[2] = state_->getQuatZ();

        imu_linear_acceleration_[0] = state_->getAccelerationX();
        imu_linear_acceleration_[1] = state_->getAccelerationY();
        imu_linear_acceleration_[2] = state_->getAccelerationZ();

        imu_angular_velocity_[0] = state_->getAngularVelocityX();
        imu_angular_velocity_[1] = state_->getAngularVelocityY();
        imu_angular_velocity_[2] = state_->getAngularVelocityZ();
    }
}

void Pigeon2Device::simRead(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (local_hardware_)
    {
        pigeon2_->GetSimState().SetRawYaw(units::radian_t{sim_yaw_});
    }
}

void Pigeon2Device::write(const ros::Time & /*time*/, const ros::Duration & /*period*/)
{
    if (!local_hardware_)
    {
        return;
    }

    // Force reconfig if the pigeon resets
    if (pigeon2_->HasResetOccurred())
    {
        command_->resetMountPoseRPY();
        command_->resetGyroTrim();
        command_->resetPigeon2Features();

        //command_->resetSetYaw(); // TODO, not sure about this, there's no telling how long ago this value was correct
    }

    ctre::phoenix6::configs::MountPoseConfigs mount_pose_configs;
    if (command_->mountPoseRPYChanged(mount_pose_configs.MountPoseYaw, mount_pose_configs.MountPosePitch, mount_pose_configs.MountPoseRoll))
    {
        mount_pose_configs.MountPoseYaw = angles::to_degrees(mount_pose_configs.MountPoseYaw);
        mount_pose_configs.MountPosePitch = angles::to_degrees(mount_pose_configs.MountPosePitch);
        mount_pose_configs.MountPoseRoll = angles::to_degrees(mount_pose_configs.MountPoseRoll);
        if (safeCall(pigeon2_->GetConfigurator().Apply(mount_pose_configs), "ApplyConfig(MountPoseConfigs"))
        {
            ROS_INFO_STREAM("Updated Pigeon2 " << getId() << " = " << getName() << " MountPose " << mount_pose_configs);
            state_->setMountPoseYaw(angles::from_degrees(mount_pose_configs.MountPoseYaw ));
            state_->setMountPosePitch(angles::from_degrees(mount_pose_configs.MountPosePitch));
            state_->setMountPoseRoll(angles::from_degrees(mount_pose_configs.MountPoseRoll));
        }
        else
        {
            command_->resetMountPoseRPY();
            return;
        }
    }

    ctre::phoenix6::configs::GyroTrimConfigs gyro_trim_configs;
    if (command_->gyroTrimChanged(gyro_trim_configs.GyroScalarX,
                                  gyro_trim_configs.GyroScalarY,
                                  gyro_trim_configs.GyroScalarZ))
    {
        gyro_trim_configs.GyroScalarX = angles::to_degrees(gyro_trim_configs.GyroScalarX);
        gyro_trim_configs.GyroScalarY = angles::to_degrees(gyro_trim_configs.GyroScalarY);
        gyro_trim_configs.GyroScalarZ = angles::to_degrees(gyro_trim_configs.GyroScalarZ);
        if (safeCall(pigeon2_->GetConfigurator().Apply(gyro_trim_configs), "ApplyConfig(GyroTrimConfigs"))
        {
            ROS_INFO_STREAM("Updated Pigeon2 " << getId() << " = " << getName() << " GyroTrim " << gyro_trim_configs);
            state_->setGyroTrimScalarX(angles::from_degrees(gyro_trim_configs.GyroScalarX ));
            state_->setGyroTrimScalarY(angles::from_degrees(gyro_trim_configs.GyroScalarY));
            state_->setGyroTrimScalarZ(angles::from_degrees(gyro_trim_configs.GyroScalarZ));
        }
        else
        {
            command_->resetGyroTrim();
            return;
        }
    }

    ctre::phoenix6::configs::Pigeon2FeaturesConfigs pigeon2_features_configs;
    if (command_->pigeon2FeaturesChanged(pigeon2_features_configs.EnableCompass,
                                         pigeon2_features_configs.DisableTemperatureCompensation,
                                         pigeon2_features_configs.DisableNoMotionCalibration))
    {
        if (safeCall(pigeon2_->GetConfigurator().Apply(pigeon2_features_configs), "ApplyConfig(Pigeon2FeaturesConfigs"))
        {
            ROS_INFO_STREAM("Updated Pigeon2 " << getId() << " = " << getName() << " Pigeon2Features " << pigeon2_features_configs);
            state_->setEnableCompass(pigeon2_features_configs.EnableCompass);
            state_->setDisableTemperatureCompensation(pigeon2_features_configs.DisableTemperatureCompensation);
            state_->setDisableNoMotionCalibration(pigeon2_features_configs.DisableNoMotionCalibration);
        }
        else
        {
            command_->resetPigeon2Features();
            return;
        }
    }

    if (command_->clearStickyFaultsChanged())
    {
        if (safeCall(pigeon2_->ClearStickyFaults(), "pigeon2_->ClearStickyFaults()"))
        {
            ROS_INFO_STREAM("Pigeon2 " << getName() << " = " << getId() << " ClearStickyFaults");
        }
        else
        {
            command_->setClearStickyFaults();
            return;
        }
    }

    if (double set_yaw; command_->setYawChanged(set_yaw))
    {
        if (safeCall(pigeon2_->SetYaw(units::radian_t{set_yaw}), "pigeon2_->SetYaw()"))
        {
            ROS_INFO_STREAM("Pigeon2 " << getName() << " = " << getId() << " SetYaw to " << set_yaw);
        }
        else
        {
            command_->resetSetYaw();
            return;
        }
    }
}

#define SAFE_READ(var, function) \
const auto var = safeRead(function, #function); \
if (!var) { tracer->stop() ; continue; }

void Pigeon2Device::read_thread(std::unique_ptr<Tracer> tracer,
                                const double poll_frequency)
{
#ifdef __linux__
	std::stringstream thread_name{"pgn2_read_"};

	if (pthread_setname_np(pthread_self(), thread_name.str().c_str()))
	{
		ROS_ERROR_STREAM("Error setting thread name " << thread_name.str() << " " << errno);
	}
#endif
    ROS_INFO_STREAM("Starting pigeon2 read thread for " << getName() << " at " << ros::Time::now());
    ros::Duration(2.952 + read_thread_state_->getDeviceNumber() * 0.04).sleep(); // Sleep for a few seconds to let CAN start up
	for (ros::Rate r(poll_frequency); ros::ok(); r.sleep())
	{
		tracer->start("pigeon2 read main_loop");

        SAFE_READ(version_major, pigeon2_->GetVersionMajor());
        SAFE_READ(version_minor, pigeon2_->GetVersionMinor());
        SAFE_READ(version_bugfix, pigeon2_->GetVersionBugfix());
        SAFE_READ(version_build, pigeon2_->GetVersionBuild());

        SAFE_READ(yaw, pigeon2_->GetYaw());
        SAFE_READ(pitch, pigeon2_->GetPitch());
        SAFE_READ(roll, pigeon2_->GetRoll());

        SAFE_READ(quat_w, pigeon2_->GetQuatW());
        SAFE_READ(quat_x, pigeon2_->GetQuatX());
        SAFE_READ(quat_y, pigeon2_->GetQuatY());
        SAFE_READ(quat_z, pigeon2_->GetQuatZ());

        SAFE_READ(gravity_vector_x, pigeon2_->GetGravityVectorX());
        SAFE_READ(gravity_vector_y, pigeon2_->GetGravityVectorY());
        SAFE_READ(gravity_vector_z, pigeon2_->GetGravityVectorZ());

        SAFE_READ(temperature, pigeon2_->GetTemperature());
        SAFE_READ(uptime, pigeon2_->GetUpTime());

        SAFE_READ(no_motion_count, pigeon2_->GetNoMotionCount());

        SAFE_READ(accum_gyro_x, pigeon2_->GetAccumGyroX());
        SAFE_READ(accum_gyro_y, pigeon2_->GetAccumGyroY());
        SAFE_READ(accum_gyro_z, pigeon2_->GetAccumGyroZ());

        SAFE_READ(angular_velocity_x, pigeon2_->GetAngularVelocityX());
        SAFE_READ(angular_velocity_y, pigeon2_->GetAngularVelocityY());
        SAFE_READ(angular_velocity_z, pigeon2_->GetAngularVelocityZ());

        SAFE_READ(acceleration_x, pigeon2_->GetAccelerationX());
        SAFE_READ(acceleration_y, pigeon2_->GetAccelerationY());
        SAFE_READ(acceleration_z, pigeon2_->GetAccelerationZ());

        SAFE_READ(supply_voltage, pigeon2_->GetSupplyVoltage());

        SAFE_READ(magnetic_field_x, pigeon2_->GetMagneticFieldX());
        SAFE_READ(magnetic_field_y, pigeon2_->GetMagneticFieldY());
        SAFE_READ(magnetic_field_z, pigeon2_->GetMagneticFieldZ());

        SAFE_READ(raw_magnetic_field_x, pigeon2_->GetRawMagneticFieldX());
        SAFE_READ(raw_magnetic_field_y, pigeon2_->GetRawMagneticFieldY());
        SAFE_READ(raw_magnetic_field_z, pigeon2_->GetRawMagneticFieldZ());

        SAFE_READ(fault_hardware, pigeon2_->GetFault_Hardware());
        SAFE_READ(fault_undervoltage, pigeon2_->GetFault_Undervoltage());
        SAFE_READ(fault_boot_during_enable, pigeon2_->GetFault_BootDuringEnable());
        SAFE_READ(fault_unlicensed_feature_in_use, pigeon2_->GetFault_UnlicensedFeatureInUse());
        SAFE_READ(fault_bootup_accelerometer, pigeon2_->GetFault_BootupAccelerometer());
        SAFE_READ(fault_bootup_gyroscope, pigeon2_->GetFault_BootupGyroscope());
        SAFE_READ(fault_bootup_magnetometer, pigeon2_->GetFault_BootupMagnetometer());
        SAFE_READ(fault_boot_into_motion, pigeon2_->GetFault_BootIntoMotion());
        SAFE_READ(fault_data_acquired_late, pigeon2_->GetFault_DataAcquiredLate());
        SAFE_READ(fault_loop_time_slow, pigeon2_->GetFault_LoopTimeSlow());
        SAFE_READ(fault_saturated_magnetometer, pigeon2_->GetFault_SaturatedMagnetometer());
        SAFE_READ(fault_saturated_accelometer, pigeon2_->GetFault_SaturatedAccelometer());
        SAFE_READ(fault_saturated_gyroscope, pigeon2_->GetFault_SaturatedGyroscope());

        SAFE_READ(sticky_fault_hardware, pigeon2_->GetStickyFault_Hardware());
        SAFE_READ(sticky_fault_undervoltage, pigeon2_->GetStickyFault_Undervoltage());
        SAFE_READ(sticky_fault_boot_during_enable, pigeon2_->GetStickyFault_BootDuringEnable());
        SAFE_READ(sticky_fault_unlicensed_feature_in_use, pigeon2_->GetStickyFault_UnlicensedFeatureInUse());
        SAFE_READ(sticky_fault_bootup_accelerometer, pigeon2_->GetStickyFault_BootupAccelerometer());
        SAFE_READ(sticky_fault_bootup_gyroscope, pigeon2_->GetStickyFault_BootupGyroscope());
        SAFE_READ(sticky_fault_bootup_magnetometer, pigeon2_->GetStickyFault_BootupMagnetometer());
        SAFE_READ(sticky_fault_boot_into_motion, pigeon2_->GetStickyFault_BootIntoMotion());
        SAFE_READ(sticky_fault_data_acquired_late, pigeon2_->GetStickyFault_DataAcquiredLate());
        SAFE_READ(sticky_fault_loop_time_slow, pigeon2_->GetStickyFault_LoopTimeSlow());
        SAFE_READ(sticky_fault_saturated_magnetometer, pigeon2_->GetStickyFault_SaturatedMagnetometer());
        SAFE_READ(sticky_fault_saturated_accelometer, pigeon2_->GetStickyFault_SaturatedAccelometer());
        SAFE_READ(sticky_fault_saturated_gyroscope, pigeon2_->GetStickyFault_SaturatedGyroscope());

        // Actually update the Pigeon2HWState shared between
		// this thread and read()
		// Do this all at once so the code minimizes the amount
		// of time with mutex locked
		{
			// Lock the state entry to make sure writes
			// are atomic - reads won't grab data in
			// the middle of a write
			std::lock_guard<std::mutex> l(*read_state_mutex_);
            read_thread_state_->setVersionMajor(*version_major);
            read_thread_state_->setVersionMinor(*version_minor);
            read_thread_state_->setVersionBugfix(*version_bugfix);
            read_thread_state_->setVersionBuild(*version_build);

            read_thread_state_->setYaw(units::radian_t{*yaw}.value());
            read_thread_state_->setPitch(units::radian_t{*pitch}.value());
            read_thread_state_->setRoll(units::radian_t{*roll}.value());

            read_thread_state_->setQuatW(*quat_w);
            read_thread_state_->setQuatX(*quat_x);
            read_thread_state_->setQuatY(*quat_y);
            read_thread_state_->setQuatZ(*quat_z);

            read_thread_state_->setGravityVectorX(*gravity_vector_x);
            read_thread_state_->setGravityVectorY(*gravity_vector_y);
            read_thread_state_->setGravityVectorZ(*gravity_vector_z);

            read_thread_state_->setTemperature(temperature->value());
            read_thread_state_->setNoMotionCount(*no_motion_count);
            read_thread_state_->setUptime(uptime->value());

            read_thread_state_->setAccumGyroX(units::radian_t{*accum_gyro_x}.value());
            read_thread_state_->setAccumGyroY(units::radian_t{*accum_gyro_y}.value());
            read_thread_state_->setAccumGyroZ(units::radian_t{*accum_gyro_z}.value());

            read_thread_state_->setAngularVelocityX(units::radians_per_second_t{*angular_velocity_x}.value());
            read_thread_state_->setAngularVelocityY(units::radians_per_second_t{*angular_velocity_y}.value());
            read_thread_state_->setAngularVelocityZ(units::radians_per_second_t{*angular_velocity_z}.value());

            read_thread_state_->setAccelerationX(units::meters_per_second_squared_t{*acceleration_x}.value());
            read_thread_state_->setAccelerationY(units::meters_per_second_squared_t{*acceleration_y}.value());
            read_thread_state_->setAccelerationZ(units::meters_per_second_squared_t{*acceleration_z}.value());

            read_thread_state_->setSupplyVoltage(supply_voltage->value());

            read_thread_state_->setMagneticFieldX(units::tesla_t{*magnetic_field_x}.value());
            read_thread_state_->setMagneticFieldY(units::tesla_t{*magnetic_field_y}.value());
            read_thread_state_->setMagneticFieldZ(units::tesla_t{*magnetic_field_z}.value());

            read_thread_state_->setRawMagneticFieldX(units::tesla_t{*raw_magnetic_field_x}.value());
            read_thread_state_->setRawMagneticFieldY(units::tesla_t{*raw_magnetic_field_y}.value());
            read_thread_state_->setRawMagneticFieldZ(units::tesla_t{*raw_magnetic_field_z}.value());

            read_thread_state_->setFaultHardware(*fault_hardware);
            read_thread_state_->setFaultUndervoltage(*fault_undervoltage);
            read_thread_state_->setFaultBootDuringEnable(*fault_boot_during_enable);
            read_thread_state_->setFaultUnlicensedFeatureInUse(*fault_unlicensed_feature_in_use);
            read_thread_state_->setFaultBootupAccelerometer(*fault_bootup_accelerometer);
            read_thread_state_->setFaultBootupGyroscope(*fault_bootup_gyroscope);
            read_thread_state_->setFaultBootupMagnetometer(*fault_bootup_magnetometer);
            read_thread_state_->setFaultBootIntoMotion(*fault_boot_into_motion);
            read_thread_state_->setFaultDataAcquiredLate(*fault_data_acquired_late);
            read_thread_state_->setFaultLoopTimeSlow(*fault_loop_time_slow);
            read_thread_state_->setFaultSaturatedMagnetometer(*fault_saturated_magnetometer);
            read_thread_state_->setFaultSaturatedAccelometer(*fault_saturated_accelometer);
            read_thread_state_->setFaultSaturatedGyroscope(*fault_saturated_gyroscope);

            read_thread_state_->setStickyFaultHardware(*sticky_fault_hardware);
            read_thread_state_->setStickyFaultUndervoltage(*sticky_fault_undervoltage);
            read_thread_state_->setStickyFaultBootDuringEnable(*sticky_fault_boot_during_enable);
            read_thread_state_->setStickyFaultUnlicensedFeatureInUse(*sticky_fault_unlicensed_feature_in_use);
            read_thread_state_->setStickyFaultBootupAccelerometer(*sticky_fault_bootup_accelerometer);
            read_thread_state_->setStickyFaultBootupGyroscope(*sticky_fault_bootup_gyroscope);
            read_thread_state_->setStickyFaultBootupMagnetometer(*sticky_fault_bootup_magnetometer);
            read_thread_state_->setStickyFaultBootIntoMotion(*sticky_fault_boot_into_motion);
            read_thread_state_->setStickyFaultDataAcquiredLate(*sticky_fault_data_acquired_late);
            read_thread_state_->setStickyFaultLoopTimeSlow(*sticky_fault_loop_time_slow);
            read_thread_state_->setStickyFaultSaturatedMagnetometer(*sticky_fault_saturated_magnetometer);
            read_thread_state_->setStickyFaultSaturatedAccelometer(*sticky_fault_saturated_accelometer);
            read_thread_state_->setStickyFaultSaturatedGyroscope(*sticky_fault_saturated_gyroscope);
        }
		tracer->report(60);
	}
}
