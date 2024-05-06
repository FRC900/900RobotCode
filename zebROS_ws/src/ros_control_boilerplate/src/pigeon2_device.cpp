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
    , mount_pose_configs_{std::make_unique<ctre::phoenix6::configs::MountPoseConfigs>()}
    , gyro_trim_configs_{std::make_unique<ctre::phoenix6::configs::GyroTrimConfigs>()}
    , pigeon2_features_configs_{std::make_unique<ctre::phoenix6::configs::Pigeon2FeaturesConfigs>()}
    , read_thread_state_{nullptr}
    , read_state_mutex_{nullptr}
    , read_thread_{nullptr}
{
    ROS_INFO_STREAM_NAMED("frc_robot_interface",
                        "Loading joint " << joint_index << "=" << joint_name <<
                        (local_update_ ? " local" : " remote") << " update, " <<
                        (local_hardware_ ? "local" : "remote") << " hardware" <<
                        " as Pigeon2 " << can_id << " on bus " << can_bus);

    if (local_hardware_)
    {
        pigeon2_ = std::make_unique<ctre::phoenix6::hardware::core::CorePigeon2>(can_id, can_bus);
        setParentDevice(pigeon2_.get());
        read_thread_state_ = std::make_unique<hardware_interface::pigeon2::Pigeon2HWState>(can_id);
        read_state_mutex_ = std::make_unique<std::mutex>();
        read_thread_ = std::make_unique<std::jthread>(&Pigeon2Device::read_thread, this,
                                                      std::make_unique<Tracer>("pigeon2_read_" + joint_name + " " + name_space),
                                                      read_hz_);
    }
}

Pigeon2Device::~Pigeon2Device(void) = default;

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
    if (local_hardware_)
    {
        std::stringstream s;
        s << "imu_" << joint_index << "_in";
        sim_sub_ = nh.subscribe<nav_msgs::Odometry>(s.str(), 1, &Pigeon2Device::imuOdomCallback, this);
        pigeon2_->GetSimState().SetSupplyVoltage(units::volt_t{12.5});
    }
}

void Pigeon2Device::imuOdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    std::scoped_lock l(sim_odom_mutex_);
    sim_odom_ = *msg;
}

void Pigeon2Device::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (local_update_)
    {
        // Fill in the full pigeon2 state using the last
        // data read from the pigeon2 read thread for this hardware
        if (std::unique_lock l(*read_state_mutex_, std::try_to_lock); l.owns_lock())
        {
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
            state_->setFaultSaturatedAccelerometer(read_thread_state_->getFaultSaturatedAccelerometer());
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
            state_->setStickyFaultSaturatedAccelerometer(read_thread_state_->getStickyFaultSaturatedAccelerometer());
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
        {
            std::scoped_lock(sim_odom_mutex_);
            const geometry_msgs::Quaternion &q = sim_odom_.pose.pose.orientation;
            tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
            double roll;
            double pitch;
            double yaw;
            tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
            pigeon2_->GetSimState().SetRoll(units::radian_t{roll});
            pigeon2_->GetSimState().SetPitch(units::radian_t{pitch});
            pigeon2_->GetSimState().SetRawYaw(units::radian_t{yaw});
            // This is run after read(), so we overwite whatever ctre's internal state is with
            // the most recent sim orientation data
            imu_orientation_[3] = q.w;
            imu_orientation_[0] = q.x;
            imu_orientation_[1] = q.y;
            imu_orientation_[2] = q.z;

            imu_angular_velocity_[0] = sim_odom_.twist.twist.angular.x;
            imu_angular_velocity_[1] = sim_odom_.twist.twist.angular.y;
            imu_angular_velocity_[2] = sim_odom_.twist.twist.angular.z;

            // No idea what to do for linear acceleration - we could try to get a delta of linear velocity?
        }
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
        ROS_WARN_STREAM("Reset detected on Pigeon2 " << getName());

        //command_->resetSetYaw(); // TODO, not sure about this, there's no telling how long ago this value was correct
    }

    if (command_->mountPoseRPYChanged(mount_pose_configs_->MountPoseYaw,
                                      mount_pose_configs_->MountPosePitch,
                                      mount_pose_configs_->MountPoseRoll))
    {
        mount_pose_configs_->MountPoseYaw = angles::to_degrees(mount_pose_configs_->MountPoseYaw);
        mount_pose_configs_->MountPosePitch = angles::to_degrees(mount_pose_configs_->MountPosePitch);
        mount_pose_configs_->MountPoseRoll = angles::to_degrees(mount_pose_configs_->MountPoseRoll);
        if (safeCall(pigeon2_->GetConfigurator().Apply(*mount_pose_configs_), "ApplyConfig(MountPoseConfigs"))
        {
            ROS_INFO_STREAM("Updated Pigeon2 " << getId() << " = " << getName() << " MountPose " << *mount_pose_configs_);
            state_->setMountPoseYaw(angles::from_degrees(mount_pose_configs_->MountPoseYaw ));
            state_->setMountPosePitch(angles::from_degrees(mount_pose_configs_->MountPosePitch));
            state_->setMountPoseRoll(angles::from_degrees(mount_pose_configs_->MountPoseRoll));
        }
        else
        {
            command_->resetMountPoseRPY();
            return;
        }
    }

    if (command_->gyroTrimChanged(gyro_trim_configs_->GyroScalarX,
                                  gyro_trim_configs_->GyroScalarY,
                                  gyro_trim_configs_->GyroScalarZ))
    {
        gyro_trim_configs_->GyroScalarX = angles::to_degrees(gyro_trim_configs_->GyroScalarX);
        gyro_trim_configs_->GyroScalarY = angles::to_degrees(gyro_trim_configs_->GyroScalarY);
        gyro_trim_configs_->GyroScalarZ = angles::to_degrees(gyro_trim_configs_->GyroScalarZ);
        if (safeCall(pigeon2_->GetConfigurator().Apply(*gyro_trim_configs_), "ApplyConfig(GyroTrimConfigs"))
        {
            ROS_INFO_STREAM("Updated Pigeon2 " << getId() << " = " << getName() << " GyroTrim " << *gyro_trim_configs_);
            state_->setGyroTrimScalarX(angles::from_degrees(gyro_trim_configs_->GyroScalarX ));
            state_->setGyroTrimScalarY(angles::from_degrees(gyro_trim_configs_->GyroScalarY));
            state_->setGyroTrimScalarZ(angles::from_degrees(gyro_trim_configs_->GyroScalarZ));
        }
        else
        {
            command_->resetGyroTrim();
            return;
        }
    }

    if (command_->pigeon2FeaturesChanged(pigeon2_features_configs_->EnableCompass,
                                         pigeon2_features_configs_->DisableTemperatureCompensation,
                                         pigeon2_features_configs_->DisableNoMotionCalibration))
    {
        if (safeCall(pigeon2_->GetConfigurator().Apply(*pigeon2_features_configs_), "ApplyConfig(Pigeon2FeaturesConfigs"))
        {
            ROS_INFO_STREAM("Updated Pigeon2 " << getId() << " = " << getName() << " Pigeon2Features " << *pigeon2_features_configs_);
            state_->setEnableCompass(pigeon2_features_configs_->EnableCompass);
            state_->setDisableTemperatureCompensation(pigeon2_features_configs_->DisableTemperatureCompensation);
            state_->setDisableNoMotionCalibration(pigeon2_features_configs_->DisableNoMotionCalibration);
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

// Set of macros for common code used to read 
// all signals with error checking.
// MAKE_SIGNAL is just an easy way to create the signal object
// that's used for subsequent reads
#define MAKE_SIGNAL(var, function) \
auto var##_signal = function; \
signals.push_back(&var##_signal);

// SAFE_READ takes the signal short name, passes the 
// previously-created signal name to the base-class
// safeRead, and checks the returned std::optional
// var. If the var is not valid, don't run any
// additional code in the loop.
#define SAFE_READ(var, function) \
const auto var = safeRead(var##_signal, #function); \
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

    std::vector<ctre::phoenix6::BaseStatusSignal *> signals;
    MAKE_SIGNAL(version_major, pigeon2_->GetVersionMajor())
    MAKE_SIGNAL(version_minor, pigeon2_->GetVersionMinor())
    MAKE_SIGNAL(version_bugfix, pigeon2_->GetVersionBugfix())
    MAKE_SIGNAL(version_build, pigeon2_->GetVersionBuild())

    MAKE_SIGNAL(yaw, pigeon2_->GetYaw())
    MAKE_SIGNAL(pitch, pigeon2_->GetPitch())
    MAKE_SIGNAL(roll, pigeon2_->GetRoll())

    MAKE_SIGNAL(quat_w, pigeon2_->GetQuatW())
    MAKE_SIGNAL(quat_x, pigeon2_->GetQuatX())
    MAKE_SIGNAL(quat_y, pigeon2_->GetQuatY())
    MAKE_SIGNAL(quat_z, pigeon2_->GetQuatZ())

    MAKE_SIGNAL(gravity_vector_x, pigeon2_->GetGravityVectorX())
    MAKE_SIGNAL(gravity_vector_y, pigeon2_->GetGravityVectorY())
    MAKE_SIGNAL(gravity_vector_z, pigeon2_->GetGravityVectorZ())

    MAKE_SIGNAL(temperature, pigeon2_->GetTemperature())
    MAKE_SIGNAL(uptime, pigeon2_->GetUpTime())

    MAKE_SIGNAL(no_motion_count, pigeon2_->GetNoMotionCount())

    MAKE_SIGNAL(accum_gyro_x, pigeon2_->GetAccumGyroX())
    MAKE_SIGNAL(accum_gyro_y, pigeon2_->GetAccumGyroY())
    MAKE_SIGNAL(accum_gyro_z, pigeon2_->GetAccumGyroZ())

    MAKE_SIGNAL(angular_velocity_x, pigeon2_->GetAngularVelocityXWorld())
    MAKE_SIGNAL(angular_velocity_y, pigeon2_->GetAngularVelocityYWorld())
    MAKE_SIGNAL(angular_velocity_z, pigeon2_->GetAngularVelocityZWorld())

    MAKE_SIGNAL(acceleration_x, pigeon2_->GetAccelerationX())
    MAKE_SIGNAL(acceleration_y, pigeon2_->GetAccelerationY())
    MAKE_SIGNAL(acceleration_z, pigeon2_->GetAccelerationZ())

    MAKE_SIGNAL(supply_voltage, pigeon2_->GetSupplyVoltage())

    MAKE_SIGNAL(magnetic_field_x, pigeon2_->GetMagneticFieldX())
    MAKE_SIGNAL(magnetic_field_y, pigeon2_->GetMagneticFieldY())
    MAKE_SIGNAL(magnetic_field_z, pigeon2_->GetMagneticFieldZ())

    MAKE_SIGNAL(raw_magnetic_field_x, pigeon2_->GetRawMagneticFieldX())
    MAKE_SIGNAL(raw_magnetic_field_y, pigeon2_->GetRawMagneticFieldY())
    MAKE_SIGNAL(raw_magnetic_field_z, pigeon2_->GetRawMagneticFieldZ())

    MAKE_SIGNAL(fault_hardware, pigeon2_->GetFault_Hardware())
    MAKE_SIGNAL(fault_undervoltage, pigeon2_->GetFault_Undervoltage())
    MAKE_SIGNAL(fault_boot_during_enable, pigeon2_->GetFault_BootDuringEnable())
    MAKE_SIGNAL(fault_unlicensed_feature_in_use, pigeon2_->GetFault_UnlicensedFeatureInUse())
    MAKE_SIGNAL(fault_bootup_accelerometer, pigeon2_->GetFault_BootupAccelerometer())
    MAKE_SIGNAL(fault_bootup_gyroscope, pigeon2_->GetFault_BootupGyroscope())
    MAKE_SIGNAL(fault_bootup_magnetometer, pigeon2_->GetFault_BootupMagnetometer())
    MAKE_SIGNAL(fault_boot_into_motion, pigeon2_->GetFault_BootIntoMotion())
    MAKE_SIGNAL(fault_data_acquired_late, pigeon2_->GetFault_DataAcquiredLate())
    MAKE_SIGNAL(fault_loop_time_slow, pigeon2_->GetFault_LoopTimeSlow())
    MAKE_SIGNAL(fault_saturated_magnetometer, pigeon2_->GetFault_SaturatedMagnetometer())
    MAKE_SIGNAL(fault_saturated_Accelerometer, pigeon2_->GetFault_SaturatedAccelerometer())
    MAKE_SIGNAL(fault_saturated_gyroscope, pigeon2_->GetFault_SaturatedGyroscope())

    MAKE_SIGNAL(sticky_fault_hardware, pigeon2_->GetStickyFault_Hardware())
    MAKE_SIGNAL(sticky_fault_undervoltage, pigeon2_->GetStickyFault_Undervoltage())
    MAKE_SIGNAL(sticky_fault_boot_during_enable, pigeon2_->GetStickyFault_BootDuringEnable())
    MAKE_SIGNAL(sticky_fault_unlicensed_feature_in_use, pigeon2_->GetStickyFault_UnlicensedFeatureInUse())
    MAKE_SIGNAL(sticky_fault_bootup_accelerometer, pigeon2_->GetStickyFault_BootupAccelerometer())
    MAKE_SIGNAL(sticky_fault_bootup_gyroscope, pigeon2_->GetStickyFault_BootupGyroscope())
    MAKE_SIGNAL(sticky_fault_bootup_magnetometer, pigeon2_->GetStickyFault_BootupMagnetometer())
    MAKE_SIGNAL(sticky_fault_boot_into_motion, pigeon2_->GetStickyFault_BootIntoMotion())
    MAKE_SIGNAL(sticky_fault_data_acquired_late, pigeon2_->GetStickyFault_DataAcquiredLate())
    MAKE_SIGNAL(sticky_fault_loop_time_slow, pigeon2_->GetStickyFault_LoopTimeSlow())
    MAKE_SIGNAL(sticky_fault_saturated_magnetometer, pigeon2_->GetStickyFault_SaturatedMagnetometer())
    MAKE_SIGNAL(sticky_fault_saturated_Accelerometer, pigeon2_->GetStickyFault_SaturatedAccelerometer())
    MAKE_SIGNAL(sticky_fault_saturated_gyroscope, pigeon2_->GetStickyFault_SaturatedGyroscope())

	for (ros::Rate r(poll_frequency); ros::ok(); r.sleep())
	{
		tracer->start("pigeon2 read main_loop");

        ctre::phoenix6::BaseStatusSignal::WaitForAll(units::second_t{0}, signals);

        SAFE_READ(version_major, pigeon2_->GetVersionMajor())
        SAFE_READ(version_minor, pigeon2_->GetVersionMinor())
        SAFE_READ(version_bugfix, pigeon2_->GetVersionBugfix())
        SAFE_READ(version_build, pigeon2_->GetVersionBuild())

        SAFE_READ(yaw, pigeon2_->GetYaw())
        SAFE_READ(pitch, pigeon2_->GetPitch())
        SAFE_READ(roll, pigeon2_->GetRoll())

        SAFE_READ(quat_w, pigeon2_->GetQuatW())
        SAFE_READ(quat_x, pigeon2_->GetQuatX())
        SAFE_READ(quat_y, pigeon2_->GetQuatY())
        SAFE_READ(quat_z, pigeon2_->GetQuatZ())

        SAFE_READ(gravity_vector_x, pigeon2_->GetGravityVectorX())
        SAFE_READ(gravity_vector_y, pigeon2_->GetGravityVectorY())
        SAFE_READ(gravity_vector_z, pigeon2_->GetGravityVectorZ())

        SAFE_READ(temperature, pigeon2_->GetTemperature())
        SAFE_READ(uptime, pigeon2_->GetUpTime())

        SAFE_READ(no_motion_count, pigeon2_->GetNoMotionCount())

        SAFE_READ(accum_gyro_x, pigeon2_->GetAccumGyroX())
        SAFE_READ(accum_gyro_y, pigeon2_->GetAccumGyroY())
        SAFE_READ(accum_gyro_z, pigeon2_->GetAccumGyroZ())

        SAFE_READ(angular_velocity_x, pigeon2_->GetAngularVelocityXWorld())
        SAFE_READ(angular_velocity_y, pigeon2_->GetAngularVelocityYWorld())
        SAFE_READ(angular_velocity_z, pigeon2_->GetAngularVelocityZWorld())

        SAFE_READ(acceleration_x, pigeon2_->GetAccelerationX())
        SAFE_READ(acceleration_y, pigeon2_->GetAccelerationY())
        SAFE_READ(acceleration_z, pigeon2_->GetAccelerationZ())

        SAFE_READ(supply_voltage, pigeon2_->GetSupplyVoltage())

        SAFE_READ(magnetic_field_x, pigeon2_->GetMagneticFieldX())
        SAFE_READ(magnetic_field_y, pigeon2_->GetMagneticFieldY())
        SAFE_READ(magnetic_field_z, pigeon2_->GetMagneticFieldZ())

        SAFE_READ(raw_magnetic_field_x, pigeon2_->GetRawMagneticFieldX())
        SAFE_READ(raw_magnetic_field_y, pigeon2_->GetRawMagneticFieldY())
        SAFE_READ(raw_magnetic_field_z, pigeon2_->GetRawMagneticFieldZ())

        SAFE_READ(fault_hardware, pigeon2_->GetFault_Hardware())
        SAFE_READ(fault_undervoltage, pigeon2_->GetFault_Undervoltage())
        SAFE_READ(fault_boot_during_enable, pigeon2_->GetFault_BootDuringEnable())
        SAFE_READ(fault_unlicensed_feature_in_use, pigeon2_->GetFault_UnlicensedFeatureInUse())
        SAFE_READ(fault_bootup_accelerometer, pigeon2_->GetFault_BootupAccelerometer())
        SAFE_READ(fault_bootup_gyroscope, pigeon2_->GetFault_BootupGyroscope())
        SAFE_READ(fault_bootup_magnetometer, pigeon2_->GetFault_BootupMagnetometer())
        SAFE_READ(fault_boot_into_motion, pigeon2_->GetFault_BootIntoMotion())
        SAFE_READ(fault_data_acquired_late, pigeon2_->GetFault_DataAcquiredLate())
        SAFE_READ(fault_loop_time_slow, pigeon2_->GetFault_LoopTimeSlow())
        SAFE_READ(fault_saturated_magnetometer, pigeon2_->GetFault_SaturatedMagnetometer())
        SAFE_READ(fault_saturated_Accelerometer, pigeon2_->GetFault_SaturatedAccelerometer())
        SAFE_READ(fault_saturated_gyroscope, pigeon2_->GetFault_SaturatedGyroscope())

        SAFE_READ(sticky_fault_hardware, pigeon2_->GetStickyFault_Hardware())
        SAFE_READ(sticky_fault_undervoltage, pigeon2_->GetStickyFault_Undervoltage())
        SAFE_READ(sticky_fault_boot_during_enable, pigeon2_->GetStickyFault_BootDuringEnable())
        SAFE_READ(sticky_fault_unlicensed_feature_in_use, pigeon2_->GetStickyFault_UnlicensedFeatureInUse())
        SAFE_READ(sticky_fault_bootup_accelerometer, pigeon2_->GetStickyFault_BootupAccelerometer())
        SAFE_READ(sticky_fault_bootup_gyroscope, pigeon2_->GetStickyFault_BootupGyroscope())
        SAFE_READ(sticky_fault_bootup_magnetometer, pigeon2_->GetStickyFault_BootupMagnetometer())
        SAFE_READ(sticky_fault_boot_into_motion, pigeon2_->GetStickyFault_BootIntoMotion())
        SAFE_READ(sticky_fault_data_acquired_late, pigeon2_->GetStickyFault_DataAcquiredLate())
        SAFE_READ(sticky_fault_loop_time_slow, pigeon2_->GetStickyFault_LoopTimeSlow())
        SAFE_READ(sticky_fault_saturated_magnetometer, pigeon2_->GetStickyFault_SaturatedMagnetometer())
        SAFE_READ(sticky_fault_saturated_Accelerometer, pigeon2_->GetStickyFault_SaturatedAccelerometer())
        SAFE_READ(sticky_fault_saturated_gyroscope, pigeon2_->GetStickyFault_SaturatedGyroscope())

        // Actually update the Pigeon2HWState shared between
		// this thread and read()
		// Do this all at once so the code minimizes the amount
		// of time with mutex locked
		{
			// Lock the state entry to make sure writes
			// are atomic - reads won't grab data in
			// the middle of a write
			std::lock_guard l(*read_state_mutex_);
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
            read_thread_state_->setNoMotionCount(no_motion_count->value());
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
            read_thread_state_->setFaultSaturatedAccelerometer(*fault_saturated_Accelerometer);
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
            read_thread_state_->setStickyFaultSaturatedAccelerometer(*sticky_fault_saturated_Accelerometer);
            read_thread_state_->setStickyFaultSaturatedGyroscope(*sticky_fault_saturated_gyroscope);
        }
		tracer->report(60);
	}
}
