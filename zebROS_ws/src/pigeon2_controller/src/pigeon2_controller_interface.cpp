#include "pigeon2_controller/pigeon2_controller_interface.h"

namespace pigeon2_controller_interface
{
Pigeon2CIParams::Pigeon2CIParams(ros::NodeHandle n)
	: DDRUpdater(n)
{
	// read in values from config file
	// Only allow one or the other here - if both are defined, error out
	readIntoScalar(n, "mount_pose_yaw", mount_pose_yaw_);
	readIntoScalar(n, "mount_pose_pitch", mount_pose_pitch_);
	readIntoScalar(n, "mount_pose_roll", mount_pose_roll_);
	readIntoScalar(n, "gyro_trim_scalar_x", gyro_trim_scalar_x_);
	readIntoScalar(n, "gyro_trim_scalar_y", gyro_trim_scalar_y_);
	readIntoScalar(n, "gyro_trim_scalar_z", gyro_trim_scalar_z_);
	readIntoScalar(n, "enable_compass", enable_compass_);
	readIntoScalar(n, "disable_temperature_compensation", disable_temperature_compensation_);
	readIntoScalar(n, "disable_no_motion_calibration_", disable_no_motion_calibration_);

	// Then hook them up to dynamic reconfigure options
	ddr_.registerVariable<double>("mount_pose_yaw",
								  [this](void) { return mount_pose_yaw_.load(std::memory_order_acquire);}, 
								  boost::bind(&Pigeon2CIParams::setVar<double>, this, _1, boost::ref(mount_pose_yaw_), false),
								  "Mount Pose Yaw",
								  -2.0 * M_PI, 2.0 * M_PI);
	ddr_.registerVariable<double>("mount_pose_pitch",
								  [this](void) { return mount_pose_pitch_.load(std::memory_order_acquire);}, 
								  boost::bind(&Pigeon2CIParams::setVar<double>, this, _1, boost::ref(mount_pose_pitch_), false),
								  "Mount Pose pitch",
								  -2.0 * M_PI, 2.0 * M_PI);
	ddr_.registerVariable<double>("mount_pose_roll",
								  [this](void) { return mount_pose_roll_.load(std::memory_order_acquire);}, 
								  boost::bind(&Pigeon2CIParams::setVar<double>, this, _1, boost::ref(mount_pose_roll_), false),
								  "Mount Pose Roll",
								  -2.0 * M_PI, 2.0 * M_PI);
	ddr_.registerVariable<double>("gyro_trim_scalar_x",
								  [this](void) { return gyro_trim_scalar_x_.load(std::memory_order_acquire);}, 
								  boost::bind(&Pigeon2CIParams::setVar<double>, this, _1, boost::ref(gyro_trim_scalar_x_), false),
								  "Gyro Trim Scalar X",
								  -2.0 * M_PI, 2.0 * M_PI);
	ddr_.registerVariable<double>("gyro_trim_scalar_y",
								  [this](void) { return gyro_trim_scalar_y_.load(std::memory_order_acquire);}, 
								  boost::bind(&Pigeon2CIParams::setVar<double>, this, _1, boost::ref(gyro_trim_scalar_y_), false),
								  "Gyro Trim Scalar Y",
								  -2.0 * M_PI, 2.0 * M_PI);
	ddr_.registerVariable<double>("gyro_trim_scalar_z",
								  [this](void) { return gyro_trim_scalar_z_.load(std::memory_order_acquire);}, 
								  boost::bind(&Pigeon2CIParams::setVar<double>, this, _1, boost::ref(gyro_trim_scalar_z_), false),
								  "Gyro Trim Scalar Z",
								  -2.0 * M_PI, 2.0 * M_PI);
	ddr_.registerVariable<bool>("compass_enable",
								[this](void) { return enable_compass_.load(std::memory_order_acquire);}, 
								boost::bind(&Pigeon2CIParams::setVar<bool>, this, _1, boost::ref(enable_compass_), false),
								"Enable Compass");
	ddr_.registerVariable<bool>("disable_temperature_compensation",
								[this](void) { return disable_temperature_compensation_.load(std::memory_order_acquire);}, 
								boost::bind(&Pigeon2CIParams::setVar<bool>, this, _1, boost::ref(disable_temperature_compensation_), false),
								"Disable Temperature Compensation");
	ddr_.registerVariable<bool>("disable_no_motion_calibration",
								[this](void) { return disable_no_motion_calibration_.load(std::memory_order_acquire);}, 
								boost::bind(&Pigeon2CIParams::setVar<bool>, this, _1, boost::ref(disable_no_motion_calibration_), false),
								"Disable No-Motion Calibration");
	ddr_.publishServicesTopics();
}

// Functions to update params from either DDR callbacks or the interface.
// First arg holds the value to set
// Second arg is true if coming from the interface, meaning it needs
// to force an update to the dynamic interface values to stay in sync
// Second arg is false if the source of the call is a DDR update. In
// that case, don't update the dynamic interface value since
// the value has already been updated there.
template <class T>
void Pigeon2CIParams::setVar(T input, std::atomic<T> &val, bool update_dynamic)
{
	const bool publish_update = update_dynamic && (input != val);
	val.store(input, std::memory_order_release);
	if (publish_update)
	{
		triggerDDRUpdate();
	}
}

Pigeon2ControllerInterface::Pigeon2ControllerInterface(ros::NodeHandle &n, const std::string &joint_name, hardware_interface::pigeon2::Pigeon2CommandHandle handle)
	: params_(ros::NodeHandle(n, joint_name))
	, handle_(handle)
{
}

void Pigeon2ControllerInterface::update(void)
{
	handle_->setMountPoseRoll(params_.mount_pose_roll_.load(std::memory_order_acquire));
	handle_->setMountPosePitch(params_.mount_pose_pitch_.load(std::memory_order_acquire));
	handle_->setMountPoseYaw(params_.mount_pose_yaw_.load(std::memory_order_acquire));
	handle_->setGyroTrimScalarX(params_.gyro_trim_scalar_x_.load(std::memory_order_acquire));
	handle_->setGyroTrimScalarY(params_.gyro_trim_scalar_y_.load(std::memory_order_acquire));
	handle_->setGyroTrimScalarZ(params_.gyro_trim_scalar_z_.load(std::memory_order_acquire));
	handle_->setEnableCompass(params_.enable_compass_.load(std::memory_order_acquire));
	handle_->setDisableTemperatureCompensation(params_.disable_temperature_compensation_.load(std::memory_order_acquire));
	handle_->setDisableNoMotionCalibration(params_.disable_no_motion_calibration_.load(std::memory_order_acquire));

	// These are 1-shots, that is, only run once when their values are set
	// Do an atomic exchange with false to clear them each time
	if (clear_sticky_faults_.exchange(false))
	{
		handle_->setClearStickyFaults();
	}
	if (set_yaw_bool_.exchange(false))
	{
		handle_->setSetYaw(set_yaw_value_.load(std::memory_order_acquire));
	}
}

void Pigeon2ControllerInterface::setClearStickyFaults(void)
{
	clear_sticky_faults_ = true;
}
void Pigeon2ControllerInterface::setYaw(const double yaw)
{
	set_yaw_value_ = yaw;
	set_yaw_bool_ = true;
}

void Pigeon2ControllerInterface::setDisableNoMotionCalibration(const bool flag)
{
	params_.disable_no_motion_calibration_.store(flag, std::memory_order_release);
}

#if 0
void Pigeon2ControllerInterface::setVelocityMeasPeriod(hardware_interface::pigeon2::SensorVelocityMeasPeriod velocity_meas_period)
{
	params_.setVelocityMeasPeriod(velocity_meas_period);
}
#endif
} // namespace pigeon2_controller_interface

