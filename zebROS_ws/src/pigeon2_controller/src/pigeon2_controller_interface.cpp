#include "pigeon2_controller/pigeon2_controller_interface.h"

namespace pigeon2_controller_interface
{
Pigeon2CIParams::Pigeon2CIParams(ros::NodeHandle n)
	: DDRUpdater(n)
{
	// Set values to sensible defaults
	mount_pose_forward_ = hardware_interface::pigeon2::AxisDirection::Undefined;
	mount_pose_up_ = hardware_interface::pigeon2::AxisDirection::Undefined;
	mount_pose_roll_ = 0;
	mount_pose_pitch_ = 0;
	mount_pose_yaw_ = 0;
	x_axis_gyro_error_ = 0;
	y_axis_gyro_error_ = 0;
	z_axis_gyro_error_ = 0;
	compass_enable_ = false;
	disable_temperature_compensation_ = false;
	disable_no_motion_calibration_ = false;

	// Then read in values from config file
	// Only allow one or the other here - if both are defined, error out
	bool read_mount_pose_axis = false;
	read_mount_pose_axis |= readIntoEnum(n, "mount_pose_forward", axis_direction_enum_map_, mount_pose_forward_);
	read_mount_pose_axis |= readIntoEnum(n, "mount_pose_up", axis_direction_enum_map_, mount_pose_up_);
	use_mount_pose_axis_ = read_mount_pose_axis;
	bool read_mount_pose_rpy = false;
	read_mount_pose_rpy |= readIntoScalar(n, "mount_pose_roll", mount_pose_roll_);
	read_mount_pose_rpy |= readIntoScalar(n, "mount_pose_pitch", mount_pose_pitch_);
	read_mount_pose_rpy |= readIntoScalar(n, "mount_pose_yaw", mount_pose_yaw_);
	use_mount_pose_rpy_ = read_mount_pose_axis;

	// Then hook them up to dynamic reconfigure options
	ddr_.registerEnumVariable<int>("mount_pose_forward",
									[this](void) { return static_cast<int>(mount_pose_forward_.load(std::memory_order_acquire));}, 
								   boost::bind(&Pigeon2CIParams::setMountPoseAxis, this, _1, boost::ref(mount_pose_forward_), false),
								   "Mount Pose Forward",
								   axis_direction_enum_map_);
	ddr_.registerEnumVariable<int>("mount_pose_up",
									[this](void) { return static_cast<int>(mount_pose_up_.load(std::memory_order_acquire));}, 
								   boost::bind(&Pigeon2CIParams::setMountPoseAxis, this, _1, boost::ref(mount_pose_up_), false),
								   "Mount Pose Up",
								   axis_direction_enum_map_);
	ddr_.registerVariable<double>("mount_pose_roll",
									[this](void) { return mount_pose_roll_.load(std::memory_order_acquire);}, 
								   boost::bind(&Pigeon2CIParams::setMountPoseRPY, this, _1, boost::ref(mount_pose_roll_), false),
								   "Mount Pose Roll",
								   0., 360.);
	ddr_.registerVariable<double>("mount_pose_pitch",
									[this](void) { return mount_pose_pitch_.load(std::memory_order_acquire);}, 
								   boost::bind(&Pigeon2CIParams::setMountPoseRPY, this, _1, boost::ref(mount_pose_pitch_), false),
								   "Mount Pose pitch",
								   0., 360.);
	ddr_.registerVariable<double>("mount_pose_yaw",
									[this](void) { return mount_pose_yaw_.load(std::memory_order_acquire);}, 
								   boost::bind(&Pigeon2CIParams::setMountPoseRPY, this, _1, boost::ref(mount_pose_yaw_), false),
								   "Mount Pose Yaw",
								   0., 360.);
	ddr_.registerVariable<bool>("compass_enable",
									[this](void) { return compass_enable_.load(std::memory_order_acquire);}, 
								   boost::bind(&Pigeon2CIParams::setVar<bool>, this, _1, boost::ref(compass_enable_), false),
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
void Pigeon2CIParams::setMountPoseAxis(int direction, std::atomic<hardware_interface::pigeon2::AxisDirection> &val, bool update_dynamic)
{
	const auto mount_pose = static_cast<hardware_interface::pigeon2::AxisDirection>(direction);
	use_mount_pose_rpy_ = false;
	use_mount_pose_axis_ = true;
	setVar(mount_pose, val, update_dynamic);
}

void Pigeon2CIParams::setMountPoseRPY(double direction, std::atomic<double> &val, bool update_dynamic)
{
	use_mount_pose_axis_ = false;
	use_mount_pose_rpy_ = true;
	setVar(direction, val, update_dynamic);
}

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
	if (params_.use_mount_pose_axis_)
	{
		handle_->setMountPoseForward(params_.mount_pose_forward_.load(std::memory_order_acquire));
		handle_->setMountPoseUp(params_.mount_pose_up_.load(std::memory_order_acquire));
	}
	if (params_.use_mount_pose_rpy_)
	{
		handle_->setMountPoseRoll(params_.mount_pose_roll_.load(std::memory_order_acquire));
		handle_->setMountPosePitch(params_.mount_pose_pitch_.load(std::memory_order_acquire));
		handle_->setMountPoseYaw(params_.mount_pose_yaw_.load(std::memory_order_acquire));
	}
	handle_->setXAxisGyroError(params_.x_axis_gyro_error_.load(std::memory_order_acquire));
	handle_->setYAxisGyroError(params_.y_axis_gyro_error_.load(std::memory_order_acquire));
	handle_->setZAxisGyroError(params_.z_axis_gyro_error_.load(std::memory_order_acquire));
	handle_->setCompassEnable(params_.compass_enable_.load(std::memory_order_acquire));
	handle_->setDisableTemperatureCompensation(params_.disable_temperature_compensation_.load(std::memory_order_acquire));
	handle_->setDisableNoMotionCalibration(params_.disable_no_motion_calibration_.load(std::memory_order_acquire));

	if (zero_gyro_bias_now_.exchange(false))
	{
		handle_->setZeroGyroBiasNow();
	}
	if (clear_sticky_faults_.exchange(false))
	{
		handle_->setClearStickyFaults();
	}
	if (set_yaw_bool_.exchange(false))
	{
		handle_->setSetYaw(set_yaw_value_.load(std::memory_order_acquire));
	}
	if (add_yaw_bool_.exchange(false))
	{
		handle_->setAddYaw(add_yaw_value_.load(std::memory_order_acquire));
	}
	if (set_yaw_to_compass_.exchange(false))
	{
		handle_->setSetYawToCompass();
	}
	if (set_accum_z_angle_.exchange(false))
	{
		handle_->setSetAccumZAngle();
	}
}

void Pigeon2ControllerInterface::zeroGyroBiasNow(void)
{
	zero_gyro_bias_now_ = true;
}
void Pigeon2ControllerInterface::setClearStickyFaults(void)
{
	clear_sticky_faults_ = true;
}
void Pigeon2ControllerInterface::setYaw(double yaw)
{
	set_yaw_value_ = yaw;
	set_yaw_bool_ = true;
}
void Pigeon2ControllerInterface::addYaw(double yaw)
{
	add_yaw_value_ = yaw;
	add_yaw_bool_ = true;
}
void Pigeon2ControllerInterface::setYawToCompass(void)
{
	set_yaw_to_compass_ = true;
}
void Pigeon2ControllerInterface::setAccumZAngle(void)
{
	set_accum_z_angle_ = true;
}

#if 0
void Pigeon2ControllerInterface::setVelocityMeasPeriod(hardware_interface::pigeon2::SensorVelocityMeasPeriod velocity_meas_period)
{
	params_.setVelocityMeasPeriod(velocity_meas_period);
}
#endif
} // namespace pigeon2_controller_interface

