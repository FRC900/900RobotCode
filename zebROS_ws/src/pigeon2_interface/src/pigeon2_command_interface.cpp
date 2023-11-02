#include "pigeon2_interface/pigeon2_command_interface.h"

namespace hardware_interface::pigeon2
{

Pigeon2HWCommand::Pigeon2HWCommand(void) = default;

void Pigeon2HWCommand::setMountPoseYaw(const double direction)
{
	if (mount_pose_yaw_ != direction)
	{
		mount_pose_yaw_ = direction;
		mount_pose_rpy_changed_ = true;
	}
}
double Pigeon2HWCommand::getMountPoseYaw(void) const
{
	return mount_pose_yaw_;

}

void Pigeon2HWCommand::setMountPosePitch(const double direction)
{
	if (mount_pose_pitch_ != direction)
	{
		mount_pose_pitch_ = direction;
		mount_pose_rpy_changed_ = true;
	}
}
double Pigeon2HWCommand::getMountPosePitch(void) const
{
	return mount_pose_pitch_;
}

void Pigeon2HWCommand::setMountPoseRoll(const double direction)
{
	if (mount_pose_roll_ != direction)
	{
		mount_pose_roll_ = direction;
		mount_pose_rpy_changed_ = true;
	}
}
double Pigeon2HWCommand::getMountPoseRoll(void) const
{
	return mount_pose_roll_;
}


bool Pigeon2HWCommand::mountPoseRPYChanged(double &yaw, double &pitch, double &roll)
{
	yaw = mount_pose_yaw_;
	pitch = mount_pose_pitch_;
	roll = mount_pose_roll_;
	const auto ret = mount_pose_rpy_changed_;
	mount_pose_rpy_changed_ = false;
	return ret;
}

void Pigeon2HWCommand::resetMountPoseRPY(void)
{
	mount_pose_rpy_changed_ = true;
}

void Pigeon2HWCommand::setGyroTrimScalarX(const double gyro_trim_scalar_x)
{
    if (gyro_trim_scalar_x != gyro_trim_scalar_x_)
	{
		gyro_trim_scalar_x_ = gyro_trim_scalar_x;
		gyro_trim_changed_ = true;
	}
}
double Pigeon2HWCommand::getGyroTrimScalarX(void) const
{
	return gyro_trim_scalar_x_;
}

void Pigeon2HWCommand::setGyroTrimScalarY(const double gyro_trim_scalar_y)
{
    if (gyro_trim_scalar_y != gyro_trim_scalar_y_)
	{
		gyro_trim_scalar_y_ = gyro_trim_scalar_y;
		gyro_trim_changed_ = true;
	}
}
double Pigeon2HWCommand::getGyroTrimScalarY(void) const
{
	return gyro_trim_scalar_y_;
}

void Pigeon2HWCommand::setGyroTrimScalarZ(const double gyro_trim_scalar_z)
{
    if (gyro_trim_scalar_z != gyro_trim_scalar_z_)
	{
		gyro_trim_scalar_z_ = gyro_trim_scalar_z;
		gyro_trim_changed_ = true;
	}
}
double Pigeon2HWCommand::getGyroTrimScalarZ(void) const
{
	return gyro_trim_scalar_z_;
}

bool Pigeon2HWCommand::gyroTrimChanged(double &gyro_trim_scalar_x,
									   double &gyro_trim_scalar_y,
									   double &gyro_trim_scalar_z)
{
	gyro_trim_scalar_x = gyro_trim_scalar_x_;
	gyro_trim_scalar_y = gyro_trim_scalar_y_;
	gyro_trim_scalar_z = gyro_trim_scalar_z_;
	const auto ret = gyro_trim_changed_;
	gyro_trim_changed_ = false;
	return ret;
}
void Pigeon2HWCommand::resetGyroTrim(void)
{
	gyro_trim_changed_ = true;
}


void Pigeon2HWCommand::setEnableCompass(const bool enable_compass)
{
	if (enable_compass_ != enable_compass)
	{
		enable_compass_ = enable_compass;
		pigeon2_features_changed_ = true;
	}
}
bool Pigeon2HWCommand::getEnableCompass(void) const
{
	return enable_compass_;
}

void Pigeon2HWCommand::setDisableTemperatureCompensation(const bool temperature_compensation)
{
    if (disable_temperature_compensation_ != temperature_compensation)
	{
		disable_temperature_compensation_ = temperature_compensation;
		pigeon2_features_changed_ = true;
	}
}
bool Pigeon2HWCommand::getDisableTemperatureCompensation(void) const
{
	return disable_temperature_compensation_;
}

void Pigeon2HWCommand::setDisableNoMotionCalibration(const bool disable_no_motions_calibration)
{
    if (disable_no_motion_calibration_ != disable_no_motions_calibration)
	{
		disable_no_motion_calibration_ = disable_no_motions_calibration;
		pigeon2_features_changed_ = true;
	}
}
bool Pigeon2HWCommand::getDisableNoMotionCalibration(void) const
{
	return disable_no_motion_calibration_;
}
bool Pigeon2HWCommand::pigeon2FeaturesChanged(bool &enable_compass,
											  bool &disable_temperature_compensation,
											  bool &disable_no_motion_calibration)
{
	enable_compass = enable_compass_;
	disable_temperature_compensation = disable_temperature_compensation_;
	disable_no_motion_calibration = disable_no_motion_calibration_;
	const auto ret = pigeon2_features_changed_;
	pigeon2_features_changed_ = false;
	return ret;
}
void Pigeon2HWCommand::resetPigeon2Features(void)
{
	pigeon2_features_changed_ = true;
}

void Pigeon2HWCommand::setSetYaw(double yaw)
{
	set_yaw_ = yaw;
	set_yaw_changed_ = true;
}
double Pigeon2HWCommand::getSetYaw(void) const
{
	return set_yaw_;
}
bool Pigeon2HWCommand::setYawChanged(double &yaw)
{
	yaw = set_yaw_;
	auto ret = set_yaw_changed_;
	set_yaw_changed_ = false;
	return ret;
}
void Pigeon2HWCommand::resetSetYaw(void)
{
	set_yaw_changed_ = true;
}

void Pigeon2HWCommand::setClearStickyFaults(void)
{
	clear_sticky_faults_ = true;
}
bool Pigeon2HWCommand::getClearStickyFaults(void) const
{
	return clear_sticky_faults_;
}
bool Pigeon2HWCommand::clearStickyFaultsChanged(void)
{
	auto ret = clear_sticky_faults_;
	clear_sticky_faults_ = false;
	return ret;
}

} // namespace
