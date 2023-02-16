#include "pigeon2_interface/pigeon2_state_interface.h"

namespace hardware_interface
{
namespace pigeon2
{

Pigeon2HWState::Pigeon2HWState(int device_number)
	: device_number_{device_number}
{
}

int Pigeon2HWState::getDeviceNumber(void) const
{
	return device_number_;
}

void Pigeon2HWState::setMountPoseForward(AxisDirection direction)
{
	mount_pose_forward_ = direction;
}
AxisDirection Pigeon2HWState::getMountPoseForward(void) const
{
	return mount_pose_forward_;
}

void Pigeon2HWState::setMountPoseUp(AxisDirection direction)
{
	mount_pose_up_ = direction;
}
AxisDirection Pigeon2HWState::getMountPoseUp(void) const
{
	return mount_pose_up_;
}

void Pigeon2HWState::setMountPoseRoll(double direction)
{
		mount_pose_roll_ = direction;
}
double Pigeon2HWState::getMountPoseRoll(void) const
{
	return mount_pose_roll_;
}

void Pigeon2HWState::setMountPosePitch(double direction)
{
	mount_pose_pitch_ = direction;
}
double Pigeon2HWState::getMountPosePitch(void) const
{
	return mount_pose_pitch_;
}

void Pigeon2HWState::setMountPoseYaw(double direction)
{
		mount_pose_yaw_ = direction;
}
double Pigeon2HWState::getMountPoseYaw(void) const
{
	return mount_pose_yaw_;
}

void Pigeon2HWState::setXAxisGyroError(double x_axis_gyro_error)
{
	x_axis_gyro_error_ = x_axis_gyro_error;
}
double Pigeon2HWState::getXAxisGyroError(void) const
{
	return x_axis_gyro_error_;
}

void Pigeon2HWState::setYAxisGyroError(double y_axis_gyro_error)
{
	y_axis_gyro_error_ = y_axis_gyro_error;
}
double Pigeon2HWState::getYAxisGyroError(void) const
{
	return y_axis_gyro_error_;
}

void Pigeon2HWState::setZAxisGyroError(double z_axis_gyro_error)
{
	z_axis_gyro_error_ = z_axis_gyro_error;
}
double Pigeon2HWState::getZAxisGyroError(void) const
{
	return z_axis_gyro_error_;
}

void Pigeon2HWState::setCompassEnable(bool compass_enable)
{
	compass_enable_ = compass_enable;
}
bool Pigeon2HWState::getCompassEnable(void) const
{
	return compass_enable_;
}

void Pigeon2HWState::setDisableTemperatureCompensation(bool temperature_compensation)
{
	disable_temperature_compensation_ = temperature_compensation;
}
bool Pigeon2HWState::getDisableTemperatureCompensation(void) const
{
	return disable_temperature_compensation_;
}

void Pigeon2HWState::setDisableNoMotionCalibration(bool disable_no_motions_calibration)
{
	disable_no_motion_calibration_ = disable_no_motions_calibration;
}
bool Pigeon2HWState::getDisableNoMotionCalibration(void) const
{
	return disable_no_motion_calibration_;
}

void Pigeon2HWState::setGravityVector(const std::array<double, 3> &gravity_vector)
{
	gravity_vector_ = gravity_vector;
}
std::array<double, 3> Pigeon2HWState::getGravityVector(void) const
{
	return gravity_vector_;
}

void Pigeon2HWState::set6dQuaternion(std::array<double, 4> &quaternion)
{
	quaternion_6d_ = quaternion;
}
std::array<double, 4> Pigeon2HWState::get6dQuatention(void) const
{
	return quaternion_6d_;
}

void Pigeon2HWState::setYaw(double yaw)
{
	yaw_ = yaw;
}
double Pigeon2HWState::getYaw(void) const
{
	return yaw_;
}

void Pigeon2HWState::setPitch(double pitch)
{
	pitch_ = pitch;
}
double Pigeon2HWState::getPitch(void) const
{
	return pitch_;
}

void Pigeon2HWState::setRoll(double roll)
{
	roll_ = roll;
}
double Pigeon2HWState::getRoll(void) const
{
	return roll_;
}

void Pigeon2HWState::setAccumGyro(std::array<double, 3> accum_gyro)
{
	accum_gyro_ = accum_gyro;
}
std::array<double, 3> Pigeon2HWState::getAccumGyro(void) const
{
	return accum_gyro_;
}

void Pigeon2HWState::setAbsoluteCompassHeading(double absolute_compass_heading)
{
	absolute_compass_heading_ = absolute_compass_heading;
}
double Pigeon2HWState::getAbsoluteCompassHeading(void) const
{
	return absolute_compass_heading_;
}

void Pigeon2HWState::setCompassHeading(double compass_heading)
{
	compass_heading_ = compass_heading;
}
double Pigeon2HWState::getCompassHeading(void) const
{
	return compass_heading_;
}

void Pigeon2HWState::setCompassFieldStrength(double compass_field_strength)
{
	compass_field_strength_ = compass_field_strength;
}
double Pigeon2HWState::getCompassFieldStrength(void) const
{
	return compass_field_strength_;
}

void Pigeon2HWState::setTemperature(double temperature)
{
	temperature_ = temperature;
}
double Pigeon2HWState::getTemperature(void) const
{
	return temperature_;
}

void Pigeon2HWState::setUptime(uint32_t uptime)
{
	uptime_ = uptime;
}
uint32_t Pigeon2HWState::getUptime(void) const
{
	return uptime_;
}

void Pigeon2HWState::setRawMagnetometer(std::array<int16_t, 3> &raw_magnetometer)
{
	raw_magnetometer_xyz_ = raw_magnetometer;
}
std::array<int16_t, 3> Pigeon2HWState::getRawMagnetometer(void) const
{
	return raw_magnetometer_xyz_;
}

void Pigeon2HWState::setBiasedMagnetometer(std::array<int16_t, 3> &biased_magnetometer)
{
	biased_magnetometer_xyz_ = biased_magnetometer;
}
std::array<int16_t, 3> Pigeon2HWState::getBiasedMagnetometer(void) const
{
	return biased_magnetometer_xyz_;
}

void Pigeon2HWState::setBiasedAccelerometer(std::array<int16_t, 3> &biased_accelerometer)
{
	biased_accelerometer_xyz_ = biased_accelerometer;
}
std::array<int16_t, 3> Pigeon2HWState::getBiasedAccelerometer(void) const
{
	return biased_accelerometer_xyz_;
}

void Pigeon2HWState::setRawGyro(std::array<double, 3> &raw_gyro)
{
	raw_gyro_ = raw_gyro;
}
std::array<double, 3> Pigeon2HWState::getRawGyro(void) const
{
	return raw_gyro_;
}

void Pigeon2HWState::setResetCount(uint32_t reset_count)
{
	reset_count_ = reset_count;
}
uint32_t Pigeon2HWState::getResetCount(void) const
{
	return reset_count_;
}

void Pigeon2HWState::setResetFlags(uint32_t reset_flags)
{
	reset_flags_ = reset_flags;
}
uint32_t Pigeon2HWState::getResetFlags(void) const
{
	return reset_flags_;
}

void Pigeon2HWState::setFirmwareVersion(uint32_t firmware_version)
{
	firmware_version_ = firmware_version;
}
uint32_t Pigeon2HWState::getFirmwareVersion(void) const
{
	return firmware_version_;
}

void Pigeon2HWState::setFaults(uint64_t faults)
{
	faults_ = faults;
}
uint64_t Pigeon2HWState::getFaults(void) const
{
	return faults_;
}

void Pigeon2HWState::setStickyFaults(uint64_t sticky_faults)
{
	sticky_faults_ = sticky_faults;
}
uint64_t Pigeon2HWState::getStickyFaults(void) const
{
	return sticky_faults_;
}

} //namespace hardware_interface
} // namespace cancoder
