#include "pigeon2_interface/pigeon2_state_interface.h"

namespace hardware_interface::pigeon2
{

Pigeon2HWState::Pigeon2HWState(const int device_number)
	: device_number_{device_number}
{
}

int Pigeon2HWState::getDeviceNumber(void) const
{
	return device_number_;
}

void Pigeon2HWState::setMountPosePitch(const double direction)
{
	mount_pose_pitch_ = direction;
}
double Pigeon2HWState::getMountPosePitch(void) const
{
	return mount_pose_pitch_;
}

void Pigeon2HWState::setMountPoseRoll(const double direction)
{
		mount_pose_roll_ = direction;
}
double Pigeon2HWState::getMountPoseRoll(void) const
{
	return mount_pose_roll_;
}

void Pigeon2HWState::setMountPoseYaw(const double direction)
{
		mount_pose_yaw_ = direction;
}
double Pigeon2HWState::getMountPoseYaw(void) const
{
	return mount_pose_yaw_;
}

void Pigeon2HWState::setGyroTrimScalarX(const double gyro_trim_scalar_x)
{
	gyro_trim_scalar_x_ = gyro_trim_scalar_x;
}
double Pigeon2HWState::getGyroTrimScalarX(void) const
{
	return gyro_trim_scalar_x_;
}

void Pigeon2HWState::setGyroTrimScalarY(const double gyro_trim_scalar_y)
{
	gyro_trim_scalar_y_ = gyro_trim_scalar_y;
}
double Pigeon2HWState::getGyroTrimScalarY(void) const
{
	return gyro_trim_scalar_y_;
}

void Pigeon2HWState::setGyroTrimScalarZ(const double gyro_trim_scalar_z)
{
	gyro_trim_scalar_z_ = gyro_trim_scalar_z;
}
double Pigeon2HWState::getGyroTrimScalarZ(void) const
{
	return gyro_trim_scalar_z_;
}

void Pigeon2HWState::setEnableCompass(const bool enable_compass)
{
	enable_compass_ = enable_compass;
}
bool Pigeon2HWState::getEnableCompass(void) const
{
	return enable_compass_;
}

void Pigeon2HWState::setDisableTemperatureCompensation(const bool disable_temperature_compensation)
{
	disable_temperature_compensation_ = disable_temperature_compensation;
}
bool Pigeon2HWState::getDisableTemperatureCompensation(void) const
{
	return disable_temperature_compensation_;
}

void Pigeon2HWState::setDisableNoMotionCalibration(const bool disable_no_motions_calibration)
{
	disable_no_motion_calibration_ = disable_no_motions_calibration;
}
bool Pigeon2HWState::getDisableNoMotionCalibration(void) const
{
	return disable_no_motion_calibration_;
}

void Pigeon2HWState::setNoMotionCount(const int no_motion_count)
{
	no_motion_count_ = no_motion_count;
}
int Pigeon2HWState::getNoMotionCount(void) const
{
	return no_motion_count_;
}

void Pigeon2HWState::setVersionMajor(const int version_major)
{
	version_major_ = version_major;
}
int Pigeon2HWState::getVersionMajor(void) const
{
	return version_major_;
}

void Pigeon2HWState::setVersionMinor(const int version_minor)
{
	version_minor_ = version_minor;
}
int Pigeon2HWState::getVersionMinor(void) const
{
	return version_minor_;
}

void Pigeon2HWState::setVersionBugfix(const int version_bugfix)
{
	version_bugfix_ = version_bugfix;
}
int Pigeon2HWState::getVersionBugfix(void) const
{
	return version_bugfix_;
}

void Pigeon2HWState::setVersionBuild(const int version_build)
{
	version_build_ = version_build;
}
int Pigeon2HWState::getVersionBuild(void) const
{
	return version_build_;
}

void Pigeon2HWState::setYaw(const double yaw)
{
	yaw_ = yaw;
}
double Pigeon2HWState::getYaw(void) const
{
	return yaw_;
}

void Pigeon2HWState::setPitch(const double pitch)
{
	pitch_ = pitch;
}
double Pigeon2HWState::getPitch(void) const
{
	return pitch_;
}

void Pigeon2HWState::setRoll(const double roll)
{
	roll_ = roll;
}
double Pigeon2HWState::getRoll(void) const
{
	return roll_;
}

void Pigeon2HWState::setQuatW(const double quat_w)
{
	quat_w_ = quat_w;
}
double Pigeon2HWState::getQuatW(void) const
{
	return quat_w_;
}

void Pigeon2HWState::setQuatX(const double quat_x)
{
	quat_x_ = quat_x;
}
double Pigeon2HWState::getQuatX(void) const
{
	return quat_x_;
}

void Pigeon2HWState::setQuatY(const double quat_y)
{
	quat_y_ = quat_y;
}
double Pigeon2HWState::getQuatY(void) const
{
	return quat_y_;
}

void Pigeon2HWState::setQuatZ(const double quat_z)
{
	quat_z_ = quat_z;
}
double Pigeon2HWState::getQuatZ(void) const
{
	return quat_z_;
}

void Pigeon2HWState::setGravityVectorX(const double gravity_vector_x)
{
	gravity_vector_x_ = gravity_vector_x;
}
double Pigeon2HWState::getGravityVectorX(void) const
{
	return gravity_vector_x_;
}

void Pigeon2HWState::setGravityVectorY(const double gravity_vector_y)
{
	gravity_vector_y_ = gravity_vector_y;
}
double Pigeon2HWState::getGravityVectorY(void) const
{
	return gravity_vector_y_;
}

void Pigeon2HWState::setGravityVectorZ(const double gravity_vector_z)
{
	gravity_vector_z_ = gravity_vector_z;
}
double Pigeon2HWState::getGravityVectorZ(void) const
{
	return gravity_vector_z_;
}

void Pigeon2HWState::setTemperature(const double temperature)
{
	temperature_ = temperature;
}
double Pigeon2HWState::getTemperature(void) const
{
	return temperature_;
}

void Pigeon2HWState::setUptime(const double uptime)
{
	uptime_ = uptime;
}
double Pigeon2HWState::getUptime(void) const
{
	return uptime_;
}

void Pigeon2HWState::setAccumGyroX(const double accum_gyro_x)
{
	accum_gyro_x_ = accum_gyro_x;
}
double Pigeon2HWState::getAccumGyroX(void) const
{
	return accum_gyro_x_;
}

void Pigeon2HWState::setAccumGyroY(const double accum_gyro_y)
{
	accum_gyro_y_ = accum_gyro_y;
}
double Pigeon2HWState::getAccumGyroY(void) const
{
	return accum_gyro_y_;
}

void Pigeon2HWState::setAccumGyroZ(const double accum_gyro_z)
{
	accum_gyro_z_ = accum_gyro_z;
}
double Pigeon2HWState::getAccumGyroZ(void) const
{
	return accum_gyro_z_;
}

void Pigeon2HWState::setAngularVelocityX(const double angular_velocity_x)
{
	angular_velocity_x_ = angular_velocity_x;
}
double Pigeon2HWState::getAngularVelocityX(void) const
{
	return angular_velocity_x_;
}

void Pigeon2HWState::setAngularVelocityY(const double angular_velocity_y)
{
	angular_velocity_y_ = angular_velocity_y;
}
double Pigeon2HWState::getAngularVelocityY(void) const
{
	return angular_velocity_y_;
}

void Pigeon2HWState::setAngularVelocityZ(const double angular_velocity_z)
{
	angular_velocity_z_ = angular_velocity_z;
}
double Pigeon2HWState::getAngularVelocityZ(void) const
{
	return angular_velocity_z_;
}

void Pigeon2HWState::setAccelerationX(const double acceleration_x)
{
	acceleration_x_ = acceleration_x;
}
double Pigeon2HWState::getAccelerationX(void) const
{
	return acceleration_x_;
}

void Pigeon2HWState::setAccelerationY(const double acceleration_y)
{
	acceleration_y_ = acceleration_y;
}
double Pigeon2HWState::getAccelerationY(void) const
{
	return acceleration_y_;
}

void Pigeon2HWState::setAccelerationZ(const double acceleration_z)
{
	acceleration_z_ = acceleration_z;
}
double Pigeon2HWState::getAccelerationZ(void) const
{
	return acceleration_z_;
}

void Pigeon2HWState::setSupplyVoltage(const double supply_voltage)
{
	supply_voltage_ = supply_voltage;
}
double Pigeon2HWState::getSupplyVoltage(void) const
{
	return supply_voltage_;
}

void Pigeon2HWState::setMagneticFieldX(const double magnetic_field_x)
{
	magnetic_field_x_ = magnetic_field_x;
}
double Pigeon2HWState::getMagneticFieldX(void) const
{
	return magnetic_field_x_;
}

void Pigeon2HWState::setMagneticFieldY(const double magnetic_field_y)
{
	magnetic_field_y_ = magnetic_field_y;
}
double Pigeon2HWState::getMagneticFieldY(void) const
{
	return magnetic_field_y_;
}

void Pigeon2HWState::setMagneticFieldZ(const double magnetic_field_z)
{
	magnetic_field_z_ = magnetic_field_z;
}
double Pigeon2HWState::getMagneticFieldZ(void) const
{
	return magnetic_field_z_;
}

void Pigeon2HWState::setRawMagneticFieldX(const double raw_magnetic_field_x)
{
	raw_magnetic_field_x_ = raw_magnetic_field_x;
}
double Pigeon2HWState::getRawMagneticFieldX(void) const
{
	return raw_magnetic_field_x_;
}

void Pigeon2HWState::setRawMagneticFieldY(const double raw_magnetic_field_y)
{
	raw_magnetic_field_y_ = raw_magnetic_field_y;
}
double Pigeon2HWState::getRawMagneticFieldY(void) const
{
	return raw_magnetic_field_y_;
}

void Pigeon2HWState::setRawMagneticFieldZ(const double raw_magnetic_field_z)
{
	raw_magnetic_field_z_ = raw_magnetic_field_z;
}
double Pigeon2HWState::getRawMagneticFieldZ(void) const
{
	return raw_magnetic_field_z_;
}

void Pigeon2HWState::setFaultHardware(const bool fault_hardware)
{
	fault_hardware_ = fault_hardware;
}
bool Pigeon2HWState::getFaultHardware(void) const
{
	return fault_hardware_;
}

void Pigeon2HWState::setFaultUndervoltage(const bool fault_undervolage)
{
	fault_undervolage_ = fault_undervolage;
}
bool Pigeon2HWState::getFaultUndervoltage(void) const
{
	return fault_undervolage_;
}

void Pigeon2HWState::setFaultBootDuringEnable(const bool fault_boot_during_enable)
{
	fault_boot_during_enable_ = fault_boot_during_enable;
}
bool Pigeon2HWState::getFaultBootDuringEnable(void) const
{
	return fault_boot_during_enable_;
}

void Pigeon2HWState::setFaultUnlicensedFeatureInUse(const bool fault_unlicensed_feature_in_use)
{
	fault_unlicensed_feature_in_use_ = fault_unlicensed_feature_in_use;
}
bool Pigeon2HWState::getFaultUnlicensedFeatureInUse(void) const
{
	return fault_unlicensed_feature_in_use_;
}

void Pigeon2HWState::setFaultBootupAccelerometer(const bool fault_bootup_accelerometer)
{
	fault_bootup_accelerometer_ = fault_bootup_accelerometer;
}
bool Pigeon2HWState::getFaultBootupAccelerometer(void) const
{
	return fault_bootup_accelerometer_;
}

void Pigeon2HWState::setFaultBootupGyroscope(const bool fault_bootup_gyroscope)
{
	fault_bootup_gyroscope_ = fault_bootup_gyroscope;
}
bool Pigeon2HWState::getFaultBootupGyroscope(void) const
{
	return fault_bootup_gyroscope_;
}

void Pigeon2HWState::setFaultBootupMagnetometer(const bool fault_bootup_magnetometer)
{
	fault_bootup_magnetometer_ = fault_bootup_magnetometer;
}
bool Pigeon2HWState::getFaultBootupMagnetometer(void) const
{
	return fault_bootup_magnetometer_;
}

void Pigeon2HWState::setFaultBootIntoMotion(const bool fault_boot_into_motion)
{
	fault_boot_into_motion_ = fault_boot_into_motion;
}
bool Pigeon2HWState::getFaultBootIntoMotion(void) const
{
	return fault_boot_into_motion_;
}

void Pigeon2HWState::setFaultDataAcquiredLate(const bool fault_data_acquired_late)
{
	fault_data_acquired_late_ = fault_data_acquired_late;
}
bool Pigeon2HWState::getFaultDataAcquiredLate(void) const
{
	return fault_data_acquired_late_;
}

void Pigeon2HWState::setFaultLoopTimeSlow(const bool fault_loop_time_slow)
{
	fault_loop_time_slow_ = fault_loop_time_slow;
}
bool Pigeon2HWState::getFaultLoopTimeSlow(void) const
{
	return fault_loop_time_slow_;
}

void Pigeon2HWState::setFaultSaturatedMagnetometer(const bool fault_saturated_magnetometer)
{
	fault_saturated_magnetometer_ = fault_saturated_magnetometer;
}
bool Pigeon2HWState::getFaultSaturatedMagnetometer(void) const
{
	return fault_saturated_magnetometer_;
}

void Pigeon2HWState::setFaultSaturatedAccelometer(const bool fault_saturated_accelerometer)
{
	fault_saturated_accelerometer_ = fault_saturated_accelerometer;
}
bool Pigeon2HWState::getFaultSaturatedAccelometer(void) const
{
	return fault_saturated_accelerometer_;
}

void Pigeon2HWState::setFaultSaturatedGyroscope(const bool fault_saturated_gyroscope)
{
	fault_saturated_gyroscope_ = fault_saturated_gyroscope;
}
bool Pigeon2HWState::getFaultSaturatedGyroscope(void) const
{
	return fault_saturated_gyroscope_;
}

void Pigeon2HWState::setStickyFaultHardware(const bool sticky_fault_hardware)
{
	sticky_fault_hardware_ = sticky_fault_hardware;
}
bool Pigeon2HWState::getStickyFaultHardware(void) const
{
	return sticky_fault_hardware_;
}

void Pigeon2HWState::setStickyFaultUndervoltage(const bool sticky_fault_undervolage)
{
	sticky_fault_undervolage_ = sticky_fault_undervolage;
}
bool Pigeon2HWState::getStickyFaultUndervoltage(void) const
{
	return sticky_fault_undervolage_;
}

void Pigeon2HWState::setStickyFaultBootDuringEnable(const bool sticky_fault_boot_during_enable)
{
	sticky_fault_boot_during_enable_ = sticky_fault_boot_during_enable;
}
bool Pigeon2HWState::getStickyFaultBootDuringEnable(void) const
{
	return sticky_fault_boot_during_enable_;
}

void Pigeon2HWState::setStickyFaultUnlicensedFeatureInUse(const bool sticky_fault_unlicensed_feature_in_use)
{
	sticky_fault_unlicensed_feature_in_use_ = sticky_fault_unlicensed_feature_in_use;
}
bool Pigeon2HWState::getStickyFaultUnlicensedFeatureInUse(void) const
{
	return sticky_fault_unlicensed_feature_in_use_;
}

void Pigeon2HWState::setStickyFaultBootupAccelerometer(const bool sticky_fault_bootup_accelerometer)
{
	sticky_fault_bootup_accelerometer_ = sticky_fault_bootup_accelerometer;
}
bool Pigeon2HWState::getStickyFaultBootupAccelerometer(void) const
{
	return sticky_fault_bootup_accelerometer_;
}

void Pigeon2HWState::setStickyFaultBootupGyroscope(const bool sticky_fault_bootup_gyroscope)
{
	sticky_fault_bootup_gyroscope_ = sticky_fault_bootup_gyroscope;
}
bool Pigeon2HWState::getStickyFaultBootupGyroscope(void) const
{
	return sticky_fault_bootup_gyroscope_;
}

void Pigeon2HWState::setStickyFaultBootupMagnetometer(const bool sticky_fault_bootup_magnetometer)
{
	sticky_fault_bootup_magnetometer_ = sticky_fault_bootup_magnetometer;
}
bool Pigeon2HWState::getStickyFaultBootupMagnetometer(void) const
{
	return sticky_fault_bootup_magnetometer_;
}

void Pigeon2HWState::setStickyFaultBootIntoMotion(const bool sticky_fault_boot_into_motion)
{
	sticky_fault_boot_into_motion_ = sticky_fault_boot_into_motion;
}
bool Pigeon2HWState::getStickyFaultBootIntoMotion(void) const
{
	return sticky_fault_boot_into_motion_;
}

void Pigeon2HWState::setStickyFaultDataAcquiredLate(const bool sticky_fault_data_acquired_late)
{
	sticky_fault_data_acquired_late_ = sticky_fault_data_acquired_late;
}
bool Pigeon2HWState::getStickyFaultDataAcquiredLate(void) const
{
	return sticky_fault_data_acquired_late_;
}

void Pigeon2HWState::setStickyFaultLoopTimeSlow(const bool sticky_fault_loop_time_slow)
{
	sticky_fault_loop_time_slow_ = sticky_fault_loop_time_slow;
}
bool Pigeon2HWState::getStickyFaultLoopTimeSlow(void) const
{
	return sticky_fault_loop_time_slow_;
}

void Pigeon2HWState::setStickyFaultSaturatedMagnetometer(const bool sticky_fault_saturated_magnetometer)
{
	sticky_fault_saturated_magnetometer_ = sticky_fault_saturated_magnetometer;
}
bool Pigeon2HWState::getStickyFaultSaturatedMagnetometer(void) const
{
	return sticky_fault_saturated_magnetometer_;
}

void Pigeon2HWState::setStickyFaultSaturatedAccelometer(const bool sticky_fault_saturated_accelerometer)
{
	sticky_fault_saturated_accelerometer_ = sticky_fault_saturated_accelerometer;
}
bool Pigeon2HWState::getStickyFaultSaturatedAccelometer(void) const
{
	return sticky_fault_saturated_accelerometer_;
}

void Pigeon2HWState::setStickyFaultSaturatedGyroscope(const bool sticky_fault_saturated_gyroscope)
{
	sticky_fault_saturated_gyroscope_ = sticky_fault_saturated_gyroscope;
}
bool Pigeon2HWState::getStickyFaultSaturatedGyroscope(void) const
{
	return sticky_fault_saturated_gyroscope_;
}

} // namespace