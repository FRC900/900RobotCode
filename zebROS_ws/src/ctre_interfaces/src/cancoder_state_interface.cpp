#include "ctre_interfaces/cancoder_state_interface.h"

namespace hardware_interface::cancoder
{

CANCoderHWState::CANCoderHWState(const int device_number)
	: device_number_{device_number}
{
}

int CANCoderHWState::getDeviceNumber(void) const
{
	return device_number_;
}

SensorDirection CANCoderHWState::getSensorDirection(void) const
{
	return sensor_direction_;
}
void CANCoderHWState::setSensorDirection(const SensorDirection sensor_direction)
{
	sensor_direction_ = sensor_direction;
}

double CANCoderHWState::getMagnetOffset(void) const
{
	return magnet_offset_;
}
void CANCoderHWState::setMagnetOffset(const double magnet_offset)
{
	magnet_offset_ = magnet_offset;
}

double CANCoderHWState::getAbsoluteSensorDiscontinuityPoint(void) const
{
	return absolute_sensor_discontinuity_point_;
}
void CANCoderHWState::setAbsoluteSensorDiscontinuityPoint(const double absolute_sensor_discontinuity_point)
{
	absolute_sensor_discontinuity_point_ = absolute_sensor_discontinuity_point;
}	

void CANCoderHWState::setConversionFactor(double conversion_factor)
{
	conversion_factor_ = conversion_factor;
}
double CANCoderHWState::getConversionFactor(void) const
{
	return conversion_factor_;
}

void CANCoderHWState::setEnableReadThread(const bool enable_read_thread)
{
	enable_read_thread_ = enable_read_thread;
}
bool CANCoderHWState::getEnableReadThread(void) const
{
	return enable_read_thread_;
}

void CANCoderHWState::setVersionMajor(const int version_major)
{
	version_major_ = version_major;
}
int CANCoderHWState::getVersionMajor(void) const
{
	return version_major_;
}

void CANCoderHWState::setVersionMinor(const int version_minor)
{
	version_minor_ = version_minor;
}
int CANCoderHWState::getVersionMinor(void) const
{
	return version_minor_;
}

void CANCoderHWState::setVersionBugfix(const int version_bugfix)
{
	version_bugfix_ = version_bugfix;
}
int CANCoderHWState::getVersionBugfix(void) const
{
	return version_bugfix_;
}

void CANCoderHWState::setVersionBuild(const int version_build)
{
	version_build_ = version_build;
}
int CANCoderHWState::getVersionBuild(void) const
{
	return version_build_;
}

double CANCoderHWState::getVelocity(void) const
{
	return velocity_;
}
void CANCoderHWState::setVelocity(const double velocity)
{
	velocity_ = velocity;
}

double CANCoderHWState::getPosition(void) const
{
	return position_;
}
void CANCoderHWState::setPosition(const double position)
{
	position_ = position;
}

double CANCoderHWState::getAbsolutePosition(void) const
{
	return absolute_position_;
}
void CANCoderHWState::setAbsolutePosition(const double absolute_position)
{
	absolute_position_ = absolute_position;
}

double CANCoderHWState::getUnfilteredVelocity(void) const
{
	return unfiltered_velocity_;
}
void CANCoderHWState::setUnfilteredVelocity(const double unfiltered_velocity)
{
	unfiltered_velocity_ = unfiltered_velocity;
}

double CANCoderHWState::getPositionSinceBoot(void) const
{
	return position_since_boot_;
}
void CANCoderHWState::setPositionSinceBoot(const double position_since_boot)
{
	position_since_boot_ = position_since_boot;
}

double CANCoderHWState::getSupplyVoltage(void) const
{
	return supply_voltage_;
}
void CANCoderHWState::setSupplyVoltage(const double supply_voltage)
{
	supply_voltage_ = supply_voltage;
}

MagnetHealth CANCoderHWState::getMagnetHealth(void) const
{
	return magnet_health_;
}
void CANCoderHWState::setMagnetHealth(const MagnetHealth magnet_health)
{
	magnet_health_ = magnet_health;
}

void CANCoderHWState::setFaultHardware(const bool fault_hardware)
{
	fault_hardware_ = fault_hardware;
}
bool CANCoderHWState::getFaultHardware(void) const
{
	return fault_hardware_;
}

void CANCoderHWState::setFaultUndervoltage(const bool fault_undervolage)
{
	fault_undervolage_ = fault_undervolage;
}
bool CANCoderHWState::getFaultUndervoltage(void) const
{
	return fault_undervolage_;
}

void CANCoderHWState::setFaultBootDuringEnable(const bool fault_boot_during_enable)
{
	fault_boot_during_enable_ = fault_boot_during_enable;
}
bool CANCoderHWState::getFaultBootDuringEnable(void) const
{
	return fault_boot_during_enable_;
}

void CANCoderHWState::setFaultUnlicensedFeatureInUse(const bool fault_unlicensed_feature_in_use)
{
	fault_unlicensed_feature_in_use_ = fault_unlicensed_feature_in_use;
}
bool CANCoderHWState::getFaultUnlicensedFeatureInUse(void) const
{
	return fault_unlicensed_feature_in_use_;
}

void CANCoderHWState::setFaultBadMagnet(const bool fault_bad_magnet)
{
	fault_bad_magnet_ = fault_bad_magnet;
}
bool CANCoderHWState::getFaultBadMagnet(void) const
{
	return fault_bad_magnet_;
}

void CANCoderHWState::setStickyFaultHardware(const bool sticky_fault_hardware)
{
	sticky_fault_hardware_ = sticky_fault_hardware;
}
bool CANCoderHWState::getStickyFaultHardware(void) const
{
	return sticky_fault_hardware_;
}

void CANCoderHWState::setStickyFaultUndervoltage(const bool sticky_fault_undervolage)
{
	sticky_fault_undervolage_ = sticky_fault_undervolage;
}
bool CANCoderHWState::getStickyFaultUndervoltage(void) const
{
	return sticky_fault_undervolage_;
}

void CANCoderHWState::setStickyFaultBootDuringEnable(const bool sticky_fault_boot_during_enable)
{
	sticky_fault_boot_during_enable_ = sticky_fault_boot_during_enable;
}
bool CANCoderHWState::getStickyFaultBootDuringEnable(void) const
{
	return sticky_fault_boot_during_enable_;
}

void CANCoderHWState::setStickyFaultUnlicensedFeatureInUse(const bool sticky_fault_unlicensed_feature_in_use)
{
	sticky_fault_unlicensed_feature_in_use_ = sticky_fault_unlicensed_feature_in_use;
}
bool CANCoderHWState::getStickyFaultUnlicensedFeatureInUse(void) const
{
	return sticky_fault_unlicensed_feature_in_use_;
}

void CANCoderHWState::setStickyFaultBadMagnet(const bool sticky_fault_bad_magnet)
{
	sticky_fault_bad_magnet_ = sticky_fault_bad_magnet;
}
bool CANCoderHWState::getStickyFaultBadMagnet(void) const
{
	return sticky_fault_bad_magnet_;
}

} // namespace 
