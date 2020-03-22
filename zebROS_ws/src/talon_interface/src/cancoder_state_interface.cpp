#include "talon_interface/cancoder_state_interface.h"

namespace hardware_interface
{
namespace cancoder
{

CANCoderHWState::CANCoderHWState(int device_number)
	: device_number_{device_number}
	  , position_{0}
	  , velocity_{0}
	  , absolute_position_{0}
	  , velocity_meas_period_{Sensor_Period_100Ms}
	  , velocity_meas_window_{64}
	  , absolute_sensor_range_{Unsigned_0_to_360}
	  , magnet_offset_{0} // NOTE : degrees
	  , initialization_strategy_{BootToZero}
	  , feedback_coefficient_{2. * M_PI / 4096.} // 2PI radians per rotation
	  , unit_string_{"rad"}
	  , time_base_{PerSecond}
	  , bus_voltage_{0}
	  , magnet_field_strength_{Invalid_Unknown}
	  , direction_{false}
	  , last_timestamp_{-std::numeric_limits<double>::max()}
	  , sensor_data_status_frame_period_{-1}
	  , vbat_and_faults_status_frame_period_{-1}
	  , firmware_version_{-1}
	  , conversion_factor_{1.0}
{
}

int CANCoderHWState::getDeviceNumber(void) const
{
	return device_number_;
}

double CANCoderHWState::getPosition(void) const
{
	return position_;
}
void CANCoderHWState::setPosition(double position)
{
	position_ = position;
}

double CANCoderHWState::getVelocity(void) const
{
	return velocity_;
}
void CANCoderHWState::setVelocity(double velocity)
{
	velocity_ = velocity;
}

double CANCoderHWState::getAbsolutePosition(void) const
{
	return absolute_position_;
}
void CANCoderHWState::setAbsolutePosition(double absolute_position)
{
	absolute_position_ = absolute_position;
}

SensorVelocityMeasPeriod CANCoderHWState::getVelocityMeasPeriod(void) const
{
	return velocity_meas_period_;
}
void CANCoderHWState::setVelocityMeasPeriod(SensorVelocityMeasPeriod velocity_meas_period)
{
	velocity_meas_period_ = velocity_meas_period;
}

int CANCoderHWState::getVelocityMeasWindow(void) const
{
	return velocity_meas_window_;
}
void CANCoderHWState::setVelocityMeasWindow(int velocity_meas_window)
{
	velocity_meas_window_ = velocity_meas_window;
}

AbsoluteSensorRange CANCoderHWState::getAbsoluteSensorRange(void) const
{
	return absolute_sensor_range_;
}
void CANCoderHWState::setAbsoluteSensorRange(AbsoluteSensorRange absolute_sensor_range)
{
	absolute_sensor_range_ = absolute_sensor_range;
}

double CANCoderHWState::getMagnetOffset(void) const
{
	return magnet_offset_;
}
void CANCoderHWState::setMagnetOffset(double magnet_offset)
{
	magnet_offset_ = magnet_offset;
}

SensorInitializationStrategy CANCoderHWState::getInitializationStrategy(void) const
{
	return initialization_strategy_;
}
void CANCoderHWState::setInitializationStrategy(SensorInitializationStrategy initialization_strategy)
{
	initialization_strategy_ = initialization_strategy;
}

double CANCoderHWState::getFeedbackCoefficient(void) const
{
	return feedback_coefficient_;
}
void CANCoderHWState::setFeedbackCoefficient(double feedback_coefficient)
{
	feedback_coefficient_ = feedback_coefficient;
}

std::string CANCoderHWState::getUnitString(void) const
{
	return unit_string_;
}
void CANCoderHWState::setUnitString(const std::string &unit_string)
{
	unit_string_ = unit_string;
}

SensorTimeBase CANCoderHWState::getTimeBase(void) const
{
	return time_base_;
}
void CANCoderHWState::setTimeBase(SensorTimeBase time_base)
{
	time_base_ = time_base;
}

double CANCoderHWState::getBusVoltage(void) const
{
	return bus_voltage_;
}
void CANCoderHWState::setBusVoltage(double bus_voltage)
{
	bus_voltage_ = bus_voltage;
}

MagnetFieldStrength CANCoderHWState::getMagnetFieldStrength(void) const
{
	return magnet_field_strength_;
}
void CANCoderHWState::setMagnetFieldStrength(MagnetFieldStrength magnet_field_strength)
{
	magnet_field_strength_ = magnet_field_strength;
}

bool CANCoderHWState::getDirection(void) const
{
	return direction_;
}
void CANCoderHWState::setDirection(bool direction)
{
	direction_ = direction;
}

double CANCoderHWState::getLastTimestamp(void) const
{
	return last_timestamp_;
}
void CANCoderHWState::setLastTimestamp(double last_timestamp)
{
	last_timestamp_ = last_timestamp;
}

int CANCoderHWState::getSensorDataStatusFramePeriod(void) const
{
	return sensor_data_status_frame_period_;
}
void CANCoderHWState::setSensorDataStatusFramePeriod(int sensor_data_status_frame_period)
{
	sensor_data_status_frame_period_ = sensor_data_status_frame_period;
}

int CANCoderHWState::getVbatAndFaultsStatusFramePeriod(void) const
{
	return vbat_and_faults_status_frame_period_;
}
void CANCoderHWState::setVbatAndFaultsStatusFramePeriod(int vbat_and_faults_status_frame_period)
{
	vbat_and_faults_status_frame_period_ = vbat_and_faults_status_frame_period;
}

int CANCoderHWState::getFirmwareVersion(void) const
{
	return firmware_version_;
}
void CANCoderHWState::setFirmwareVersion(int firmware_version)
{
	firmware_version_ = firmware_version;
}

int CANCoderHWState::getFaults(void) const
{
	return faults_;
}
void CANCoderHWState::setFaults(int faults)
{
	faults_ = faults;
}
int CANCoderHWState::getStickyFaults(void) const
{
	return sticky_faults_;
}
void CANCoderHWState::setStickyFaults(int sticky_faults)
{
	sticky_faults_ = sticky_faults;
}

void CANCoderHWState::setConversionFactor(double conversion_factor)
{
	conversion_factor_ = conversion_factor;
}
double CANCoderHWState::getConversionFactor(void) const
{
	return conversion_factor_;
}

} //namespace hardware_interface
} // namespace cancoder
