#include "talon_interface/cancoder_command_interface.h"

namespace hardware_interface
{
namespace cancoder
{

CANCoderHWCommand::CANCoderHWCommand(void)
	: position_{0}
	, position_changed_{false}
	, set_position_to_absolute_{false}
	, velocity_meas_period_{Sensor_Period_100Ms}
	, velocity_meas_period_changed_{true}
	, velocity_meas_window_{64}
	, velocity_meas_window_changed_{true}
	, absolute_sensor_range_{Unsigned_0_to_360}
	, absolute_sensor_range_changed_{true}
	, magnet_offset_{0}
	, magnet_offset_changed_{false}
	, initialization_strategy_{BootToZero}
	, initialization_strategy_changed_{true}
	, feedback_coefficient_{2. * M_PI / 4096.} // 2PI radians per rotation
	, unit_string_{"rad"}
	, time_base_{PerSecond}
	, feedback_coefficient_changed_{true}
	, direction_{false}
	, direction_changed_{true}
	, sensor_data_status_frame_period_{100}
	, sensor_data_status_frame_period_changed_{true}
	, vbat_and_faults_status_frame_period_{100}
	, vbat_and_faults_status_frame_period_changed_{true}
	, clear_sticky_faults_{false}
	, conversion_factor_{1.0}
{
}

double CANCoderHWCommand::getPosition(void) const
{
	return position_;
}
void CANCoderHWCommand::setPosition(double position)
{
	if (position != position_)
	{
		position_ = position;
		position_changed_ = true;
	}
}
bool CANCoderHWCommand::positionChanged(double &position)
{
	position = position_;
	bool rc = position_changed_;
	position_changed_ = false;
	return rc;
}
void CANCoderHWCommand::resetPosition(void)
{
	position_changed_ = true;
}

bool CANCoderHWCommand::getPositionToAbsolute(void) const
{
	return set_position_to_absolute_;
}
void CANCoderHWCommand::setPositionToAbsolute(void)
{
	set_position_to_absolute_ = true;
}
bool CANCoderHWCommand::CANCoderHWCommand::positionToAbsoluteChanged(void)
{
	bool rc = set_position_to_absolute_;
	set_position_to_absolute_ = false;
	return rc;
}

SensorVelocityMeasPeriod CANCoderHWCommand::getVelocityMeasPeriod(void) const
{
	return velocity_meas_period_;
}
void CANCoderHWCommand::setVelocityMeasPeriod(SensorVelocityMeasPeriod velocity_meas_period)
{
	if (velocity_meas_period != velocity_meas_period_)
	{
		velocity_meas_period_ = velocity_meas_period;
		velocity_meas_period_changed_ = true;
	}
}
bool CANCoderHWCommand::velocityMeasPeriodChanged(SensorVelocityMeasPeriod &velocity_meas_period)
{
	velocity_meas_period = velocity_meas_period_;
	bool rc = velocity_meas_period_changed_;
	velocity_meas_period_changed_ = false;
	return rc;
}
void CANCoderHWCommand::resetVelocityMeasPeriod(void)
{
	velocity_meas_period_changed_ = true;
}

int CANCoderHWCommand::getVelocityMeasWindow(void) const
{
	return velocity_meas_window_;
}
void CANCoderHWCommand::setVelocityMeasWindow(int velocity_meas_window)
{
	if (velocity_meas_window != velocity_meas_window_)
	{
		velocity_meas_window_ = velocity_meas_window;
		velocity_meas_window_changed_ = true;
	}
}
bool CANCoderHWCommand::velocityMeasWindowChanged(int &velocity_meas_window)
{
	velocity_meas_window = velocity_meas_window_;
	bool rc = velocity_meas_window_changed_;
	velocity_meas_window_changed_ = false;
	return rc;
}
void CANCoderHWCommand::resetVelocityMeasWindow(void)
{
	velocity_meas_window_changed_ = true;
}

AbsoluteSensorRange CANCoderHWCommand::getAbsoluteSensorRange(void) const
{
	return absolute_sensor_range_;
}
void CANCoderHWCommand::setAbsoluteSensorRange(AbsoluteSensorRange absolute_sensor_range)
{
	if (absolute_sensor_range != absolute_sensor_range_)
	{
		absolute_sensor_range_ = absolute_sensor_range;
		absolute_sensor_range_changed_ = true;
	}
}
bool CANCoderHWCommand::absoluteSensorRangeChanged(AbsoluteSensorRange &absolute_sensor_range)
{
	absolute_sensor_range = absolute_sensor_range_;
	bool rc = absolute_sensor_range_changed_;
	absolute_sensor_range_changed_ = false;
	return rc;
}
void CANCoderHWCommand::resetAbsoluteSensorRange(void)
{
	absolute_sensor_range_changed_ = true;
}

double CANCoderHWCommand::getMagnetOffset(void) const
{
	return magnet_offset_;
}
void CANCoderHWCommand::setMagnetOffset(double magnet_offset)
{
	if (magnet_offset != magnet_offset_)
	{
		magnet_offset_ = magnet_offset;
		magnet_offset_changed_ = true;
	}
}
bool CANCoderHWCommand::magnetOffsetChanged(double &magnet_offset)
{
	magnet_offset = magnet_offset_;
	bool rc = magnet_offset_changed_;
	magnet_offset_changed_ = false;
	return rc;
}
void CANCoderHWCommand::resetMagnetOffset(void)
{
	magnet_offset_changed_ = true;
}

double CANCoderHWCommand::getInitializationStrategy(void) const
{
	return initialization_strategy_;
}
void CANCoderHWCommand::setInitializationStrategy(SensorInitializationStrategy initialization_strategy)
{
	if (initialization_strategy != initialization_strategy_)
	{
		initialization_strategy_ = initialization_strategy;
		initialization_strategy_changed_ = true;
	}
}
bool CANCoderHWCommand::InitializationStrategyChanged(SensorInitializationStrategy &initialization_strategy)
{
	initialization_strategy = initialization_strategy_;
	bool rc = initialization_strategy_changed_;
	initialization_strategy_changed_ = false;
	return rc;
}
void CANCoderHWCommand::resetInitializationStrategy(void)
{
	initialization_strategy_changed_ = true;
}

double CANCoderHWCommand::getFeedbackCoefficient(void) const
{
	return feedback_coefficient_;
}
void CANCoderHWCommand::setFeedbackCoefficient(double feedback_coefficient)
{
	if (feedback_coefficient_ != feedback_coefficient_)
	{
		feedback_coefficient_ = feedback_coefficient;
		feedback_coefficient_changed_ = true;
	}
}

std::string CANCoderHWCommand::getUnitString(void) const
{
	return unit_string_;
}
void CANCoderHWCommand::setUnitString(const std::string &unit_string)
{
	if (unit_string_ != unit_string)
	{
		unit_string_ = unit_string;
		feedback_coefficient_changed_ = true;
	}
}
SensorTimeBase CANCoderHWCommand::getTimeBase(void) const
{
	return time_base_;
}
void CANCoderHWCommand::setTimeBase(SensorTimeBase time_base)
{
	if (time_base_ != time_base)
	{
		time_base_ = time_base;
		feedback_coefficient_changed_ = true;
	}
}
bool CANCoderHWCommand::feedbackCoefficientChanged(
		double &feedback_coefficient,
		std::string &unit_string,
		SensorTimeBase &time_base)
{
	feedback_coefficient = feedback_coefficient_;
	unit_string = unit_string_;
	time_base = time_base_;
	bool rc = feedback_coefficient_changed_;
	feedback_coefficient_changed_ = false;
	return rc;
}
void CANCoderHWCommand::resetFeedbackCoefficient(void)
{
	feedback_coefficient_changed_ = true;
}

double CANCoderHWCommand::getDirection(void) const
{
	return direction_;
}
void CANCoderHWCommand::setDirection(bool direction)
{
	if (direction != direction_)
	{
		direction_ = direction;
		direction_changed_ = true;
	}
}
bool CANCoderHWCommand::directionChanged(bool &direction)
{
	direction = direction_;
	bool rc = direction_changed_;
	direction_changed_ = false;
	return rc;
}
void CANCoderHWCommand::resetDirection(void)
{
	direction_changed_ = true;
}

int CANCoderHWCommand::getSensorDataStatusFramePeriod(void) const
{
	return sensor_data_status_frame_period_;
}
void CANCoderHWCommand::setSensorDataStatusFramePeriod(int sensor_data_status_frame_period)
{
	if (sensor_data_status_frame_period != sensor_data_status_frame_period_)
	{
		sensor_data_status_frame_period_ = sensor_data_status_frame_period;
		sensor_data_status_frame_period_changed_ = true;
	}
}
bool CANCoderHWCommand::sensorDataStatusFramePeriodChanged(int &sensor_data_status_frame_period)
{
	sensor_data_status_frame_period = sensor_data_status_frame_period_;
	bool rc = sensor_data_status_frame_period_changed_;
	sensor_data_status_frame_period_changed_ = false;
	return rc;
}
void CANCoderHWCommand::resetSensorDataStatusFramePeriod(void)
{
	sensor_data_status_frame_period_changed_ = true;
}

int CANCoderHWCommand::getVBatAndFaultsStatusFramePeriod(void) const
{
	return vbat_and_faults_status_frame_period_;
}
void CANCoderHWCommand::setVBatAndFaultsStatusFramePeriod(int vbat_and_faults_status_frame_period)
{
	if (vbat_and_faults_status_frame_period != vbat_and_faults_status_frame_period_)
	{
		vbat_and_faults_status_frame_period_ = vbat_and_faults_status_frame_period;
		vbat_and_faults_status_frame_period_changed_ = true;
	}
}
bool CANCoderHWCommand::vbatAndFaultsStatusFramePeriodChanged(int &vbat_and_faults_status_frame_period)
{
	vbat_and_faults_status_frame_period = vbat_and_faults_status_frame_period_;
	bool rc = vbat_and_faults_status_frame_period_changed_;
	vbat_and_faults_status_frame_period_changed_ = false;
	return rc;
}
void CANCoderHWCommand::resetVBatAndFaultsStatusFramePeriod(void)
{
	vbat_and_faults_status_frame_period_changed_ = true;
}
void CANCoderHWCommand::setClearStickyFaults(void)
{
	clear_sticky_faults_ = true;
}
bool CANCoderHWCommand::getClearStickyFaults(void) const
{
	return clear_sticky_faults_;
}
bool CANCoderHWCommand::clearStickyFaultsChanged(void)
{
	auto ret = clear_sticky_faults_;
	clear_sticky_faults_ = false;
	return ret;
}

void CANCoderHWCommand::setConversionFactor(double conversion_factor)
{
	conversion_factor_ = conversion_factor;
}
double CANCoderHWCommand::getConversionFactor(void) const
{
	return conversion_factor_;
}

} // namespace hardware_interface
} // namespace cancoder

