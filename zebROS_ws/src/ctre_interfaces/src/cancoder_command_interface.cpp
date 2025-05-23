#include "ctre_interfaces/cancoder_command_interface.h"

namespace hardware_interface::cancoder
{

CANCoderHWCommand::CANCoderHWCommand(void) = default;

double CANCoderHWCommand::getSetPosition(void) const
{
	return set_position_;
}
void CANCoderHWCommand::setSetPosition(const double set_position)
{
	if (set_position != set_position_)
	{
		set_position_ = set_position;
		set_position_changed_ = true;
	}
}
bool CANCoderHWCommand::setPositionChanged(double &set_position)
{
	set_position = set_position_;
	bool rc = set_position_changed_;
	set_position_changed_ = false;
	return rc;
}
void CANCoderHWCommand::resetSetPosition(void)
{
	set_position_changed_ = true;
}

SensorDirection CANCoderHWCommand::getSensorDirection(void) const
{
	return sensor_direction_;
}
void CANCoderHWCommand::setSensorDirection(const SensorDirection sensor_direction)
{
	if (sensor_direction != sensor_direction_)
	{
		sensor_direction_ = sensor_direction;
		magnet_sensor_configs_changed_ = true;
	}
}

double CANCoderHWCommand::getMagnetOffset(void) const
{
	return magnet_offset_;
}
void CANCoderHWCommand::setMagnetOffset(const double magnet_offset)
{
	if (magnet_offset != magnet_offset_)
	{
		magnet_offset_ = magnet_offset;
		magnet_sensor_configs_changed_ = true;
	}
}

double CANCoderHWCommand::getAbsoluteSensorDiscontinuityPoint(void) const
{
	return absolute_sensor_discontinuity_point_;
}
void CANCoderHWCommand::setAbsoluteSensorDiscontinuityPoint(const double absolute_sensor_discontinuity_point)
{
	if (absolute_sensor_discontinuity_point != absolute_sensor_discontinuity_point_)
	{
		absolute_sensor_discontinuity_point_ = absolute_sensor_discontinuity_point;
		magnet_sensor_configs_changed_ = true;
	}
}

bool CANCoderHWCommand::magnetSensorConfigsChanged(SensorDirection &sensor_direction,
												   double &magnet_offset,
												   double &absolute_sensor_discontinuity_point)
{
	sensor_direction = sensor_direction_;
	magnet_offset = magnet_offset_;
	absolute_sensor_discontinuity_point = absolute_sensor_discontinuity_point_;
	const auto ret = magnet_sensor_configs_changed_;
	magnet_sensor_configs_changed_ = false;
	return ret;
}

void CANCoderHWCommand::resetMagnetSensorConfigs(void)
{
	magnet_sensor_configs_changed_ = true;
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

void CANCoderHWCommand::setConversionFactor(const double conversion_factor)
{
	conversion_factor_ = conversion_factor;
}
double CANCoderHWCommand::getConversionFactor(void) const
{
	return conversion_factor_;
}

void CANCoderHWCommand::setEnableReadThread(const bool enable_read_thread)
{
	enable_read_thread_ = enable_read_thread;
}
bool CANCoderHWCommand::getEnableReadThread(void) const
{
	return enable_read_thread_;
}

// namespace hardware_interface
} // namespace cancoder

