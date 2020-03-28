#include "ros_control_boilerplate/as726x_convert.h"

namespace as726x_convert
{
bool AS726xConvert::indLedCurrentLimit(const hardware_interface::as726x::IndLedCurrentLimits input,
		as726x::ind_led_current_limits &output) const
{
	switch (input)
	{
		case hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_1MA:
			output = as726x::ind_led_current_limits::LIMIT_1MA;
			break;
		case hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_2MA:
			output = as726x::ind_led_current_limits::LIMIT_2MA;
			break;
		case hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_4MA:
			output = as726x::ind_led_current_limits::LIMIT_4MA;
			break;
		case hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_8MA:
			output = as726x::ind_led_current_limits::LIMIT_8MA;
			break;
		default:
			ROS_ERROR("Invalid input in convertAS72xIndLedCurrentLimit");
			return false;
	}
	return true;
}
bool AS726xConvert::drvLedCurrentLimit(const hardware_interface::as726x::DrvLedCurrentLimits input,
		as726x::drv_led_current_limits &output) const
{
	switch (input)
	{
		case hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_12MA5:
			output = as726x::drv_led_current_limits::LIMIT_12MA5;
			break;
		case hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_25MA:
			output = as726x::drv_led_current_limits::LIMIT_25MA;
			break;
		case hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_50MA:
			output = as726x::drv_led_current_limits::LIMIT_50MA;
			break;
		case hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_100MA:
			output = as726x::drv_led_current_limits::LIMIT_100MA;
			break;
		default:
			ROS_ERROR("Invalid input in convertAS72xDrvLedCurrentLimit");
			return false;
	}
	return true;
}
bool AS726xConvert::conversionType(const hardware_interface::as726x::ConversionTypes input,
		as726x::conversion_types &output) const
{
	switch (input)
	{
		case hardware_interface::as726x::ConversionTypes::MODE_0:
			output = as726x::conversion_types::MODE_0;
			break;
		case hardware_interface::as726x::ConversionTypes::MODE_1:
			output = as726x::conversion_types::MODE_1;
			break;
		case hardware_interface::as726x::ConversionTypes::MODE_2:
			output = as726x::conversion_types::MODE_2;
			break;
		case hardware_interface::as726x::ConversionTypes::ONE_SHOT:
			output = as726x::conversion_types::ONE_SHOT;
			break;
		default:
			ROS_ERROR("Invalid input in convertAS726xConversionType");
			return false;
	}
	return true;
}
bool AS726xConvert::channelGain(const hardware_interface::as726x::ChannelGain input,
		as726x::channel_gain &output) const
{
	switch (input)
	{
		case hardware_interface::as726x::ChannelGain::GAIN_1X:
			output = as726x::channel_gain::GAIN_1X;
			break;
		case hardware_interface::as726x::ChannelGain::GAIN_3X7:
			output = as726x::channel_gain::GAIN_3X7;
			break;
		case hardware_interface::as726x::ChannelGain::GAIN_16X:
			output = as726x::channel_gain::GAIN_16X;
			break;
		case hardware_interface::as726x::ChannelGain::GAIN_64X:
			output = as726x::channel_gain::GAIN_64X;
			break;
		default:
			ROS_ERROR("Invalid input in convertAS726xChannelGain");
			return false;
	}
	return true;
}

} // namespace as726x_convert
