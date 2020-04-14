// Classes to hold state and command for AS726x hardware interface integration
// The state class hold data read from the hardware interface ::read() method from
//   the sensor hardware. This data is used by controllers that need to access
//   sensor state
// The command class holds commands written to hardware by the hardware interface
//   ::write() method. Controllers can write to this class to send commands to
//   the hardware itself
#include "as726x_interface/as726x_interface.h"

namespace hardware_interface
{
namespace as726x
{

AS726xState::AS726xState(const std::string &port, int address)
	: port_(port)
	, address_(address)
	, ind_led_current_limit_(IND_LIMIT_1MA)
	, ind_led_enable_(false)
	, drv_led_current_limit_(DRV_LIMIT_12MA5)
	, drv_led_enable_(false)
	, conversion_type_(ONE_SHOT)
	, gain_(GAIN_1X)
	, integration_time_(50)
	, temperature_(0)
	, raw_channel_data_{0}
	, calibrated_channel_data_{0.0}
{
}
std::string                              AS726xState::getPort(void) const                  { return port_; }
int                                      AS726xState::getAddress(void) const               { return address_; }
IndLedCurrentLimits                      AS726xState::getIndLedCurrentLimit(void) const    { return ind_led_current_limit_; }
bool                                     AS726xState::getIndLedEnable(void) const          { return ind_led_enable_; }
DrvLedCurrentLimits                      AS726xState::getDrvLedCurrentLimit(void) const    { return drv_led_current_limit_; }
bool                                     AS726xState::getDrvLedEnable(void) const          { return drv_led_enable_; }
ConversionTypes                          AS726xState::getConversionType(void) const        { return conversion_type_; }
ChannelGain                              AS726xState::getGain(void) const                  { return gain_; }
uint8_t                                  AS726xState::getIntegrationTime(void) const       { return integration_time_; }
uint8_t                                  AS726xState::getTemperature(void) const           { return temperature_; }
std::array<uint16_t, COLOR_CHANNEL_LAST> AS726xState::getRawChannelData(void) const        { return raw_channel_data_; }
std::array<float, COLOR_CHANNEL_LAST>    AS726xState::getCalibratedChannelData(void) const { return calibrated_channel_data_; }

void AS726xState::setIndLedCurrentLimit(const IndLedCurrentLimits &ind_led_current_limit)
{
	if ((ind_led_current_limit < IND_LIMIT_1MA) || (ind_led_current_limit >= IND_LIMIT_LAST))
	{
		ROS_ERROR_STREAM("enum argument out of range in " << __FUNCTION__);
		return;
	}
	ind_led_current_limit_ = ind_led_current_limit;
}
void AS726xState::setIndLedEnable(bool ind_led_enable)
{
	ind_led_enable_ = ind_led_enable;
}
void AS726xState::setDrvLedCurrentLimit(const DrvLedCurrentLimits &drv_led_current_limit)
{
	if ((drv_led_current_limit < DRV_LIMIT_12MA5) || (drv_led_current_limit >= DRV_LIMIT_LAST))
	{
		ROS_ERROR_STREAM("enum argument out of range in " << __FUNCTION__);
		return;
	}
	drv_led_current_limit_ = drv_led_current_limit;
}
void AS726xState::setDrvLedEnable(bool drv_led_enable)
{
	drv_led_enable_ = drv_led_enable;
}
void AS726xState::setConversionType(const ConversionTypes &conversion_type)
{
	if ((conversion_type < MODE_0) || (conversion_type >= ConversionTypes_Last))
	{
		ROS_ERROR_STREAM("enum argument out of range in " << __FUNCTION__);
		return;
	}
	conversion_type_ = conversion_type;
}
void AS726xState::setGain(const ChannelGain &gain)
{
	if ((gain < GAIN_1X) || (gain >= GAIN_LAST))
	{
		ROS_ERROR_STREAM("enum argument out of range in " << __FUNCTION__);
		return;
	}
	gain_ = gain;
}
void AS726xState::setIntegrationTime(uint8_t integration_time)
{
	integration_time_ = integration_time;
}
void AS726xState::setTemperature(uint8_t temperature)
{
	temperature_ = temperature;
}
void AS726xState::setRawChannelData(const std::array<uint16_t, COLOR_CHANNEL_LAST> &raw_channel_data)
{
	raw_channel_data_ = raw_channel_data;
}
void AS726xState::setCalibratedChannelData(const std::array<float, COLOR_CHANNEL_LAST> &calibrated_channel_data)
{
	calibrated_channel_data_ = calibrated_channel_data;
}

AS726xCommand::AS726xCommand(void)
	: ind_led_current_limit_(IND_LIMIT_1MA)
	, ind_led_current_limit_changed_(true)
	, ind_led_enable_(false)
	, ind_led_enable_changed_(true)
	, drv_led_current_limit_(DRV_LIMIT_12MA5)
	, drv_led_current_limit_changed_(true)
	, drv_led_enable_(false)
	, drv_led_enable_changed_(true)
	, conversion_type_(ONE_SHOT)
	, conversion_type_changed_(true)
	, gain_(GAIN_1X)
	, gain_changed_(true)
	, integration_time_(50)
	, integration_time_changed_(true)
{
}

IndLedCurrentLimits AS726xCommand::getIndLedCurrentLimit(void) const  { return ind_led_current_limit_; }
bool                AS726xCommand::getIndLedEnable(void) const        { return ind_led_enable_; }
DrvLedCurrentLimits AS726xCommand::getDrvLedCurrentLimit(void) const  { return drv_led_current_limit_; }
bool                AS726xCommand::getDrvLedEnable(void) const        { return drv_led_enable_; }
ConversionTypes     AS726xCommand::getConversionType(void) const      { return conversion_type_; }
ChannelGain         AS726xCommand::getGain(void) const                { return gain_; }
uint8_t             AS726xCommand::getIntegrationTime(void) const     { return integration_time_; }

		void AS726xCommand::setIndLedCurrentLimit(IndLedCurrentLimits ind_led_current_limit)
		{
			if ((ind_led_current_limit < IND_LIMIT_1MA) || (ind_led_current_limit >= IND_LIMIT_LAST))
			{
				ROS_ERROR_STREAM("enum argument out of range in " << __FUNCTION__);
				return;
			}
			if (ind_led_current_limit != ind_led_current_limit_)
			{
				ind_led_current_limit_ = ind_led_current_limit;
				ind_led_current_limit_changed_ = true;
			}
		}
		void AS726xCommand::setIndLedEnable(bool ind_led_enable)
		{
			if (ind_led_enable != ind_led_enable_)
			{
				ind_led_enable_ = ind_led_enable;
				ind_led_enable_changed_ = true;
			}
		}
		void AS726xCommand::setDrvLedCurrentLimit(DrvLedCurrentLimits drv_led_current_limit)
		{
			if ((drv_led_current_limit < DRV_LIMIT_12MA5) || (drv_led_current_limit >= DRV_LIMIT_LAST))
			{
				ROS_ERROR_STREAM("enum argument out of range in " << __FUNCTION__);
				return;
			}
			if (drv_led_current_limit != drv_led_current_limit_)
			{
				drv_led_current_limit_ = drv_led_current_limit;
				drv_led_current_limit_changed_ = true;
			}
		}
		void AS726xCommand::setDrvLedEnable(bool drv_led_enable)
		{
			if (drv_led_enable != drv_led_enable_)
			{
				drv_led_enable_ = drv_led_enable;
				drv_led_enable_changed_ = true;
			}
		}
		void AS726xCommand::setConversionType(ConversionTypes conversion_type)
		{
			if ((conversion_type < MODE_0) || (conversion_type >= ConversionTypes_Last))
			{
				ROS_ERROR_STREAM("enum argument out of range in " << __FUNCTION__);
				return;
			}
			if (conversion_type != conversion_type_)
			{
				conversion_type_ = conversion_type;
				conversion_type_changed_ = true;
			}
		}
		void AS726xCommand::setGain(ChannelGain gain)
		{
			if ((gain < GAIN_1X) || (gain >= GAIN_LAST))
			{
				ROS_ERROR_STREAM("enum argument out of range in " << __FUNCTION__);
				return;
			}
			if (gain != gain_)
			{
				gain_ = gain;
				gain_changed_ = true;
			}
		}
		void AS726xCommand::setIntegrationTime(uint8_t integration_time)
		{
			if (integration_time != integration_time_)
			{
				integration_time_ = integration_time;
				integration_time_changed_ = true;
			}
		}

		bool AS726xCommand::indLedCurrentLimitChanged(IndLedCurrentLimits &ind_led_current_limit)
		{
			ind_led_current_limit = ind_led_current_limit_;
			const bool ret = ind_led_current_limit_changed_;
			ind_led_current_limit_changed_ = false;
			return ret;
		}

		bool AS726xCommand::indLedEnableChanged(bool &ind_led_enable)
		{
			ind_led_enable = ind_led_enable_;
			const bool ret = ind_led_enable_changed_;
			ind_led_enable_changed_ = false;
			return ret;
		}

		bool AS726xCommand::drvLedCurrentLimitChanged(DrvLedCurrentLimits &drv_led_current_limit)
		{
			drv_led_current_limit = drv_led_current_limit_;
			const bool ret = drv_led_current_limit_changed_;
			drv_led_current_limit_changed_ = false;
			return ret;
		}

		bool AS726xCommand::drvLedEnableChanged(bool &drv_led_enable)
		{
			drv_led_enable = drv_led_enable_;
			const bool ret = drv_led_enable_changed_;
			drv_led_enable_changed_ =false;
			return ret;
		}

		bool AS726xCommand::conversionTypeChanged(ConversionTypes &conversion_type)
		{
			conversion_type = conversion_type_;
			const bool ret  = conversion_type_changed_;
			conversion_type_changed_ = false;
			return ret;
		}

		bool AS726xCommand::gainChanged(ChannelGain &gain)
		{
			gain = gain_;
			const bool ret = gain_changed_;
			gain_changed_ = false;
			return ret;
		}

		bool AS726xCommand::integrationTimeChanged(uint8_t &integration_time)
		{
			integration_time = integration_time_;
			const bool ret = integration_time_changed_;
			integration_time_changed_ = false;
			return ret;
		}

		void AS726xCommand::resetIndLedCurrentLimit(void) { ind_led_current_limit_changed_ = true; }
		void AS726xCommand::resetIndLedEnable(void)       { ind_led_enable_changed_ = true; }
		void AS726xCommand::resetDrvLedCurrentLimit(void) { drv_led_current_limit_changed_ = true; }
		void AS726xCommand::resetDrvLedEnable(void)       { drv_led_enable_changed_ = true; }
		void AS726xCommand::resetConversionType(void)     { conversion_type_changed_ = true; }
		void AS726xCommand::resetGain(void)               { gain_changed_ = true; }
		void AS726xCommand::resetIntegrationTime(void)    { integration_time_changed_ = true; }

} // namespace as726x
} // namespace hardware_interface
