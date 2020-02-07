#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include "state_handle/command_handle.h"

namespace hardware_interface
{
namespace as726x
{
enum ConversionTypes
{
	MODE_0,
	MODE_1,
	MODE_2,
	ONE_SHOT,
	ConversionTypes_Last
};

/**************************************************************************/
/*!
  @brief gain settings. Default is 1x gain
  */
/**************************************************************************/
enum ChannelGain
{
	GAIN_1X, //default
	GAIN_3X7,
	GAIN_16X,
	GAIN_64X,
	GAIN_LAST
};

/**************************************************************************/
/*!
  @brief  indicator LED current limit settings. Default is 1mA
  */
/**************************************************************************/
enum IndLedCurrentLimits
{
	IND_LIMIT_1MA, //default
	IND_LIMIT_2MA,
	IND_LIMIT_4MA,
	IND_LIMIT_8MA,
	IND_LIMIT_LAST
};

/**************************************************************************/
/*!
  @brief  Driver LED current limit settings. Default is 12.5 mA
  */
/**************************************************************************/
enum DrvLedCurrentLimits
{
	DRV_LIMIT_12MA5, //default
	DRV_LIMIT_25MA,
	DRV_LIMIT_50MA,
	DRV_LIMIT_100MA,
	DRV_LIMIT_LAST,
};


/*=========================================================================*/

constexpr uint8_t AS726x_NUM_CHANNELS = 6; ///< number of sensor channels

/**************************************************************************/
/*!
  @brief  Color definitions used by the library
  */
/**************************************************************************/
enum ColorChannel {
	COLOR_CHANNEL_VIOLET = 0,
	COLOR_CHANNEL_BLUE,
	COLOR_CHANNEL_GREEN,
	COLOR_CHANNEL_YELLOW,
	COLOR_CHANNEL_ORANGE,
	COLOR_CHANNEL_RED,
	COLOR_CHANNEL_LAST
};

class AS726xState
{
	public:
		AS726xState(const std::string &port, int address)
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

		std::string                              getPort(void) const                  { return port_; }
		int                                      getAddress(void) const               { return address_; }
		IndLedCurrentLimits                      getIndLedCurrentLimit(void) const    { return ind_led_current_limit_; }
		bool                                     getIndLedEnable(void) const          { return ind_led_enable_; }
		DrvLedCurrentLimits                      getDrvLedCurrentLimit(void) const    { return drv_led_current_limit_; }
		bool                                     getDrvLedEnable(void) const          { return drv_led_enable_; }
		ConversionTypes                          getConversionType(void) const        { return conversion_type_; }
		ChannelGain                              getGain(void) const                  { return gain_; }
		uint8_t                                  getIntegrationTime(void) const       { return integration_time_; }
		uint8_t                                  getTemperature(void) const           { return temperature_; }
		std::array<uint16_t, COLOR_CHANNEL_LAST> getRawChannelData(void) const        { return raw_channel_data_; }
		std::array<float, COLOR_CHANNEL_LAST>    getCalibratedChannelData(void) const { return calibrated_channel_data_; }


		void setIndLedCurrentLimit(const IndLedCurrentLimits &ind_led_current_limit)
		{
			if ((ind_led_current_limit < IND_LIMIT_1MA) || (ind_led_current_limit >= IND_LIMIT_LAST))
			{
				ROS_ERROR_STREAM("enum argument out of range in " << __FUNCTION__);
				return;
			}
			ind_led_current_limit_ = ind_led_current_limit;
		}
		void setIndLedEnable(bool ind_led_enable)
		{
			ind_led_enable_ = ind_led_enable;
		}
		void setDrvLedCurrentLimit(const DrvLedCurrentLimits &drv_led_current_limit)
		{
			if ((drv_led_current_limit < DRV_LIMIT_12MA5) || (drv_led_current_limit >= DRV_LIMIT_LAST))
			{
				ROS_ERROR_STREAM("enum argument out of range in " << __FUNCTION__);
				return;
			}
			drv_led_current_limit_ = drv_led_current_limit;
		}
		void setDrvLedEnable(bool drv_led_enable)
		{
			drv_led_enable_ = drv_led_enable;
		}
		void setConversionType(const ConversionTypes &conversion_type)
		{
			if ((conversion_type < MODE_0) || (conversion_type >= ConversionTypes_Last))
			{
				ROS_ERROR_STREAM("enum argument out of range in " << __FUNCTION__);
				return;
			}
			conversion_type_ = conversion_type;
		}
		void setGain(const ChannelGain &gain)
		{
			if ((gain < GAIN_1X) || (gain >= GAIN_LAST))
			{
				ROS_ERROR_STREAM("enum argument out of range in " << __FUNCTION__);
				return;
			}
			gain_ = gain;
		}
		void setIntegrationTime(uint8_t integration_time)
		{
			integration_time_ = integration_time;
		}
		void setTemperature(uint8_t temperature)
		{
			temperature_ = temperature;
		}
		void setRawChannelData(const std::array<uint16_t, COLOR_CHANNEL_LAST> &raw_channel_data)
		{
			raw_channel_data_ = raw_channel_data;
		}
		void setCalibratedChannelData(const std::array<float, COLOR_CHANNEL_LAST> &calibrated_channel_data)
		{
			calibrated_channel_data_ = calibrated_channel_data;
		}

	private:
		std::string                              port_;
		int                                      address_;
		IndLedCurrentLimits                      ind_led_current_limit_;
		bool                                     ind_led_enable_;
		DrvLedCurrentLimits                      drv_led_current_limit_;
		bool                                     drv_led_enable_;
		ConversionTypes                          conversion_type_;
		ChannelGain                              gain_;
		uint8_t                                  integration_time_;
		uint8_t                                  temperature_;
		std::array<uint16_t, COLOR_CHANNEL_LAST> raw_channel_data_;
		std::array<float, COLOR_CHANNEL_LAST>    calibrated_channel_data_;
};

class AS726xCommand
{
	public:
		AS726xCommand()
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

		IndLedCurrentLimits getIndLedCurrentLimit(void) const  { return ind_led_current_limit_; }
		bool                getIndLedEnable(void) const        { return ind_led_enable_; }
		DrvLedCurrentLimits getDrvLedCurrentLimit(void) const  { return drv_led_current_limit_; }
		bool                getDrvLedEnable(void) const        { return drv_led_enable_; }
		ConversionTypes     getConversionType(void) const      { return conversion_type_; }
		ChannelGain         getGain(void) const                { return gain_; }
		uint8_t             getIntegrationTime(void) const     { return integration_time_; }

		void setIndLedCurrentLimit(IndLedCurrentLimits ind_led_current_limit)
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
		void setIndLedEnable(bool ind_led_enable)
		{
			if (ind_led_enable != ind_led_enable_)
			{
				ind_led_enable_ = ind_led_enable;
				ind_led_enable_changed_ = true;
			}
		}
		void setDrvLedCurrentLimit(DrvLedCurrentLimits drv_led_current_limit)
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
		void setDrvLedEnable(bool drv_led_enable)
		{
			if (drv_led_enable != drv_led_enable_)
			{
				drv_led_enable_ = drv_led_enable;
				drv_led_enable_changed_ = true;
			}
		}
		void setConversionType(ConversionTypes conversion_type)
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
		void setGain(ChannelGain gain)
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
		void setIntegrationTime(uint8_t integration_time)
		{
			if (integration_time != integration_time_)
			{
				integration_time_ = integration_time;
				integration_time_changed_ = true;
			}
		}

		bool indLedCurrentLimitChanged(IndLedCurrentLimits &ind_led_current_limit)
		{
			ind_led_current_limit = ind_led_current_limit_;
			const bool ret = ind_led_current_limit_changed_;
			ind_led_current_limit_changed_ = false;
			return ret;
		}

		bool indLedEnableChanged(bool &ind_led_enable)
		{
			ind_led_enable = ind_led_enable_;
			const bool ret = ind_led_enable_changed_;
			ind_led_enable_changed_ = false;
			return ret;
		}

		bool drvLedCurrentLimitChanged(DrvLedCurrentLimits &drv_led_current_limit)
		{
			drv_led_current_limit = drv_led_current_limit_;
			const bool ret = drv_led_current_limit_changed_;
			drv_led_current_limit_changed_ = false;
			return ret;
		}

		bool drvLedEnableChanged(bool &drv_led_enable)
		{
			drv_led_enable = drv_led_enable_;
			const bool ret = drv_led_enable_changed_;
			drv_led_enable_changed_ =false;
			return ret;
		}

		bool conversionTypeChanged(ConversionTypes &conversion_type)
		{
			conversion_type = conversion_type_;
			const bool ret  = conversion_type_changed_;
			conversion_type_changed_ = false;
			return ret;
		}

		bool gainChanged(ChannelGain &gain)
		{
			gain = gain_;
			const bool ret = gain_changed_;
			gain_changed_ = false;
			return ret;
		}

		bool integrationTimeChanged(uint8_t &integration_time)
		{
			integration_time = integration_time_;
			const bool ret = integration_time_changed_;
			integration_time_changed_ = false;
			return ret;
		}

		void resetIndLedCurrentLimit(void) { ind_led_current_limit_changed_ = true; }
		void resetIndLedEnable(void)       { ind_led_enable_changed_ = true; }
		void resetDrvLedCurrentLimit(void) { drv_led_current_limit_changed_ = true; }
		void resetDrvLedEnable(void)       { drv_led_enable_changed_ = true; }
		void resetConversionType(void)     { conversion_type_changed_ = true; }
		void resetGain(void)               { gain_changed_ = true; }
		void resetIntegrationTime(void)    { integration_time_changed_ = true; }

	private:
		IndLedCurrentLimits ind_led_current_limit_;
		bool                ind_led_current_limit_changed_;
		bool                ind_led_enable_;
		bool                ind_led_enable_changed_;
		DrvLedCurrentLimits drv_led_current_limit_;
		bool                drv_led_current_limit_changed_;
		bool                drv_led_enable_;
		bool                drv_led_enable_changed_;
		ConversionTypes     conversion_type_;
		bool                conversion_type_changed_;
		ChannelGain         gain_;
		bool                gain_changed_;
		uint8_t             integration_time_;
		bool                integration_time_changed_;
};

typedef StateHandle<const AS726xState> AS726xStateHandle;
typedef StateHandle<AS726xState> AS726xWritableStateHandle;
class AS726xStateInterface: public HardwareResourceManager<AS726xStateHandle> {};
class RemoteAS726xStateInterface : public HardwareResourceManager<AS726xWritableStateHandle, ClaimResources> {};

// Handle - used by each controller to get, by name of the
// corresponding joint, an interface with which to send commands
// to a AS726x
typedef CommandHandle<AS726xCommand, AS726xState, AS726xStateHandle> AS726xCommandHandle;

// Use ClaimResources here since we only want 1 controller
// to be able to access a given AS726x at any particular time
class AS726xCommandInterface : public HardwareResourceManager<AS726xCommandHandle, ClaimResources> {};

} // namespace as726x
} // namespace hardware_interface
