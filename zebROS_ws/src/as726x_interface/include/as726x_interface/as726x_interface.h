// Classes to hold state and command for AS726x hardware interface integration
// The state class hold data read from the hardware interface ::read() method from
//   the sensor hardware. This data is used by controllers that need to access
//   sensor state
// The command class holds commands written to hardware by the hardware interface
//   ::write() method. Controllers can write to this class to send commands to
//   the hardware itself
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
		AS726xState(const std::string &port, int address);

		std::string                              getPort(void) const;
		int                                      getAddress(void) const;
		IndLedCurrentLimits                      getIndLedCurrentLimit(void) const;
		bool                                     getIndLedEnable(void) const;
		DrvLedCurrentLimits                      getDrvLedCurrentLimit(void) const;
		bool                                     getDrvLedEnable(void) const;
		ConversionTypes                          getConversionType(void) const;
		ChannelGain                              getGain(void) const;
		uint8_t                                  getIntegrationTime(void) const;
		uint8_t                                  getTemperature(void) const;
		std::array<uint16_t, COLOR_CHANNEL_LAST> getRawChannelData(void) const;
		std::array<float, COLOR_CHANNEL_LAST>    getCalibratedChannelData(void) const;


		void setIndLedCurrentLimit(const IndLedCurrentLimits &ind_led_current_limit);
		void setIndLedEnable(bool ind_led_enable);
		void setDrvLedCurrentLimit(const DrvLedCurrentLimits &drv_led_current_limit);
		void setDrvLedEnable(bool drv_led_enable);
		void setConversionType(const ConversionTypes &conversion_type);
		void setGain(const ChannelGain &gain);
		void setIntegrationTime(uint8_t integration_time);
		void setTemperature(uint8_t temperature);
		void setRawChannelData(const std::array<uint16_t, COLOR_CHANNEL_LAST> &raw_channel_data);
		void setCalibratedChannelData(const std::array<float, COLOR_CHANNEL_LAST> &calibrated_channel_data);

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
		AS726xCommand(void);

		IndLedCurrentLimits getIndLedCurrentLimit(void) const;
		bool                getIndLedEnable(void) const;
		DrvLedCurrentLimits getDrvLedCurrentLimit(void) const;
		bool                getDrvLedEnable(void) const;
		ConversionTypes     getConversionType(void) const;
		ChannelGain         getGain(void) const;
		uint8_t             getIntegrationTime(void) const;

		void setIndLedCurrentLimit(IndLedCurrentLimits ind_led_current_limit);
		void setIndLedEnable(bool ind_led_enable);
		void setDrvLedCurrentLimit(DrvLedCurrentLimits drv_led_current_limit);
		void setDrvLedEnable(bool drv_led_enable);
		void setConversionType(ConversionTypes conversion_type);
		void setGain(ChannelGain gain);
		void setIntegrationTime(uint8_t integration_time);

		bool indLedCurrentLimitChanged(IndLedCurrentLimits &ind_led_current_limit);
		bool indLedEnableChanged(bool &ind_led_enable);
		bool drvLedCurrentLimitChanged(DrvLedCurrentLimits &drv_led_current_limit);
		bool drvLedEnableChanged(bool &drv_led_enable);
		bool conversionTypeChanged(ConversionTypes &conversion_type);
		bool gainChanged(ChannelGain &gain);
		bool integrationTimeChanged(uint8_t &integration_time);

		void resetIndLedCurrentLimit(void);
		void resetIndLedEnable(void);
		void resetDrvLedCurrentLimit(void);
		void resetDrvLedEnable(void);
		void resetConversionType(void);
		void resetGain(void);
		void resetIntegrationTime(void);

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
