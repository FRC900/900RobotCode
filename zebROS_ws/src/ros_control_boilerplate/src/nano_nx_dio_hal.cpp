// Code to handle IO with Nano NX carrier board general-purpose IO
// This includes a class to map from board pin number to chip GPIO number
// Also includes code which uses libgpiod to access those pins

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "hal/DIO.h"
#include "hal/Types.h"
#include "DigitalInternal.h"

#include <gpiod.h>

// These arrays contain tuples of all the relevant GPIO data for each Jetson
// Platform. The fields are:
// - Linux GPIO pin number,
// - GPIO chip sysfs directory
// - Pin number (BOARD mode)
// - Pin number (BCM mode)
// - Pin name (CVM mode)
// - Pin name (TEGRA_SOC mode)
// - PWM chip sysfs directory
// - PWM ID within PWM chip
// The values are use to generate dictionaries that map the corresponding pin
// mode numbers to the Linux GPIO pin number and GPIO chip directory

class GPIOPin
{
	public:
		GPIOPin(
				uint8_t            linuxPin,
				std::string        sysfsDir,
				uint8_t            boardPin,
				uint8_t            bcmPin,
				const std::string &pinNameCVM,
				const std::string &pinNameTegraSOC,
				const std::string &pwmSysfsDir,
				uint8_t            pwmID)
	: linuxPin_{linuxPin}
	, sysfsDir_{sysfsDir}
	, boardPin_{boardPin}
	, bcmPin_{bcmPin}
	, pinNameCVM_{pinNameCVM}
	, pinNameTegraSOC_{pinNameTegraSOC}
	, pwmSysfsDir_{pwmSysfsDir}
	, pwmID_{pwmID}
	, inUse_{0}
    , isInput_{false}
	, isPWM_{false}
	{
	}

	uint8_t     linuxPin_;
	std::string sysfsDir_;
	uint8_t     boardPin_;
	uint8_t     bcmPin_;
	std::string pinNameCVM_;
	std::string pinNameTegraSOC_;
	std::string pwmSysfsDir_;
	uint8_t     pwmID_;
	size_t      inUse_;
	bool        isInput_;
	bool        isPWM_;
};

static const std::vector<GPIOPin> JETSON_NX_PIN_DEFS = {
    {148, "/sys/devices/2200000.gpio", 7, 4, "GPIO09", "AUD_MCLK", "", 0},
    {140, "/sys/devices/2200000.gpio", 11, 17, "UART1_RTS", "UART1_RTS", "", 0},
    {157, "/sys/devices/2200000.gpio", 12, 18, "I2S0_SCLK", "DAP5_SCLK", "", 0},
    {192, "/sys/devices/2200000.gpio", 13, 27, "SPI1_SCK", "SPI3_SCK", "", 0},
    {20,  "/sys/devices/c2f0000.gpio", 15, 22, "GPIO12", "TOUCH_CLK", "", 0},
    {196, "/sys/devices/2200000.gpio", 16, 23, "SPI1_CS1", "SPI3_CS1_N", "", 0},
    {195, "/sys/devices/2200000.gpio", 18, 24, "SPI1_CS0", "SPI3_CS0_N", "", 0},
    {205, "/sys/devices/2200000.gpio", 19, 10, "SPI0_MOSI", "SPI1_MOSI", "", 0},
    {204, "/sys/devices/2200000.gpio", 21, 9, "SPI0_MISO", "SPI1_MISO", "", 0},
    {193, "/sys/devices/2200000.gpio", 22, 25, "SPI1_MISO", "SPI3_MISO", "", 0},
    {203, "/sys/devices/2200000.gpio", 23, 11, "SPI0_SCK", "SPI1_SCK", "", 0},
    {206, "/sys/devices/2200000.gpio", 24, 8, "SPI0_CS0", "SPI1_CS0_N", "", 0},
    {207, "/sys/devices/2200000.gpio", 26, 7, "SPI0_CS1", "SPI1_CS1_N", "", 0},
    {133, "/sys/devices/2200000.gpio", 29, 5, "GPIO01", "SOC_GPIO41", "", 0},
    {134, "/sys/devices/2200000.gpio", 31, 6, "GPIO11", "SOC_GPIO42", "", 0},
    {136, "/sys/devices/2200000.gpio", 32, 12, "GPIO07", "SOC_GPIO44", "/sys/devices/32f0000.pwm", 0},
    {105, "/sys/devices/2200000.gpio", 33, 13, "GPIO13", "SOC_GPIO54", "/sys/devices/3280000.pwm", 0},
    {160, "/sys/devices/2200000.gpio", 35, 19, "I2S0_FS", "DAP5_FS", "", 0},
    {141, "/sys/devices/2200000.gpio", 36, 16, "UART1_CTS", "UART1_CTS", "", 0},
    {194, "/sys/devices/2200000.gpio", 37, 26, "SPI1_MOSI", "SPI3_MOSI", "", 0},
    {159, "/sys/devices/2200000.gpio", 38, 20, "I2S0_DIN", "DAP5_DIN", "", 0},
    {158, "/sys/devices/2200000.gpio", 40, 21, "I2S0_DOUT", "DAP5_DOUT", "", 0}
};

static const std::vector<GPIOPin> JETSON_NANO_PIN_DEFS = {
    {216, "/sys/devices/6000d000.gpio", 7, 4, "GPIO9", "AUD_MCLK", "", 0},
    {50, "/sys/devices/6000d000.gpio", 11, 17, "UART1_RTS", "UART2_RTS", "", 0},
    {79, "/sys/devices/6000d000.gpio", 12, 18, "I2S0_SCLK", "DAP4_SCLK", "", 0},
    {14, "/sys/devices/6000d000.gpio", 13, 27, "SPI1_SCK", "SPI2_SCK", "", 0},
    {194, "/sys/devices/6000d000.gpio", 15, 22, "GPIO12", "LCD_TE", "", 0},
    {232, "/sys/devices/6000d000.gpio", 16, 23, "SPI1_CS1", "SPI2_CS1", "", 0},
    {15, "/sys/devices/6000d000.gpio", 18, 24, "SPI1_CS0", "SPI2_CS0", "", 0},
    {16, "/sys/devices/6000d000.gpio", 19, 10, "SPI0_MOSI", "SPI1_MOSI", "", 0},
    {17, "/sys/devices/6000d000.gpio", 21, 9, "SPI0_MISO", "SPI1_MISO", "", 0},
    {13, "/sys/devices/6000d000.gpio", 22, 25, "SPI1_MISO", "SPI2_MISO", "", 0},
    {18, "/sys/devices/6000d000.gpio", 23, 11, "SPI0_SCK", "SPI1_SCK", "", 0},
    {19, "/sys/devices/6000d000.gpio", 24, 8, "SPI0_CS0", "SPI1_CS0", "", 0},
    {20, "/sys/devices/6000d000.gpio", 26, 7, "SPI0_CS1", "SPI1_CS1", "", 0},
    {149, "/sys/devices/6000d000.gpio", 29, 5, "GPIO01", "CAM_AF_EN", "", 0},
    {200, "/sys/devices/6000d000.gpio", 31, 6, "GPIO11", "GPIO_PZ0", "", 0},
    // Older versions of L4T have a DT bug which instantiates a bogus device
    // which prevents this library from using this PWM channel.
    {168, "/sys/devices/6000d000.gpio", 32, 12, "GPIO07", "LCD_BL_PW", "/sys/devices/7000a000.pwm", 0},
    {38, "/sys/devices/6000d000.gpio", 33, 13, "GPIO13", "GPIO_PE6", "/sys/devices/7000a000.pwm", 2},
    {76, "/sys/devices/6000d000.gpio", 35, 19, "I2S0_FS", "DAP4_FS", "", 0},
    {51, "/sys/devices/6000d000.gpio", 36, 16, "UART1_CTS", "UART2_CTS", "", 0},
    {12, "/sys/devices/6000d000.gpio", 37, 26, "SPI1_MOSI", "SPI2_MOSI", "", 0},
    {77, "/sys/devices/6000d000.gpio", 38, 20, "I2S0_DIN", "DAP4_DIN", "", 0},
    {78, "/sys/devices/6000d000.gpio", 40, 21, "I2S0_DOUT", "DAP4_DOUT", "", 0}
};

static const std::vector<GPIOPin> JETSON_XAVIER_PIN_DEFS = {
    {134, "/sys/devices/2200000.gpio", 7, 4, "MCLK05", "SOC_GPIO42", "", 0},
    {140, "/sys/devices/2200000.gpio", 11, 17, "UART1_RTS", "UART1_RTS", "", 0},
    {63, "/sys/devices/2200000.gpio", 12, 18, "I2S2_CLK", "DAP2_SCLK", "", 0},
    {136, "/sys/devices/2200000.gpio", 13, 27, "PWM01", "SOC_GPIO44", "/sys/devices/32f0000.pwm", 0},
    // Older versions of L4T don"t enable this PWM controller in DT, so this PWM
    // channel may not be available.
    {105, "/sys/devices/2200000.gpio", 15, 22, "GPIO27", "SOC_GPIO54", "/sys/devices/3280000.pwm", 0},
    {8, "/sys/devices/c2f0000.gpio", 16, 23, "GPIO8", "CAN1_STB", "", 0},
    {56, "/sys/devices/2200000.gpio", 18, 24, "GPIO35", "SOC_GPIO12", "/sys/devices/32c0000.pwm", 0},
    {205, "/sys/devices/2200000.gpio", 19, 10, "SPI1_MOSI", "SPI1_MOSI", "", 0},
    {204, "/sys/devices/2200000.gpio", 21, 9, "SPI1_MISO", "SPI1_MISO", "", 0},
    {129, "/sys/devices/2200000.gpio", 22, 25, "GPIO17", "SOC_GPIO21", "", 0},
    {203, "/sys/devices/2200000.gpio", 23, 11, "SPI1_CLK", "SPI1_SCK", "", 0},
    {206, "/sys/devices/2200000.gpio", 24, 8, "SPI1_CS0_N", "SPI1_CS0_N", "", 0},
    {207, "/sys/devices/2200000.gpio", 26, 7, "SPI1_CS1_N", "SPI1_CS1_N", "", 0},
    {3, "/sys/devices/c2f0000.gpio", 29, 5, "CAN0_DIN", "CAN0_DIN", "", 0},
    {2, "/sys/devices/c2f0000.gpio", 31, 6, "CAN0_DOUT", "CAN0_DOUT", "", 0},
    {9, "/sys/devices/c2f0000.gpio", 32, 12, "GPIO9", "CAN1_EN", "", 0},
    {0, "/sys/devices/c2f0000.gpio", 33, 13, "CAN1_DOUT", "CAN1_DOUT", "", 0},
    {66, "/sys/devices/2200000.gpio", 35, 19, "I2S2_FS", "DAP2_FS", "", 0},
    // Input-only {due to base board}
    {141, "/sys/devices/2200000.gpio", 36, 16, "UART1_CTS", "UART1_CTS", "", 0},
    {1, "/sys/devices/c2f0000.gpio", 37, 26, "CAN1_DIN", "CAN1_DIN", "", 0},
    {65, "/sys/devices/2200000.gpio", 38, 20, "I2S2_DIN", "DAP2_DIN", "", 0},
    {64, "/sys/devices/2200000.gpio", 40, 21, "I2S2_DOUT", "DAP2_DOUT", "", 0}
};

class GPIOPins
{
	using PinHandle = std::map<uint8_t, GPIOPin>::iterator;
	public :
		// TODO - make the GPIO pin array depend on what's read from compat file
		GPIOPins(void)
		{
			for (const auto &p: JETSON_NX_PIN_DEFS)
			{
				gpioPinMap_.emplace(p.boardPin_, p);
			}
		}
		bool allocateDigitalIO(uint8_t pinNum, bool input)
		{
			PinHandle ph;
			if (!getByPinNum(pinNum, ph))
			{
				return false;
			}
			// Allow duplicate references to the same pin
			// if the input/output direction is the same for all of them
			if (ph->second.inUse_)
			{
				if ((ph->second.isInput_ != input) || ph->second.isPWM_)
				return false;
			}
			// Set the pin in use, set the correct io direction and mark it as non-PWM (PWM is a TODO)
			ph->second.inUse_   += 1;
			ph->second.isInput_  = input;
			ph->second.isPWM_    = false;
			return true;
		}
		// Remove a reference count to this device
		// Return count of remaining references
		size_t freeDigitalIO(uint8_t pinNum)
		{
			PinHandle ph;
			if (!getByPinNum(pinNum, ph))
			{
				return std::numeric_limits<size_t>::max();
			}
			if (!ph->second.inUse_)
			{
				return std::numeric_limits<size_t>::max();
			}
			ph->second.inUse_ -= 1;
			return ph->second.inUse_;
		}
		bool getLinuxPin(uint8_t pinNum, uint8_t &linuxPin)
		{
			PinHandle ph;
			if (!getByPinNum(pinNum, ph))
			{
				return false;
			}
			linuxPin = ph->second.linuxPin_;
			return true;
		}
		bool getPinOutputEnable(uint8_t pinNum, bool &outputEnable)
		{
			PinHandle ph;
			if (!getByPinNum(pinNum, ph))
			{
				return false;
			}
			outputEnable = !ph->second.isInput_;
			return true;
		}
		bool isValidChannel(uint8_t pinNum)
		{
			return gpioPinMap_.find(pinNum) != gpioPinMap_.end();
		}
	private:
		bool getByPinNum(uint8_t pinNum, PinHandle &pin)
		{
			pin = gpioPinMap_.find(pinNum);
			if (pin == gpioPinMap_.end())
				return false;
			return true;
		}
		// Map from pin number to GPIO data
		std::map<uint8_t, GPIOPin> gpioPinMap_;
} gpioPins;


#ifdef __cplusplus
extern "C" {
#endif

static std::map<uint8_t, struct gpiod_line *> gpiodLineMap;
static struct gpiod_chip *gpiodChip = nullptr;
HAL_DigitalHandle HAL_InitializeDIOPort(HAL_PortHandle portHandle,
                                        HAL_Bool input, int32_t* status)
{
	ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " portHandle = " << portHandle << " input = " << static_cast<int>(input));
	if (*status)
		return HAL_kInvalidHandle;

	// portHandle will be a NX pin number
	// check that pin is usable as DIn/Out
	//   - all should be OK, assuming they're not already allocated to
	//     a different direction (i.e. can't reuse an out and an in

	if (!gpioPins.allocateDigitalIO(portHandle, input))
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : allocateDigitalIO failed for port " << portHandle);
		*status = HAL_HANDLE_ERROR;
		return HAL_kInvalidHandle;
	}
	// Create gpiod struct needed to access GPIO hardware
	uint8_t offset;
	if (!gpioPins.getLinuxPin(portHandle, offset))
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : getLinuxPins failed for port " << portHandle);
		*status = HAL_HANDLE_ERROR;
		return HAL_kInvalidHandle;
	}
	ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " : getLinuxPin returned offset = " << static_cast<int>(offset));
	if (!gpiodChip)
		gpiodChip = gpiod_chip_open_lookup("tegra-gpio");
	if (!gpiodChip)
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : gpiod_chip_open_lookup(\"tegra-gpio\") failed");
		*status = HAL_HANDLE_ERROR;
		return HAL_kInvalidHandle;
	}
	// Grab and store line info - this lets the code access a particular
	// pin connected to a particular device on the board
	const auto line = gpiod_chip_get_line(gpiodChip, offset);
	if (!line)
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : gpiod_chip_ge_line(gpiodChip, " << offset << ") failed (pin = " << portHandle << ") - wrong pin -> offset?");
		*status = HAL_HANDLE_ERROR;
		return HAL_kInvalidHandle;
	}
	ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " : line = " << line << ", line_is_free = " << static_cast<int>(gpiod_line_is_free(line)) << " line_is_requested = " << static_cast<int>(gpiod_line_is_requested(line)));
	gpiodLineMap[portHandle] = line;

	// Set the line to either input or output. Since we don't ever
	// change it it is fine to set once and leave it for the remainder
	// of the lifetime of the code
	errno = 0;
	gpiod_line_release(line);
	if (input)
	{
		if (gpiod_line_request_input(line, "") < 0)
		{
			ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : dioPortHandle = " << portHandle << " gpiod_line_request_input() failed, errno = " << errno);
			*status = HAL_HANDLE_ERROR;
			return HAL_kInvalidHandle;
		}
	}
	else if (gpiod_line_request_output(line, "", 0) < 0)
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : dioPortHandle = " << portHandle << " : gpiod_line_request_output failed, errno = " << errno);
		*status = HAL_HANDLE_ERROR;
		return HAL_kInvalidHandle;
	}

	return portHandle;
}


HAL_Bool HAL_CheckDIOChannel(int32_t channel)
{
	return gpioPins.isValidChannel(channel);
}


void HAL_FreeDIOPort(HAL_DigitalHandle dioPortHandle)
{
	if (gpioPins.freeDigitalIO(dioPortHandle) == 0)
	{
		auto it = gpiodLineMap.find(dioPortHandle);
		if (it != gpiodLineMap.end())
		{
			gpiodLineMap.erase(it);
		}
	}
}


void HAL_SetDIOSimDevice(HAL_DigitalHandle handle, HAL_SimDeviceHandle device)
{
}


HAL_DigitalPWMHandle HAL_AllocateDigitalPWM(int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
	return HAL_kInvalidHandle;
}


void HAL_FreeDigitalPWM(HAL_DigitalPWMHandle pwmGenerator, int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
}


void HAL_SetDigitalPWMRate(double rate, int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
}


void HAL_SetDigitalPWMDutyCycle(HAL_DigitalPWMHandle pwmGenerator,
                                double dutyCycle, int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
}


void HAL_SetDigitalPWMOutputChannel(HAL_DigitalPWMHandle pwmGenerator,
                                    int32_t channel, int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
}


void HAL_SetDIO(HAL_DigitalHandle dioPortHandle, HAL_Bool value,
                int32_t* status)
{
	if (*status)
	{
		return;
	}

	// Check pin is valid output
	bool outputEnable;
	if (!gpioPins.getPinOutputEnable(dioPortHandle, outputEnable) || !outputEnable)
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : dioPortHandle = " << dioPortHandle << " is not an output");
		*status = HAL_HANDLE_ERROR;
		return;
	}
	// Get value from gpiodLineMap
	const auto gpiodLine = gpiodLineMap.find(dioPortHandle);
	if (gpiodLine == gpiodLineMap.end())
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : dioPortHandle = " << dioPortHandle << " is not in gpiodLineMap");
		*status = HAL_HANDLE_ERROR;
		return;
	}

	// Actually change the value of the pin
	errno = 0;
	const auto rc = gpiod_line_set_value(gpiodLine->second, value);
	if (rc < 0)
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : dioPortHandle = " << dioPortHandle << " gpiod_line_set_value() failed : errno = " << errno);
		*status = HAL_HANDLE_ERROR;
	}
}


void HAL_SetDIODirection(HAL_DigitalHandle dioPortHandle, HAL_Bool input,
                         int32_t* status)
{
		*status = HAL_HANDLE_ERROR;
		return;
}


HAL_Bool HAL_GetDIO(HAL_DigitalHandle dioPortHandle, int32_t* status)
{
	if (*status)
	{
		return false;
	}

	// Check pin is valid output
	bool outputEnable;
	if (!gpioPins.getPinOutputEnable(dioPortHandle, outputEnable) || outputEnable)
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : dioPortHandle = " << dioPortHandle << " is not an input");
		*status = HAL_HANDLE_ERROR;
		return false;
	}
	// Get value from gpiodLineMap
	const auto gpiodLine = gpiodLineMap.find(dioPortHandle);
	if (gpiodLine == gpiodLineMap.end())
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : dioPortHandle = " << dioPortHandle << " is not in gpiodLineMap");
		*status = HAL_HANDLE_ERROR;
		return false;
	}

	errno = 0;
	const auto rc = gpiod_line_get_value(gpiodLine->second);
	if (rc < 0)
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : dioPortHandle = " << dioPortHandle << " gpiod_line_get_value() failed : errno = " << errno);
		*status = HAL_HANDLE_ERROR;
		return false;
	}

	return static_cast<bool>(rc);
}


HAL_Bool HAL_GetDIODirection(HAL_DigitalHandle dioPortHandle, int32_t* status)
{
	// Check pin is valid output
	bool outputEnable;
	if (!gpioPins.getPinOutputEnable(dioPortHandle, outputEnable))
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " : dioPortHandle = " << dioPortHandle << " : failed to read from gpioPins");
		*status = HAL_HANDLE_ERROR;
		return false;
	}
	return outputEnable;
}


void HAL_Pulse(HAL_DigitalHandle dioPortHandle, double pulseLength,
               int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
	return;
}


HAL_Bool HAL_IsPulsing(HAL_DigitalHandle dioPortHandle, int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
	return false;
}


HAL_Bool HAL_IsAnyPulsing(int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
	return false;
}


void HAL_SetFilterSelect(HAL_DigitalHandle dioPortHandle, int32_t filterIndex,
                         int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
	return;
}


int32_t HAL_GetFilterSelect(HAL_DigitalHandle dioPortHandle, int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
	return 0;
}


void HAL_SetFilterPeriod(int32_t filterIndex, int64_t value, int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
	return;
}


int64_t HAL_GetFilterPeriod(int32_t filterIndex, int32_t* status)
{
	*status = HAL_HANDLE_ERROR;
	return 0;
}

#ifdef __cplusplus
}  // extern "C"
#endif


#ifdef __cplusplus
extern "C" {
#endif
// From wpilib HAL.cpp
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

HAL_PortHandle HAL_GetPort(int32_t channel) {
  // Dont allow a number that wouldn't fit in a uint8_t
  if (channel < 0 || channel >= 255) return HAL_kInvalidHandle;
  return hal::createPortHandle(channel, 1);
}

int32_t HAL_GetNumAccumulators(void) { return 0; }
int32_t HAL_GetNumAnalogTriggers(void) { return 0; }
int32_t HAL_GetNumAnalogInputs(void) { return 0; }
int32_t HAL_GetNumAnalogOutputs(void) { return 0; }
int32_t HAL_GetNumCounters(void) { return 0; }
int32_t HAL_GetNumDigitalHeaders(void) { return 1; }
int32_t HAL_GetNumPWMHeaders(void) { return 1; }
int32_t HAL_GetNumDigitalChannels(void) { return 40; }
int32_t HAL_GetNumPWMChannels(void) { return 0; }
int32_t HAL_GetNumDigitalPWMOutputs(void) { return 2; }
int32_t HAL_GetNumEncoders(void) { return 0; }
int32_t HAL_GetNumInterrupts(void) { return 0; }
int32_t HAL_GetNumRelayChannels(void) { return 0; }
int32_t HAL_GetNumRelayHeaders(void) { return 0; }
int32_t HAL_GetNumPCMModules(void) { return 0; }
int32_t HAL_GetNumSolenoidChannels(void) { return 0; }
int32_t HAL_GetNumPDPModules(void) { return 0; }
int32_t HAL_GetNumPDPChannels(void) { return 0; }
int32_t HAL_GetNumDutyCycles(void) { return 0; }
int32_t HAL_GetNumAddressableLEDs(void) { return 0; }

HAL_Bool HAL_CheckAnalogOutputChannel(int32_t channel) {
	return false; // Nope
}
HAL_Bool HAL_CheckAnalogInputChannel(int32_t channel) {
	return false; // Nope
}
HAL_Bool HAL_CheckPWMChannel(int32_t channel) {
	return (channel > 0) && (channel < HAL_GetNumPWMChannels());
}

HAL_Bool HAL_CheckRelayChannel(int32_t channel) {
	return false; // Nope
}

#ifdef __cplusplus
}  // extern "C"
#endif

#include <frc/DriverStation.h>
double frc::DriverStation::GetMatchTime(void) const
{
	ROS_ERROR("Called DriverStation::GetMatchTime() on unsupported platform");
	return std::numeric_limits<double>::max();
}

#include <wpi/SmallString.h>
#include <hal/DriverStation.h>
void frc::DriverStation::ReportError(bool isError, int32_t code,
                                const wpi::Twine& error,
                                const wpi::Twine& location,
                                const wpi::Twine& stack) {
  wpi::SmallString<128> errorTemp;
  wpi::SmallString<128> locationTemp;
  wpi::SmallString<128> stackTemp;
  HAL_SendError(isError, code, 0,
                error.toNullTerminatedStringRef(errorTemp).data(),
                location.toNullTerminatedStringRef(locationTemp).data(),
                stack.toNullTerminatedStringRef(stackTemp).data(), 1);
}


#include <frc/Timer.h>
namespace frc {
double GetTime()
{
	using std::chrono::duration;
	using std::chrono::duration_cast;
	using std::chrono::system_clock;
	return duration_cast<duration<double>>(system_clock::now().time_since_epoch()).count();
}
}

