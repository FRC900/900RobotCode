/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/SPI.h"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cstring>
#include <map>
#include <sstream>

#include <wpi/mutex.h>
#include <wpi/raw_ostream.h>

#include "HALInitializer.h"
#include "hal/HAL.h"
#include "hal/handles/HandlesInternal.h"

using namespace hal;

namespace hal
{
namespace init
{
void InitializeSPI() {}
}  // namespace init
}  // namespace hal

extern "C" {
void HAL_InitializeSPI(HAL_SPIPort port, int32_t *status)
{
	if (*status != 0)
	{
		return;
	}
	if (port < 0)
	{
		*status = PARAMETER_OUT_OF_RANGE;
		return;
	}

	if (HAL_GetSPIHandle(port) != -1)
	{
		return;
	}
	std::stringstream s;
	s << "/dev/spidev" << ((port / 10) % 10) << "." << (port % 10);
	const auto handle = open(s.str().c_str(), O_RDWR);
	if (handle < 0)
	{
		std::printf("Failed to open SPI port %d: %s\n", port, std::strerror(errno));
		*status = PARAMETER_OUT_OF_RANGE;
	}
	HAL_SetSPIHandle(port, handle);
}

int32_t HAL_TransactionSPI(HAL_SPIPort port, const uint8_t *dataToSend,
						   uint8_t *dataReceived, int32_t size)
{
	const auto handle = HAL_GetSPIHandle(port);
	if (handle < 0)
	{
		return -1;
	}

	struct spi_ioc_transfer xfer;
	std::memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = (__u64)dataToSend;
	xfer.rx_buf = (__u64)dataReceived;
	xfer.len = size;

	return ioctl(handle, SPI_IOC_MESSAGE(1), &xfer);
}

int32_t HAL_WriteSPI(HAL_SPIPort port, const uint8_t *dataToSend,
					 int32_t sendSize)
{
	const auto handle = HAL_GetSPIHandle(port);
	if (handle < 0)
	{
		return -1;
	}

	struct spi_ioc_transfer xfer;
	std::memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = (__u64)dataToSend;
	xfer.len = sendSize;

	return ioctl(handle, SPI_IOC_MESSAGE(1), &xfer);
}

int32_t HAL_ReadSPI(HAL_SPIPort port, uint8_t *buffer, int32_t count)
{
	const auto handle = HAL_GetSPIHandle(port);
	if (handle < 0)
	{
		return -1;
	}

	struct spi_ioc_transfer xfer;
	std::memset(&xfer, 0, sizeof(xfer));
	xfer.rx_buf = (__u64)buffer;
	xfer.len = count;

	return ioctl(HAL_GetSPIHandle(port), SPI_IOC_MESSAGE(1), &xfer);
}

void HAL_CloseSPI(HAL_SPIPort port)
{
	const auto handle = HAL_GetSPIHandle(port);
	if (handle < 0)
	{
		return;
	}

	close(handle);

	HAL_SetSPIHandle(port, -1);
}

void HAL_SetSPISpeed(HAL_SPIPort port, int32_t speed)
{
	const auto handle = HAL_GetSPIHandle(port);
	if (handle < 0)
	{
		return;
	}
	ioctl(handle, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
}

void HAL_SetSPIOpts(HAL_SPIPort port, HAL_Bool msbFirst,
					HAL_Bool sampleOnTrailing, HAL_Bool clkIdleHigh)
{
	const auto handle = HAL_GetSPIHandle(port);
	if (handle < 0)
	{
		return;
	}

	uint8_t mode = 0;
	mode |= (!msbFirst ? 8 : 0);
	mode |= (clkIdleHigh ? 2 : 0);
	mode |= (sampleOnTrailing ? 1 : 0);

	ioctl(handle, SPI_IOC_WR_MODE, &mode);
}

void HAL_SetSPIChipSelectActiveHigh(HAL_SPIPort port, int32_t *status)
{
	if (*status != 0)
	{
		return;
	}
	*status = PARAMETER_OUT_OF_RANGE;
	return;
}

void HAL_SetSPIChipSelectActiveLow(HAL_SPIPort port, int32_t *status)
{
	if (*status != 0)
	{
		return;
	}
	*status = PARAMETER_OUT_OF_RANGE;
	return;
}

static std::map<HAL_SPIPort, int32_t> spiHandleMap{};

int32_t HAL_GetSPIHandle(HAL_SPIPort port)
{
	if (port < 0)
	{
		return -1;
	}

	const auto h = spiHandleMap.find(port);
	if (h == spiHandleMap.cend())
	{
		return -1;
	}
	return h->second;
}

void HAL_SetSPIHandle(HAL_SPIPort port, int32_t handle)
{
	if (port < 0)
	{
		return;
	}

	spiHandleMap[port] = handle;
}

void HAL_InitSPIAuto(HAL_SPIPort port, int32_t bufferSize, int32_t *status)
{
	*status = PARAMETER_OUT_OF_RANGE;
	return;
}

void HAL_StartSPIAutoRate(HAL_SPIPort port, double period, int32_t *status)
{
	*status = INCOMPATIBLE_STATE;
	return;
}

void HAL_StartSPIAutoTrigger(HAL_SPIPort port, HAL_Handle digitalSourceHandle,
							 HAL_AnalogTriggerType analogTriggerType,
							 HAL_Bool triggerRising, HAL_Bool triggerFalling,
							 int32_t *status)
{
	*status = INCOMPATIBLE_STATE;
	return;
}

void HAL_StopSPIAuto(HAL_SPIPort port, int32_t *status)
{
	*status = INCOMPATIBLE_STATE;
	return;
}

void HAL_SetSPIAutoTransmitData(HAL_SPIPort port, const uint8_t *dataToSend,
								int32_t dataSize, int32_t zeroSize,
								int32_t *status)
{
	*status = INCOMPATIBLE_STATE;
	return;
}

void HAL_ForceSPIAutoRead(HAL_SPIPort port, int32_t *status)
{
	*status = INCOMPATIBLE_STATE;
	return;
}

int32_t HAL_ReadSPIAutoReceivedData(HAL_SPIPort port, uint32_t *buffer,
									int32_t numToRead, double timeout,
									int32_t *status)
{
	*status = INCOMPATIBLE_STATE;
	return 0;
}

int32_t HAL_GetSPIAutoDroppedCount(HAL_SPIPort port, int32_t *status)
{
	*status = INCOMPATIBLE_STATE;
	return 0;
}

void HAL_ConfigureSPIAutoStall(HAL_SPIPort port, int32_t csToSclkTicks,
							   int32_t stallTicks, int32_t pow2BytesPerRead,
							   int32_t *status)
{
	*status = INCOMPATIBLE_STATE;
	return;
}

}  // extern "C"

