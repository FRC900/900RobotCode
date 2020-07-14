// Original copyright :
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "hal/I2C.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <mutex>
#include <sstream>

#include "HALInitializer.h"
#include "hal/HAL.h"

using namespace hal;

static std::vector<std::unique_ptr<std::mutex>> i2CMutex;
static std::vector<uint8_t> i2COjbCount; // because everyone needs 2^64 i2c devices
static std::vector<int> i2CHandle;

namespace hal
{
namespace init
{
void InitializeI2C() {}
}  // namespace init
}  // namespace hal

extern "C" {

void HAL_InitializeI2C(HAL_I2CPort port, int32_t *status)
{
	if (*status != 0)
	{
		return;
	}

	const auto portNum = static_cast<size_t>(port);
	// Want to make size 1 greater than portNum, since
	// portNum is the index and vector indexes run from 0..size-1
	while (i2CMutex.size() <= portNum)
	{
		i2CMutex.push_back(std::make_unique<std::mutex>());
		i2COjbCount.push_back(0);
		i2CHandle.push_back(-1);
	}

	std::scoped_lock lock(*(i2CMutex[portNum]));
	i2COjbCount[portNum] += 1;
	if (i2COjbCount[portNum] > 1)
	{
		return;
	}
	std::stringstream s;
	s << "/dev/i2c-" << portNum;
	const int handle = open(s.str().c_str(), O_RDWR);
	if (handle < 0)
	{
		std::printf("Failed to open onboard i2c bus %s: %s\n",
				s.str().c_str(), std::strerror(errno));
		return;
	}
	i2CHandle[portNum] = handle;
}

int32_t HAL_TransactionI2C(HAL_I2CPort port, int32_t deviceAddress,
						   const uint8_t *dataToSend, int32_t sendSize,
						   uint8_t *dataReceived, int32_t receiveSize)
{
	const auto portNum = static_cast<size_t>(port);
	if (portNum >= i2CMutex.size())
	{
		// Set port out of range error here
		return -1;
	}

	struct i2c_msg msgs[2];
	msgs[0].addr = deviceAddress;
	msgs[0].flags = 0;
	msgs[0].len = sendSize;
	msgs[0].buf = const_cast<uint8_t *>(dataToSend);
	msgs[1].addr = deviceAddress;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = receiveSize;
	msgs[1].buf = dataReceived;

	struct i2c_rdwr_ioctl_data rdwr;
	rdwr.msgs = msgs;
	rdwr.nmsgs = 2;

	std::scoped_lock lock(*(i2CMutex[portNum]));
	return ioctl(i2CHandle[portNum], I2C_RDWR, &rdwr);
}

int32_t HAL_WriteI2C(HAL_I2CPort port, int32_t deviceAddress,
					 const uint8_t *dataToSend, int32_t sendSize)
{
	const auto portNum = static_cast<size_t>(port);
	if (portNum >= i2CMutex.size())
	{
		// Set port out of range error here
		return -1;
	}

	struct i2c_msg msg;
	msg.addr = deviceAddress;
	msg.flags = 0;
	msg.len = sendSize;
	msg.buf = const_cast<uint8_t *>(dataToSend);

	struct i2c_rdwr_ioctl_data rdwr;
	rdwr.msgs = &msg;
	rdwr.nmsgs = 1;

	std::scoped_lock lock(*(i2CMutex[portNum]));
	return ioctl(i2CHandle[portNum], I2C_RDWR, &rdwr);
}

int32_t HAL_ReadI2C(HAL_I2CPort port, int32_t deviceAddress, uint8_t *buffer,
					int32_t count)
{
	const auto portNum = static_cast<size_t>(port);
	if (portNum >= i2CMutex.size())
	{
		// Set port out of range error here
		return -1;
	}

	struct i2c_msg msg;
	msg.addr = deviceAddress;
	msg.flags = I2C_M_RD;
	msg.len = count;
	msg.buf = buffer;

	struct i2c_rdwr_ioctl_data rdwr;
	rdwr.msgs = &msg;
	rdwr.nmsgs = 1;

	std::scoped_lock lock(*(i2CMutex[portNum]));
	return ioctl(i2CHandle[portNum], I2C_RDWR, &rdwr);
}

void HAL_CloseI2C(HAL_I2CPort port)
{
	const auto portNum = static_cast<size_t>(port);
	if (portNum >= i2CMutex.size())
	{
		// Set port out of range error here
		return;
	}
	std::scoped_lock lock(*(i2CMutex[portNum]));
	if (i2COjbCount[portNum] == 1)
	{
		close(i2CHandle[portNum]);
	}
	if (i2COjbCount[portNum] > 0)
	{
		i2COjbCount[portNum] -= 1;
	}
}

}  // extern "C"

