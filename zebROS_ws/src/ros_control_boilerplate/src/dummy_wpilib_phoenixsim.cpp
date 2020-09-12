#include <ros/console.h>
#include <string>             // for string

namespace hal {
namespace init {
void InitializeAccelerometerData()
{
	ROS_INFO("Called InitializeAccelerometerData()");
}
void InitializeAddressableLED()
{
	ROS_INFO("Called InitializeSAddressableLED()");
}
void InitializeAddressableLEDData()
{
	ROS_INFO("Called InitializeSAddressableLEDData()");
}
void InitializeAnalogGyroData()
{
	ROS_INFO("Called InitializeAnalogGyroData()");
}
void InitializeDutyCycle()
{
	ROS_INFO("Called InitializeSDutyCycle()");
}
void InitializeDutyCycleData()
{
	ROS_INFO("Called InitializeSDutyCycleData()");
}
void InitializeEncoderData()
{
	ROS_INFO("Called InitializeEncoderData()");
}
void InitializeI2CData()
{
	ROS_INFO("Called InitializeI2CData()");
}
void InitializeSPIAccelerometerData()
{
	ROS_INFO("Called InitializeSPIAccelerometerData()");
}
void InitializeSPIData()
{
	ROS_INFO("Called InitializeSPIData()");
}
void InitializeAccelerometer()
{
	ROS_INFO("Called InitializeAccelerometer()");
}
void InitializeAnalogGyro()
{
	ROS_INFO("Called InitializeAnalogGyro()");
}
void InitializeEncoder()
{
	ROS_INFO("Called InitializeEncoder()");
}
void InitializeI2C()
{
	ROS_INFO("Called InitializeI2C()");
}
void InitializePCMInternal()
{
	ROS_INFO("Called InitializePCMInternal()");
}
void InitializeSerialPort()
{
	ROS_INFO("Called InitializeSerialPort()");
}
void InitializeSPI()
{
	ROS_INFO("Called InitializeSSPI()");
}
}
}

#include "cscore_c.h"         // for CS_Source, CS_Status
#include "cscore_cpp.h"       // for GetSourceName
// Avoid including all of cscore- this is the only function so far that
// is needed by sim code
std::string cs::GetSourceName(CS_Source source, CS_Status* status) {
	ROS_ERROR("Called cs::GetSourceName (CS_Sourcesource, CS_Status *status) on unsupported platform");
	return "Not a CS Source";
}
