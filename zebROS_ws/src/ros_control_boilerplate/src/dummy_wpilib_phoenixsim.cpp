#include <ros/ros.h>


namespace hal {
namespace init {
void InitializeAccelerometerData()
{
	ROS_INFO("Called InitializeAccelerometerData()");
}
void InitializeAnalogGyroData()
{
	ROS_INFO("Called InitializeAnalogGyroData()");
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

#include <frc/MotorSafety.h>
void frc::MotorSafety::CheckMotors()
{
	ROS_INFO("Called frc::MotorSafety::CheckMotors()");
}
