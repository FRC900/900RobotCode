// Stub code for wpilib I2C functions - used to make the wpilib
// code compile on platforms which don't support I2C hardware
#include <ros/ros.h>
#include <frc/I2C.h>
frc::I2C::I2C(frc::I2C::Port port, int deviceAddress)
    : m_port(static_cast<HAL_I2CPort>(port)), m_deviceAddress(deviceAddress)
{
	ROS_ERROR("Called I2C::I2C() on unsupported platform");
}
frc::I2C::~I2C()
{
	ROS_ERROR("Called I2C::~I2C() on unsupported platform");
}
bool frc::I2C::Write(int registerAddress, uint8_t data)
{
	ROS_ERROR("Called I2C::Write() on unsupported platform");
	return false;
}
bool frc::I2C::Read(int registerAddress, int count, uint8_t* buffer)
{
	ROS_ERROR("Called I2C::Read() on unsupported platform");
	return false;
}

