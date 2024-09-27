// Extra stub functions needed to get wpilib to link against
// the hardware interface on x86 targets
#include <ros/console.h>

#include <frc/DigitalInput.h>
frc::DigitalInput::DigitalInput(int)
{
	ROS_ERROR("Called DigitalInput::DigitalInput(int) on unsupported platform");
}
frc::DigitalInput::~DigitalInput()
{
	ROS_ERROR("Called DigitalInput::~DigitalInput() on unsupported platform");
}
bool frc::DigitalInput::Get() const
{
	ROS_ERROR("Called DigitalInput::Get() const on unsupported platform");
	return false;
}
int frc::DigitalInput::GetChannel() const
{
	ROS_ERROR("Called frc::DigitalInput::GetChannel() const on unsupported platform");
	return std::numeric_limits<int>::max();
}
HAL_Handle frc::DigitalInput::GetPortHandleForRouting() const
{
	ROS_ERROR("Called frc::DigitalInput::GetPortHandleForRouting() const on unsupported platform");
	return HAL_Handle();
}
frc::AnalogTriggerType frc::DigitalInput::GetAnalogTriggerTypeForRouting() const
{
	ROS_ERROR("Called frc::DigitalInput::GetAnalogTriggerTypeForRouting() const on unsupported platform");
	return frc::AnalogTriggerType::kInWindow;
}
bool frc::DigitalInput::IsAnalogTrigger() const
{
	ROS_ERROR("Called frc::DigitalInput::IsAnalogTrigger() const on unsupported platform");
	return false;
}
void frc::DigitalInput::InitSendable(wpi::SendableBuilder&)
{
	ROS_ERROR("Called frc::DigitalInput::InitSendable(SendableBuilder& builder) on unsupported platform");
}

#include <frc/DigitalOutput.h>
frc::DigitalOutput::DigitalOutput(int)
{
	ROS_ERROR("Called DigitalOutput::DigitalOutput(int) on unsupported platform");
}

frc::DigitalOutput::~DigitalOutput()
{
	ROS_ERROR("Called DigitalOutput::~DigitalOutput() on unsupported platform");
}

void frc::DigitalOutput::Set(bool)
{
	ROS_ERROR("Called DigitalOutput::set(bool) on unsupported platform");
}

void frc::DigitalOutput::InitSendable(wpi::SendableBuilder&)
{
	ROS_ERROR("Called frc::DigitalOutput::InitSendable(SendableBuilder& builder) on unsupported platform");
}

HAL_Handle frc::DigitalOutput::GetPortHandleForRouting() const
{
	ROS_ERROR("Called DigitalOutput::GetPortHandleForRouting() on unsupported platform");
	return HAL_kInvalidHandle;
}

frc::AnalogTriggerType frc::DigitalOutput::GetAnalogTriggerTypeForRouting() const
{
	ROS_ERROR("Called DigitalOutput::GetAnalogTriggerTypeForRouting() on unsupported platform");
	return static_cast<frc::AnalogTriggerType>(-1);
}

bool frc::DigitalOutput::IsAnalogTrigger() const
{
	ROS_ERROR("Called DigitalOutput::IsAnalogTrigger() on unsupported platform");
	return false;
}

int frc::DigitalOutput::GetChannel() const
{
	ROS_ERROR("Called DigitalOutput::GetChannel() on unsupported platform");
	return std::numeric_limits<int>::max();
}

// Jetson has DIO pins, so make this stub x86-specific
extern "C" {
HAL_Bool HAL_CheckDIOChannel(int32_t)
{
	ROS_ERROR("Called HAL_CheckDIOChannel(int32_t) on unsupported platform");
	return false;
}

HAL_Bool HAL_CheckAnalogInputChannel(int32_t)
{
	ROS_ERROR("Called HAL_CheckAnalogInputChannel(int32_t) on unsupported platform");
	return false;
}
HAL_Bool HAL_CheckAnalogOutputChannel(int32_t)
{
	ROS_ERROR("Called HAL_CheckAnalogOutputChannel(int32_t) on unsupported platform");
	return false;
}
HAL_Bool HAL_CheckRelayChannel(int32_t)
{
	ROS_ERROR("Called HAL_CheckRelayChannel(int32_t) on unsupported platform");
	return false;
}
HAL_Bool HAL_CheckPWMChannel(int32_t)
{
	ROS_ERROR("Called HAL_CheckPWMChannel(int32_t) on unsupported platform");
	return false;
}

int32_t HAL_GetNumAnalogInputs(void) {
  return 0;
}
int32_t HAL_GetNumAnalogOutputs(void) {
  return 0;
}
int32_t HAL_GetNumDigitalChannels(void) {
  return 0;
}
int32_t HAL_GetNumPWMChannels(void) {
  return 0;
}
int32_t HAL_GetNumRelayHeaders(void) {
  return 0;
}

const char *HALSIM_GetSimDeviceName(HAL_SimDeviceHandle /*handle*/)
{
	ROS_ERROR_STREAM("Call to " << __PRETTY_FUNCTION__ << " on unsupported platform");
	return "";
}

HAL_SimDeviceHandle HALSIM_GetSimValueDeviceHandle(HAL_SimValueHandle /*handle*/)
{
	ROS_ERROR_STREAM("Call to " << __PRETTY_FUNCTION__ << " on unsupported platform");
	return 0;
}

void HALSIM_CancelSimValueChangedCallback(int32_t /*uid*/)
{
	ROS_ERROR_STREAM("Call to " << __PRETTY_FUNCTION__ << " on unsupported platform");
}

#include "hal/simulation/SimDeviceData.h"
int32_t HALSIM_RegisterSimValueChangedCallback(HAL_SimValueHandle /*handle*/,
											   void */*param*/,
											   HALSIM_SimValueCallback /*callback*/,
											   HAL_Bool /*initialNotify*/)
{
	ROS_ERROR_STREAM("Call to " << __PRETTY_FUNCTION__ << " on unsupported platform");
	return 0;
}

#include "hal/simulation/MockHooks.h"
int32_t HALSIM_RegisterSimPeriodicBeforeCallback(HALSIM_SimPeriodicCallback /*callback*/, void */*param*/)
{
	ROS_ERROR_STREAM("Call to " << __PRETTY_FUNCTION__ << " on unsupported platform");
	return 0;
}

void HALSIM_CancelSimPeriodicBeforeCallback(int32_t /*uid*/)
{
	ROS_ERROR_STREAM("Call to " << __PRETTY_FUNCTION__ << " on unsupported platform");
}

}  // extern "C"