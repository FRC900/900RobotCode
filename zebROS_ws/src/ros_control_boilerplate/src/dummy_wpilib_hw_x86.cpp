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
void frc::DigitalInput::InitSendable(SendableBuilder&)
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
void frc::DigitalOutput::InitSendable(SendableBuilder&)
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

#include <frc/ErrorBase.h>
frc::ErrorBase::ErrorBase()
{
	ROS_ERROR("Called ErrorBase::ErrorBase() on unsupported platform");
}

void frc::ErrorBase::ClearError() const
{
	ROS_ERROR("Called ErrorBase::ClearError() const on unsupported platform");
}
void frc::ErrorBase::CloneError(frc::ErrorBase const&) const
{
	ROS_ERROR("Called ErrorBase::CloneError(frc::ErrorBase const&) const on unsupported platform");
}
frc::Error &frc::ErrorBase::GetError()
{
	ROS_ERROR("Called ErrorBase::GetError() on unsupported platform");
	static frc::Error e;
	return e;
}
const frc::Error &frc::ErrorBase::GetError() const
{
	ROS_ERROR("Called ErrorBase::GetError() on unsupported platform");
	static frc::Error e;
	return e;
}
void frc::ErrorBase::SetErrnoError(wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const
{
	ROS_ERROR("Called ErrorBase::SetErrnoError(wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const on unsupported platform");
}
void frc::ErrorBase::SetError(int, wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const
{
	ROS_ERROR("Called ErrorBase::SetError(int, wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const on unsupported platform");
}
void frc::ErrorBase::SetErrorRange(int, int, int, int, wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const
{
	ROS_ERROR("Called ErrorBase::SetErrorRange(int, int, int, int, wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const on unsupported platform");
}
void frc::ErrorBase::SetImaqError(int, wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const
{
	ROS_ERROR("Called ErrorBase::SetImaqError(int, wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const on unsupported platform");
}
void frc::ErrorBase::SetWPIError(wpi::Twine const&, int, wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const
{
	ROS_ERROR("Called ErrorBase::SetWPIError(wpi::Twine const&, int, wpi::Twine const&, wpi::StringRef, wpi::StringRef, int) const on unsupported platform");
}
bool frc::ErrorBase::StatusIsFatal() const
{
	ROS_ERROR("Called ErrorBase::StatusIsFatal() const on unsupported platform");
	return false;
}

void frc::ErrorBase::SetGlobalError(int, wpi::Twine const&, wpi::StringRef, wpi::StringRef, int)
{
	ROS_ERROR_STREAM("Called " << __PRETTY_FUNCTION__ << "on unsupported platform");
}

