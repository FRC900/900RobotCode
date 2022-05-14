// Stub functions to replace WPIlib code which accesses hardware
// which doesn't exist on certain platforms.
#include <ros/console.h>

#include <WPILibVersion.h>
const char* GetWPILibVersion(void)
{
	ROS_ERROR("Called GetWPILibVersion on unsupported platform");
	return "900.2022";
}

#include <AHRS.h>
AHRS::AHRS(frc::SPI::Port)
{
	ROS_ERROR("Called AHRS(SPI::Port spi_port_id) on unsupported platform");
}

float  AHRS::AHRS::GetPitch()
{
	ROS_ERROR("Called GetPitch() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::AHRS::GetRoll()
{
	ROS_ERROR("Called GetRoll() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::AHRS::GetYaw()
{
	ROS_ERROR("Called GetYaw() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetWorldLinearAccelX()
{
	ROS_ERROR("Called GetWorldLinearAccelX() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetWorldLinearAccelY()
{
	ROS_ERROR("Called GetWorldLinearAccelY() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetWorldLinearAccelZ()
{
	ROS_ERROR("Called GetWorldLinearAccelZ() on unsupported platform");
	return std::numeric_limits<float>::max();
}

float  AHRS::GetVelocityX()
{
	ROS_ERROR("Called GetVelocityX() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetVelocityY()
{
	ROS_ERROR("Called GetVelocityY() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetVelocityZ()
{
	ROS_ERROR("Called GetVelocityZ() on unsupported platform");
	return std::numeric_limits<float>::max();
}
double AHRS::GetAngle() const
{
	ROS_ERROR("Called GetAngle() on unsupported platform");
	return std::numeric_limits<double>::max();
}
double AHRS::GetRate() const
{
	ROS_ERROR("Called GetRate() on unsupported platform");
	return std::numeric_limits<double>::max();
}
void   AHRS::Reset()
{
	ROS_ERROR("Called Reset() on unsupported platform");
}
void AHRS::InitSendable(wpi::SendableBuilder&)
{
	ROS_ERROR("Called AHRS::InitSendable(wpi::SendableBuilder&) on unsupported platform");
}
void AHRS::Calibrate()
{
	ROS_ERROR("Called AHRS::Calibrate() on unsupported platform");
}

#include <frc/motorcontrol/NidecBrushless.h>
frc::NidecBrushless::NidecBrushless(int pwmChannel, int dioChannel) : m_dio(dioChannel), m_pwm(pwmChannel)
{
	ROS_ERROR("Called NidecBrushless::NidecBrushless(int, int) on unsupported platform");
}

void frc::NidecBrushless::Set(double)
{
	ROS_ERROR("Called ::NidecBrushless::Set(double speed) on unsupported platform");
}
double frc::NidecBrushless::Get() const
{
	ROS_ERROR("Called ::NidecBrushless::Get() const on unsupported platform");
	return std::numeric_limits<double>::max();
}
void frc::NidecBrushless::SetInverted(bool)
{
	ROS_ERROR("Called ::NidecBrushless::SetInverted(bool isInverted) on unsupported platform");
}
bool frc::NidecBrushless::GetInverted() const
{
	ROS_ERROR("Called ::NidecBrushless::GetInverted() const on unsupported platform");
	return false;
}
void frc::NidecBrushless::Disable()
{
	ROS_ERROR("Called ::NidecBrushless::Disable() on unsupported platform");
}
void frc::NidecBrushless::StopMotor()
{
	ROS_ERROR("Called ::NidecBrushless::StopMotor() on unsupported platform");
}

std::string frc::NidecBrushless::GetDescription() const
{
	ROS_ERROR("Called ::NidecBrushless::GetDescription(wpi::raw_ostream& desc) const on unsupported platform");
	return "";
}

void frc::NidecBrushless::InitSendable(wpi::SendableBuilder&)
{
	ROS_ERROR("Called ::NidecBrushless::InitSendable(SendableBuilder& builder) on unsupported platform");
}

#include <frc/PWM.h>
frc::PWM::PWM(int, bool)
{
	ROS_ERROR("Called PWM::PWM(int) on unsupported platform");
}
frc::PWM::~PWM()
{
	ROS_ERROR("Called PWM::~PWM() on unsupported platform");
}
void frc::PWM::SetRaw(uint16_t)
{
	ROS_ERROR("Called PWM::SetRaw(uint16_t value) on unsupported platform");
}
uint16_t frc::PWM::GetRaw() const
{
	ROS_ERROR("Called PWM::GetRaw() const on unsupported platform");
	return -1;
}
void frc::PWM::SetPosition(double)
{
	ROS_ERROR("Called PWM::SetPosition(double pos) on unsupported platform");
}
double frc::PWM::GetPosition() const
{
	ROS_ERROR("Called PWM::GetPosition() const on unsupported platform");
	return std::numeric_limits<double>::max();
}
void frc::PWM::SetSpeed(double)
{
	ROS_ERROR("Called PWM::SetSpeed(double speed) on unsupported platform");
}
double frc::PWM::GetSpeed() const
{
	ROS_ERROR("Called PWM::GetSpeed() const on unsupported platform");
	return std::numeric_limits<double>::max();
}
void frc::PWM::SetDisabled()
{
	ROS_ERROR("Called PWM::SetDisabled() on unsupported platform");
}
void frc::PWM::InitSendable(wpi::SendableBuilder&)
{
	ROS_ERROR("Called PWM::InitSendable(wpi::SendableBuilder& builder) on unsupported platform");
}

#include <frc/PIDSource.h>
void frc::PIDSource::SetPIDSourceType(PIDSourceType)
{
	ROS_ERROR("Called frc::PIDSource::SetPIDSourceType(PIDSourceType pidSource) on unsupported platform");
}
frc::PIDSourceType frc::PIDSource::GetPIDSourceType() const
{
	ROS_ERROR("Called frc::PIDSource::GetPIDSourceType() on unsupported platform");
	return frc::PIDSourceType::kDisplacement;
}

#include <frc/SpeedController.h>
void frc::SpeedController::SetVoltage(units::volt_t /*output*/)
{
	ROS_ERROR("Called frc::SpeedController::Set(volt_t output) on unsupported platform");
}

// Code to stub out various Senable implementations. These should quietly do nothing.
// The plan is bypassing the default WPIlib code will let us get rid of a lot of other
// unused functions - networktables, etc.
//
#include "wpi/sendable/SendableRegistry.h"

namespace wpi{

void SendableRegistry::AddLW(Sendable* /*sendable*/, std::string_view /*moduleType*/, int /*channel*/)
{
	//ROS_ERROR("Called SendableRegistry::AddLW(Sendable *, std::string_view, int) on unsupported platform");
}

void SendableRegistry::AddLW(Sendable* /*sendable*/, std::string_view /*moduleType*/, int /*moduleNumber*/, int /*channel*/)
{
	//ROS_ERROR("Called SendableRegistry::AddLW(Sendable *, std::string_view, int, int) on unsupported platform");
}

bool SendableRegistry::Remove(Sendable* /*sendable*/)
{
return true;
}

}  // namespace wpi

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.inc"
namespace nt
{

NetworkTable::NetworkTable(NT_Inst /*inst*/, std::string_view /*path*/, NetworkTable::private_init const & /*pi*/)
{
}

NetworkTable::~NetworkTable()
{
}

}

#include "wpi/json.h"
const char *wpi::json::type_name() const noexcept
{
	ROS_ERROR("Called wpi::json::type_name() const on unsupported platform");
	return nullptr;
}

wpi::json::const_reference wpi::json::at(std::string_view /*key*/) const
{
	ROS_ERROR("Called wpi::json::at(string_view) const on unsupported platform");
	return *this;
}

wpi::json::json(wpi::json::initializer_list_t /*init*/,
               bool /*type_deduction = true*/,
               wpi::json::value_t/*manual_type = value_t::array*/)
{
	ROS_ERROR("Called wpi::json::json(wpi::json::initializer_list_t, bool, wpi::json::value_t) on unsupported platform");
}

void wpi::json::json_value::destroy(wpi::detail::value_t) noexcept
{
	ROS_ERROR("Called wpi::json::json_value::destroy(wpi::detal::value_t) on unsupported platform");
}

wpi::detail::type_error wpi::detail::type_error::create(int,std::string_view , std::string_view )
{
	ROS_ERROR("Called static wpi::detail::type_error::create(int, std::string_view, std::string_view) const on unsupported platform");
}

