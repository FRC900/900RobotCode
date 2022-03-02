// Stub functions to replace WPIlib code which accesses hardware
// which doesn't exist on certain platforms.
#include <ros/console.h>

#include <WPILibVersion.h>
const char* GetWPILibVersion(void)
{
	ROS_ERROR("Called GetWPILibVersion on unsupported platform");
	return "900.2020";
}

#include <AHRS.h>
AHRS::AHRS(frc::SPI::Port)
{
	ROS_ERROR("Called AHRS(SPI::Port spi_port_id) on unsupported platform");
}
AHRS::AHRS(frc::I2C::Port)
{
	ROS_ERROR("Called AHRS(I2C::Port i2c_port_id) on unsupported platform");
}
AHRS::AHRS(frc::SerialPort::Port)
{
	ROS_ERROR("Called AHRS(SerialPort::Port serial_port_id) on unsupported platform");
}

AHRS::AHRS(frc::SPI::Port, uint8_t)
{
	ROS_ERROR("Called AHRS(SPI::Port spi_port_id, uint8_t update_rate_hz) on unsupported platform");
}
AHRS::AHRS(frc::SPI::Port, uint32_t, uint8_t)
{
	ROS_ERROR("Called AHRS(SPI::Port spi_port_id, uint32_t spi_bitrate, uint8_t update_rate_hz) on unsupported platform");
}

AHRS::AHRS(frc::I2C::Port, uint8_t)
{
	ROS_ERROR("Called AHRS(I2C::Port i2c_port_id, uint8_t update_rate_hz) on unsupported platform");
}

AHRS::AHRS(frc::SerialPort::Port, AHRS::SerialDataType , uint8_t)
{
	ROS_ERROR("Called AHRS(SerialPort::Port serial_port_id, AHRS::SerialDataType data_type, uint8_t update_rate_hz) on unsupported platform");
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
float  AHRS::GetCompassHeading()
{
	ROS_ERROR("Called GetCompassHeading() on unsupported platform");
	return std::numeric_limits<float>::max();
}
void   AHRS::ZeroYaw()
{
	ROS_ERROR("Called ZeroYaw() on unsupported platform");
}
bool   AHRS::IsCalibrating()
{
	ROS_ERROR("Called IsCalibrating() on unsupported platform");
	return false;
}
bool   AHRS::IsConnected()
{
	ROS_ERROR("Called IsConnected() on unsupported platform");
	return false;
}
double AHRS::GetByteCount()
{
	ROS_ERROR("Called GetByteCount() on unsupported platform");
	return std::numeric_limits<double>::max();
}
double AHRS::GetUpdateCount()
{
	ROS_ERROR("Called GetUpdateCount() on unsupported platform");
	return std::numeric_limits<double>::max();
}
long   AHRS::GetLastSensorTimestamp()
{
	ROS_ERROR("Called GetLastSensorTimestamp() on unsupported platform");
	return std::numeric_limits<long>::max();
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
bool   AHRS::IsMoving()
{
	ROS_ERROR("Called IsMoving() on unsupported platform");
	return false;
}
bool   AHRS::IsRotating()
{
	ROS_ERROR("Called IsRotating() on unsupported platform");
	return false;
}
float  AHRS::GetBarometricPressure()
{
	ROS_ERROR("Called GetBarometricPressure() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetAltitude()
{
	ROS_ERROR("Called GetAltitude() on unsupported platform");
	return std::numeric_limits<float>::max();
}
bool   AHRS::IsAltitudeValid()
{
	ROS_ERROR("Called IsAltitudeValid() on unsupported platform");
	return false;
}
float  AHRS::GetFusedHeading()
{
	ROS_ERROR("Called GetFusedHeading() on unsupported platform");
	return std::numeric_limits<float>::max();
}
bool   AHRS::IsMagneticDisturbance()
{
	ROS_ERROR("Called IsMagneticDisturbance() on unsupported platform");
	return false;
}
bool   AHRS::IsMagnetometerCalibrated()
{
	ROS_ERROR("Called IsMagnetometerCalibrated() on unsupported platform");
	return false;
}
float  AHRS::GetQuaternionW()
{
	ROS_ERROR("Called GetQuaternionW() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetQuaternionX()
{
	ROS_ERROR("Called GetQuaternionX() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetQuaternionY()
{
	ROS_ERROR("Called GetQuaternionY() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetQuaternionZ()
{
	ROS_ERROR("Called GetQuaternionZ() on unsupported platform");
	return std::numeric_limits<float>::max();
}
void   AHRS::ResetDisplacement()
{
	ROS_ERROR("Called ResetDisplacement() on unsupported platform");
}
void   AHRS::UpdateDisplacement( float , float ,
						   int , bool )
{
	ROS_ERROR("Called UpdateDisplacement( float accel_x_g, float accel_y_g, int update_rate_hz, bool is_moving ) on unsupported platform");
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
float  AHRS::GetDisplacementX()
{
	ROS_ERROR("Called GetDisplacementX() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetDisplacementY()
{
	ROS_ERROR("Called GetDisplacementY() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetDisplacementZ()
{
	ROS_ERROR("Called GetDisplacementZ() on unsupported platform");
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
void   AHRS::SetAngleAdjustment(double)
{
	ROS_ERROR("Called double angle) on unsupported platform");
}
double AHRS::GetAngleAdjustment()
{
	ROS_ERROR("Called GetAngleAdjustment() on unsupported platform");
	return std::numeric_limits<double>::max();
}
void   AHRS::Reset()
{
	ROS_ERROR("Called Reset() on unsupported platform");
}
float  AHRS::GetRawGyroX()
{
	ROS_ERROR("Called GetRawGyroX() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetRawGyroY()
{
	ROS_ERROR("Called GetRawGyroY() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetRawGyroZ()
{
	ROS_ERROR("Called GetRawGyroZ() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetRawAccelX()
{
	ROS_ERROR("Called GetRawAccelX() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetRawAccelY()
{
	ROS_ERROR("Called GetRawAccelY() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetRawAccelZ()
{
	ROS_ERROR("Called GetRawAccelZ() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetRawMagX()
{
	ROS_ERROR("Called GetRawMagX() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetRawMagY()
{
	ROS_ERROR("Called GetRawMagY() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetRawMagZ()
{
	ROS_ERROR("Called GetRawMagZ() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetPressure()
{
	ROS_ERROR("Called GetPressure() on unsupported platform");
	return std::numeric_limits<float>::max();
}
float  AHRS::GetTempC()
{
	ROS_ERROR("Called GetTempC() on unsupported platform");
	return std::numeric_limits<float>::max();
}
AHRS::BoardYawAxis AHRS::GetBoardYawAxis()
{
	ROS_ERROR("Called GetBoardYawAxis() on unsupported platform");
	return AHRS::BoardYawAxis();
}
std::string AHRS::GetFirmwareVersion()
{
	ROS_ERROR("Called GetFirmwareVersion() on unsupported platform");
	return std::string();
}

bool AHRS::RegisterCallback( ITimestampedDataSubscriber *, void *)
{
	ROS_ERROR("Called *callback_context) on unsupported platform");
	return false;
}
bool AHRS::DeregisterCallback( ITimestampedDataSubscriber *)
{
	ROS_ERROR("Called *callback ) on unsupported platform");
	return false;
}

int AHRS::GetActualUpdateRate()
{
	ROS_ERROR("Called GetActualUpdateRate() on unsupported platform");
	return std::numeric_limits<int>::max();
}
int AHRS::GetRequestedUpdateRate()
{
	ROS_ERROR("Called GetRequestedUpdateRate() on unsupported platform");
	return std::numeric_limits<int>::max();
}

void AHRS::EnableLogging(bool)
{
	ROS_ERROR("Called bool AHRS::EnableLogging(bool enable) on unsupported platform");
}
void AHRS::SPIInit(frc::SPI::Port, uint32_t, uint8_t)
{
	ROS_ERROR("Called AHRS::SPIInit( SPI::Port spi_port_id, uint32_t bitrate, uint8_t update_rate_hz ) on unsupported platform");
}
void AHRS::I2CInit(frc::I2C::Port, uint8_t)
{
	ROS_ERROR("Called AHRS::I2CInit( I2C::Port i2c_port_id, uint8_t update_rate_hz ) on unsupported platform");
}
void AHRS::SerialInit(frc::SerialPort::Port, AHRS::SerialDataType, uint8_t)
{
	ROS_ERROR("Called AHRS::SerialInit(SerialPort::Port serial_port_id, AHRS::SerialDataType data_type, uint8_t update_rate_hz) on unsupported platform");
}
void AHRS::commonInit( uint8_t)
{
	ROS_ERROR("Called AHRS::commonInit( uint8_t update_rate_hz ) on unsupported platform");
}
int AHRS::ThreadFunc(IIOProvider *)
{
	ROS_ERROR("Called int AHRS::ThreadFunc(IIOProvider *io_provider) on unsupported platform");
	return -1;
}
void AHRS::InitSendable(wpi::SendableBuilder&)
{
	ROS_ERROR("Called AHRS::InitSendable(wpi::SendableBuilder&) on unsupported platform");
}
uint8_t AHRS::GetActualUpdateRateInternal(uint8_t)
{
	ROS_ERROR("Called AHRS::GetActualUpdateRateInternal(uint8_t update_rate) on unsupported platform");
	return std::numeric_limits<uint8_t>::max();
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

void frc::NidecBrushless::Enable()
{
	ROS_ERROR("Called ::NidecBrushless::Enable() on unsupported platform");
}

std::string frc::NidecBrushless::GetDescription() const
{
	ROS_ERROR("Called ::NidecBrushless::GetDescription(wpi::raw_ostream& desc) const on unsupported platform");
}

int frc::NidecBrushless::GetChannel() const
{
	ROS_ERROR("Called ::NidecBrushless::GetChannel() const on unsupported platform");
	return -1;
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
void frc::PWM::SetPeriodMultiplier(PeriodMultiplier)
{
	ROS_ERROR("Called PWM::SetPeriodMultiplier(PeriodMultiplier mult) on unsupported platform");
}
void frc::PWM::SetZeroLatch()
{
	ROS_ERROR("Called PWM::SetZeroLatch() on unsupported platform");
}
void frc::PWM::EnableDeadbandElimination(bool)
{
	ROS_ERROR("Called PWM::EnableDeadbandElimination(bool eliminateDeadband) on unsupported platform");
}
void frc::PWM::SetBounds(double, double, double, double, double)
{
	ROS_ERROR("Called PWM::SetBounds(double max, double deadbandMax, double center, double deadbandMin, double min) on unsupported platform");
}
void frc::PWM::SetRawBounds(int, int, int, int, int)
{
	ROS_ERROR("Called PWM::SetRawBounds(int max, int deadbandMax, int center, int deadbandMin, int min) on unsupported platform");
}
void frc::PWM::GetRawBounds(int32_t*, int32_t*, int32_t*, int32_t*, int32_t*)
{
	ROS_ERROR("Called PWM::GetRawBounds(int32_t* max, int32_t* deadbandMax, int32_t* center, int32_t* deadbandMin, int32_t* min) on unsupported platform");
}
void frc::PWM::InitSendable(wpi::SendableBuilder&)
{
	ROS_ERROR("Called PWM::InitSendable(wpi::SendableBuilder& builder) on unsupported platform");
}

#include <frc/RobotBase.h>
bool frc::RobotBase::IsAutonomous() const
{
	ROS_ERROR("Called RobotBase::IsAutonomous() const on unsupported platform");
	return false;
}
bool frc::RobotBase::IsDisabled() const
{
	ROS_ERROR("Called RobotBase::IsDisabled() const on unsupported platform");
	return false;
}
bool frc::RobotBase::IsOperatorControl() const
{
	ROS_ERROR("Called RobotBase::IsOperatorControl() const on unsupported platform");
	return false;
}
frc::RobotBase::RobotBase()
{
	ROS_ERROR("Called RobotBase::RobotBase() on unsupported platform");
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

#include <frc/Notifier.h>
frc::Notifier::Notifier(std::function<void ()>)
{
	ROS_ERROR("Called frc::Notifier::Notifier(std::function<void ()>) on unsupported platform");
}
frc::Notifier::~Notifier()
{
	ROS_ERROR("Called frc::Notifier::~Notifier() on unsupported platform");
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
#if 0

struct SendableRegistry::Impl
{
	bool foo;
};

SendableRegistry& SendableRegistry::GetInstance()
{
	static SendableRegistry s;
	return s;
}

void SendableRegistry::Add(Sendable* sendable, const wpi::Twine& name)
{
}

void SendableRegistry::Add(Sendable* sendable, const wpi::Twine& moduleType, int channel)
{
}

void SendableRegistry::Add(Sendable* sendable, const wpi::Twine& moduleType, int moduleNumber, int channel)
{
}

void SendableRegistry::Add(Sendable* sendable, const wpi::Twine& subsystem, const wpi::Twine& name)
{
}
#endif

void SendableRegistry::AddLW(Sendable* /*sendable*/, std::string_view /*name*/)
{
	//ROS_ERROR("Called SendableRegistry::AddLW(Sendable *, std::string_view) on unsupported platform");
}

void SendableRegistry::AddLW(Sendable* /*sendable*/, std::string_view /*moduleType*/, int /*channel*/)
{
	//ROS_ERROR("Called SendableRegistry::AddLW(Sendable *, std::string_view, int) on unsupported platform");
}

void SendableRegistry::AddLW(Sendable* /*sendable*/, std::string_view /*moduleType*/, int /*moduleNumber*/, int /*channel*/)
{
	//ROS_ERROR("Called SendableRegistry::AddLW(Sendable *, std::string_view, int, int) on unsupported platform");
}

#if 0

void AddLW(Sendable* sendable, const wpi::Twine& subsystem,
		 const wpi::Twine& name);

void AddChild(Sendable* parent, Sendable* child);

void AddChild(Sendable* parent, void* child);
#endif

bool SendableRegistry::Remove(Sendable* /*sendable*/)
{
return true;
}


void SendableRegistry::Move(Sendable* /*to*/, Sendable* /*from*/)
{
}

#if 0
bool Contains(const Sendable* sendable) const

std::string GetName(const Sendable* sendable) const

void SetName(Sendable* /*sendable*/, std::string_view /* name*/)
{
}
#endif


void SendableRegistry::SetName(Sendable* /*sendable*/, std::string_view /*moduleType*/, int /*channel*/)
{
}

#if 0
void SetName(Sendable* sendable, const wpi::Twine& moduleType,
		   int moduleNumber, int channel);

void SendableRegistry::SetName(Sendable* sendable, const wpi::Twine& subsystem,
		   const wpi::Twine& name)
{
}

std::string GetSubsystem(const Sendable* sendable) const

void SetSubsystem(Sendable* sendable, const wpi::Twine& subsystem);

int GetDataHandle();

std::shared_ptr<void> SetData(Sendable* sendable, int handle,
							std::shared_ptr<void> data);

std::shared_ptr<void> GetData(Sendable* sendable, int handle);

void EnableLiveWindow(Sendable* sendable);

void DisableLiveWindow(Sendable* sendable);

UID GetUniqueId(Sendable* sendable);

Sendable* GetSendable(UID uid);

void Publish(UID sendableUid, std::shared_ptr<NetworkTable> table);

void Update(UID sendableUid);

void ForeachLiveWindow(
  int dataHandle,
  wpi::function_ref<void(CallbackData& cbdata)> callback) const

SendableRegistry::SendableRegistry()
{
}

#endif
}  // namespace wpi

#if 0
#include "wpi/sendable/SendableBase.h"
namespace frc
{

	SendableBase::SendableBase(bool)
	{
	}
}
#endif

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"
namespace nt
{
Value::Value()
{
}
Value::Value(NT_Type /*type*/, uint64_t /*time*/, const private_init&)
{
}
Value::~Value()
{
}

bool SetEntryValue(NT_Entry, std::shared_ptr<Value>)
{
	return true;
}
void SetEntryTypeValue(NT_Entry /*entry*/, std::shared_ptr<Value> /*value*/)
{
}

NT_Inst GetDefaultInstance()
{
	static NT_Inst nti;
	return nti;
}

NetworkTable::NetworkTable(NT_Inst /*inst*/, std::string_view /*path*/, NetworkTable::private_init const & /*pi*/)
{
}

NetworkTable::~NetworkTable()
{
}

std::shared_ptr<NetworkTable> NetworkTableInstance::GetTable(std::string_view /*key*/) const
{
	return std::make_shared<NetworkTable>(0, "", NetworkTable::private_init{});
}

NetworkTableEntry NetworkTable::GetEntry(std::string_view /*key*/) const
{
	return NetworkTableEntry();
}


}
#if 0
#include "ntcore_cpp.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

namespace nt
{

bool SetEntryValue(NT_Entry /*entry*/, std::shared_ptr<Value> /*value*/)
{
	return true;
}
void SetEntryTypeValue(NT_Entry /*entry*/, std::shared_ptr<Value> /*value*/)
{
}
NT_Inst GetDefaultInstance()
{
	static NT_Inst nti;
	return nti;
}

Value::Value()
{
}
Value::Value(NT_Type /*type*/, uint64_t /*time*/, const private_init&)
{
}
Value::~Value()
{
}

NetworkTable::NetworkTable(NT_Inst /*inst*/, wpi::Twine const & /*path*/, NetworkTable::private_init const & /*pi*/)
{
}

NetworkTable::~NetworkTable()
{
}

bool NetworkTable::ContainsKey(const Twine& /*key*/) const
{
	return true;
}

bool NetworkTable::ContainsSubTable(const Twine& /*key*/) const
{
	return true;
}
std::shared_ptr<NetworkTable> NetworkTable::GetSubTable(const Twine& /*key*/) const
{
	NetworkTable::private_init pi;
	return std::make_shared<NetworkTable>(0, "", pi);
}

std::vector<std::string> NetworkTable::GetKeys(int /*types*/) const
{
	return std::vector<std::string>();
}

std::vector<std::string> NetworkTable::GetSubTables() const
{
	return std::vector<std::string>();
}

NetworkTableEntry NetworkTable::GetEntry(const Twine& /*key*/) const
{
	return NetworkTableEntry();
}

void NetworkTable::SetPersistent(StringRef /*key*/)
{
}

void NetworkTable::ClearPersistent(StringRef /*key*/)
{
}

bool NetworkTable::IsPersistent(StringRef /*key*/) const
{
	return false;
}
void NetworkTable::SetFlags(StringRef /*key*/, unsigned int /*flags*/)
{
}

void NetworkTable::ClearFlags(StringRef /*key*/, unsigned int /*flags*/)
{
}

unsigned int NetworkTable::GetFlags(StringRef /*key*/) const
{
	return 0UL;
}

void NetworkTable::Delete(const Twine& /*key*/)
{
}

bool NetworkTable::PutNumber(StringRef /*key*/, double /*value*/)
{
	return true;
}

bool NetworkTable::SetDefaultNumber(StringRef /*key*/, double /*defaultValue*/)
{
	return true;
}

double NetworkTable::GetNumber(StringRef /*key*/, double /*defaultValue*/) const
{
	return 0.0;
}

bool NetworkTable::PutString(StringRef /*key*/, StringRef /*value*/)
{
	return true;
}

bool NetworkTable::SetDefaultString(StringRef /*key*/, StringRef /*defaultValue*/)
{
	return true;
}

std::string NetworkTable::GetString(StringRef /*key*/, StringRef /*defaultValue*/) const
{
	return std::string();
}

bool NetworkTable::PutBoolean(StringRef /*key*/, bool /*value*/)
{
	return true;
}

bool NetworkTable::SetDefaultBoolean(StringRef /*key*/, bool /*defaultValue*/)
{
	return true;
}

bool NetworkTable::GetBoolean(StringRef /*key*/, bool /*defaultValue*/) const
{
	return true;
}

bool NetworkTable::PutBooleanArray(StringRef /*key*/, ArrayRef<int> /*value*/)
{
	return true;
}

bool NetworkTable::SetDefaultBooleanArray(StringRef /*key*/,
		ArrayRef<int> /*defaultValue*/)
{
	return true;
}

std::vector<int> NetworkTable::GetBooleanArray(StringRef /*key*/,
		ArrayRef<int> /*defaultValue*/) const
{
	return std::vector<int>();
}

bool NetworkTable::PutNumberArray(StringRef /*key*/, ArrayRef<double> /*value*/)
{
	return true;
}

bool NetworkTable::SetDefaultNumberArray(StringRef /*key*/,
		ArrayRef<double> /*defaultValue*/)
{
	return true;
}

std::vector<double> NetworkTable::GetNumberArray(
		StringRef /*key*/, ArrayRef<double> /*defaultValue*/) const
{
	return std::vector<double>();
}

bool NetworkTable::PutStringArray(StringRef /*key*/, ArrayRef<std::string> /*value*/)
{
	return true;
}

bool NetworkTable::SetDefaultStringArray(StringRef /*key*/,
		ArrayRef<std::string> /*defaultValue*/)
{
	return true;
}

std::vector<std::string> NetworkTable::GetStringArray(
		StringRef /*key*/, ArrayRef<std::string> /*defaultValue*/) const
{
	return std::vector<std::string>();
}

bool NetworkTable::PutRaw(StringRef /*key*/, StringRef /*value*/)
{
	return true;
}

bool NetworkTable::SetDefaultRaw(StringRef /*key*/, StringRef /*defaultvalue*/)
{
	return true;
}

std::string NetworkTable::GetRaw(StringRef /*key*/, StringRef /*defaultvalue*/) const
{
	return std::string();
}

bool NetworkTable::PutValue(const Twine& /*key*/, std::shared_ptr<Value> /*value*/)
{
	return true;
}

bool NetworkTable::SetDefaultValue(const Twine& /*key*/,
		std::shared_ptr<Value> /*defaultValue*/)
{
	return true;
}

std::shared_ptr<Value> NetworkTable::GetValue(const Twine& /*key*/) const
{
	return std::make_shared<Value>();
}

StringRef NetworkTable::GetPath() const
{
	return StringRef();
}
#endif

/*
const char* NetworkTable::SaveEntries(const Twine& filename) const
{
	return nullptr;
}

const char* LoadEntries(
		const Twine& filename,
		std::function<void(size_t line, const char* msg)> warn)
{
	return nullptr;
}
*/
#if 0
void NetworkTable::AddTableListener(ITableListener* /*listener*/)
{
}

void NetworkTable::AddTableListener(ITableListener* /*listener*/,
		bool /*immediateNotify*/)
{
}

void NetworkTable::AddTableListenerEx(ITableListener* /*listener*/,
		unsigned int /*flags*/)
{
}

void NetworkTable::AddTableListener(StringRef /*key*/, ITableListener* /*listener*/,
		bool /*immediateNotify*/)
{
}

void NetworkTable::AddTableListenerEx(StringRef /*key*/, ITableListener* /*listener*/,
		unsigned int /*flags*/)
{
}

void NetworkTable::AddSubTableListener(ITableListener* /*listener*/)
{
}

void NetworkTable::AddSubTableListener(ITableListener* /*listener*/, bool /*localNotify*/)
{
}

void NetworkTable::RemoveTableListener(NT_EntryListener /*listener*/)
{
}

void NetworkTable::RemoveTableListener(ITableListener* /*listener*/)
{
}


std::shared_ptr<NetworkTable> NetworkTableInstance::GetTable(const Twine& /*key*/) const
{
	NetworkTable::private_init pi;
	return std::make_shared<NetworkTable>(0, "", pi);
}

} // namespace nt

#endif

#include "wpi/json.h"
const char *wpi::json::type_name() const noexcept
{
	ROS_ERROR("Called wpi::json::type_name() const on unsupported platform");
	return nullptr;
}

wpi::json::reference wpi::json::at(std::string_view /*key*/)
{
	ROS_ERROR("Called wpi::json::at(string_view) on unsupported platform");
	return *this;
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

wpi::detail::type_error wpi::detail::type_error::create(int,std::string_view what_arg, std::string_view type_info)
{
	ROS_ERROR("Called static wpi::detail::type_error::create(int, std::string_view, std::string_view) const on unsupported platform");
}


#if 0
#include <rev/CANSparkMaxLowLevel.h>

rev::CANSparkMaxLowLevel::CANSparkMaxLowLevel(int deviceID, rev::CANSparkMaxLowLevel::MotorType type)
	: m_motorType(type)
	, m_deviceID(deviceID)
{
	ROS_ERROR("Called CANSparkMaxLowLevel(int deviceID, MotorType type) on unsupported platform");
}
rev::CANSparkMaxLowLevel::~CANSparkMaxLowLevel()
{
	ROS_ERROR("Called CANSparkMaxLowLevel::CANSparkMaxLowLevel() on unsupported platform");
}


#include <rev/CANSparkMax.h>
rev::CANSparkMax::CANSparkMax(int deviceID, rev::CANSparkMaxLowLevel::MotorType type)
	: CANSparkMaxLowLevel(deviceID, type)
{
	ROS_ERROR("And also called CANSparkMax::CANSparkMax(int deviceID, MotorType type) on unsupported platform");
}

void rev::CANSparkMax::Set(double speed)
{
	(void)speed;
	ROS_ERROR("Called CANSparkMax::Set(double speed) on unsupported platform");
}

double rev::CANSparkMax::Get(void) const
{
	ROS_ERROR("Called CANSparkMax::Get() on unsupported platform");
	return 900;
}

void rev::CANSparkMax::SetInverted(bool isInverted)
{
	(void)isInverted;
	ROS_ERROR("Called CANSparkMax::SetInverted(bool isInverted) on unsupported platform");
}
bool rev::CANSparkMax::GetInverted() const
{
	ROS_ERROR("Called CANSparkMax::GetInverted() on unsupported platform");
	return false;
}
void rev::CANSparkMax::Disable()
{
	ROS_ERROR("Called CANSparkMax::Disable() on unsupported platform");
}
void rev::CANSparkMax::StopMotor()
{
	ROS_ERROR("Called CANSparkMax::StopMotor() on unsupported platform");
}
void rev::CANSparkMax::PIDWrite(double output)
{
	(void)output;
	ROS_ERROR("Called CANSparkMax::PIDWrite() on unsupported platform");
}

bool rev::CANSparkMax::IsFollower()
{
	ROS_ERROR("Called CANSparkMax::IsFollower() on unsupported platform");
	return false;
}

uint16_t rev::CANSparkMax::GetFaults()
{
	ROS_ERROR("Called CANSparkMax::GetFaults() on unsupported platform");
	return 0xFFFF;
}

uint16_t rev::CANSparkMax::GetStickyFaults()
{
	ROS_ERROR("Called CANSparkMax::GetStickyFaults() on unsupported platform");
	return 0xFFFF;
}
#endif

