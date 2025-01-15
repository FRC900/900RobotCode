// Functions referenced by various WPILIB code used on non-Rio targets but not actually
// called. These functions let us build wpilib-dependent code on x86 and Jetson targets
// by stubbing out functions which aren't actually used
#include <ros/ros.h>
#include <ctre/phoenix/platform/Platform.hpp>
#include <ros/console.h>
#include <chrono>

#include <hal/CAN.h>

static std::string canBus{};
// Not actually a HAL call, added here to set the canBus string from inside the HWI
void HAL_SetCANBusString(const std::string &bus) { canBus = bus; }

extern "C"
{
	static constexpr uint32_t PCM_CONTROL_1 = 0x09041C00;	/* PCM_Control */
	static constexpr uint32_t PCM_CONTROL_2 = 0x09041C40;	/* PCM_SupplemControl */
	static constexpr uint32_t PCM_CONTROL_3 = 0x09041C80;	/* PcmControlSetOneShotDur_t */

	// This is the path for calls going through the new CANAPI.  Accesses
	// via these functions have already been through the CAN status
	// cache and were not found
	void HAL_CAN_SendMessage(uint32_t messageID, const uint8_t *data, uint8_t dataSize, int32_t periodMs, int32_t *status)
	{
		// PCM arbIDs - need to filter out writes to these from the Jetson
		// otherwise they overwrite legitimate commands from the Rio
		if (const uint32_t arbId = messageID & 0xFFFFFFC0;
		    (arbId == PCM_CONTROL_1) || (arbId == PCM_CONTROL_2) || (arbId == PCM_CONTROL_3))
			return;

		ctre::phoenix::platform::can::CANComm_SendMessage(messageID, data, dataSize, status, canBus.c_str());
	}
	void HAL_CAN_ReceiveMessage(uint32_t *messageID, uint32_t /*messageIDMask*/, uint8_t *data, uint8_t *dataSize, uint32_t *timeStamp, int32_t *status)
	{
		ctre::phoenix::platform::can::canframe_t canframe{};
		ctre::phoenix::platform::can::CANComm_ReceiveMessage(*messageID, canframe, status, canBus.c_str());
		*dataSize = canframe.len;
		std::memcpy(data, canframe.data, *dataSize);

		*timeStamp = static_cast<uint32_t>(canframe.swTimestampUs / 1000);
	}
	void HAL_CAN_OpenStreamSession(uint32_t* sessionHandle, uint32_t messageID,
						   uint32_t messageIDMask, uint32_t maxMessages,
						   int32_t* status) {
			ctre::phoenix::platform::can::CANComm_OpenStreamSession(
							sessionHandle, messageID, messageIDMask, maxMessages, status, canBus.c_str());
	}
	void HAL_CAN_CloseStreamSession(uint32_t sessionHandle) {
			ctre::phoenix::platform::can::CANComm_CloseStreamSession(sessionHandle, canBus.c_str());
	}
	void HAL_CAN_ReadStreamSession(uint32_t sessionHandle,
					struct HAL_CANStreamMessage* messages,
					uint32_t messagesToRead, uint32_t* messagesRead,
					int32_t* status) {
			ctre::phoenix::platform::can::canframe_t localMessages[messagesToRead];
			ctre::phoenix::platform::can::CANComm_ReadStreamSession(
							sessionHandle, localMessages,
							messagesToRead, messagesRead, status, canBus.c_str());
			for (uint32_t i = 0; i < *messagesRead; i++)
			{
					messages[i].messageID = localMessages[i].arbID;
					messages[i].timeStamp = static_cast<uint32_t>(localMessages[i].swTimestampUs);
					memcpy(messages[i].data, localMessages[i].data, sizeof(messages[i].data));
					messages[i].dataSize = localMessages[i].len;
			}
	}
}

#include <frc/AnalogInput.h>
frc::AnalogInput::AnalogInput(int)
{
	ROS_ERROR("Called AnalogInput::AnalogInput(int) on unsupported platform");
}

int frc::AnalogInput::GetValue() const
{
	ROS_ERROR("Called frc::AnalogInput::GetValue() const on unsupported platform");
	return -1;
}

void frc::AnalogInput::InitSendable(wpi::SendableBuilder&)
{
	ROS_ERROR("Called frc::AnalogInput::InitSendable(SendableBuilder& builder) on unsupported platform");
}

#include <hal/DriverStation.h>
#include <frc/DriverStation.h>
bool frc::DriverStation::IsEnabled(void)
{
        HAL_ControlWord controlWord;
        HAL_GetControlWord(&controlWord);
        return controlWord.enabled && controlWord.dsAttached;
}
bool frc::DriverStation::IsDisabled() {
	HAL_ControlWord controlWord;
	HAL_GetControlWord(&controlWord);
	return !(controlWord.enabled && controlWord.dsAttached);
}
bool frc::DriverStation::IsAutonomous() {
	HAL_ControlWord controlWord;
	HAL_GetControlWord(&controlWord);
	return controlWord.autonomous;
}
bool frc::DriverStation::IsTeleop() {
  HAL_ControlWord controlWord;
  HAL_GetControlWord(&controlWord);
  return !(controlWord.autonomous || controlWord.test);
}
bool frc::DriverStation::IsTest() {
  HAL_ControlWord controlWord;
  HAL_GetControlWord(&controlWord);
  return controlWord.test;
}
void frc::DriverStation::RefreshData() {
	ROS_ERROR_STREAM("frc::DriverStation::RefreshData() called on unsupported platform");
}

void HAL_ProvideNewDataEventHandle(WPI_EventHandle handle)
{
	ROS_ERROR_STREAM("HAL_ProvideNewDataEventHandle() called on unspported platform");
}
void HAL_RemoveNewDataEventHandle(WPI_EventHandle handle)
{
	ROS_ERROR_STREAM("HAL_RemoveNewDataEventHandle() called on unspported platform");
}


#include <frc/GenericHID.h>
frc::GenericHID::GenericHID(int)
{
	ROS_ERROR("Called GenericHID::GenericHID(int) on unsupported platform");
}
int frc::GenericHID::GetPOV(int) const
{
	ROS_ERROR("Called GenericHID::GetPOV(int) const on unsupported platform");
	return -1;
}
double frc::GenericHID::GetRawAxis(int) const
{
	ROS_ERROR("Called GenericHID::GetRawAxis(int) const on unsupported platform");
	return std::numeric_limits<double>::max();
}
bool frc::GenericHID::GetRawButton(int) const
{
	ROS_ERROR("Called GenericHID::GetRawButton(int) const on unsupported platform");
	return false;
}
bool frc::GenericHID::GetRawButtonPressed(int)
{
	ROS_ERROR("Called GenericHID::GetRawButtonPressed(int) on unsupported platform");
	return false;
}
bool frc::GenericHID::GetRawButtonReleased(int)
{
	ROS_ERROR("Called GenericHID::GetRawButtonReleased(int) on unsupported platform");
	return false;
}

int frc::GenericHID::GetButtonCount() const
{
	ROS_ERROR("Called frc::Joystick::GetButtonCount() const on unsupported platform");
	return -1;
}

int frc::GenericHID::GetAxisCount() const
{
	ROS_ERROR("Called frc::Joystick::GetAxisCount() const on unsupported platform");
	return -1;
}

int frc::GenericHID::GetPOVCount() const
{
	ROS_ERROR("Called frc::Joystick::GetPOVCount() const on unsupported platform");
	return -1;
}

#include <hal/Types.h>
#include <HALInitializer.h>
extern "C" {

uint64_t HAL_GetFPGATime(int32_t* status)
{
	*status = 0;
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

HAL_Bool HAL_Initialize(int32_t, int32_t)
{
	hal::init::HAL_IsInitialized.store(true);
	return true;
}

int32_t HAL_GetFPGAVersion(int32_t* status)
{
	ROS_ERROR("Called HAL_GetFPGAVersion() on unsupported platform");
	*status = 0;
	return -900;  // Automatically script this at some point
}

int64_t HAL_GetFPGARevision(int32_t* status)
{
	ROS_ERROR("Called HAL_GetFPGARevision() on unsupported platform");
	*status = 0;
	return -900;  // TODO: Find a better number to return;
}

HAL_Bool HAL_GetFPGAButton(int32_t* status)
{
	ROS_ERROR("Called HAL_GetFPGAButton() on unsupported platform");
	*status = 0;
	return false;
}

HAL_Bool HAL_GetSystemActive(int32_t* status)
{
	ROS_ERROR("Called HAL_GetSystemActive() on unsupported platform");
	*status = 0;
	return true;
}

HAL_Bool HAL_GetBrownedOut(int32_t* status)
{
	ROS_ERROR("Called HAL_GetBrownedOut() on unsupported platform");
	*status = 0;
	return false;
}

double HAL_GetVinVoltage(int32_t* status)
{
	//ROS_ERROR("Called HAL_GetVinVoltage() on unsupported platform");
	*status = 0;
	return 12.; /* Just to be safe, fake a reasonable voltage */
}
double HAL_GetVinCurrent(int32_t* status)
{
	ROS_ERROR("Called HAL_GetVinCurrent() on unsupported platform");
	*status = 0;
	return -1;
}
double HAL_GetUserVoltage6V(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserVoltage6V() on unsupported platform");
	*status = 0;
	return -1;
}
double HAL_GetUserCurrent6V(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserCurrent6V() on unsupported platform");
	*status = 0;
	return -1;
}
HAL_Bool HAL_GetUserActive6V(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserActive6V() on unsupported platform");
	*status = 0;
	return false;
}
int32_t HAL_GetUserCurrentFaults6V(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserCurrentFaults6V() on unsupported platform");
	*status = 0;
	return -1;
}
double HAL_GetUserVoltage5V(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserVoltage5V() on unsupported platform");
	*status = 0;
	return -1;
}
double HAL_GetUserCurrent5V(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserCurrent5V() on unsupported platform");
	*status = 0;
	return -1;
}
HAL_Bool HAL_GetUserActive5V(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserActive5V() on unsupported platform");
	*status = 0;
	return false;
}
int32_t HAL_GetUserCurrentFaults5V(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserCurrentFaults5V() on unsupported platform");
	*status = 0;
	return -1;
}
double HAL_GetUserVoltage3V3(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserVoltage3V3() on unsupported platform");
	*status = 0;
	return -1;
}
double HAL_GetUserCurrent3V3(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserCurrent3V3() on unsupported platform");
	*status = 0;
	return -1;
}
HAL_Bool HAL_GetUserActive3V3(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserActive3V3() on unsupported platform");
	*status = 0;
	return false;
}
int32_t HAL_GetUserCurrentFaults3V3(int32_t* status)
{
	ROS_ERROR("Called HAL_GetUserCurrentFaults3V3() on unsupported platform");
	*status = 0;
	return -1;
}
void HAL_SetBrownoutVoltage(double , int32_t* status)
{
	ROS_ERROR("Called HAL_SetBrownoutVoltage(double, int32_t*) on unsupported platform");
	*status = 0;
}
double HAL_GetBrownoutVoltage(int32_t* status) {
	ROS_ERROR("Called HAL_GetBrownoutVoltage(int32_t*) on unsupported platform");
	*status = 0;
	return -1;
}
void HAL_CAN_GetCANStatus(float* percentBusUtilization, uint32_t* busOffCount,
                          uint32_t* txFullCount, uint32_t* receiveErrorCount,
                          uint32_t* transmitErrorCount, int32_t* status)
{
	ROS_ERROR("Called HAL_CAN_GetCANStatus() on unsupported platform");
	*percentBusUtilization = -1;
	*busOffCount = -1;
	*txFullCount = -1;
	*receiveErrorCount = -1;
	*transmitErrorCount = -1;
	*status = 0;
}

int64_t HAL_Report(int32_t resource, int32_t instanceNumber,
		int32_t context, const char* feature)
{
	ROS_INFO_STREAM("HAL_Report resource = " << resource << " instanceNumber = " << instanceNumber <<
			" context = " << context << " feature = " << feature);
	return -1;
}

static HAL_ControlWord HALSIM_controlword = {0,0,0,0,0,0,0};
int32_t HAL_GetControlWord(HAL_ControlWord *controlword)
{
	*controlword = HALSIM_controlword;
	return 0;
}

// Allow non-DS attached HW interfaces to set a simulated
// control word. Used to keep DriverStation::Is* calls in
// sync with actual robot state
void HALSIM_SetControlWord(HAL_ControlWord controlword)
{
	HALSIM_controlword = controlword;
}

void HALSIM_SetDIOValue(int32_t index, HAL_Bool value)
{
	ROS_INFO_STREAM("Called HALSIM_SetDIOValue() from hardware interface?");
}

void HALSIM_SetAnalogInVoltage(int32_t index, double value)
{
	ROS_INFO_STREAM("Called HALSIM_SetAnalogInVoltage() from hardware interface?");
}

void HALSIM_SetDriverStationAllianceStationId(HAL_AllianceStationID allianceStationId)
{
	ROS_INFO_STREAM("Called HALSIM_SetDriverStationAllianceStationId on unsupported platform?");
}

void HALSIM_SetDriverStationEnabled(HAL_Bool enabled)
{
	ROS_INFO_STREAM("Called HALSIM_SetDriverStationEnabled on unsupported platform?");
}

void HALSIM_SetDriverStationAutonomous(HAL_Bool enabled)
{
	ROS_INFO_STREAM("Called HALSIM_SetDriverStationAutonomous on unsupported platform?");
}

void HALSIM_SetDriverStationDsAttached(HAL_Bool enabled)
{
	ROS_INFO_STREAM("Called HALSIM_SetDriverStationDsAttached on unsupported platform?");
}

void HALSIM_SetDriverStationFmsAttached(HAL_Bool enabled)
{
	ROS_INFO_STREAM("Called HALSIM_SetDriverStationFmsAttached on unsupported platform?");
}

void HALSIM_SetDriverStationTest(HAL_Bool enabled)
{
	ROS_INFO_STREAM("Called HALSIM_SetDriverStationTest on unsupported platform?");
}

void HALSIM_SetDriverStationEStop(HAL_Bool enabled)
{
	ROS_INFO_STREAM("Called HALSIM_SetDriverStationEStop on unsupported platform?");
}

void HALSIM_SetMatchInfo(const HAL_MatchInfo* info)
{
	ROS_INFO_STREAM("Called HALSIM_SetMatchInfo on unsupported platform?");
}

void HALSIM_NotifyDriverStationNewData(void)
{
	ROS_INFO_STREAM("Called HALSIM_NotifyDriverStationNewData on unsupported platform?");
}

void HALSIM_SetDriverStationMatchTime(double matchTime)
{
	ROS_INFO_STREAM("Called HALSIM_SetDriverStationMatchTime on unsupported platform?");
}

void HALSIM_SetJoystickPOVs(int32_t joystickNum, const HAL_JoystickPOVs* povs)
{
	ROS_INFO_STREAM("Called HALSIM_SetJoystickPOVs on unsupported platform?");
}

void HALSIM_SetJoystickAxes(int32_t joystickNum, const HAL_JoystickAxes* povs)
{
	ROS_INFO_STREAM("Called HALSIM_SetJoystickAxes on unsupported platform?");
}

void HALSIM_SetJoystickButtons(int32_t joystickNum, const HAL_JoystickButtons* povs)
{
	ROS_INFO_STREAM("Called HALSIM_SetJoystickButtons on unsupported platform?");
}


HAL_Bool HAL_SetCurrentThreadPriority(HAL_Bool realTime, int32_t priority,
									  int32_t *status)
{
	ROS_ERROR_STREAM("Call to " << __PRETTY_FUNCTION__ << " on unsupported platform");
	return true;
}

HAL_Bool HAL_SetNotifierThreadPriority(HAL_Bool realTime, int32_t priority,
									   int32_t *status)
{
	return true;
}

void HAL_SetNotifierName(HAL_NotifierHandle notifierHandle, const char *name,
						 int32_t *status)
{
	ROS_ERROR_STREAM("Call to " << __PRETTY_FUNCTION__ << " on unsupported platform");
}
void HAL_CancelNotifierAlarm(HAL_NotifierHandle notifierHandle,
							 int32_t *status)

{
	ROS_ERROR_STREAM("Call to " << __PRETTY_FUNCTION__ << " on unsupported platform");
}
HAL_AllianceStationID HAL_GetAllianceStation(int32_t* status)
{
	ROS_INFO_STREAM("Called HAL_GetAllianceStation() on unsupported platform");
	*status = 0;
	return static_cast<HAL_AllianceStationID>(-1);
}
double HAL_GetMatchTime(int32_t* status)
{
	ROS_INFO_STREAM("Called HAL_GetMatchTime() on unsupported platform");
	*status = 0;
	return -1;
}
int32_t HAL_GetMatchInfo(HAL_MatchInfo*)
{
	ROS_INFO_STREAM("Called HAL_GetMatchInfo() on unsupported platform");
	return -1;
}

void HAL_ObserveUserProgramAutonomous(void)
{
	ROS_ERROR("Called HAL_ObserveUserProgramAutonomous(void) on unsupported platform");
}
void HAL_ObserveUserProgramDisabled(void)
{
	ROS_ERROR("Called HAL_ObserveUserProgramDisabled(void) on unsupported platform");
}
void HAL_ObserveUserProgramStarting(void)
{
	ROS_ERROR("Called HAL_ObserveUserProgramStarting(void) on unsupported platform");
}
void HAL_ObserveUserProgramTeleop(void)
{
	ROS_ERROR("Called HAL_ObserveUserProgramTeleop(void) on unsupported platform");
}
void HAL_ObserveUserProgramTest(void)
{
	ROS_ERROR("Called HAL_ObserveUserProgramTest(void) on unsupported platform");
}
int32_t HAL_SetJoystickOutputs(int32_t, int64_t,
                               int32_t, int32_t)
{
	ROS_ERROR("Called HAL_SetJoystickOutputs(int32_t joystickNum, int64_t outputs, int32_t leftRumble, int32_t rightRumble) on unsupported device");
	return -1;
}

int32_t HAL_GetTeamNumber(void)
{
	ROS_WARN("HAL_GetTeamNumber() called on unsupported platform");
	return -1;
}

HAL_Bool HAL_GetRSLState(int32_t *status)
{
	*status = -1;
	ROS_WARN("HAL_GetRSLState() called on unsupported platform");
	return false;
}
HAL_Bool HAL_GetSystemTimeValid(int32_t *status)
{
	*status = -1;
	ROS_WARN("HAL_GetSystemTimeValid() called on unsupported platform");
	return false;
}

void HAL_SetUserRailEnabled6V(HAL_Bool, int32_t* status) {
	*status = -1;
	ROS_WARN("HAL_SetUserRailEnabled6V() called on unsupported platform");
}
void HAL_SetUserRailEnabled5V(HAL_Bool, int32_t* status) {
	*status = -1;
	ROS_WARN("HAL_SetUserRailEnabled5V() called on unsupported platform");
}
void HAL_SetUserRailEnabled3V3(HAL_Bool, int32_t* status) {
	*status = -1;
	ROS_WARN("HAL_SetUserRailEnabled3V3() called on unsupported platform");
}
double HAL_GetCPUTemp(int32_t *status)
{
	*status = -1;
	ROS_WARN("HAL_GetCPUTemp() called on unsupported platform");
	return -1;
}
} /// extern "C"
