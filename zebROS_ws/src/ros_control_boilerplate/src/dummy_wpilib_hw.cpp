// Functions referenced by various WPILIB code used on non-Rio targets but not actually
// called. These functions let us build wpilib-dependent code on x86 and Jetson targets
// by stubbing out functions which aren't actually used
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
		const uint32_t arbId = messageID & 0xFFFFFFC0;
		if ((arbId == PCM_CONTROL_1) || (arbId == PCM_CONTROL_2) || (arbId == PCM_CONTROL_3))
			return;

		ctre::phoenix::platform::can::CANComm_SendMessage(messageID, data, dataSize, status, canBus.c_str());
	}
	void HAL_CAN_ReceiveMessage(uint32_t *messageID, uint32_t messageIDMask, uint8_t *data, uint8_t *dataSize, uint32_t *timeStamp, int32_t *status)
	{
		ctre::phoenix::platform::can::canframe_t canframe{};
		ctre::phoenix::platform::can::CANComm_ReceiveMessage(*messageID, canframe, status, canBus.c_str());
		*dataSize = canframe.len;
		std::memcpy(data, canframe.data, *dataSize);

		*timeStamp = canframe.swTimestampUs / 1000;
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
					messages[i].timeStamp = localMessages[i].swTimestampUs;
					memcpy(messages[i].data, localMessages[i].data, sizeof(localMessages[i].data));
					messages[i].dataSize = localMessages[i].len;
			}
	}
}

#include <frc/AnalogInput.h>
frc::AnalogInput::AnalogInput(int)
{
	ROS_ERROR("Called AnalogInput::AnalogInput(int) on unsupported platform");
}
frc::AnalogInput::~AnalogInput()
{
	ROS_ERROR("Called ::AnalogInput::~AnalogInput( on unsupported platform");
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

#include "frc/DSControlWord.h"
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
} /// extern "C"

#include <ros_control_boilerplate/error_queue.h>
#include <hal/HALBase.h>
extern "C"
{
int32_t HAL_SendError(HAL_Bool /*isError*/, int32_t errorCode, HAL_Bool /*isLVCode*/, const char *details, const char *, const char *, HAL_Bool )
{
    ROS_ERROR_STREAM("HAL_SendError called : errorCode = " << errorCode
			<< " = \"" <<  HAL_GetErrorMessage(errorCode)
			<< "\" : details = \"" << details << "\"");

	errorQueue->enqueue(errorCode, std::string(details));
	return 0;
}
}

#include <frc/Timer.h>
units::second_t frc::Timer::GetFPGATimestamp()
{
	//ROS_ERROR("Called frc::Timer::GetFPGATimestamp() on unsupported platform");
	return static_cast<units::second_t>(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
}


// From wpilib HAL.cpp
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "hal/Errors.h"
#include "../sim/ErrorsInternal.h"
//#include "ctre/ctre.h"
//#include "FRC_FPGA_ChipObject/fpgainterfacecapi/NiFpga.h"
#include "FRC_NetworkCommunication/FRCComm.h"
//#include "visa/visa.h"

extern "C" {
const char* HAL_GetErrorMessage(int32_t code) {
  switch (code) {
    case 0:
      return "";
    case NiFpga_Status_FifoTimeout:
      return NiFpga_Status_FifoTimeout_MESSAGE;
    case NiFpga_Status_TransferAborted:
      return NiFpga_Status_TransferAborted_MESSAGE;
    case NiFpga_Status_MemoryFull:
      return NiFpga_Status_MemoryFull_MESSAGE;
    case NiFpga_Status_SoftwareFault:
      return NiFpga_Status_SoftwareFault_MESSAGE;
    case NiFpga_Status_InvalidParameter:
      return NiFpga_Status_InvalidParameter_MESSAGE;
    case NiFpga_Status_ResourceNotFound:
      return NiFpga_Status_ResourceNotFound_MESSAGE;
    case NiFpga_Status_ResourceNotInitialized:
      return NiFpga_Status_ResourceNotInitialized_MESSAGE;
    case NiFpga_Status_HardwareFault:
      return NiFpga_Status_HardwareFault_MESSAGE;
    case NiFpga_Status_IrqTimeout:
      return NiFpga_Status_IrqTimeout_MESSAGE;
    case SAMPLE_RATE_TOO_HIGH:
      return SAMPLE_RATE_TOO_HIGH_MESSAGE;
    case VOLTAGE_OUT_OF_RANGE:
      return VOLTAGE_OUT_OF_RANGE_MESSAGE;
    case LOOP_TIMING_ERROR:
      return LOOP_TIMING_ERROR_MESSAGE;
    case SPI_WRITE_NO_MOSI:
      return SPI_WRITE_NO_MOSI_MESSAGE;
    case SPI_READ_NO_MISO:
      return SPI_READ_NO_MISO_MESSAGE;
    case SPI_READ_NO_DATA:
      return SPI_READ_NO_DATA_MESSAGE;
    case INCOMPATIBLE_STATE:
      return INCOMPATIBLE_STATE_MESSAGE;
    case NO_AVAILABLE_RESOURCES:
      return NO_AVAILABLE_RESOURCES_MESSAGE;
    case RESOURCE_IS_ALLOCATED:
      return RESOURCE_IS_ALLOCATED_MESSAGE;
    case RESOURCE_OUT_OF_RANGE:
      return RESOURCE_OUT_OF_RANGE_MESSAGE;
    case HAL_INVALID_ACCUMULATOR_CHANNEL:
      return HAL_INVALID_ACCUMULATOR_CHANNEL_MESSAGE;
    case HAL_HANDLE_ERROR:
      return HAL_HANDLE_ERROR_MESSAGE;
    case NULL_PARAMETER:
      return NULL_PARAMETER_MESSAGE;
    case ANALOG_TRIGGER_LIMIT_ORDER_ERROR:
      return ANALOG_TRIGGER_LIMIT_ORDER_ERROR_MESSAGE;
    case ANALOG_TRIGGER_PULSE_OUTPUT_ERROR:
      return ANALOG_TRIGGER_PULSE_OUTPUT_ERROR_MESSAGE;
    case PARAMETER_OUT_OF_RANGE:
      return PARAMETER_OUT_OF_RANGE_MESSAGE;
    case HAL_COUNTER_NOT_SUPPORTED:
      return HAL_COUNTER_NOT_SUPPORTED_MESSAGE;
    case HAL_ERR_CANSessionMux_InvalidBuffer:
      return ERR_CANSessionMux_InvalidBuffer_MESSAGE;
    case HAL_ERR_CANSessionMux_MessageNotFound:
      return ERR_CANSessionMux_MessageNotFound_MESSAGE;
    case HAL_WARN_CANSessionMux_NoToken:
      return WARN_CANSessionMux_NoToken_MESSAGE;
    case HAL_ERR_CANSessionMux_NotAllowed:
      return ERR_CANSessionMux_NotAllowed_MESSAGE;
    case HAL_ERR_CANSessionMux_NotInitialized:
      return ERR_CANSessionMux_NotInitialized_MESSAGE;
    case VI_ERROR_SYSTEM_ERROR:
      return VI_ERROR_SYSTEM_ERROR_MESSAGE;
    case VI_ERROR_INV_OBJECT:
      return VI_ERROR_INV_OBJECT_MESSAGE;
    case VI_ERROR_RSRC_LOCKED:
      return VI_ERROR_RSRC_LOCKED_MESSAGE;
    case VI_ERROR_RSRC_NFOUND:
      return VI_ERROR_RSRC_NFOUND_MESSAGE;
    case VI_ERROR_INV_RSRC_NAME:
      return VI_ERROR_INV_RSRC_NAME_MESSAGE;
    case VI_ERROR_QUEUE_OVERFLOW:
      return VI_ERROR_QUEUE_OVERFLOW_MESSAGE;
    case VI_ERROR_IO:
      return VI_ERROR_IO_MESSAGE;
    case VI_ERROR_ASRL_PARITY:
      return VI_ERROR_ASRL_PARITY_MESSAGE;
    case VI_ERROR_ASRL_FRAMING:
      return VI_ERROR_ASRL_FRAMING_MESSAGE;
    case VI_ERROR_ASRL_OVERRUN:
      return VI_ERROR_ASRL_OVERRUN_MESSAGE;
    case VI_ERROR_RSRC_BUSY:
      return VI_ERROR_RSRC_BUSY_MESSAGE;
    case VI_ERROR_INV_PARAMETER:
      return VI_ERROR_INV_PARAMETER_MESSAGE;
    case HAL_PWM_SCALE_ERROR:
      return HAL_PWM_SCALE_ERROR_MESSAGE;
    case HAL_SERIAL_PORT_NOT_FOUND:
      return HAL_SERIAL_PORT_NOT_FOUND_MESSAGE;
    case HAL_THREAD_PRIORITY_ERROR:
      return HAL_THREAD_PRIORITY_ERROR_MESSAGE;
    case HAL_THREAD_PRIORITY_RANGE_ERROR:
      return HAL_THREAD_PRIORITY_RANGE_ERROR_MESSAGE;
    case HAL_SERIAL_PORT_OPEN_ERROR:
      return HAL_SERIAL_PORT_OPEN_ERROR_MESSAGE;
    case HAL_SERIAL_PORT_ERROR:
      return HAL_SERIAL_PORT_ERROR_MESSAGE;
    case HAL_CAN_TIMEOUT:
      return HAL_CAN_TIMEOUT_MESSAGE;
    case ERR_FRCSystem_NetCommNotResponding:
      return ERR_FRCSystem_NetCommNotResponding_MESSAGE;
    case ERR_FRCSystem_NoDSConnection:
      return ERR_FRCSystem_NoDSConnection_MESSAGE;
    case HAL_CAN_BUFFER_OVERRUN:
      return HAL_CAN_BUFFER_OVERRUN_MESSAGE;
    case HAL_LED_CHANNEL_ERROR:
      return HAL_LED_CHANNEL_ERROR_MESSAGE;
    case HAL_INVALID_DMA_STATE:
      return HAL_INVALID_DMA_STATE_MESSAGE;
    case HAL_INVALID_DMA_ADDITION:
      return HAL_INVALID_DMA_ADDITION_MESSAGE;
    case HAL_USE_LAST_ERROR:
      return HAL_USE_LAST_ERROR_MESSAGE;
    case HAL_CONSOLE_OUT_ENABLED_ERROR:
      return HAL_CONSOLE_OUT_ENABLED_ERROR_MESSAGE;
    default:
      return "Unknown error status";
  }
}

} // extern "C"

#include "hal/Notifier.h"
HAL_NotifierHandle HAL_InitializeNotifier(int32_t* status) {
        *status = 0;
        return 1;
}
void HAL_StopNotifier(HAL_NotifierHandle notifierHandle, int32_t* status) {
        (void)notifierHandle;
        *status = 0;
}

void HAL_CleanNotifier(HAL_NotifierHandle notifierHandle, int32_t* status) {
        (void)notifierHandle;
        *status = 0;
}
void HAL_UpdateNotifierAlarm(HAL_NotifierHandle notifierHandle,
                             uint64_t triggerTime, int32_t* status) {
        (void)notifierHandle;
        (void)triggerTime;
        *status = 0;
}
uint64_t HAL_WaitForNotifierAlarm(HAL_NotifierHandle notifierHandle,
                                  int32_t* status) {
        (void)notifierHandle;
        *status = 0;
        return 1;
}

size_t HAL_GetSerialNumber(char* buffer, size_t size) {
		*buffer = '\0';
		return 0;
}

size_t HAL_GetComments(char* buffer, size_t size) {
		*buffer = '\0';
		return 0;
}

