#ifndef INC_TALON_STATE_TYPES__
#define INC_TALON_STATE_TYPES__

#include <cstddef>
#include <cstdint>

namespace hardware_interface
{
// These should mirror the modes listed in ControlModes.h
// Need a separate copy here though, since sim won't be
// including that header file - sim shouldn't require
// anything specifically from the actual motor controller
// hardware
enum TalonMode
{
	TalonMode_First = -1,
	TalonMode_PercentOutput,
	TalonMode_Position,      // CloseLoop
	TalonMode_Velocity,      // CloseLoop
	TalonMode_Current,       // CloseLoop
	TalonMode_Follower,
	TalonMode_MotionProfile,
	TalonMode_MotionMagic,
	TalonMode_MotionProfileArc,
    TalonMode_Music,
	TalonMode_Disabled,
	TalonMode_Last
};

enum DemandType
{
	DemandType_Neutral,
	DemandType_AuxPID,
	DemandType_ArbitraryFeedForward,
	DemandType_Last
};

enum NeutralMode
{
	NeutralMode_Uninitialized,
	NeutralMode_EEPROM_Setting,
	NeutralMode_Coast,
	NeutralMode_Brake,
	NeutralMode_Last
};

enum FeedbackDevice
{
	FeedbackDevice_Uninitialized,
	FeedbackDevice_QuadEncoder,
	FeedbackDevice_CTRE_MagEncoder_Relative = FeedbackDevice_QuadEncoder,
	FeedbackDevice_IntegratedSensor,
	FeedbackDevice_Analog,
	FeedbackDevice_Tachometer,
	FeedbackDevice_PulseWidthEncodedPosition,
	FeedbackDevice_CTRE_MagEncoder_Absolute = FeedbackDevice_PulseWidthEncodedPosition,
	FeedbackDevice_SensorSum,
	FeedbackDevice_SensorDifference,
	FeedbackDevice_RemoteSensor0,
	FeedbackDevice_RemoteSensor1,
	FeedbackDevice_None,
	FeedbackDevice_SoftwareEmulatedSensor,
	FeedbackDevice_Last
};

enum RemoteFeedbackDevice
{
	RemoteFeedbackDevice_SensorSum,
	RemoteFeedbackDevice_SensorDifference,
	RemoteFeedbackDevice_RemoteSensor0,
	RemoteFeedbackDevice_RemoteSensor1,
	RemoteFeedbackDevice_None,
	RemoteFeedbackDevice_SoftwareEmulatedSensor,
	RemoteFeedbackDevice_Last
};

enum RemoteSensorSource
{
	RemoteSensorSource_Off,
	RemoteSensorSource_TalonSRX_SelectedSensor,
	RemoteSensorSource_Pigeon_Yaw,
	RemoteSensorSource_Pigeon_Pitch,
	RemoteSensorSource_Pigeon_Roll,
	RemoteSensorSource_CANifier_Quadrature,
	RemoteSensorSource_CANifier_PWMInput0,
	RemoteSensorSource_CANifier_PWMInput1,
	RemoteSensorSource_CANifier_PWMInput2,
	RemoteSensorSource_CANifier_PWMInput3,
	RemoteSensorSource_GadgeteerPigeon_Yaw,
	RemoteSensorSource_GadgeteerPigeon_Pitch,
	RemoteSensorSource_GadgeteerPigeon_Roll,
	RemoteSensorSource_CANCoder,
	RemoteSensorSource_Last
};

enum SensorTerm
{
	SensorTerm_Sum0,
	SensorTerm_Sum1,
	SensorTerm_Diff0,
	SensorTerm_Diff1,
	SensorTerm_Last
};

enum LimitSwitchSource
{
	LimitSwitchSource_Uninitialized,
	LimitSwitchSource_FeedbackConnector,
	LimitSwitchSource_RemoteTalonSRX,
	LimitSwitchSource_RemoteCANifier,
	LimitSwitchSource_Deactivated,
	LimitSwitchSource_Last
};

enum RemoteLimitSwitchSource
{
	RemoteLimitSwitchSource_Uninitialized,
	RemoteLimitSwitchSource_RemoteTalonSRX,
	RemoteLimitSwitchSource_RemoteCANifier,
	RemoteLimitSwitchSource_Deactivated,
	RemoteLimitSwitchSource_Last
};

enum LimitSwitchNormal
{
	LimitSwitchNormal_Uninitialized,
	LimitSwitchNormal_NormallyOpen,
	LimitSwitchNormal_NormallyClosed,
	LimitSwitchNormal_Disabled,
	LimitSwitchNormal_Last
};

enum VelocityMeasurementPeriod {
	Period_1Ms = 1,
	Period_2Ms = 2,
	Period_5Ms = 5,
	Period_10Ms = 10,
	Period_20Ms = 20,
	Period_25Ms = 25,
	Period_50Ms = 50,
	Period_100Ms = 100,
};

enum StatusFrame
{
	Status_1_General,
	Status_2_Feedback0,
	Status_3_Quadrature,
	Status_4_AinTempVbat,
	Status_6_Misc,
	Status_7_CommStatus,
	Status_8_PulseWidth,
	Status_9_MotProfBuffer,
	Status_10_MotionMagic,
	Status_10_Targets = Status_10_MotionMagic,
	Status_11_UartGadgeteer,
	Status_12_Feedback1,
	Status_13_Base_PIDF0,
	Status_14_Turn_PIDF1,
	Status_15_FirmwareApiStatus,
	Status_17_Targets1,
	Status_Brushless_Current,
	Status_Last
};
constexpr uint8_t status_1_general_default = 10;
constexpr uint8_t status_2_feedback0_default = 20;
constexpr uint8_t status_3_quadrature_default = 160;
constexpr uint8_t status_4_aintempvbat_default = 160;
constexpr uint8_t status_6_misc_default = 0;
constexpr uint8_t status_7_commstatus_default = 0;
constexpr uint8_t status_8_pulsewidth_default = 160;
constexpr uint8_t status_9_motprofbuffer_default = 250;
constexpr uint8_t status_10_motionmagic_default = 160;
constexpr uint8_t status_11_uartgadgeteer_default = 250;
constexpr uint8_t status_12_feedback1_default = 250;
constexpr uint8_t status_13_base_pidf0_default = 160;
constexpr uint8_t status_14_turn_pidf1_default = 250;
constexpr uint8_t status_15_firmwareapistatus_default = 160;
constexpr uint8_t status_17_targets1_default = 250;
constexpr uint8_t status_brushless_current_default = 50;

enum ControlFrame
{
	Control_3_General,
	Control_4_Advanced,
	Control_5_FeedbackOutputOverride,
	Control_6_MotProfAddTrajPoint,
	Control_Last
};
// TODO : what should these defaults be?
constexpr uint8_t control_3_general_default = 0;
constexpr uint8_t control_4_advanced_default = 0;
constexpr uint8_t control_5_feedbackoutputoverride_default = 0;
constexpr uint8_t control_6_motprofaddtrajpoint_default = 0;

// Match up with CTRE Motion profile struct
enum SetValueMotionProfile
{
	Disable = 0, Enable = 1, Hold = 2,
};

enum class MotorCommutation {
	Trapezoidal //!< Trapezoidal Commutation
};

struct MotionProfileStatus
{
	int  topBufferRem;
	int  topBufferCnt;
	int  btmBufferCnt;
	bool hasUnderrun;
	bool isUnderrun;
	bool activePointValid;
	bool isLast;
	int  profileSlotSelect0;
	int  profileSlotSelect1;
	SetValueMotionProfile outputEnable;
	int  timeDurMs;

	MotionProfileStatus(void):
		topBufferRem(0),
		topBufferCnt(0),
		btmBufferCnt(0),
		hasUnderrun(false),
		isUnderrun(false),
		activePointValid(false),
		isLast(false),
		profileSlotSelect0(0),
		profileSlotSelect1(0),
		outputEnable(Disable),
		timeDurMs(0)
	{
	}

};

constexpr size_t TALON_PIDF_SLOTS = 4;

} // namespace hardware_interface

#endif
