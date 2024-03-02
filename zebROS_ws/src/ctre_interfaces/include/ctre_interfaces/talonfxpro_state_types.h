#ifndef TALONFXPRO_STATE_TYPES_INC__
#define TALONFXPRO_STATE_TYPES_INC__

namespace hardware_interface::talonfxpro
{

enum class Inverted
{
	Uninitialized,
	CounterClockwise_Positive,
	Clockwise_Positive,
	Last
};

enum class NeutralMode
{
	Uninitialized,
	Coast,
	Brake,
	Last
};

enum class FeedbackSensorSource
{
	Uninitialized,
	RotorSensor,
	RemoteCANcoder,
	RemotePigeon2_Yaw,
	RemotePigeon2_Pitch,
	RemotePigeon2_Roll,
	FusedCANcoder,
	SyncCANcoder,
	Last
};

enum class LimitType
{
	Uninitialized,
	NormallyOpen,
	NormallyClosed,
	Last
};

enum class LimitSource
{
	Uninitialized,
	LimitSwitchPin,
	RemoteTalonFX,
	RemoteCANifier,
	RemoteCANcoder,
	Disabled,
	Last
};

enum class TalonMode
{
	First,
	DutyCycleOut,
	TorqueCurrentFOC,
	VoltageOut,
	PositionDutyCycle,
	PositionVoltage,
	PositionTorqueCurrentFOC,
	VelocityDutyCycle,
	VelocityVoltage,
	VelocityTorqueCurrentFOC,
	MotionMagicDutyCycle,
	MotionMagicVoltage,
	MotionMagicTorqueCurrentFOC,
	MotionMagicExpoDutyCycle,
	MotionMagicExpoVoltage,
	MotionMagicExpoTorqueCurrentFOC,
	MotionMagicVelocityDutyCycle,
	MotionMagicVelocityVoltage,
	MotionMagicVelocityTorqueCurrentFOC,
	DynamicMotionMagicDutyCycle,
	DynamicMotionMagicVoltage,
	DynamicMotionMagicTorqueCurrentFOC,
	Follower,
	StrictFollower,
	NeutralOut,
	CoastOut,
	StaticBrake,
	DifferentialDutyCycleOut,
	DifferentialVoltageOut,
	DifferentialPositionDutyCycle,
	DifferentialPositionVoltage,
	DifferentialVelocityDutyCycle,
	DifferentialVelocityVoltage,
	DifferentialMotionMagicDutyCycle,
	DifferentialMotionMagicVoltage,
	DifferentialFollower,
	DifferentialStrictFollower,
	Disabled,
	Last
};

enum class BridgeOutput
{
	First,
	Coast,
	Brake,
	Trapez,
	FOCTorque,
	MusicTone,
	FOCEasy,
	FaultBrake,
	FaultCoast,
	Last
};

enum class GravityType
{
	First,
	Elevator_Static,
	Arm_Cosine,
	Last
};

enum class DifferentialSensorSource
{
	First,
	Disabled,
    RemoteTalonFX_Diff,
    RemotePigeon2_Yaw,
    RemotePigeon2_Pitch,
    RemotePigeon2_Roll,
    RemoteCANcoder,
	Last
};

enum class DifferentialControlMode
{
	First,
	DisabledOutput,
	NeutralOut,
	StaticBrake,
	DutyCycleOut,
	PositionDutyCycle,
	VelocityDutyCycle,
	MotionMagicDutyCycle,
	DutyCycleFOC,
	PositionDutyCycleFOC,
	VelocityDutyCycleFOC,
	MotionMagicDutyCycleFOC,
	VoltageOut,
	PositionVoltage,
	VelocityVoltage,
	MotionMagicVoltage,
	VoltageFOC,
	PositionVoltageFOC,
	VelocityVoltageFOC,
	MotionMagicVoltageFOC,
	TorqueCurrentFOC,
	PositionTorqueCurrentFOC,
	VelocityTorqueCurrentFOC,
	MotionMagicTorqueCurrentFOC,
	Follower,
	Reserved,
	CoastOut,
	Last
};

constexpr size_t TALON_PIDF_SLOTS = 3;

} // namespace 

#endif