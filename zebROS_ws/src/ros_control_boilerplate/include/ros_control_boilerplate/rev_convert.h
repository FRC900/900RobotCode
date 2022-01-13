#include <rev/CANSparkMax.h>
#include "spark_max_interface/spark_max_state_interface.h"

namespace rev_convert
{

class RevConvert
{
	public:
		bool motorType(const hardware_interface::MotorType input,
				rev::CANSparkMaxLowLevel::MotorType &output) const;
		bool limitSwitchPolarity(const hardware_interface::LimitSwitchPolarity input,
				rev::CANDigitalInput::LimitSwitchPolarity &output) const;
		bool encoderType(const hardware_interface::SensorType input,
				rev::CANEncoder::EncoderType &output) const;
		bool controlType(const hardware_interface::ControlType input,
				rev::ControlType &output) const;
		bool arbFFUnits(const hardware_interface::ArbFFUnits input,
				rev::CANPIDController::ArbFFUnits &output) const;
		bool idleMode(const hardware_interface::IdleMode input,
				rev::CANSparkMax::IdleMode &output) const;
		bool externalFollower(const hardware_interface::ExternalFollower input,
				rev::CANSparkMax::ExternalFollower &output) const;

};
} // namespace rev_convert

