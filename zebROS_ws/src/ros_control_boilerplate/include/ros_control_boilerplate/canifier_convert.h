#include <ctre/phoenix/CANifier.h>

#include "talon_interface/canifier_state_interface.h"

namespace canifier_convert
{

class CANifierConvert
{
	public:
		bool generalPin(const hardware_interface::canifier::GeneralPin input,
				ctre::phoenix::CANifier::GeneralPin &output) const;
		bool PWMChannel(hardware_interface::canifier::PWMChannel input,
				ctre::phoenix::CANifier::PWMChannel &output) const;
		bool LEDChannel(hardware_interface::canifier::LEDChannel input,
				ctre::phoenix::CANifier::LEDChannel &output) const;
		bool velocityMeasurementPeriod(hardware_interface::canifier::CANifierVelocityMeasPeriod input,
				ctre::phoenix::CANifierVelocityMeasPeriod &output) const;
		bool statusFrame(hardware_interface::canifier::CANifierStatusFrame input,
				ctre::phoenix::CANifierStatusFrame &output) const;
		bool controlFrame(hardware_interface::canifier::CANifierControlFrame input,
				ctre::phoenix::CANifierControlFrame &output) const;
};

} // namespace canifier_convert
