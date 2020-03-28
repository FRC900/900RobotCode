#include <ctre/phoenix/sensors/CANCoderStatusFrame.h>
#include <ctre/phoenix/sensors/AbsoluteSensorRange.h>
#include <ctre/phoenix/sensors/MagnetFieldStrength.h>
#include <ctre/phoenix/sensors/SensorInitializationStrategy.h>
#include <ctre/phoenix/sensors/SensorVelocityMeasPeriod.h>
#include <ctre/phoenix/sensors/SensorTimeBase.h>
#include "talon_interface/cancoder_state_interface.h"

namespace cancoder_convert
{
class CANCoderConvert
{
	public:
		bool magnetFieldStrength(ctre::phoenix::sensors::MagnetFieldStrength input,
				hardware_interface::cancoder::MagnetFieldStrength &output) const;
		bool velocityMeasPeriod(hardware_interface::cancoder::SensorVelocityMeasPeriod input,
				ctre::phoenix::sensors::SensorVelocityMeasPeriod &output) const;
		bool absoluteSensorRange(hardware_interface::cancoder::AbsoluteSensorRange input,
				ctre::phoenix::sensors::AbsoluteSensorRange &output) const;
		bool initializationStrategy(hardware_interface::cancoder::SensorInitializationStrategy input,
				ctre::phoenix::sensors::SensorInitializationStrategy &output) const;
		bool timeBase(hardware_interface::cancoder::SensorTimeBase input,
				ctre::phoenix::sensors::SensorTimeBase &output) const;
};

} // namespace cancoder_convert
