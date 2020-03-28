#include "talon_interface/talon_state_interface.h"
#include <ctre/phoenix/motorcontrol/ControlFrame.h>
#include <ctre/phoenix/motorcontrol/ControlMode.h>
#include <ctre/phoenix/motorcontrol/DemandType.h>
#include <ctre/phoenix/motorcontrol/FeedbackDevice.h>
#include <ctre/phoenix/motorcontrol/LimitSwitchType.h>
#include <ctre/phoenix/motorcontrol/MotorCommutation.h>
#include <ctre/phoenix/motorcontrol/NeutralMode.h>
#include <ctre/phoenix/motorcontrol/RemoteSensorSource.h>
#include <ctre/phoenix/motorcontrol/StatusFrame.h>
#include <ctre/phoenix/motorcontrol/VelocityMeasPeriod.h>
#include <ctre/phoenix/sensors/AbsoluteSensorRange.h>
#include <ctre/phoenix/sensors/SensorInitializationStrategy.h>


// Convert from internal version of hardware mode ID
// to one to write to actual Talon hardware
// Return true if conversion is OK, false if
// an unknown mode is hit.

namespace talon_convert
{
class TalonConvert
{
	public:
		bool controlMode(const hardware_interface::TalonMode input_mode,
								ctre::phoenix::motorcontrol::ControlMode &output_mode) const;
		bool demand1Type( const hardware_interface::DemandType input,
				ctre::phoenix::motorcontrol::DemandType &output) const;
		bool neutralMode(const hardware_interface::NeutralMode input_mode,
								ctre::phoenix::motorcontrol::NeutralMode &output_mode) const;
		bool feedbackDevice(
			const hardware_interface::FeedbackDevice input_fd,
			ctre::phoenix::motorcontrol::FeedbackDevice &output_fd) const;
		bool remoteFeedbackDevice(
			const hardware_interface::RemoteFeedbackDevice input_fd,
			ctre::phoenix::motorcontrol::RemoteFeedbackDevice &output_fd) const;
		bool remoteSensorSource(
				const hardware_interface::RemoteSensorSource input_rss,
				ctre::phoenix::motorcontrol::RemoteSensorSource &output_rss) const;
		bool limitSwitchSource(
			const hardware_interface::LimitSwitchSource input_ls,
			ctre::phoenix::motorcontrol::LimitSwitchSource &output_ls) const;
		bool remoteLimitSwitchSource(
			const hardware_interface::RemoteLimitSwitchSource input_ls,
			ctre::phoenix::motorcontrol::RemoteLimitSwitchSource &output_ls) const;
		bool limitSwitchNormal(
			const hardware_interface::LimitSwitchNormal input_ls,
			ctre::phoenix::motorcontrol::LimitSwitchNormal &output_ls) const;
		bool velocityMeasurementPeriod(
			const hardware_interface::VelocityMeasurementPeriod input_v_m_p,
			ctre::phoenix::motorcontrol::VelocityMeasPeriod &output_v_m_period) const;
		bool statusFrame(const hardware_interface::StatusFrame input,
			ctre::phoenix::motorcontrol::StatusFrameEnhanced &output) const;
		bool controlFrame(const hardware_interface::ControlFrame input,
			ctre::phoenix::motorcontrol::ControlFrame &output) const;
		bool motorCommutation(const hardware_interface::MotorCommutation input,
			ctre::phoenix::motorcontrol::MotorCommutation &output) const;
		bool absoluteSensorRange(const hardware_interface::AbsoluteSensorRange input,
			ctre::phoenix::sensors::AbsoluteSensorRange &output) const;
		bool sensorInitializationStrategy(const hardware_interface::SensorInitializationStrategy input,
			ctre::phoenix::sensors::SensorInitializationStrategy &output) const;
};

} // namespace talon_convert
