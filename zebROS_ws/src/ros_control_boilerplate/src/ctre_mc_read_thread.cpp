#ifdef __linux__
#include <pthread.h>                                  // for pthread_self
#endif
#include "ros_control_boilerplate/frc_robot_interface.h"
#include "ctre/phoenix/motion/MotionProfileStatus.h"             // for Moti...
#include "ctre/phoenix/motorcontrol/Faults.h"                    // for Faults
#include "ctre/phoenix/motorcontrol/IMotorController.h"          // for IMot...
#include "ctre/phoenix/motorcontrol/IMotorControllerEnhanced.h"  // for IMot...
#include "ctre/phoenix/motorcontrol/SensorCollection.h"          // for Sens...
#include "ctre/phoenix/motorcontrol/StickyFaults.h"              // for Stic...
#include "ctre/phoenix/motorcontrol/TalonFXSensorCollection.h"   // for Talo...
#include "ctre/phoenix/motorcontrol/can/TalonFX.h"               // for TalonFX
#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"              // for Talo...#include <ros_control_boilerplate/frc_robot_interface.h>
namespace ros_control_boilerplate
{
// Each talon/victor gets their own read thread. The thread loops at a fixed rate
// reading all state from that talon/victor. The state is copied to a shared buffer
// at the end of each iteration of the loop.
// The code tries to only read status when we expect there to be new
// data given the update rate of various CAN messages.
void FRCRobotInterface::ctre_mc_read_thread(std::shared_ptr<ctre::phoenix::motorcontrol::IMotorController> ctre_mc,
											std::shared_ptr<hardware_interface::TalonHWState> state,
											std::shared_ptr<std::mutex> mutex,
											std::unique_ptr<Tracer> tracer,
											size_t index,
											double poll_frequency)
{
#ifdef __linux__
	std::stringstream thread_name;
	// Use abbreviations since pthread_setname will fail if name is >= 16 characters
	thread_name << "ctre_mc_rd_" << state->getCANID();
	if (pthread_setname_np(pthread_self(), thread_name.str().c_str()))
	{
		ROS_ERROR_STREAM("Error setting thread name " << thread_name.str() << " " << errno);
	}
#endif
	ros::Duration(2.75 + index * 0.05).sleep(); // Sleep for a few seconds to let CAN start up
	ROS_INFO_STREAM("Starting ctre_mc " << state->getCANID() << " thread at " << ros::Time::now());

	const auto victor = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::IMotorController>(ctre_mc);
	const auto talon = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::IMotorControllerEnhanced>(ctre_mc);
	const auto talonsrx = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::can::TalonSRX>(ctre_mc);
	const auto talonfx = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::can::TalonFX>(ctre_mc);

	for (ros::Rate r(poll_frequency); ros::ok(); r.sleep())
	{
		tracer->start("talon read main_loop");

		hardware_interface::TalonMode talon_mode;
		hardware_interface::FeedbackDevice encoder_feedback;
		int encoder_ticks_per_rotation;
		double conversion_factor;

		// Update local status with relevant global config
		// values set by write(). This way, items configured
		// by controllers will be reflected in the state here
		// used when reading from talons.
		// Realistically they won't change much (except maybe mode)
		// but unless it causes performance problems reading them
		// each time through the loop is easier than waiting until
		// they've been correctly set by write() before using them
		// here.
		// Note that this isn't a complete list - only the values
		// used by the read thread are copied over.  Update
		// as needed when more are read
		{
			std::lock_guard<std::mutex> l(*mutex);
			if (!state->getEnableReadThread())
				return;
			talon_mode = state->getTalonMode();
			encoder_feedback = state->getEncoderFeedback();
			encoder_ticks_per_rotation = state->getEncoderTicksPerRotation();
			conversion_factor = state->getConversionFactor();
		}

		// TODO : in main read() loop copy status from talon being followed
		// into follower talon state?
		if (talon_mode == hardware_interface::TalonMode_Follower)
			return;

		const double radians_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, hardware_interface::TalonMode_Position) * conversion_factor;
		const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, hardware_interface::TalonMode_Velocity) * conversion_factor;

		const double motor_output_percent = victor->GetMotorOutputPercent();
		safeTalonCall(victor->GetLastError(), "GetMotorOutputPercent", state->getCANID());

		ctre::phoenix::motorcontrol::Faults faults;
		safeTalonCall(victor->GetFaults(faults), "GetFaults", state->getCANID());

		// applied control mode - cached
		// soft limit and limit switch override - cached

		const double position = victor->GetSelectedSensorPosition(pidIdx) * radians_scale;
		safeTalonCall(victor->GetLastError(), "GetSelectedSensorPosition", state->getCANID());

		const double velocity = victor->GetSelectedSensorVelocity(pidIdx) * radians_per_second_scale;
		safeTalonCall(victor->GetLastError(), "GetSelectedSensorVelocity", state->getCANID());

		double output_current = -1;
		if (talon)
		{
			output_current = talon->GetOutputCurrent();
			safeTalonCall(victor->GetLastError(), "GetOutputCurrent", state->getCANID());
		}

		ctre::phoenix::motorcontrol::StickyFaults sticky_faults;
		safeTalonCall(victor->GetStickyFaults(sticky_faults), "GetStickyFault", state->getCANID());

		const double bus_voltage = victor->GetBusVoltage();
		safeTalonCall(victor->GetLastError(), "GetBusVoltage", state->getCANID());

		const double temperature = victor->GetTemperature(); //returns in Celsius
		safeTalonCall(victor->GetLastError(), "GetTemperature", state->getCANID());

		const double output_voltage = victor->GetMotorOutputVoltage();
		safeTalonCall(victor->GetLastError(), "GetMotorOutputVoltage", state->getCANID());

		double closed_loop_error = 0;
		double integral_accumulator = 0;
		double error_derivative = 0;
		double closed_loop_target = 0;

		if ((talon_mode == hardware_interface::TalonMode_Position) ||
			(talon_mode == hardware_interface::TalonMode_Velocity) ||
			(talon_mode == hardware_interface::TalonMode_Current ) ||
			(talon_mode == hardware_interface::TalonMode_MotionProfile) ||
			(talon_mode == hardware_interface::TalonMode_MotionMagic)   ||
			(talon_mode == hardware_interface::TalonMode_MotionProfileArc))
		{
			const double closed_loop_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, talon_mode) * conversion_factor;

			closed_loop_error = victor->GetClosedLoopError(pidIdx) * closed_loop_scale;
			safeTalonCall(victor->GetLastError(), "GetClosedLoopError", state->getCANID());

			integral_accumulator = victor->GetIntegralAccumulator(pidIdx) * closed_loop_scale;
			safeTalonCall(victor->GetLastError(), "GetIntegralAccumulator", state->getCANID());

			error_derivative = victor->GetErrorDerivative(pidIdx) * closed_loop_scale;
			safeTalonCall(victor->GetLastError(), "GetErrorDerivative", state->getCANID());

			closed_loop_target = victor->GetClosedLoopTarget(pidIdx) * closed_loop_scale;
			safeTalonCall(victor->GetLastError(), "GetClosedLoopTarget", state->getCANID());
		}

		// Targets Status 10 - 160 mSec default
		double active_trajectory_position = 0.0;
		double active_trajectory_velocity = 0.0;
		double active_trajectory_heading = 0.0;
		if ((talon_mode == hardware_interface::TalonMode_MotionProfile) ||
			(talon_mode == hardware_interface::TalonMode_MotionMagic)   ||
			(talon_mode == hardware_interface::TalonMode_MotionProfileArc))
		{
			active_trajectory_position = victor->GetActiveTrajectoryPosition() * radians_scale;
			safeTalonCall(victor->GetLastError(), "GetActiveTrajectoryPosition", state->getCANID());

			active_trajectory_velocity = victor->GetActiveTrajectoryVelocity() * radians_per_second_scale;
			safeTalonCall(victor->GetLastError(), "GetActiveTrajectoryVelocity", state->getCANID());

			if (talon_mode == hardware_interface::TalonMode_MotionProfileArc)
			{
				active_trajectory_heading = victor->GetActiveTrajectoryPosition(1) * 2. * M_PI / 360.; //returns in degrees
				safeTalonCall(victor->GetLastError(), "GetActiveTrajectoryHeading", state->getCANID());
			}
		}

		int mp_top_level_buffer_count = 0;
		hardware_interface::MotionProfileStatus internal_status;
		if (talon_mode == hardware_interface::TalonMode_MotionProfile)
		{
			mp_top_level_buffer_count = victor->GetMotionProfileTopLevelBufferCount();
			ctre::phoenix::motion::MotionProfileStatus talon_status;
			safeTalonCall(victor->GetMotionProfileStatus(talon_status), "GetMotionProfileStatus", state->getCANID());

			internal_status.topBufferRem = talon_status.topBufferRem;
			internal_status.topBufferCnt = talon_status.topBufferCnt;
			internal_status.btmBufferCnt = talon_status.btmBufferCnt;
			internal_status.hasUnderrun = talon_status.hasUnderrun;
			internal_status.isUnderrun = talon_status.isUnderrun;
			internal_status.activePointValid = talon_status.activePointValid;
			internal_status.isLast = talon_status.isLast;
			internal_status.profileSlotSelect0 = talon_status.profileSlotSelect0;
			internal_status.profileSlotSelect1 = talon_status.profileSlotSelect1;
			internal_status.outputEnable = static_cast<hardware_interface::SetValueMotionProfile>(talon_status.outputEnable);
			internal_status.timeDurMs = talon_status.timeDurMs;
		}

		bool forward_limit_switch = false;
		bool reverse_limit_switch = false;
		if (talonsrx)
		{
			auto sensor_collection = talonsrx->GetSensorCollection();

			forward_limit_switch = sensor_collection.IsFwdLimitSwitchClosed();
			reverse_limit_switch = sensor_collection.IsRevLimitSwitchClosed();
		}
		else if (talonfx)
		{
			auto sensor_collection = talonfx->GetSensorCollection();

			forward_limit_switch = sensor_collection.IsFwdLimitSwitchClosed();
			reverse_limit_switch = sensor_collection.IsRevLimitSwitchClosed();
		}

		const int firmware_version = victor->GetFirmwareVersion();

		// Actually update the TalonHWState shared between
		// this thread and read()
		// Do this all at once so the code minimizes the amount
		// of time with mutex locked
		{
			// Lock the state entry to make sure writes
			// are atomic - reads won't grab data in
			// the middle of a write
			std::lock_guard<std::mutex> l(*mutex);

			if (talon_mode == hardware_interface::TalonMode_MotionProfile)
			{
				state->setMotionProfileStatus(internal_status);
				state->setMotionProfileTopLevelBufferCount(mp_top_level_buffer_count);
			}

			state->setMotorOutputPercent(motor_output_percent);
			state->setFaults(faults.ToBitfield());

			state->setForwardSoftlimitHit(faults.ForwardSoftLimit);
			state->setReverseSoftlimitHit(faults.ReverseSoftLimit);

			state->setPosition(position);
			state->setSpeed(velocity);
			if (talon)
				state->setOutputCurrent(output_current);
			state->setStickyFaults(sticky_faults.ToBitfield());

			state->setBusVoltage(bus_voltage);
			state->setTemperature(temperature);
			state->setOutputVoltage(output_voltage);

			if ((talon_mode == hardware_interface::TalonMode_Position) ||
				(talon_mode == hardware_interface::TalonMode_Velocity) ||
				(talon_mode == hardware_interface::TalonMode_Current ) ||
				(talon_mode == hardware_interface::TalonMode_MotionProfile) ||
				(talon_mode == hardware_interface::TalonMode_MotionMagic)   ||
				(talon_mode == hardware_interface::TalonMode_MotionProfileArc))
			{
				state->setClosedLoopError(closed_loop_error);
				state->setIntegralAccumulator(integral_accumulator);
				state->setErrorDerivative(error_derivative);
				// Reverse engineer the individual P,I,D,F components used
				// to generate closed-loop control signals to the motor
				// This is just for debugging PIDF tuning
				const auto closed_loop_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, talon_mode) * conversion_factor;
				const auto pidf_slot = state->getSlot();
				const auto kp = state->getPidfP(pidf_slot);
				const auto ki = state->getPidfI(pidf_slot);
				const auto kd = state->getPidfD(pidf_slot);
				const auto native_closed_loop_error = closed_loop_error / closed_loop_scale;
				state->setPTerm(native_closed_loop_error * kp);
				state->setITerm(integral_accumulator * ki);
				state->setDTerm(error_derivative * kd);
				if ((talon_mode != hardware_interface::TalonMode_MotionProfile) &&
					(talon_mode != hardware_interface::TalonMode_MotionMagic) &&
					(talon_mode != hardware_interface::TalonMode_MotionProfileArc))
				{
					state->setClosedLoopTarget(closed_loop_target);

					const double kf = state->getPidfF(pidf_slot);
					state->setFTerm(closed_loop_target / closed_loop_scale * kf);
				}
			}

			if ((talon_mode == hardware_interface::TalonMode_MotionProfile) ||
				(talon_mode == hardware_interface::TalonMode_MotionMagic)   ||
				(talon_mode == hardware_interface::TalonMode_MotionProfileArc))
			{
				state->setActiveTrajectoryPosition(active_trajectory_position);
				state->setActiveTrajectoryVelocity(active_trajectory_velocity);
				if (talon_mode == hardware_interface::TalonMode_MotionProfileArc)
				{
					state->setActiveTrajectoryHeading(active_trajectory_heading);
				}
			}

			if (talon)
			{
				state->setForwardLimitSwitch(forward_limit_switch);
				state->setReverseLimitSwitch(reverse_limit_switch);
			}

			state->setFirmwareVersion(firmware_version);
		}
		tracer->report(60);
	}
}

} // namespace
