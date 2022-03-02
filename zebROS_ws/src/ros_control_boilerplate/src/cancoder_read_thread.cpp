
#include <ros_control_boilerplate/frc_robot_interface.h>
namespace ros_control_boilerplate
{
// Each cancoder gets their own read thread. The thread loops at a fixed rate
// reading all state from that cancoder. The state is copied to a shared buffer
// at the end of each iteration of the loop.
// The code tries to only read status when we expect there to be new
// data given the update rate of various CAN messages.
void FRCRobotInterface::cancoder_read_thread(std::shared_ptr<ctre::phoenix::sensors::CANCoder> cancoder,
											std::shared_ptr<hardware_interface::cancoder::CANCoderHWState> state,
											std::shared_ptr<std::mutex> mutex,
											std::unique_ptr<Tracer> tracer,
											double poll_frequency)
{
#ifdef __linux__
	std::stringstream thread_name{"cancdr_read_"};

	if (pthread_setname_np(pthread_self(), thread_name.str().c_str()))
	{
		ROS_ERROR_STREAM("Error setting thread name " << thread_name.str() << " " << errno);
	}
#endif
	ros::Duration(2.452 + state->getDeviceNumber() * 0.07).sleep(); // Sleep for a few seconds to let CAN start up
	for (ros::Rate r(poll_frequency); ros::ok(); r.sleep())
	{
		tracer->start("cancoder read main_loop");

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
			conversion_factor = state->getConversionFactor();
		}
		//TODO redo using feedback coefficent

		// Use FeedbackDevice_QuadEncoder to force getConversionFactor to use the encoder_ticks_per_rotation
		// variable to calculate these values
		const double position = cancoder->GetPosition() * conversion_factor;
		const double velocity = cancoder->GetVelocity() * conversion_factor;
		const double absolute_position = cancoder->GetAbsolutePosition() * conversion_factor;
		const double bus_voltage = cancoder->GetBusVoltage();
		const auto ctre_magnet_field_strength = cancoder->GetMagnetFieldStrength();
		hardware_interface::cancoder::MagnetFieldStrength magnet_field_strength;
		cancoder_convert_.magnetFieldStrength(ctre_magnet_field_strength, magnet_field_strength);
		const double last_timestamp = cancoder->GetLastTimestamp();
		const int firmware_version = cancoder->GetFirmwareVersion();

		ctre::phoenix::sensors::CANCoderFaults ctre_faults;
		cancoder->GetFaults(ctre_faults);
		const unsigned faults = ctre_faults.ToBitfield();
		ctre::phoenix::sensors::CANCoderStickyFaults ctre_sticky_faults;
		cancoder->GetStickyFaults(ctre_sticky_faults);
		const unsigned sticky_faults = ctre_sticky_faults.ToBitfield();

		// Actually update the CANCoderHWState shared between
		// this thread and read()
		// Do this all at once so the code minimizes the amount
		// of time with mutex locked
		{
			// Lock the state entry to make sure writes
			// are atomic - reads won't grab data in
			// the middle of a write
			std::lock_guard<std::mutex> l(*mutex);
			state->setPosition(position);
			state->setVelocity(velocity);
			state->setAbsolutePosition(absolute_position);
			state->setBusVoltage(bus_voltage);
			state->setMagnetFieldStrength(magnet_field_strength);
			state->setLastTimestamp(last_timestamp);
			state->setFirmwareVersion(firmware_version);
			state->setFaults(faults);
			state->setStickyFaults(sticky_faults);
		}
		tracer->report(60);
	}
}

} // namespace
