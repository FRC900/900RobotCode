
#include <ros_control_boilerplate/frc_robot_interface.h>
#include <angles/angles.h>
namespace ros_control_boilerplate
{
// Each pigeon2 gets their own read thread. The thread loops at a fixed rate
// reading all state from that pigeon2. The state is copied to a shared buffer
// at the end of each iteration of the loop.
// The code tries to only read status when we expect there to be new
// data given the update rate of various CAN messages.
void FRCRobotInterface::pigeon2_read_thread(std::shared_ptr<ctre::phoenix::sensors::Pigeon2> pigeon2,
											std::shared_ptr<hardware_interface::pigeon2::Pigeon2HWState> state,
											std::shared_ptr<std::mutex> mutex,
											std::unique_ptr<Tracer> tracer,
											double poll_frequency)
{
#ifdef __linux__
	std::stringstream thread_name{"pgn2_read_"};

	if (pthread_setname_np(pthread_self(), thread_name.str().c_str()))
	{
		ROS_ERROR_STREAM("Error setting thread name " << thread_name.str() << " " << errno);
	}
#endif
	ros::Duration(2.952 + state->getDeviceNumber() * 0.04).sleep(); // Sleep for a few seconds to let CAN start up
	for (ros::Rate r(poll_frequency); ros::ok(); r.sleep())
	{
		tracer->start("pigeon2 read main_loop");

		std::array<double, 3> gravity_vector;
		pigeon2->GetGravityVector(&gravity_vector[0]);

		ctre::phoenix::sensors::Pigeon2_Faults ctre_faults;
		pigeon2->GetFaults(ctre_faults);
		const unsigned faults = ctre_faults.ToBitfield();
		ctre::phoenix::sensors::Pigeon2_StickyFaults ctre_sticky_faults;
		pigeon2->GetStickyFaults(ctre_sticky_faults);
		const unsigned sticky_faults = ctre_sticky_faults.ToBitfield();

		std::array<double, 4> quaternion_6d;
		pigeon2->Get6dQuaternion(&quaternion_6d[0]);

		std::array<double, 3> ypr;
		pigeon2->GetYawPitchRoll(&ypr[0]);

		std::array<double, 3> accum_gyro;
		pigeon2->GetAccumGyro(&accum_gyro[0]);

		const double absolute_compass_heading = pigeon2->GetAbsoluteCompassHeading();
		const double compass_heading = pigeon2->GetCompassHeading();
		const double compass_field_strength = pigeon2->GetCompassFieldStrength();
		const double temperature = pigeon2->GetTemp();
		const uint32_t uptime = pigeon2->GetUpTime();
		
		std::array<int16_t, 3> raw_magnetometer;
		pigeon2->GetRawMagnetometer(&raw_magnetometer[0]);

		std::array<int16_t, 3> biased_magnetometer;
		pigeon2->GetBiasedMagnetometer(&biased_magnetometer[0]);

		std::array<int16_t, 3> biased_accelerometer;
		pigeon2->GetBiasedAccelerometer(&biased_accelerometer[0]);

		std::array<double, 3> raw_gyro;
		pigeon2->GetRawGyro(&raw_gyro[0]);
		for (auto &r : raw_gyro)
		{
			r = angles::from_degrees(r);
		}

		//const uint32_t reset_count = pigeon2->GetResetCount();
		const uint32_t reset_flags = pigeon2->GetResetFlags();
		const uint32_t firmware_version = pigeon2->GetFirmVers();

		// Actually update the Pigeon2HWState shared between
		// this thread and read()
		// Do this all at once so the code minimizes the amount
		// of time with mutex locked
		{
			// Lock the state entry to make sure writes
			// are atomic - reads won't grab data in
			// the middle of a write
			std::lock_guard<std::mutex> l(*mutex);
			state->setGravityVector(gravity_vector);
			state->setFaults(faults);
			state->setStickyFaults(sticky_faults);

			state->set6dQuaternion(quaternion_6d);
			state->setYaw(angles::from_degrees(ypr[0]));
			state->setPitch(angles::from_degrees(ypr[1]));
			state->setRoll(angles::from_degrees(ypr[2]));
			state->setAccumGyro({angles::from_degrees(accum_gyro[0]),
								angles::from_degrees(accum_gyro[1]),
								angles::from_degrees(accum_gyro[2])});
			state->setAbsoluteCompassHeading(angles::from_degrees(absolute_compass_heading));
			state->setCompassHeading(angles::from_degrees(compass_heading));
			state->setCompassFieldStrength(compass_field_strength);
			state->setTemperature(temperature);
			state->setUptime(uptime);
			state->setRawMagnetometer(raw_magnetometer);
			state->setBiasedMagnetometer(biased_magnetometer);
			state->setBiasedAccelerometer(biased_accelerometer);
			state->setRawGyro(raw_gyro);
			//state->setResetCount(reset_count);
			state->setResetFlags(reset_flags);
			state->setFirmwareVersion(firmware_version);

		}
		tracer->report(60);
	}
}

} // namespace
