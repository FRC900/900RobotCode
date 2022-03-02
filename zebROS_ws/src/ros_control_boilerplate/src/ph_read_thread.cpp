#include <ros_control_boilerplate/frc_robot_interface.h>
#include "frc/PneumaticHub.h"

namespace ros_control_boilerplate
{
// The PH state reads happen in their own thread. This thread
// loops at 20Hz to match the update rate of PH CAN
// status messages.  Each iteration, data read from the
// PH is copied to a state buffer shared with the main read
// thread.
void FRCRobotInterface::ph_read_thread(std::shared_ptr<frc::PneumaticHub> ph_handle,
										std::shared_ptr<hardware_interface::PHHWState> state,
										std::shared_ptr<std::mutex> mutex,
										std::unique_ptr<Tracer> tracer,
										double poll_frequency)
{
	const auto ph_id = ph_handle->GetModuleNumber();
#ifdef __linux__
	std::stringstream s;
	s << "ph_read_" << ph_id;
	if (pthread_setname_np(pthread_self(), s.str().c_str()))
	{
		ROS_ERROR_STREAM("Error setting thread name " << s.str() << " " << errno);
	}
#endif
	ros::Duration(1.7 + ph_id /10.).sleep(); // Sleep for a few seconds to let CAN start up
	ROS_INFO_STREAM("Starting ph " << ph_id << " read thread at " << ros::Time::now());
	for (ros::Rate r(poll_frequency); ros::ok(); r.sleep())
	{
		tracer->start("main loop");

		hardware_interface::PHHWState ph_state(ph_id);
		ph_state.setCompressorEnabled(ph_handle->GetCompressor());
		ph_state.setPressureSwitch(ph_handle->GetPressureSwitch());
		ph_state.setCompressorCurrent(static_cast<double>(ph_handle->GetCompressorCurrent().value()));
		for (size_t i = 0; i < 2; i++)
		{
			ph_state.setAnalogVoltage(static_cast<double>(ph_handle->GetAnalogVoltage(i).value()), i);
			ph_state.setPressure(static_cast<double>(ph_handle->GetPressure(i).value()), i);
		}

		{
			// Copy to state shared with read() thread
			// Put this in a separate scope so lock_guard is released
			// as soon as the state is finished copying
			std::lock_guard<std::mutex> l(*mutex);
			*state = ph_state;
		}

		tracer->report(60);
	}
}

} // namespace
