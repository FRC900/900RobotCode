#include <ros_control_boilerplate/frc_robot_interface.h>
#include "frc/PneumaticsControlModule.h"

namespace ros_control_boilerplate
{
// The PCM state reads happen in their own thread. This thread
// loops at 20Hz to match the update rate of PCM CAN
// status messages.  Each iteration, data read from the
// PCM is copied to a state buffer shared with the main read
// thread.
void FRCRobotInterface::pcm_read_thread(std::shared_ptr<frc::PneumaticsControlModule> pcm_handle,
										std::shared_ptr<hardware_interface::PCMState> state,
										std::shared_ptr<std::mutex> mutex,
										std::unique_ptr<Tracer> tracer,
										double poll_frequency)
{
	const auto pcm_id = pcm_handle->GetModuleNumber();
#ifdef __linux__
	std::stringstream s;
	s << "pcm_read_" << pcm_handle->GetModuleNumber();
	if (pthread_setname_np(pthread_self(), s.str().c_str()))
	{
		ROS_ERROR_STREAM("Error setting thread name " << s.str() << " " << errno);
	}
#endif
	ros::Duration(2.1 + pcm_id / 10.).sleep(); // Sleep for a few seconds to let CAN start up
	ROS_INFO_STREAM("Starting pcm " << pcm_id << " read thread at " << ros::Time::now());
	//pcm_handle->ClearAllStickyFaults(); TODO broken in wpilib
	for (ros::Rate r(poll_frequency); ros::ok(); r.sleep())
	{
		tracer->start("main loop");

		hardware_interface::PCMState pcm_state(pcm_id);
		pcm_state.setCompressorEnabled(pcm_handle->GetCompressor());
		pcm_state.setPressureSwitch(pcm_handle->GetPressureSwitch());
		pcm_state.setCompressorCurrent(static_cast<double>(pcm_handle->GetCompressorCurrent().value()));
		pcm_state.setClosedLoopControl(pcm_handle->GetCompressorConfigType() != frc::CompressorConfigType::Disabled);
		pcm_state.setCurrentTooHigh(pcm_handle->GetCompressorCurrentTooHighFault());
		pcm_state.setCurrentTooHighSticky(pcm_handle->GetCompressorCurrentTooHighStickyFault());

		pcm_state.setShorted(pcm_handle->GetCompressorShortedFault());
		pcm_state.setShortedSticky(pcm_handle->GetCompressorShortedStickyFault());
		pcm_state.setNotConntected(pcm_handle->GetCompressorNotConnectedFault());
		pcm_state.setNotConnecteSticky(pcm_handle->GetCompressorNotConnectedStickyFault());
		pcm_state.setVoltageFault(pcm_handle->GetSolenoidVoltageFault());
		pcm_state.setVoltageSticky(pcm_handle->GetSolenoidVoltageStickyFault());
		pcm_state.setSolenoidDisabledList(pcm_handle->GetSolenoidDisabledList());

		{
			// Copy to state shared with read() thread
			// Put this in a separate scope so lock_guard is released
			// as soon as the state is finished copying
			std::lock_guard<std::mutex> l(*mutex);
			*state = pcm_state;
		}
		tracer->report(60);
	}
}

} // namespace
