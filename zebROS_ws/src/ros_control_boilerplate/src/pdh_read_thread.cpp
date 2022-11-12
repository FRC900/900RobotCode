#include <ros_control_boilerplate/frc_robot_interface.h>
#include "REVPDH.h"

namespace ros_control_boilerplate
{
// The PDH reads happen in their own thread. This thread
// loops at 20Hz to match the update rate of PDH CAN
// status messages.  Each iteration, data read from the
// PDP is copied to a state buffer shared with the main read
// thread.
void FRCRobotInterface::pdh_read_thread(int32_t pdh,
		std::shared_ptr<hardware_interface::PDHHWState> state,
		std::shared_ptr<std::mutex> mutex,
		std::unique_ptr<Tracer> tracer,
		double poll_frequency)
{
#ifdef __linux__
	if (pthread_setname_np(pthread_self(), "pdh_read"))
	{
		ROS_ERROR_STREAM("Error setting thread name pdh_read " << errno);
	}
#endif
	ros::Duration(1.9).sleep(); // Sleep for a few seconds to let CAN start up
	ROS_INFO_STREAM("Starting pdh read thread at " << ros::Time::now());
	int32_t status = 0;
	HAL_ClearREVPDHStickyFaults(pdh, &status);
	if (status)
		ROS_ERROR_STREAM("pdh_read_thread error clearing faults : status = " << status);
	status = 0;
	hardware_interface::PDHHWState pdh_state(HAL_GetREVPDHModuleNumber(pdh, &status));
	if (status)
		ROS_ERROR_STREAM("pdh_read_thread error reading PDH module number: status = " << status);
	status = 0;
	HAL_PowerDistributionVersion pdh_version;
	HAL_GetREVPDHVersion(pdh, &pdh_version, &status);
	if (status)
		ROS_ERROR_STREAM("pdh_read_thread error reading PDH version : status = " << status);
	pdh_state.setFirmwareMajor(pdh_version.firmwareMajor);
	pdh_state.setFirmwareMinor(pdh_version.firmwareMinor);
	pdh_state.setFirmwareFix(pdh_version.firmwareFix);
	pdh_state.setHardwareMajor(pdh_version.hardwareMajor);
	pdh_state.setHardwareMinor(pdh_version.hardwareMinor);
	pdh_state.setUniqueID(pdh_version.uniqueId);
	for (ros::Rate r(poll_frequency); ros::ok(); r.sleep())
	{
		tracer->start("main loop");

		//read info from the PDH hardware
		status = 0;
		pdh_state.setVoltage(HAL_GetREVPDHVoltage(pdh, &status));
		if (status)
		{
			ROS_ERROR_STREAM("voltage pdh_read_thread error : status = " << status << ":" << HAL_GetErrorMessage(status));
		}
		HAL_PowerDistributionFaults faults;
		HAL_GetREVPDHFaults(pdh, &faults, &status);
		if (status)
		{
			ROS_ERROR_STREAM("faults pdh_read_thread error : status = " << status << ":" << HAL_GetErrorMessage(status));
		}
		HAL_PowerDistributionStickyFaults sticky_faults;
		HAL_GetREVPDHStickyFaults(pdh, &sticky_faults, &status);
		if (status)
		{
			ROS_ERROR_STREAM("sticky pdh_read_thread error : status = " << status << ":" << HAL_GetErrorMessage(status));
		}


#if 0
		pdh_state.setEnabled(HAL_IsREVPDHEnabled(pdh, &status));
#endif
		pdh_state.setBrownout(faults.brownout);
		pdh_state.setStickyBrownout(sticky_faults.brownout);
		pdh_state.setCANWarning(faults.canWarning);
		pdh_state.setStickyCANWarning(sticky_faults.canWarning);
		pdh_state.setStickyCANBusOff(sticky_faults.canBusOff);
		pdh_state.setHardwareFault(faults.hardwareFault);
		pdh_state.setChannelBreakerFault(faults.channel0BreakerFault, 0);
		pdh_state.setChannelBreakerFault(faults.channel1BreakerFault, 1);
		pdh_state.setChannelBreakerFault(faults.channel2BreakerFault, 2);
		pdh_state.setChannelBreakerFault(faults.channel3BreakerFault, 3);
		pdh_state.setChannelBreakerFault(faults.channel4BreakerFault, 4);
		pdh_state.setChannelBreakerFault(faults.channel5BreakerFault, 5);
		pdh_state.setChannelBreakerFault(faults.channel6BreakerFault, 6);
		pdh_state.setChannelBreakerFault(faults.channel7BreakerFault, 7);
		pdh_state.setChannelBreakerFault(faults.channel8BreakerFault, 8);
		pdh_state.setChannelBreakerFault(faults.channel9BreakerFault, 9);
		pdh_state.setChannelBreakerFault(faults.channel10BreakerFault, 10);
		pdh_state.setChannelBreakerFault(faults.channel11BreakerFault, 11);
		pdh_state.setChannelBreakerFault(faults.channel12BreakerFault, 12);
		pdh_state.setChannelBreakerFault(faults.channel13BreakerFault, 13);
		pdh_state.setChannelBreakerFault(faults.channel14BreakerFault, 14);
		pdh_state.setChannelBreakerFault(faults.channel15BreakerFault, 15);
		pdh_state.setChannelBreakerFault(faults.channel16BreakerFault, 16);
		pdh_state.setChannelBreakerFault(faults.channel17BreakerFault, 17);
		pdh_state.setChannelBreakerFault(faults.channel18BreakerFault, 18);
		pdh_state.setChannelBreakerFault(faults.channel19BreakerFault, 19);
		pdh_state.setChannelBreakerFault(faults.channel20BreakerFault, 20);
		pdh_state.setChannelBreakerFault(faults.channel21BreakerFault, 21);
		pdh_state.setChannelBreakerFault(faults.channel22BreakerFault, 22);
		pdh_state.setChannelBreakerFault(faults.channel23BreakerFault, 23);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel0BreakerFault, 0);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel1BreakerFault, 1);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel2BreakerFault, 2);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel3BreakerFault, 3);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel4BreakerFault, 4);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel5BreakerFault, 5);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel6BreakerFault, 6);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel7BreakerFault, 7);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel8BreakerFault, 8);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel9BreakerFault, 9);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel10BreakerFault, 10);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel11BreakerFault, 11);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel12BreakerFault, 12);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel13BreakerFault, 13);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel14BreakerFault, 14);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel15BreakerFault, 15);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel16BreakerFault, 16);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel17BreakerFault, 17);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel18BreakerFault, 18);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel19BreakerFault, 19);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel20BreakerFault, 20);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel21BreakerFault, 21);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel22BreakerFault, 22);
		pdh_state.setStickyChannelBreakerFault(sticky_faults.channel23BreakerFault, 23);
		pdh_state.setStickyHasReset(sticky_faults.hasReset);
		pdh_state.setTotalCurrent(HAL_GetREVPDHTotalCurrent(pdh, &status));
		if (status)
		{
			ROS_ERROR_STREAM("line 130 pdh_read_thread error : status = " << status << ":" << HAL_GetErrorMessage(status));
		}
		pdh_state.setSwitchableChannelState(HAL_GetREVPDHSwitchableChannelState(pdh, &status));

		for (size_t channel = 0; channel < 20; channel++)
		{
			pdh_state.setChannelCurrent(HAL_GetREVPDHChannelCurrent(pdh, channel, &status), channel);
			if (status)
		{
			ROS_ERROR_STREAM("line 130 pdh_read_thread error : status = " << status << ":" << HAL_GetErrorMessage(status));
		}
		}
		if (status)
		{
			ROS_ERROR_STREAM("pdh_read_thread error : status = " << status << ":" << HAL_GetErrorMessage(status));
		}

		{
			// Copy to state shared with read() thread
			// Put this in a separate scope so lock_guard is released
			// as soon as the state is finished copying
			std::lock_guard<std::mutex> l(*mutex);
			*state = pdh_state;
		}
		tracer->report(60);
	}
}

} // namespace
