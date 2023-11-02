#include <thread>

#include <ros/node_handle.h>

#include "CTREPDP.h"
#include "hal/HALBase.h"                              // for HAL_GetErrorMes...
#include "hal/FRCUsageReporting.h"

#include "frc_interfaces/pdp_state_interface.h"
#include "ros_control_boilerplate/pdp_device.h"
#include "ros_control_boilerplate/tracer.h"

PDPDevice::PDPDevice(const std::string &name_space,
                     const int joint_index,
                     const std::string &joint_name,
                     const int pdp_module,
                     const bool local,
                     const double read_hz_)
    : name_{joint_name}
    , pdp_module_{pdp_module}
    , local_{local}
    , state_{std::make_unique<hardware_interface::PDPHWState>()}
    , read_thread_state_{local ? std::make_unique<hardware_interface::PDPHWState>() : nullptr}
    , read_state_mutex_{local ? std::make_unique<std::mutex>() : nullptr}
{
    if (local_)
    {
        int32_t status = 0;
        const auto pdp_handle = HAL_InitializePDP(pdp_module_, __FUNCTION__, &status);

        read_thread_ = std::make_unique<std::jthread>(&PDPDevice::read_thread,
                                                      this,
                                                      pdp_handle,
                                                      std::make_unique<Tracer>("pdp_read_" + joint_name + " " + name_space),
                                                      read_hz_);
        HAL_Report(HALUsageReporting::kResourceType_PDP, pdp_module_);
    }

    ROS_INFO_STREAM("Loading joint " << joint_index << "=" << name_ <<
                    " local=" << local_ <<
                    " as PDP module " << pdp_module_);
}

PDPDevice::~PDPDevice(void) = default;

void PDPDevice::registerInterfaces(hardware_interface::PDPStateInterface &state_interface,
                                   hardware_interface::RemotePDPStateInterface &remote_state_interface) const
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for PDP : " << name_ << " as pdp module " << pdp_module_);

    hardware_interface::PDPStateHandle pdp_state_handle(name_, state_.get());
    state_interface.registerHandle(pdp_state_handle);
    
    if (!local_)
    {
        hardware_interface::PDPWritableStateHandle remote_handle(name_, state_.get());
        remote_state_interface.registerHandle(remote_handle);
    }
}

void PDPDevice::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (local_)
    {
        std::unique_lock l(*read_state_mutex_, std::try_to_lock);
        if (l.owns_lock())
        {
            *state_ = *read_thread_state_;
        }
    }
}

void PDPDevice::read_thread(int32_t pdp,
                            std::unique_ptr<Tracer> tracer,
                            double poll_frequency)
{
#ifdef __linux__
	if (pthread_setname_np(pthread_self(), "pdp_read"))
	{
		ROS_ERROR_STREAM("Error setting thread name pdp_read " << errno);
	}
#endif
	ros::Duration(1.9).sleep(); // Sleep for a few seconds to let CAN start up
	ROS_INFO_STREAM("Starting pdp read thread at " << ros::Time::now());
	int32_t status = 0;
	HAL_ClearPDPStickyFaults(pdp, &status);
	if (status)
	{
		ROS_ERROR_STREAM("pdp_read_thread error clearing sticky faults : status = " << status);
	}
    status = 0;
	HAL_ResetPDPTotalEnergy(pdp, &status);
	if (status)
	{
		ROS_ERROR_STREAM("pdp_read_thread error reset total energy : status = " << status);
	}
	hardware_interface::PDPHWState pdp_state;
	for (ros::Rate r(poll_frequency); ros::ok(); r.sleep())
	{
		tracer->start("main loop");

		//read info from the PDP hardware into local read thread state buffer
		status = 0;
		pdp_state.setVoltage(HAL_GetPDPVoltage(pdp, &status));
		if (status)
		{
			ROS_ERROR_STREAM("pdp_read_thread error : GetPDPVoltage status = " << status << " : " << HAL_GetErrorMessage(status));
		}

        status = 0;
		pdp_state.setTemperature(HAL_GetPDPTemperature(pdp, &status));
		if (status)
		{
			ROS_ERROR_STREAM("pdp_read_thread error : GetPDPTemperature status = " << status << " : " << HAL_GetErrorMessage(status));
		}

        status = 0;
		pdp_state.setTotalCurrent(HAL_GetPDPTotalCurrent(pdp, &status));
		if (status)
		{
			ROS_ERROR_STREAM("pdp_read_thread error : GetPDPTotalCurrent status = " << status << " : " << HAL_GetErrorMessage(status));
		}

        status = 0;
		pdp_state.setTotalPower(HAL_GetPDPTotalPower(pdp, &status));
		if (status)
		{
			ROS_ERROR_STREAM("pdp_read_thread error : GetPDPTotalPower status = " << status << " : " << HAL_GetErrorMessage(status));
		}

        status = 0;
		pdp_state.setTotalEnergy(HAL_GetPDPTotalEnergy(pdp, &status));
		if (status)
		{
			ROS_ERROR_STREAM("pdp_read_thread error : GetPDPTotalEnergy status = " << status << " : " << HAL_GetErrorMessage(status));
		}

		for (int channel = 0; channel <= 15; channel++)
		{
            status = 0;
            pdp_state.setCurrent(HAL_GetPDPChannelCurrent(pdp, channel, &status), channel);
            if (status)
            {
                ROS_ERROR_STREAM("pdp_read_thread error : GetPDPChannelCurrent(" << channel << ") status = " << status << " : " << HAL_GetErrorMessage(status));
            }
        }

		{
			// Copy to state shared with read() thread
			// Put this in a separate scope so lock_guard is released
			// as soon as the state is finished copying
			std::lock_guard l(*read_state_mutex_);
			*read_thread_state_ = pdp_state;
		}
		tracer->report(60);
	}
}
