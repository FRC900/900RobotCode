#include <thread>

#include "ros/node_handle.h"

#include "REVPDH.h"
#include "hal/HALBase.h"

#include "frc_interfaces/pdh_command_interface.h"
#include "ros_control_boilerplate/pdh_device.h"
#include "ros_control_boilerplate/tracer.h"

PDHDevice::PDHDevice(const std::string &name_space,
                     const int joint_index,
                     const std::string &joint_name,
                     const int pdh_module,
                     const bool local_read,
                     const bool local_write,
                     const double read_hz_)
    : name_{joint_name}
    , pdh_module_{pdh_module}
    , local_read_{local_read}
    , local_write_{local_write}
    , state_{std::make_unique<hardware_interface::PDHHWState>(pdh_module)}
    , command_{std::make_unique<hardware_interface::PDHHWCommand>()}
    , read_thread_state_{local_read ? std::make_unique<hardware_interface::PDHHWState>(pdh_module) : nullptr}
    , read_state_mutex_{local_read ? std::make_unique<std::mutex>() : nullptr}
{
    if (local_read_ || local_write)
    {
        if (!HAL_CheckREVPDHModuleNumber(pdh_module))
        {
            ROS_ERROR_STREAM("Invalid PDH module number " << pdh_module);
            return;
        }
        else
        {
            int32_t status = 0;
            pdh_handle_ = HAL_InitializeREVPDH(pdh_module, __FUNCTION__, &status);
            if ((pdh_handle_ == HAL_kInvalidHandle) || status)
            {
                ROS_ERROR_STREAM("Could not initialize PDH module, status = " << status);
                return;
            }
            else
            {
                if (local_read_)
                {
                    read_thread_ = std::make_unique<std::thread>(&PDHDevice::read_thread, this, std::make_unique<Tracer>("pdh_read_" + joint_name + " " + name_space), read_hz_);
                }
                //HAL_Report(HALUsageReporting::kResourceType_PDH, pdh_modules_[i]);
            }
        }
    }
    ROS_INFO_STREAM("Loading joint " << joint_index << "=" << name_ <<
                    (local_read_ ? " local" : " remote") << " read, " <<
                    (local_write_ ? "local" : "remote") << " write" <<
                    " as PDH " << pdh_module_);
}

PDHDevice::~PDHDevice()
{
    if (read_thread_ && read_thread_->joinable())
    {
        read_thread_->join();
    }
}

void PDHDevice::registerInterfaces(hardware_interface::PDHStateInterface &state_interface,
                                  hardware_interface::PDHCommandInterface &command_interface,
                                  hardware_interface::RemotePDHStateInterface &remote_state_interface)
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for PDH : " << name_ << " as pdh module " << pdh_module_);

    hardware_interface::PDHStateHandle state_handle(name_, state_.get());
    state_interface.registerHandle(state_handle);

    hardware_interface::PDHCommandHandle command_handle(state_handle, &(*command_));
    command_interface.registerHandle(command_handle);

    if (!local_read_)
    {
        hardware_interface::PDHWritableStateHandle remote_handle(name_, state_.get());
        remote_state_interface.registerHandle(remote_handle);
    }
}

void PDHDevice::read(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (local_read_)
    {
        std::unique_lock<std::mutex> l(*read_state_mutex_, std::try_to_lock);
        if (l.owns_lock())
        {
            *state_ = *read_thread_state_;
        }
    }
}

void PDHDevice::write(const ros::Time &/*time*/, const ros::Duration &/*period*/)
{
    if (bool enable; command_->switchableChannelEnableChanged(enable))
    {
        if (local_write_)
        {
            int32_t status = 0;
            HAL_SetREVPDHSwitchableChannel(pdh_handle_, enable, &status);
            if (status == 0)
            {
                ROS_INFO_STREAM("Set PDH " << name_ << " enable = " << static_cast<int>(enable));
            }
            else
            {
                ROS_ERROR_STREAM("Error setting PDH " << name_
                                << " enable = " << static_cast<int>(enable)
                                << " : status = " << status
                                << " : " << HAL_GetErrorMessage(status));
                command_->resetSwitchableChannelEnable();
            }
        }
    }

    if (command_->clearStickyFaultsChanged())
    {
        if (local_write_)
        {
            int32_t status = 0;
            HAL_ClearREVPDHStickyFaults(pdh_module_, &status);
            if (status == 0)
            {
                ROS_INFO_STREAM("Cleared sticky faults on PDH " << name_);
            }
            else
            {
                ROS_ERROR_STREAM("Error clearing sticky faults on " << name_
                                << " : status = " << status
                                << " : " << HAL_GetErrorMessage(status));
                command_->setClearStickyFaults();
            }
        }
    }
}

void PDHDevice::read_thread(std::unique_ptr<Tracer> tracer,
                            const double poll_frequency)
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
    HAL_ClearREVPDHStickyFaults(pdh_handle_, &status);
    if (status)
    {
        ROS_ERROR_STREAM("pdh_read_thread error clearing faults : status = " << status);
    }
    status = 0;
    hardware_interface::PDHHWState pdh_state(HAL_GetREVPDHModuleNumber(pdh_module_, &status));
    if (status)
    {
        ROS_ERROR_STREAM("pdh_read_thread error reading PDH module number: status = " << status);
    }
    status = 0;
    HAL_PowerDistributionVersion pdh_version;
    HAL_GetREVPDHVersion(pdh_handle_, &pdh_version, &status);
    if (status)
    {
        ROS_ERROR_STREAM("pdh_read_thread error reading PDH version : status = " << status);
    }
    pdh_state.setFirmwareMajor(pdh_version.firmwareMajor);
    pdh_state.setFirmwareMinor(pdh_version.firmwareMinor);
    pdh_state.setFirmwareFix(pdh_version.firmwareFix);
    pdh_state.setHardwareMajor(pdh_version.hardwareMajor);
    pdh_state.setHardwareMinor(pdh_version.hardwareMinor);
    pdh_state.setUniqueID(pdh_version.uniqueId);
    for (ros::Rate r(poll_frequency); ros::ok(); r.sleep())
    {
        tracer->start("main loop");

        // read info from the PDH hardware
        status = 0;
        pdh_state.setVoltage(HAL_GetREVPDHVoltage(pdh_handle_, &status));
        if (status)
        {
            ROS_ERROR_STREAM("voltage pdh_read_thread error : status = " << status << ":" << HAL_GetErrorMessage(status));
        }
        HAL_PowerDistributionFaults faults;
        status = 0;
        HAL_GetREVPDHFaults(pdh_handle_, &faults, &status);
        if (status)
        {
            ROS_ERROR_STREAM("faults pdh_read_thread error : status = " << status << ":" << HAL_GetErrorMessage(status));
        }
        HAL_PowerDistributionStickyFaults sticky_faults;
        status = 0;
        HAL_GetREVPDHStickyFaults(pdh_handle_, &sticky_faults, &status);
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
        status = 0;
        pdh_state.setTotalCurrent(HAL_GetREVPDHTotalCurrent(pdh_handle_, &status));
        if (status)
        {
            ROS_ERROR_STREAM("line " << __LINE__ << " pdh_read_thread error : status = " << status << ":" << HAL_GetErrorMessage(status));
        }
        status = 0;
        pdh_state.setSwitchableChannelState(HAL_GetREVPDHSwitchableChannelState(pdh_handle_, &status));
        if (status)
        {
            ROS_ERROR_STREAM("line " << __LINE__ << " pdh_read_thread error : status = " << status << ":" << HAL_GetErrorMessage(status));
        }

        for (size_t channel = 0; channel < 20; channel++)
        {
            status = 0;
            pdh_state.setChannelCurrent(HAL_GetREVPDHChannelCurrent(pdh_handle_, channel, &status), channel);
            if (status)
            {
                ROS_ERROR_STREAM("line " << __LINE__ << " pdh_read_thread error : status = " << status << ":" << HAL_GetErrorMessage(status));
            }
        }

        {
            // Copy to state shared with read() thread
            // Put this in a separate scope so lock_guard is released
            // as soon as the state is finished copying
            std::lock_guard<std::mutex> l(*read_state_mutex_);
            *read_thread_state_ = pdh_state;
        }
        tracer->report(60);
    }
}
