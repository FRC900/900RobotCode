#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "hal/DriverStationTypes.h"

#include "frc_interfaces/match_data_interface.h"
#include "ros_control_boilerplate/match_data_devices.h"
#include "ros_control_boilerplate/match_data_device.h"

template <class DEVICE_TYPE>
MatchDataDevices<DEVICE_TYPE>::MatchDataDevices(ros::NodeHandle &root_nh)
    : state_interface_{std::make_unique<hardware_interface::MatchStateInterface>()}
    , remote_state_interface_{std::make_unique<hardware_interface::RemoteMatchStateInterface>()}
{
    devices_.emplace_back(std::make_unique<DEVICE_TYPE>(root_nh));
}

template <class DEVICE_TYPE>
MatchDataDevices<DEVICE_TYPE>::~MatchDataDevices() = default;

template <class DEVICE_TYPE>
hardware_interface::InterfaceManager *MatchDataDevices<DEVICE_TYPE>::registerInterface()
{
    for (auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_, *remote_state_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    interface_manager_.registerInterface(remote_state_interface_.get());
    return &interface_manager_;
}

template <class DEVICE_TYPE>
void MatchDataDevices<DEVICE_TYPE>::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
{
    tracer.start_unique("match data");
    if (isReady())
    {
        for (auto &d : devices_)
        {
            d->read(time, period);
        }
    }
}

template <class DEVICE_TYPE>
void MatchDataDevices<DEVICE_TYPE>::simInit(ros::NodeHandle &nh)
{
    devices_[0]->simInit(nh);
}

template <class DEVICE_TYPE>
std::optional<bool> MatchDataDevices<DEVICE_TYPE>::isEnabled(void) const
{
    return devices_[0]->isEnabled();
}

template <class DEVICE_TYPE>
bool MatchDataDevices<DEVICE_TYPE>::getControlWord(HAL_ControlWord &cw) const
{
    return devices_[0]->getControlWord(cw);
    return true;
}