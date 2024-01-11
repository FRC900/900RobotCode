#include <hardware_interface/robot_hw.h> // for hardware_interface::InterfaceManager
#include "hal/DriverStationTypes.h"

#include "frc_interfaces/match_data_interface.h"
#include "ros_control_boilerplate/match_data_devices.h"
#include "ros_control_boilerplate/match_data_device.h"

template <bool SIM>
MatchDataDevices<SIM>::MatchDataDevices(ros::NodeHandle &root_nh)
    : state_interface_{std::make_unique<hardware_interface::match_data::MatchStateInterface>()}
    , remote_state_interface_{std::make_unique<hardware_interface::match_data::RemoteMatchStateInterface>()}
{
    devices_.emplace_back(std::make_unique<DEVICE_TYPE>(root_nh));
}

template <bool SIM>
MatchDataDevices<SIM>::~MatchDataDevices() = default;

template <bool SIM>
hardware_interface::InterfaceManager *MatchDataDevices<SIM>::registerInterface()
{
    for (auto &d : devices_)
    {
        d->registerInterfaces(*state_interface_, *remote_state_interface_);
    }
    interface_manager_.registerInterface(state_interface_.get());
    interface_manager_.registerInterface(remote_state_interface_.get());
    return &interface_manager_;
}

template <bool SIM>
void MatchDataDevices<SIM>::read(const ros::Time& time, const ros::Duration& period, Tracer &tracer)
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

template <bool SIM>
void MatchDataDevices<SIM>::simInit(ros::NodeHandle &nh)
{
    devices_[0]->simInit(nh);
}

template <bool SIM>
std::optional<bool> MatchDataDevices<SIM>::isEnabled(void) const
{
    return devices_[0]->isEnabled();
}

template <bool SIM>
bool MatchDataDevices<SIM>::getControlWord(HAL_ControlWord &cw) const
{
    return devices_[0]->getControlWord(cw);
    return true;
}