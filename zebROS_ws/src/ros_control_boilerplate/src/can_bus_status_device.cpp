#include "ctre/phoenix6/CANBus.hpp"

#include "frc_interfaces/can_bus_status_interface.h"
#include "periodic_interval_counter/periodic_interval_counter.h"
#include "ros_control_boilerplate/can_bus_status_device.h"

CANBusStatusDevice::CANBusStatusDevice(const std::string &name, const double read_hz)
    : name_{name}
    , state_{std::make_unique <hardware_interface::can_bus_status::CANBusStatusHWState>(name_)}
    , interval_counter_{std::make_unique<PeriodicIntervalCounter>(read_hz)}
    
    , can_bus_{std::make_unique<ctre::phoenix6::CANBus>(name_)}
{
    ROS_INFO_STREAM("Loading CAN Bus Status Device " << name << " running at " << read_hz << "Hz");
}

CANBusStatusDevice::~CANBusStatusDevice() = default;

void CANBusStatusDevice::registerInterfaces(hardware_interface::can_bus_status::CANBusStatusStateInterface &state_interface) const
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for Robot Controller");
    hardware_interface::can_bus_status::CANBusStatusStateHandle state_handle(state_->getName(), state_.get());
    state_interface.registerHandle(state_handle);
}

void CANBusStatusDevice::read(const ros::Time &/*time*/, const ros::Duration &period)
{
    // check if sufficient time has passed since last read
    if (interval_counter_->update(period))
    {
        const auto can_bus_status = can_bus_->GetStatus();

        if (can_bus_status.Status == ctre::phoenix::StatusCode::OK)
        {
            state_->setBusUtilization(can_bus_status.BusUtilization);
            state_->setBusOffCount(can_bus_status.BusOffCount);
            state_->setTXFullCount(can_bus_status.TxFullCount);
            state_->setREC(can_bus_status.REC);
            state_->setTEC(can_bus_status.TEC);
        }
        else
        {
            ROS_ERROR_STREAM("Failed to read CAN bus status for " << state_->getName() << " : " << can_bus_status.Status.GetName() << " (" << can_bus_status.Status << ")");
        }
    }
}
