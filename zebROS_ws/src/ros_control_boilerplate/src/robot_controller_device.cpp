#include "hal/CAN.h"                                  // for HAL_CAN_GetCANS...
#include "hal/HALBase.h"
#include "hal/Power.h"                                // for HAL_GetVinVoltage

#include "frc_interfaces/robot_controller_interface.h"
#include "periodic_interval_counter/periodic_interval_counter.h"
#include "ros_control_boilerplate/robot_controller_device.h"

RobotControllerDevice::RobotControllerDevice(ros::NodeHandle &nh)
    : state_{std::make_unique <hardware_interface::RobotControllerState>()}
{
    ros::NodeHandle param_nh(nh, "generic_hw_control_loop");
    double read_hz = 20;
    if(!param_nh.param("robot_controller_read_hz", read_hz, read_hz))
    {
		ROS_WARN("Failed to read robot_controller_read_hz in frc_robot_interface");
	}
	interval_counter_ = std::make_unique<PeriodicIntervalCounter>(read_hz);

    ROS_INFO_STREAM("Loading Robot Controller running at " << read_hz << "Hz");
}

RobotControllerDevice::~RobotControllerDevice() = default;

void RobotControllerDevice::registerInterfaces(hardware_interface::RobotControllerStateInterface &state_interface) const
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for Robot Controller");
    hardware_interface::RobotControllerStateHandle state_handle("robot_controller_name", state_.get());
    state_interface.registerHandle(state_handle);
}

void RobotControllerDevice::read(const ros::Time &/*time*/, const ros::Duration &period)
{
    // check if sufficient time has passed since last read
    if (interval_counter_->update(period))
    {
        int status = 0;
        state_->SetFPGAVersion(HAL_GetFPGAVersion(&status));
        state_->SetFPGAVersionStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetFPGARevision(HAL_GetFPGARevision(&status));
        state_->SetFPGARevisionStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetFPGATime(HAL_GetFPGATime(&status));
        state_->SetFPGATimeStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetUserButton(HAL_GetFPGAButton(&status));
        state_->SetUserButtonStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetIsSysActive(HAL_GetSystemActive(&status));
        state_->SetIsSysActiveStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetIsBrownedOut(HAL_GetBrownedOut(&status));
        state_->SetIsBrownedOutStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetInputVoltage(HAL_GetVinVoltage(&status));
        state_->SetInputVoltageStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetInputCurrent(HAL_GetVinCurrent(&status));
        state_->SetInputCurrentStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetVoltage3V3(HAL_GetUserVoltage3V3(&status));
        state_->SetVoltage3V3Status(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetCurrent3V3(HAL_GetUserCurrent3V3(&status));
        state_->SetCurrent3V3Status(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetEnabled3V3(HAL_GetUserActive3V3(&status));
        state_->SetEnabled3V3Status(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetFaultCount3V3(HAL_GetUserCurrentFaults3V3(&status));
        state_->SetFaultCount3V3Status(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetVoltage5V(HAL_GetUserVoltage5V(&status));
        state_->SetVoltage5VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetCurrent5V(HAL_GetUserCurrent5V(&status));
        state_->SetCurrent5VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetEnabled5V(HAL_GetUserActive5V(&status));
        state_->SetEnabled5VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetFaultCount5V(HAL_GetUserCurrentFaults5V(&status));
        state_->SetFaultCount5VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetVoltage6V(HAL_GetUserVoltage6V(&status));
        state_->SetVoltage6VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetCurrent6V(HAL_GetUserCurrent6V(&status));
        state_->SetCurrent6VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetEnabled6V(HAL_GetUserActive6V(&status));
        state_->SetEnabled6VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        status = 0;
        state_->SetFaultCount6V(HAL_GetUserCurrentFaults6V(&status));
        state_->SetFaultCount6VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

        float percent_bus_utilization;
        uint32_t bus_off_count;
        uint32_t tx_full_count;
        uint32_t receive_error_count;
        uint32_t transmit_error_count;
        status = 0;
        HAL_CAN_GetCANStatus(&percent_bus_utilization, &bus_off_count,
                             &tx_full_count, &receive_error_count,
                             &transmit_error_count, &status);

        state_->SetCANPercentBusUtilization(percent_bus_utilization);
        state_->SetCANBusOffCount(bus_off_count);
        state_->SetCANTxFullCount(tx_full_count);
        state_->SetCANReceiveErrorCount(receive_error_count);
        state_->SetCANTransmitErrorCount(transmit_error_count);

        state_->SetCANDataStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));
    }
}
