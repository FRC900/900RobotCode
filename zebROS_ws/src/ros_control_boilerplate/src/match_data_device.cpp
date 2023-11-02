#include "frc/DriverStation.h"                        // for DriverStation
#include "hal/DriverStation.h"                        // for HAL_GetAlliance...
#include "hal/DriverStationTypes.h"                   // for HAL_ControlWord
#include "hal/HALBase.h"                              // for HAL_GetErrorMes...
#include "hal/Power.h"                                // for HAL_GetVinVoltage
#include "hal/simulation/DriverStationData.h"

#include "frc_interfaces/match_data_interface.h"
#include "periodic_interval_counter/periodic_interval_counter.h"
#include "ros_control_boilerplate/match_data_device.h"

MatchDataDevice::MatchDataDevice(const ros::NodeHandle &nh)
    : state_{std::make_unique<hardware_interface::MatchHWState>()}
{
    ros::NodeHandle hwi_nh(nh, "hardware_interface");
    hwi_nh.param("run_hal_robot", local_, local_);
    if (local_)
    {
        ros::NodeHandle param_nh(nh, "generic_hw_control_loop");
        double read_hz = 2;
        if (!param_nh.param("match_data_read_hz", read_hz, read_hz))
        {
            ROS_WARN("Failed to read match_data_read_hz in frc_robot_interface");
        }

        interval_counter_ = std::make_unique<PeriodicIntervalCounter>(read_hz);

        ROS_INFO_STREAM("Loading Match Data running at " << read_hz << "Hz");
    }
    else
    {
        ROS_INFO_STREAM("Loading remote Match Data");
    }

}

MatchDataDevice::~MatchDataDevice() = default;

void MatchDataDevice::registerInterfaces(hardware_interface::MatchStateInterface &state_interface,
                                         hardware_interface::RemoteMatchStateInterface &remote_state_interface) const
{
    ROS_INFO_STREAM("FRCRobotInterface: Registering interface for Match Data");
    hardware_interface::MatchStateHandle state_handle("match_data", state_.get());
    state_interface.registerHandle(state_handle);

    if (!local_)
    {
        hardware_interface::MatchStateWritableHandle remote_state_handle("match_data", state_.get());
        remote_state_interface.registerHandle(remote_state_handle);
    }
}

void MatchDataDevice::read(const ros::Time &/*time*/, const ros::Duration &period)
{
    if (!local_)
    {
        return;
    }
    // check if sufficient time has passed since last read
    if (interval_counter_->update(period))
    {
        int32_t status = 0;
        state_->setMatchTimeRemaining(HAL_GetMatchTime(&status));
        state_->setGetMatchTimeStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));
        HAL_MatchInfo info{};
        HAL_GetMatchInfo(&info);

        state_->setGameSpecificData(std::vector<uint8_t>(info.gameSpecificMessage, info.gameSpecificMessage + info.gameSpecificMessageSize));
        state_->setEventName(info.eventName);

        status = 0;
        auto allianceStationID = HAL_GetAllianceStation(&status);
        state_->setGetAllianceStationStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));
        frc::DriverStation::Alliance color;
        switch (allianceStationID)
        {
        case HAL_AllianceStationID_kRed1:
        case HAL_AllianceStationID_kRed2:
        case HAL_AllianceStationID_kRed3:
            color = frc::DriverStation::kRed;
            break;
        case HAL_AllianceStationID_kBlue1:
        case HAL_AllianceStationID_kBlue2:
        case HAL_AllianceStationID_kBlue3:
            color = frc::DriverStation::kBlue;
            break;
        default:
            color = frc::DriverStation::kInvalid;
        }
        state_->setAllianceColor(color);

        state_->setMatchType(static_cast<frc::DriverStation::MatchType>(info.matchType));

        int station_location;
        switch (allianceStationID)
        {
        case HAL_AllianceStationID_kRed1:
        case HAL_AllianceStationID_kBlue1:
            station_location = 1;
            break;
        case HAL_AllianceStationID_kRed2:
        case HAL_AllianceStationID_kBlue2:
            station_location = 2;
            break;
        case HAL_AllianceStationID_kRed3:
        case HAL_AllianceStationID_kBlue3:
            station_location = 3;
            break;
        default:
            station_location = 0;
        }
        state_->setDriverStationLocation(station_location);

        state_->setMatchNumber(info.matchNumber);
        state_->setReplayNumber(info.replayNumber);
        status = 0;
        state_->setBatteryVoltage(HAL_GetVinVoltage(&status));
        state_->setGetVinVoltageStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));
    }
    // read control word match data at full speed - contains enable info, and reads should be v fast
    HAL_ControlWord controlWord;
    HAL_GetControlWord(&controlWord);
    state_->setEnabled(controlWord.enabled && controlWord.dsAttached);
    state_->setDisabled(!(controlWord.enabled && controlWord.dsAttached));
    state_->setAutonomous(controlWord.autonomous);
    state_->setOperatorControl(!(controlWord.autonomous || controlWord.test));
    state_->setTest(controlWord.test);
    state_->setDSAttached(controlWord.dsAttached);
    state_->setFMSAttached(controlWord.fmsAttached);
    state_->setEStopped(controlWord.eStop);
}

std::optional<bool> MatchDataDevice::isEnabled(void) const
{
    return state_->isEnabled();
}

bool MatchDataDevice::getControlWord(HAL_ControlWord &cw) const
{
        cw.enabled = state_->isEnabled();
        cw.autonomous = state_->isAutonomous();
        cw.test = state_->isTest();
        cw.eStop = state_->isEStopped();
        cw.fmsAttached = state_->isFMSAttached();
        cw.dsAttached = state_->isDSAttached();
        cw.control_reserved = 0;
        return true;
}