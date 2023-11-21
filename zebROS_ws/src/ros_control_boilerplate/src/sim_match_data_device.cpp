#include "frc/DriverStation.h"                        // for DriverStation
#include "hal/DriverStation.h"                        // for HAL_GetAlliance...
#include "hal/DriverStationTypes.h"                   // for HAL_ControlWord
#include "hal/HALBase.h"                              // for HAL_GetErrorMes...
#include "hal/Power.h"                                // for HAL_GetVinVoltage
#include "hal/simulation/DriverStationData.h"

#include "ros_control_boilerplate/sim_match_data_device.h"

SimMatchDataDevice::SimMatchDataDevice(const ros::NodeHandle &nh)
    : MatchDataDevice(nh)
    , mutex_{std::make_unique<std::mutex>()}
{
    ROS_INFO_STREAM("Loading sim Match Data");
}

SimMatchDataDevice::~SimMatchDataDevice() = default;

void SimMatchDataDevice::read(const ros::Time &time, const ros::Duration &period)
{
    std::unique_lock l(*mutex_, std::try_to_lock);
    if (l.owns_lock())
    {
        MatchDataDevice::read(time, period);
        return;
    }
}

void SimMatchDataDevice::simInit(ros::NodeHandle nh)
{
    if (getLocal())
    {
        sim_sub_ = nh.subscribe("/frcrobot_rio/match_data_in", 1, &SimMatchDataDevice::matchDataCallback, this);
    }
}

void SimMatchDataDevice::matchDataCallback(const frc_msgs::MatchSpecificData &match_data) {
    std::unique_lock l(*mutex_);
	HALSIM_SetDriverStationMatchTime(match_data.matchTimeRemaining);
	HAL_AllianceStationID alliance_station_id = HAL_AllianceStationID_kRed1;
	if (match_data.allianceColor == frc::DriverStation::kRed)
	{
		if (match_data.driverStationLocation == 1)
		{
			alliance_station_id = HAL_AllianceStationID_kRed1;
		}
		else if (match_data.driverStationLocation == 2)
		{
			alliance_station_id = HAL_AllianceStationID_kRed2;
		}
		else if (match_data.driverStationLocation == 3)
		{
			alliance_station_id = HAL_AllianceStationID_kRed3;
		}
	}
	else if (match_data.allianceColor == frc::DriverStation::kBlue)
	{
		if (match_data.driverStationLocation == 1)
		{
			alliance_station_id = HAL_AllianceStationID_kBlue1;
		}
		else if (match_data.driverStationLocation == 2)
		{
			alliance_station_id = HAL_AllianceStationID_kBlue2;
		}
		else if (match_data.driverStationLocation == 3)
		{
			alliance_station_id = HAL_AllianceStationID_kBlue3;
		}
	}
	HALSIM_SetDriverStationAllianceStationId(alliance_station_id);
	HALSIM_SetDriverStationEnabled(match_data.Enabled);
	HALSIM_SetDriverStationAutonomous(match_data.Autonomous);
	HALSIM_SetDriverStationDsAttached(match_data.DSAttached);
	HALSIM_SetDriverStationFmsAttached(match_data.FMSAttached);
	HALSIM_SetDriverStationTest(match_data.Test);
	HALSIM_SetDriverStationEStop(match_data.EStopped);
	// TODO - HALSIM_SetDriverStationBatteryVoltage(match_data.BatteryVoltage);

	HAL_MatchInfo hal_match_info;
	strncpy(hal_match_info.eventName, match_data.eventName.c_str(), sizeof(hal_match_info.eventName));
	hal_match_info.matchType = static_cast<HAL_MatchType>(match_data.matchType);
	hal_match_info.matchNumber = static_cast<unsigned char>(match_data.matchNumber);
	hal_match_info.replayNumber = match_data.replayNumber;
	for (size_t i = 0; i < std::min(match_data.gameSpecificData.size(), sizeof(hal_match_info.gameSpecificMessage)); i++)
	{
		hal_match_info.gameSpecificMessage[i] = match_data.gameSpecificData[i];
	}

	hal_match_info.gameSpecificMessageSize = match_data.gameSpecificData.size();
	HALSIM_SetMatchInfo(&hal_match_info);
	HALSIM_NotifyDriverStationNewData();
}

std::optional<bool> SimMatchDataDevice::isEnabled(void) const
{
    std::unique_lock l(*mutex_, std::try_to_lock);
    if (l.owns_lock())
    {
        return MatchDataDevice::isEnabled();
    }
    return std::nullopt;
}

bool SimMatchDataDevice::getControlWord(HAL_ControlWord &cw) const
{
    std::unique_lock l(*mutex_, std::try_to_lock);
    if (l.owns_lock())
    {
        return MatchDataDevice::getControlWord(cw);
    }
    return false;
}