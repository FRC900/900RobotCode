#include "frc_state_controllers/match_state_controller.h"
#include <pluginlib/class_list_macros.h>

namespace match_state_controller
{
    bool MatchStateController::init(hardware_interface::MatchStateInterface *hw,
			ros::NodeHandle					&root_nh,
			ros::NodeHandle					&controller_nh)

	 {
	     ROS_INFO_STREAM_NAMED("match_state_controller", "init is running");

	     //get match name, check if there is more or less than 1 match
	     std::vector<std::string> match_names = hw->getNames();
		 if (match_names.size() > 1) {
			 ROS_ERROR_STREAM("Cannot initialize multiple match state joints");
			 return false; }
		 else if (match_names.size() < 1) {
			 ROS_ERROR_STREAM("Cannot initialize zero match state joints");
			 return false; }

	     const std::string match_name = match_names[0];

	     //get publish rate from config file
	     if (!controller_nh.getParam("publish_rate", publish_rate_))
			 ROS_ERROR("Could not read publish_rate in match state controller");

	     //set up publisher
	     realtime_pub_.reset(new realtime_tools::RealtimePublisher<frc_msgs::MatchSpecificData>(root_nh, "match_data", 4));

	     auto &m = realtime_pub_->msg_;

	     m.matchTimeRemaining = 0.0;
	     m.gameSpecificData = "";
	     m.eventName = "";
	     m.allianceColor = 0.0;
	     m.matchType = 0.0;
	     m.driverStationLocation = 0.0;
	     m.matchNumber = 0.0;
	     m.replayNumber = 0.0;
	     m.Enabled = false;
	     m.Disabled = false;
	     m.Autonomous = false;
		 m.DSAttached = false;
		 m.FMSAttached = false;
	     m.OperatorControl = false;
	     m.Test = false;
	     m.BatteryVoltage = 0.0;

	     match_state_ = hw->getHandle(match_name);

	     return true;
	 }

    void MatchStateController::starting(const ros::Time &time)
    {
		last_publish_time_ = time;
    }

    void MatchStateController::update(const ros::Time &time, const ros::Duration & )
	{
		//ROS_INFO_STREAM("pdp pub: " << publish_rate_);
		if ((publish_rate_ > 0.0) && (last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time))
		{
			if (realtime_pub_->trylock())
			{
				last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

				auto &m = realtime_pub_->msg_;

				m.header.stamp = time;

				auto &ms = match_state_;

				//read from the object and stuff it in a msg
				m.matchTimeRemaining = ms->getMatchTimeRemaining();
				m.gameSpecificData = ms->getGameSpecificData();
				m.eventName = ms->getEventName();
				m.allianceColor = ms->getAllianceColor();
				m.matchType = ms->getMatchType();
				m.driverStationLocation = ms->getDriverStationLocation();
				m.matchNumber = ms->getMatchNumber();
				m.replayNumber = ms->getReplayNumber();
				m.Enabled = ms->isEnabled();
				m.Disabled = ms->isDisabled();
				m.Autonomous = ms->isAutonomous();
				m.DSAttached = ms->isDSAttached();
				m.FMSAttached = ms->isFMSAttached();
				m.OperatorControl = ms->isOperatorControl();
				m.Test = ms->isTest();
				m.BatteryVoltage = ms->getBatteryVoltage();

				realtime_pub_->unlockAndPublish();
			}
		}
	}

    void MatchStateController::stopping(const ros::Time & )
    {}
}

PLUGINLIB_EXPORT_CLASS(match_state_controller::MatchStateController, controller_interface::ControllerBase)
