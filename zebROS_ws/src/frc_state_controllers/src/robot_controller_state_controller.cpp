#include <pluginlib/class_list_macros.h>
#include "frc_state_controllers/robot_controller_state_controller.h"

namespace robot_controller_state_controller 
{
bool RobotControllerStateController::init(hardware_interface::RobotControllerStateInterface *hw,
										  ros::NodeHandle                                   &root_nh,
										  ros::NodeHandle                                   &controller_nh)
{

	// get publishing period
	if (!controller_nh.getParam("publish_rate", publish_rate_))
		ROS_ERROR("Parameter 'publish_rate' not set");

	// get all joint names from the hardware interface,
	// hope there is only one defined
	std::vector<std::string> rc_names = hw->getNames();
	if (rc_names.size() > 1)
	{
		ROS_ERROR_STREAM("Cannot initialize multiple RobotControllers.");
		return false;
	}
	else if (rc_names.size() < 1)
	{
		ROS_ERROR_STREAM("Cannot initialize zero RobotControllers.");
		return false;
	}

	rc_state_ = hw->getHandle(rc_names[0]);

	realtime_pub_.reset(new realtime_tools::RealtimePublisher<frc_msgs::RobotControllerData>(root_nh, "robot_controller_states", 4));
	return true;
}

void RobotControllerStateController::starting(const ros::Time &time)
{
	last_publish_time_ = time;
}

void RobotControllerStateController::update(const ros::Time &time, const ros::Duration & )
{
	if ((publish_rate_ > 0.0) && (last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time))
	{
		if (realtime_pub_->trylock())
		{
			last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

			auto &m = realtime_pub_->msg_;

			m.header.stamp = time;

			auto &rcs = rc_state_;

			m.fpgaVersion = rcs->GetFPGAVersion();
			m.fpgaRevision = rcs->GetFPGARevision();
			m.fpgaTime = rcs->GetFPGATime();
			m.userButton = rcs->GetUserButton();
			m.isSysActive = rcs->GetIsSysActive();
			m.isBrownedOut = rcs->GetIsBrownedOut();
			m.inputVoltage = rcs->GetInputVoltage();
			m.inputCurrent = rcs->GetInputCurrent();
			m.voltage3V3 = rcs->GetVoltage3V3();
			m.current3V3 = rcs->GetCurrent3V3();
			m.enabled3V3 = rcs->GetEnabled3V3();
			m.faultCount3V3 = rcs->GetFaultCount3V3();
			m.voltage5V = rcs->GetVoltage5V();
			m.current5V = rcs->GetCurrent5V();
			m.enabled5V = rcs->GetEnabled5V();
			m.faultCount5V = rcs->GetFaultCount5V();
			m.voltage6V = rcs->GetVoltage6V();
			m.current6V = rcs->GetCurrent6V();
			m.enabled6V = rcs->GetEnabled6V();
			m.faultCount6V = rcs->GetFaultCount6V();
			m.canData.percentBusUtilization = rcs->GetCANPercentBusUtilization();
			m.canData.busOffCount = rcs->GetCANBusOffCount();
			m.canData.txFullCount = rcs->GetCANTxFullCount();
			m.canData.receiveErrorCount = rcs->GetCANReceiveErrorCount();
			m.canData.transmitErrorCount = rcs->GetCANTransmitErrorCount();

			m.fpgaVersion_status = rcs->GetFPGAVersionStatus();
			m.fpgaRevision_status = rcs->GetFPGARevisionStatus();
			m.fpgaTime_status = rcs->GetFPGATimeStatus();
			m.userButton_status = rcs->GetUserButtonStatus();
			m.isSysActive_status = rcs->GetIsSysActiveStatus();
			m.isBrownedOut_status = rcs->GetIsBrownedOutStatus();
			m.inputVoltage_status = rcs->GetInputVoltageStatus();
			m.inputCurrent_status = rcs->GetInputCurrentStatus();
			m.voltage3V3_status = rcs->GetVoltage3V3Status();
			m.current3V3_status = rcs->GetCurrent3V3Status();
			m.enabled3V3_status = rcs->GetEnabled3V3Status();
			m.faultCount3V3_status = rcs->GetFaultCount3V3Status();
			m.voltage5V_status = rcs->GetVoltage5VStatus();
			m.current5V_status = rcs->GetCurrent5VStatus();
			m.enabled5V_status = rcs->GetEnabled5VStatus();
			m.faultCount5V_status = rcs->GetFaultCount5VStatus();
			m.voltage6V_status = rcs->GetVoltage6VStatus();
			m.current6V_status = rcs->GetCurrent6VStatus();
			m.enabled6V_status = rcs->GetEnabled6VStatus();
			m.faultCount6V_status = rcs->GetFaultCount6VStatus();
			m.canData_status = rcs->GetCANDataStatus();


			realtime_pub_->unlockAndPublish();
		}
	}
}

void RobotControllerStateController::stopping(const ros::Time & )
{
}

}

PLUGINLIB_EXPORT_CLASS(robot_controller_state_controller::RobotControllerStateController, controller_interface::ControllerBase)
