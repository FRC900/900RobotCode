#include "controllers_2020/turret_controller.h"

#include <pluginlib/class_list_macros.h> //to compile as a controller

namespace turret_controller
{
	bool TurretController::init(hardware_interface::RobotHW *hw,
									 ros::NodeHandle                 &/*root_nh*/,
									 ros::NodeHandle                 &controller_nh)
	{
		hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();

		//get turret motor params
		XmlRpc::XmlRpcValue turret_params;
		if ( !controller_nh.getParam("turret_joint", turret_params)) //grabbing the config value under the controller's section in the main config file
		{
			ROS_ERROR_STREAM("Could not read turret_params");
			return false;
		}

		//get turret zero timeout param
		if ( !controller_nh.getParam("turret_zero_timeout", turret_zero_timeout_)) //grabbing the config value under the controller's section in the main config file
		{
			ROS_ERROR_STREAM("Could not read turret_zero_timeout");
			return false;
		}

		//get turret zero percent out param
		if ( !controller_nh.getParam("turret_zero_percent_output", turret_zero_percent_output_)) //grabbing the config value under the controller's section in the main config file
		{
			ROS_ERROR_STREAM("Could not read turret_zero_percent_output");
			return false;
		}

		//get turret zeroing angle
		if ( !controller_nh.getParam("turret_zero_angle", turret_zero_angle_)) //grabbing the config value under the controller's section in the main config file
		{
			ROS_ERROR_STREAM("Could not read turret_zero_angle");
			return false;
		}

		//initialize motor joint using those config values
		if ( !turret_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, turret_params) ){
			ROS_ERROR("Cannot initialize turret_joint!");
			return false;
		}
		else
		{
			ROS_INFO("Initialized turret joint");
		}
		//Initialize your ROS server
		turret_service_ = controller_nh.advertiseService("turret_command", &TurretController::cmdService, this);

		return true;
	}

	void TurretController::starting(const ros::Time &/*time*/) {
		turret_joint_.setSelectedSensorPosition(0.0); //resets the encoder position to 0

		zeroed_ = false;
		last_zeroed_  = false;
		last_time_moving_ = ros::Time::now();
		cmd_buffer_.writeFromNonRT(0.0);
	}

	void TurretController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
		// If we hit the limit switch, (re)zero the position.
		if (turret_joint_.getReverseLimitSwitch())
		{
			ROS_INFO_THROTTLE(2, "TurretController : hit limit switch");
			if (!last_zeroed_)
			{
				zeroed_ = true;
				last_zeroed_ = true;
				turret_joint_.setSelectedSensorPosition(turret_zero_angle_);
			}
		}
		else
		{
			last_zeroed_ = false;
		}


		if (zeroed_) // run normally, seeking to various positions
		{
			turret_joint_.setMode(hardware_interface::TalonMode_MotionMagic);

			//grab value from command buffer and write it
			const double turret_cmd = *(cmd_buffer_.readFromRT());
			turret_joint_.setCommand(turret_cmd);
		}
		else
		{
			turret_joint_.setMode(hardware_interface::TalonMode_PercentOutput);
			//Check if turret is timed out
			if ((ros::Time::now() - last_time_moving_).toSec() < turret_zero_timeout_)
			{
				// Not yet zeroed. Run the turret over slowly until the limit switch is set.
				ROS_INFO_STREAM_THROTTLE(0.25, "Zeroing turret with percent output: "
										 << turret_zero_percent_output_);
				turret_joint_.setCommand(turret_zero_percent_output_);
			}
			else
			{
				// Stop moving to prevent motor from burning out
				ROS_INFO_STREAM_THROTTLE(0.25, "Turret timed out in turret controller while zeroing");
				turret_joint_.setCommand(0);
			}

			// If not zeroed but enabled, check if the turret is moving
			if ((turret_joint_.getMode() == hardware_interface::TalonMode_Disabled) ||
				(turret_joint_.getSpeed() > 0)) // TODO : param
			{
				// If moving down, or disabled and thus not expected to move down, reset the timer
				last_time_moving_ = ros::Time::now();
			}
		}
	}

	void TurretController::stopping(const ros::Time &/*time*/) {
	}

	bool TurretController::cmdService(controllers_2020_msgs::TurretSrv::Request &req, controllers_2020_msgs::TurretSrv::Response &/*response*/) {
		if(isRunning())
		{
			//assign request value to command buffer
			cmd_buffer_.writeFromNonRT(req.set_point);
		}
		else
		{
			ROS_ERROR_STREAM("Can't accept new commands. TurretController is not running.");
			return false;
		}
		return true;
	}


}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(turret_controller::TurretController, controller_interface::ControllerBase)
