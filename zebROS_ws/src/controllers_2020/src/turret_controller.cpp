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

		//give command buffer an initial value
		cmd_buffer_.writeFromNonRT(0.0);
	}

	void TurretController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
		//grab value from command buffer and write it
		const double turret_cmd = *(cmd_buffer_.readFromRT());
		turret_joint_.setCommand(turret_cmd);
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
