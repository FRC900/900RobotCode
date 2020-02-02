#include "controllers_2020/control_panel_controller.h"

namespace control_panel_controller
{
	bool ControlPanelController::init(hardware_interface::RobotHW *hw,
			ros::NodeHandle                 &/*root_nh*/,
			ros::NodeHandle                 &controller_nh)
	{
		hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
		hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();

		//initialize control panel arm joint (pnumatic piston for vertical actuation)
		control_panel_arm_joint_ = pos_joint_iface->getHandle("control_panel_arm_joint"); //joint_name comes from ros_control_boilerplate/config/[insert_year]_compbot_base_jetson.yaml

		//initialize control_panel_joint (motor for panel rotation)
		//get control panel params from config file
		XmlRpc::XmlRpcValue control_panel_params;
		if ( !controller_nh.getParam("control_panel_joint", control_panel_params)) //grabbing the config value under the controller's section in the main config file
		{
			ROS_ERROR_STREAM("Could not read control_panel_params");
			return false;
		}
		controller_nh.getParam("control_panel_diameter", control_panel_diameter_);
		controller_nh.getParam("wheel_diameter", wheel_diameter_);

		ROS_INFO_STREAM("Control Panel Diameter:" << control_panel_diameter_);
		ROS_INFO_STREAM("Wheel Diameter:" << wheel_diameter_);

		//initialize motor joint using those config values
		if (!control_panel_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, control_panel_params)) {
			ROS_ERROR("Cannot initialize control_panel_joint!");
			return false;
		}
		else
		{
			ROS_INFO("Initialized control panel joint");
		}

		control_panel_service_ = controller_nh.advertiseService("control_panel_command", &ControlPanelController::cmdService, this);

		return true;
	}

	void ControlPanelController::starting(const ros::Time &/*time*/) {
		//give command buffer(s) an initial value
		/* Ex:
		   cmd_buffer_.writeFromNonRT(true);
		   */
		control_panel_joint_.setSelectedSensorPosition(0.0); //set current encoder position to 0
		control_panel_cmd_.writeFromNonRT(ControlPanelCommand(0,false));
	}

	void ControlPanelController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
		//grab value from command buffer(s)
		/* Ex:
		   const bool extend_cmd = *(cmd_buffer_.readFromRT());
		   */
		const ControlPanelCommand control_panel_cmd = *(control_panel_cmd_.readFromRT());
		double control_panel_arm_double;
		if(control_panel_cmd.panel_arm_extend_ == true){
			control_panel_arm_double = 1;
		}
		else {
			control_panel_arm_double = 0;
		}
		control_panel_joint_.setCommand(control_panel_cmd.set_point_); //set the position command to the control panel motor
		control_panel_arm_joint_.setCommand(control_panel_cmd.panel_arm_extend_);//set the extend/retract command to the control panel solenoid
	}


	void ControlPanelController::stopping(const ros::Time &/*time*/) {
	}
	bool ControlPanelController::cmdService(controllers_2020_msgs::ControlPanelSrv::Request &req, controllers_2020_msgs::ControlPanelSrv::Response &/*response*/) {
		if(isRunning())
		{
			//assign request value to command buffer(s)
			double rotation_ratio = (control_panel_diameter_/wheel_diameter_);
			double set_point = (req.control_panel_rotations * rotation_ratio) + control_panel_joint_.getPosition();

			control_panel_cmd_.writeFromNonRT(ControlPanelCommand(set_point , req.panel_arm_extend));
		}
		else
		{
			ROS_ERROR_STREAM("Can't accept new commands. ControlPanelController is not running.");
			return false;
		}
		return true;
	}
}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(control_panel_controller::ControlPanelController, controller_interface::ControllerBase)

