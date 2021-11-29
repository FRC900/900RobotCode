//TEMPLATE FOR WRITING A CONTROLLER
//replace "mech" with the name of your mechanism, words_separated_by_underscores
//replace "Mech" with the name of your mechanism, ThisIsTheFormatForThat
//replace "package" with the name of the controllers package
#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time

//controller interfaces
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <talon_controllers/talon_controller_interface.h>

#include "template_controller/MechSrv.h"

namespace mech_controller
{
//this is the controller class, so it stores all of the  update() functions and the actual handle from the joint interface
//if it was only one type, can do controller_interface::Controller<TalonCommandInterface or PositionJointInterface> here
class MechController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface, hardware_interface::TalonCommandInterface> //including both talons and pistons so can copy paste this w/o having to change it if you want to add a talon
{

	bool init(hardware_interface::RobotHW *hw,
			  ros::NodeHandle             &/*root_nh*/,
			  ros::NodeHandle             &controller_nh) override
	{
		//get interface
		hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();
		hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
		//Initialize piston joints
		/* Ex:
		push_joint_ = pos_joint_iface->getHandle("joint_name"); //joint name comes from ros_control_boilerplate/config/[insert_year]_compbot_base_jetson.yaml
		*/

		//Initialize motor joints
		/* Ex:

		//get params from config file
		XmlRpc::XmlRpcValue intake_motor_params;
		if ( !controller_nh.getParam("config_value_name", intake_motor_params)) //grabbing the config value under the controller's section in the main config file
		{
			ROS_ERROR_STREAM("Could not read _______ params");
			return false;
		}
		//initialize motor joint using those config values
		if ( !motor_name_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, intake_motor_params) ) {
			ROS_ERROR("Cannot initialize ______ joint!");
			return false;
		}

		*/


		//Initialize your ROS server
		/* Ex:
		mech_service_ = controller_nh.advertiseService("mech_command", &MechController::cmdService, this);
		*/

		return true;
	}

	void starting(const ros::Time &/*time*/) override {
		//give command buffer(s) an initial value
		/* Ex:
		cmd_buffer_.writeFromNonRT(true);
		*/
	}

	void update(const ros::Time &/*time*/, const ros::Duration &/*period*/) override {
		//grab value from command buffer(s)
		/* Ex:
		const bool extend_cmd = *(cmd_buffer_.readFromRT());
		*/


		//Set values of the pistons based on the command. Can be 1.0, 0.0, or -1.0. -1.0 is only used with double solenoids
		/* Syntax: push_joint_.setCommand(1.0); */

		//for motors, it's the same syntax, but the meaning of the argument passed to setCommand() differs based on what motor mode you're using
	}

	void stopping(const ros::Time &/*time*/) override {
	}

private:
	bool cmdService(template_controller::MechSrv::Request &req, template_controller::MechSrv::Response &/*response*/) {
		if(isRunning())
		{
			//assign request value to command buffer(s)
			/* Ex:
			cmd_buffer_.writeFromNonRT(req.claw_release);
			*/
		}
		else
		{
			ROS_ERROR_STREAM("Can't accept new commands. MechController is not running.");
			return false;
		}
		return true;
	}

	//variable for piston joint
	/* Ex:
	   hardware_interface::JointHandle push_joint_; //handle for accessing the piston joint command buffer
	   */

	//variable for motor joint
	/* Ex:
	   talon_controllers::TalonPercentOutputControllerInterface motor_name_joint_; //other types exist FYI
	   */

	//set up your ROS server and buffer
	/* Ex:
	   ros::ServiceServer mech_service_; //service for receiving commands
	   realtime_tools::RealtimeBuffer<bool> cmd_buffer_; //buffer for commands
	   */

}; // class

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
#include <pluginlib/class_list_macros.h> //to compile as a controller
PLUGINLIB_EXPORT_CLASS(mech_controller::MechController, controller_interface::ControllerBase)
