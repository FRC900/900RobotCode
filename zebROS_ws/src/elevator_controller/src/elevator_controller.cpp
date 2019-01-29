#include "elevator_controller/elevator_controller.h"
namespace elevator_controller
{

	bool ElevatorController::init(hardware_interface::RobotHW *hw,
			ros::NodeHandle                 &root_nh,
			ros::NodeHandle                 &controller_nh)
	{
		//create the interface used to initialize the talon joint
		hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();

		hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();
		
	    //initializes climber engage joint
		climber_engage_joint_ = pos_joint_iface->getHandle("climber_engage_joint");

		//get config values for the elevator talon
		XmlRpc::XmlRpcValue elevator_params;
		if (!controller_nh.getParam("elevator_joint_",elevator_params))
		{ ROS_ERROR("Could not find elevator_joint_");
			return false;
		}
	
		//initialize the elevator joint
		if(!elevator_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, elevator_params))
		{
			ROS_ERROR("Cannot initialize elevator joint!");
		}



		// intake_in_ = pos_joint_iface->getHandle("clamp");
		/*
		//read intake name from config file
		XmlRpc::XmlRpcValue intake_params;
		if (!controller_nh.getParam("intake_joint", intake_params))
		{
		ROS_ERROR_STREAM("Can not read intake name");
		return false;
		}
		*/
		//initialize joint with that name
		/* if (!intake_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, intake_params))
		   {
		   ROS_ERROR("Cannot initialize intake joint!");
		   return false;
		   }
		   else
		   {
		   ROS_INFO("Initialized intake joint!");
		   }

*/
		elevator_service_ = controller_nh.advertiseService("elevator_service", &ElevatorController::cmdService, this);
		
		climber_engage_service_ = controller_nh.advertiseService("climber_engage_service", &ElevatorController::climberEngageCallback, this);

		return true;
	}


	void ElevatorController::starting(const ros::Time &/*time*/) {
	}

	void ElevatorController::update(const  ros::Time &time,const  ros::Duration &duration) 	// set the command to the spinny part of the intake
			{
			elevator_joint_.setCommand(*spin_command_.readFromRT());

			bool climber_engage_cmd = *(climber_engage_cmd_.readFromRT());
			if(climber_engage_cmd == true) {
				climber_engage_joint_.setCommand(-1.0);
			}
			else if(climber_engage_cmd == false) {
				climber_engage_joint_.setCommand(1.0);
			}

			}

			void ElevatorController::stopping(const ros::Time &time) {
			}


			//Command Service Function

			bool ElevatorController::cmdService(elevator_controller::ElevatorSrv::Request &req, elevator_controller::ElevatorSrv::Response &response) {
			if(isRunning())
			{
			spin_command_.writeFromNonRT(req.position); //take the service request for a certain amount of power (-1 to 1) and write it to the command variable
			//	intake_in_cmd_.writeFromNonRT(req.intake_in); //take the service request for in/out (true/false???) and write to a command variable
			}
			else
			{
			ROS_ERROR_STREAM("Can't accept new commands. ElevatorController is not running.");
			return false;
			}
			return true;
			}
			
			bool ElevatorController::ClimberEngageCallback(elevator_controller::ClimberEngageSrv::Request &req, elevator_controller::ClimberEngageSrv::Response &response) {
			if(isRunning())
			{
			climber_enagage_cmd_.writeFromNonRT(req.engage); //take the service request for a certain amount of power (-1 to 1) and write it to the command variable
			//	intake_in_cmd_.writeFromNonRT(req.intake_in); //take the service request for in/out (true/false???) and write to a command variable
			}
			else
			{
			ROS_ERROR_STREAM("Can't accept new commands. ElevatorController is not running.");
			return false;
			}
			return true;
			}
}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(elevator_controller::ElevatorController, controller_interface::ControllerBase)
