#include "elevator_controller/elevator_controller.h"

namespace elevator_controller
{
bool ElevatorController::init(hardware_interface::RobotHW *hw,
							  ros::NodeHandle             &root_nh,
							  ros::NodeHandle             &controller_nh)
{
	//create the interface used to initialize the talon joint
	hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();

	//hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();

	if (!controller_nh.getParam("arb_feed_forward_up", arb_feed_forward_up_))
	{
<<<<<<< HEAD
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
=======
		ROS_ERROR("Could not find arb_feed_forward_up");
		return false;
	}
	if (!controller_nh.getParam("elevator_zeroing_percent_output", elevator_zeroing_percent_output_))
	{
		ROS_ERROR("Could not find elevator_zeroing_percent_output");
		return false;
	}
	if (!controller_nh.getParam("elevator_sensor_bad_distance", elevator_sensor_bad_distance_))
	{
		ROS_ERROR("Could not find elevator_sensor_bad_distance");
		return false;
	}

	//get config values for the elevator talon
	XmlRpc::XmlRpcValue elevator_params;
	if (!controller_nh.getParam("elevator_joint", elevator_params))
	{
		ROS_ERROR("Could not find elevator_joint");
		return false;
	}

	//initialize the elevator joint
	if(!elevator_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, elevator_params))
	{
		ROS_ERROR("Cannot initialize elevator joint!");
	}
	// Grab initial position, use it to sanity check encoder position
	// while looking for zero on the way down. If we go too far, assume
	// the limit switch is broken and stop driving the motor
	initial_position_ = elevator_joint_.getPosition();

	elevator_service_ = controller_nh.advertiseService("elevator_service", &ElevatorController::cmdService, this);

	zeroed_ = false;
>>>>>>> 01edc2a6eec83020ceb9c4a874eff9132aa57e3d

	return true;
}

void ElevatorController::starting(const ros::Time &/*time*/) {
}

void ElevatorController::update(const ros::Time &/*time*/, const ros::Duration &/*duration*/)
{
	// If we hit the limit switch, zero the position.
	// TODO : should we re-zero every time we hit it during a match?
	if (!zeroed_ && elevator_joint_.getForwardLimitSwitch())
	{
		ROS_INFO("ElevatorController : hit limit switch");
		zeroed_ = true;
		elevator_joint_.setSelectedSensorPosition(0);
	}

	if (zeroed_) // run normally, seeking to various positions
	{
		const double setpoint = *(position_command_.readFromRT());
		elevator_joint_.setMode(hardware_interface::TalonMode_MotionMagic);
		elevator_joint_.setPIDFSlot(0);
		elevator_joint_.setCommand(setpoint);

		// Add arbirary feed forward for upwards motion
		// We could have arb ff for both up and down, but seems
		// easier (and good enough) to tune PID for down motion
		// and add an arb FF correction for up
		if (setpoint > elevator_joint_.getPosition())
		{
			elevator_joint_.setDemand1Type(hardware_interface::DemandType_AuxPID);
			elevator_joint_.setDemand1Value(arb_feed_forward_up_);
		}
		else
		{
			elevator_joint_.setDemand1Type(hardware_interface::DemandType_Neutral);
			elevator_joint_.setDemand1Value(0);
		}
<<<<<<< HEAD
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
=======
>>>>>>> 01edc2a6eec83020ceb9c4a874eff9132aa57e3d
	}
	else
	{
		// Not yet zeroed. Run the elevator down slowly until the limit switch is set.
		elevator_joint_.setMode(hardware_interface::TalonMode_PercentOutput);

<<<<<<< HEAD

	void ElevatorController::starting(const ros::Time &/*time*/) {
=======
		// TODO : check sign - expecting down to be negative
		if ((initial_position_ - elevator_joint_.getPosition()) > elevator_sensor_bad_distance_)
		{
			elevator_joint_.setCommand(elevator_zeroing_percent_output_); // TODO : configuration param
		}
		else
		{
			// If we've moved further than the height of the elevator
			// and haven't seen a limit switch, there's a good chance
			// the limit switch is not working. In that case, stop driving
			// down since things just aren't going to work.
			elevator_joint_.setCommand(0);
		}
>>>>>>> 01edc2a6eec83020ceb9c4a874eff9132aa57e3d
	}
}

<<<<<<< HEAD
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
=======
void ElevatorController::stopping(const ros::Time &/*time*/)
{
}

//Command Service Function
bool ElevatorController::cmdService(elevator_controller::ElevatorSrv::Request  &req,
									elevator_controller::ElevatorSrv::Response &/*response*/)
{
	if(isRunning())
	{
		position_command_.writeFromNonRT(req.position);
		ROS_INFO_STREAM("writing " << std::to_string(req.position) << " to elevator controller");
	}
	else
	{
		ROS_ERROR_STREAM("Can't accept new commands. ElevatorController is not running.");
		return false;
	}
	return true;
}

>>>>>>> 01edc2a6eec83020ceb9c4a874eff9132aa57e3d
}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(elevator_controller::ElevatorController, controller_interface::ControllerBase)
