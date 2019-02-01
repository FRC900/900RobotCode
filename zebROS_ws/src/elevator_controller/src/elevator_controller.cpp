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

	elevator_service_ = controller_nh.advertiseService("elevator_service", &ElevatorController::cmdService, this);

	zeroed_ = false;

	return true;
}

void ElevatorController::starting(const ros::Time &/*time*/) {
}

void ElevatorController::update(const ros::Time &/*time*/, const ros::Duration &/*duration*/)
{
	double elevator_position = elevator_joint_.getPosition(); //so we call the getPosition() function fewer times?

	// Grab initial position, use it to sanity check encoder position
	// while looking for zero on the way down. If we go too far, assume
	// the limit switch is broken and stop driving the motor
	static bool initial_position_found = false;
	if(elevator_position == 0)
	{
		ROS_ERROR_STREAM("not running elevator_controller yet; waiting for encoder data to come in");
		return;
	}
	else if(!initial_position_found)
	{
		initial_position_ = elevator_position;
		ROS_INFO_STREAM("ElevatorController: initial elevator position = " << elevator_position);
		initial_position_found = true;
	}

	// If we hit the limit switch, zero the position.
	// TODO : should we re-zero every time we hit it during a match?
	if (!zeroed_ && elevator_joint_.getReverseLimitSwitch())
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
		if (setpoint > elevator_position)
		{
			elevator_joint_.setDemand1Type(hardware_interface::DemandType_AuxPID);
			elevator_joint_.setDemand1Value(arb_feed_forward_up_);
		}
		else
		{
			elevator_joint_.setDemand1Type(hardware_interface::DemandType_Neutral);
			elevator_joint_.setDemand1Value(0);
		}
	}
	else
	{
		// Not yet zeroed. Run the elevator down slowly until the limit switch is set.
		elevator_joint_.setMode(hardware_interface::TalonMode_PercentOutput);

		// TODO : check sign - expecting down to be negative
		if ((initial_position_ - elevator_position) > elevator_sensor_bad_distance_)
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
	}
}

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

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(elevator_controller::ElevatorController, controller_interface::ControllerBase)
