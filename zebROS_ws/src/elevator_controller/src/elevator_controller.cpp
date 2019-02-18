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

	if (!controller_nh.getParam("elevator_zeroing_timeout", elevator_zeroing_timeout_))
	{
		ROS_ERROR("Could not find elevator_zeroing_timeout");
		return false;
	}

	if (!controller_nh.getParam("slow_peak_output", slow_peak_output_))
	{
		ROS_ERROR("Elevator controller: could not find slow_peak_output");
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
	last_time_down_ = ros::Time::now();

	return true;
}

void ElevatorController::starting(const ros::Time &/*time*/) {
}

void ElevatorController::update(const ros::Time &/*time*/, const ros::Duration &/*duration*/)
{
	// If we hit the limit switch, (re)zero the position.
	if (elevator_joint_.getReverseLimitSwitch())
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
	}
	else
	{
		elevator_joint_.setMode(hardware_interface::TalonMode_PercentOutput);
		if ((ros::Time::now() - last_time_down_).toSec() < elevator_zeroing_timeout_)
		{
			// Not yet zeroed. Run the elevator down slowly until the limit switch is set.
			elevator_joint_.setCommand(elevator_zeroing_percent_output_);
		}
		else
		{
			elevator_joint_.setCommand(0);
		}

		// If not zeroed but enabled, check if the arm is moving down
		if ((elevator_joint_.getMode() == hardware_interface::TalonMode_Disabled) ||
			(elevator_joint_.getSpeed() < 0)) // TODO : param
		{
			// If moving down, or disabled and thus not expected to move down, reset the timer
			last_time_down_ = ros::Time::now();
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
		//adjust peak output appropriately
		if(req.go_slow)
		{
			elevator_joint_.setPeakOutputForward(slow_peak_output_);
			elevator_joint_.setPeakOutputReverse(-slow_peak_output_);
			ROS_INFO("Elevator controller: reduced peak output +/- %f",slow_peak_output_);
		}
		else { //reset to default
			elevator_joint_.setPeakOutputForward(1.0);
			elevator_joint_.setPeakOutputReverse(-1.0);
			ROS_INFO("Elevator controller: normal peak output");
		}

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
