#include "elevator_controller/elevator_controller.h"

namespace elevator_controller
{
bool ElevatorController::init(hardware_interface::RobotHW *hw,
							  ros::NodeHandle             &/*root_nh*/,
							  ros::NodeHandle             &controller_nh)
{
	//create the interface used to initialize the talon joint
	hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();

	//hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();

	if (!controller_nh.getParam("arb_feed_forward_up", config_.arb_feed_forward_up))
	{
		ROS_ERROR("Could not find arb_feed_forward_up");
		return false;
	}

	if (!controller_nh.getParam("arb_feed_forward_down", config_.arb_feed_forward_down))
	{
		ROS_ERROR("Could not find arb_feed_forward_down");
		return false;
	}

	if (!controller_nh.getParam("elevator_zeroing_percent_output", config_.elevator_zeroing_percent_output))
	{
		ROS_ERROR("Could not find elevator_zeroing_percent_output");
		return false;
	}

	if (!controller_nh.getParam("elevator_zeroing_timeout", config_.elevator_zeroing_timeout))
	{
		ROS_ERROR("Could not find elevator_zeroing_timeout");
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

	dynamic_reconfigure_server_.init(controller_nh, boost::bind(&ElevatorController::callback, this, _1, _2));

	return true;
}

void ElevatorController::starting(const ros::Time &/*time*/) {
	go_slow_ = false;
	zeroed_ = false;
	last_time_down_ = ros::Time::now();
	position_command_.writeFromNonRT(0);
}

void ElevatorController::update(const ros::Time &/*time*/, const ros::Duration &/*duration*/)
{
	// If we hit the limit switch, (re)zero the position.
	if (elevator_joint_.getReverseLimitSwitch())
	{
		ROS_INFO_THROTTLE(2, "ElevatorController : hit limit switch");
		zeroed_ = true;
		elevator_joint_.setSelectedSensorPosition(0);
	}

	if (zeroed_) // run normally, seeking to various positions
	{
		if (elevator_joint_.getMode() == hardware_interface::TalonMode_Disabled && last_mode_ != hardware_interface::TalonMode_Disabled)
		{
			position_command_.writeFromNonRT(elevator_joint_.getPosition());
		}
		const double setpoint = *(position_command_.readFromRT());
		elevator_joint_.setCommand(setpoint);

		//if we're not climbing, add an arbitrary feed forward to hold the elevator up
		if(!go_slow_)
		{
			elevator_joint_.setMode(hardware_interface::TalonMode_Position);
			elevator_joint_.setPIDFSlot(0);
			// Add arbitrary feed forward for upwards motion
			// We could have arb ff for both up and down, but seems
			// easier (and good enough) to tune PID for down motion
			// and add an arb FF correction for up
			if(elevator_joint_.getPosition() > 0.8 && last_position_ < 0.8) {
				elevator_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
				elevator_joint_.setDemand1Value(config_.arb_feed_forward_up);
			}
			else if (elevator_joint_.getPosition() < 0.8 && last_position_ > 0.8) {
				elevator_joint_.setDemand1Type(hardware_interface::DemandType_Neutral);
				elevator_joint_.setDemand1Value(0);
			}

			//for now, up and down PID is the same, so slot 1 is used for climbing
			/*
			if(last_setpoint_ != setpoint) {
				if(setpoint > elevator_joint_.getPosition()) {
					elevator_joint_.setPIDFSlot(0);
				}
				else {
					elevator_joint_.setPIDFSlot(1);
				}
			}
			last_setpoint_ = setpoint;
			*/
		}
		else //climbing
		{
			elevator_joint_.setMode(hardware_interface::TalonMode_MotionMagic);
			elevator_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
			elevator_joint_.setDemand1Value(config_.arb_feed_forward_down);
			//elevator_joint_.setPeakOutputForward(0.0);
			elevator_joint_.setPIDFSlot(1);
		}
	}
	else
	{
		elevator_joint_.setMode(hardware_interface::TalonMode_PercentOutput);
		if ((ros::Time::now() - last_time_down_).toSec() < config_.elevator_zeroing_timeout)
		{
			// Not yet zeroed. Run the elevator down slowly until the limit switch is set.
			ROS_INFO_STREAM_THROTTLE(0.25, "Zeroing elevator with percent output: "
									 << config_.elevator_zeroing_percent_output);
			elevator_joint_.setCommand(config_.elevator_zeroing_percent_output);
		}
		else
		{
			// Stop moving to prevent motor from burning out
			ROS_INFO_STREAM_THROTTLE(0.25, "Elevator timed out");
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
        last_position_ = elevator_joint_.getPosition();
        last_mode_ = elevator_joint_.getMode();
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
		//adjust talon mode, arb feed forward, and PID slot appropriately
		if(req.go_slow)
		{
			go_slow_ = true;
			ROS_INFO("Elevator controller: now in climbing mode");
		}
		else { //reset to default -- although, this should never be called after endgame
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

void ElevatorController::callback(elevator_controller::ElevatorConfig &config, uint32_t level)
{
	(void)level;
	config_ = config;
}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(elevator_controller::ElevatorController, controller_interface::ControllerBase)
