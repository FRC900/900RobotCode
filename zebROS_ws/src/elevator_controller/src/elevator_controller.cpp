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

	if (!controller_nh.getParam("arb_feed_forward_up_high", config_.arb_feed_forward_up_high))
	{
		ROS_ERROR("Could not find arb_feed_forward_up_high");
		return false;
	}

	if (!controller_nh.getParam("arb_feed_forward_up_low", config_.arb_feed_forward_up_low))
	{
		ROS_ERROR("Could not find arb_feed_forward_up_low");
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

	if (!controller_nh.getParam("stage_2_height", config_.stage_2_height))
	{
		ROS_ERROR("Could not find stage_2_height");
		return false;
	}

	if (!controller_nh.getParam("motion_magic_velocity_fast", config_.motion_magic_velocity_fast))
	{
		ROS_ERROR("Could not find motion_magic_velocity_fast");
		return false;
	}

	if (!controller_nh.getParam("motion_magic_velocity_slow", config_.motion_magic_velocity_slow))
	{
		ROS_ERROR("Could not find motion_magic_velocity_slow");
		return false;
	}

	if (!controller_nh.getParam("motion_magic_acceleration_fast", config_.motion_magic_acceleration_fast))
	{
		ROS_ERROR("Could not find motion_magic_acceleration_fast");
		return false;
	}

	if (!controller_nh.getParam("motion_magic_acceleration_slow", config_.motion_magic_acceleration_slow))
	{
		ROS_ERROR("Could not find motion_magic_acceleration_slow");
		return false;
	}

	/*
	if (!controller_nh.getParam("motion_s_curve_strength",config_.motion_s_curve_strength))
	{
		ROS_ERROR("Could not find motion_s_curve_strength");
		return false;
	}*/
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

	dynamic_reconfigure_server_.init(controller_nh, config_);

	return true;
}

void ElevatorController::starting(const ros::Time &/*time*/) {
	zeroed_ = false;
	last_zeroed_  = false;
	last_time_down_ = ros::Time::now();
	last_mode_ = hardware_interface::TalonMode_Disabled;
	last_position_ = -1; // give nonsense position to force update on first time through update()
	position_command_.writeFromNonRT(ElevatorCommand());
}

void ElevatorController::update(const ros::Time &/*time*/, const ros::Duration &/*duration*/)
{
	// If we hit the limit switch, (re)zero the position.
	if (elevator_joint_.getReverseLimitSwitch())
	{
		ROS_INFO_THROTTLE(2, "ElevatorController : hit limit switch");
		if (!last_zeroed_)
		{
			zeroed_ = true;
			last_zeroed_ = true;
			elevator_joint_.setSelectedSensorPosition(0);
			elevator_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
			elevator_joint_.setDemand1Value(config_.arb_feed_forward_up_low);
		}
	}
	else
	{
		last_zeroed_ = false;
	}


	if (zeroed_) // run normally, seeking to various positions
	{
		elevator_joint_.setMode(hardware_interface::TalonMode_MotionMagic);
		if (elevator_joint_.getMode() == hardware_interface::TalonMode_Disabled && last_mode_ != hardware_interface::TalonMode_Disabled)
		{
			position_command_.writeFromNonRT(ElevatorCommand (elevator_joint_.getPosition(),false));
		}
		const ElevatorCommand setpoint = *(position_command_.readFromRT());
		elevator_joint_.setCommand(setpoint.GetPosition());

		//if we're not climbing, add an arbitrary feed forward to hold the elevator up
		if(!setpoint.GetGoSlow())
		{
			elevator_joint_.setMotionAcceleration(config_.motion_magic_acceleration_fast);
			elevator_joint_.setMotionCruiseVelocity(config_.motion_magic_velocity_fast);
			elevator_joint_.setPIDFSlot(0);
			// Add arbitrary feed forward for upwards motion
			// We could have arb ff for both up and down, but seems
			// easier (and good enough) to tune PID for down motion
			// and add an arb FF correction for up

			if(elevator_joint_.getPosition() >= config_.stage_2_height && last_position_ <= config_.stage_2_height) {
				elevator_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
				elevator_joint_.setDemand1Value(config_.arb_feed_forward_up_high);
			}
			else if (elevator_joint_.getPosition() <= config_.stage_2_height && last_position_ >= config_.stage_2_height) {
				elevator_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
				elevator_joint_.setDemand1Value(config_.arb_feed_forward_up_low);
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
			elevator_joint_.setMotionAcceleration(config_.motion_magic_acceleration_slow);
			elevator_joint_.setMotionCruiseVelocity(config_.motion_magic_velocity_slow);
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
	if (req.position > 1.7) // TODO : get real measurement, make a param
	{
		ROS_ERROR_STREAM("Elevator controller: req.position too large : " << req.position);
		return false;
	}
	if(isRunning())
	{
		//adjust talon mode, arb feed forward, and PID slot appropriately
		if(req.go_slow)
		{
			ROS_INFO("Elevator controller: now in climbing mode");
		}
		else
		{ //reset to default -- although, this should never be called after endgame
			ROS_INFO("Elevator controller: normal peak output");
		}

		position_command_.writeFromNonRT(ElevatorCommand (req.position, req.go_slow));
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
