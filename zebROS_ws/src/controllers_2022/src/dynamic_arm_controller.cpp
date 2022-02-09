#include "controllers_2022/dynamic_arm_controller.h"

namespace dynamic_arm_controller
{
bool DynamicArmController::init(hardware_interface::RobotHW *hw,
                ros::NodeHandle             &/*root_nh*/,
                ros::NodeHandle             &controller_nh)
{
  // create the interface used to initialize the talon joint
  hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();

  //hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();

  if (!controller_nh.getParam("dynamic_arm_zeroing_percent_output", config_.dynamic_arm_zeroing_percent_output))
	{
		ROS_ERROR("dynamic_arm_controller : Could not find dynamic_arm_zeroing_percent_output");
		return false;
	}

	if (!controller_nh.getParam("dynamic_arm_zeroing_timeout", config_.dynamic_arm_zeroing_timeout))
	{
		ROS_ERROR("dynamic_arm_controller : Could not find dynamic_arm_zeroing_timeout");
		return false;
	}

  if (!controller_nh.getParam("motion_magic_velocity_fast", config_.motion_magic_velocity_fast))
  {
    ROS_ERROR("dynamic_arm_controller : Could not find motion_magic_velocity_fast");
    return false;
  }

  if (!controller_nh.getParam("motion_magic_velocity_slow", config_.motion_magic_velocity_slow))
  {
    ROS_ERROR("dynamic_arm_controller : Could not find motion_magic_velocity_slow");
    return false;
  }

  if (!controller_nh.getParam("motion_magic_acceleration_fast", config_.motion_magic_acceleration_fast))
  {
    ROS_ERROR("dynamic_arm_controller : Could not find motion_magic_acceleration_fast");
    return false;
  }

  if (!controller_nh.getParam("motion_magic_acceleration_slow", config_.motion_magic_acceleration_slow))
  {
    ROS_ERROR("dynamic_arm_controller : Could not find motion_magic_acceleration_slow");
    return false;
  }

  /*
  if (!controller_nh.getParam("motion_s_curve_strength",config_.motion_s_curve_strength))
  {
    ROS_ERROR("dynamic_arm_controller : Could not find motion_s_curve_strength");
    return false;
  }*/
  // get config values for the dynamic arm talons
  XmlRpc::XmlRpcValue dynamic_arm_params;
  if (!controller_nh.getParam("dynamic_arm_joint", dynamic_arm_params))
  {
    ROS_ERROR("dynamic_arm_controller : Could not find dynamic_arm_joint");
    return false;
  }

  //initialize the dynamic_arm joint
  if(!dynamic_arm_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, dynamic_arm_params))
  {
    ROS_ERROR("dynamic_arm_controller : Cannot initialize dynamic_arm joint!");
  }

  dynamic_arm_service_ = controller_nh.advertiseService("command", &DynamicArmController::cmdService, this);

  dynamic_reconfigure_server_.init(controller_nh, config_);

  return true;
}

void DynamicArmController::starting(const ros::Time &time) {
  zeroed_ = false;
  last_zeroed_  = false;
  last_time_down_ = ros::Time::now();
  command_buffer_.writeFromNonRT(DynamicArmCommand());
}

void DynamicArmController::update(const ros::Time &time, const ros::Duration &/*duration*/)
{
  // If we hit the limit switch, (re)zero the position.
  if (dynamic_arm_joint_.getReverseLimitSwitch())
  {
    ROS_INFO_THROTTLE(2, "dynamic_arm_controller : hit bottom limit switch");
    if (!last_zeroed_)
    {
      zeroed_ = true;
      last_zeroed_ = true;
      dynamic_arm_joint_.setSelectedSensorPosition(0);
      // dynamic_arm_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
      // dynamic_arm_joint_.setDemand1Value(config_.arb_feed_forward_up_low);
    }
  }
  else
  {
    last_zeroed_ = false;
  }

  if (zeroed_) // run normally, seeking to various positions
  {
    const DynamicArmCommand setpoint = *(command_buffer_.readFromRT());

    if (setpoint.GetUsingPercentOutput()) {
      dynamic_arm_joint_.setMode(hardware_interface::TalonMode_PercentOutput);
    } else {
      dynamic_arm_joint_.setMode(hardware_interface::TalonMode_MotionMagic);
    }

    dynamic_arm_joint_.setCommand(setpoint.GetData());

    //if we're not climbing, add an arbitrary feed forward to hold the dynamic_arm up
    if(!setpoint.GetGoSlow())
    {
      dynamic_arm_joint_.setMotionAcceleration(config_.motion_magic_acceleration_fast);
      dynamic_arm_joint_.setMotionCruiseVelocity(config_.motion_magic_velocity_fast);
      dynamic_arm_joint_.setPIDFSlot(0);
      // Add arbitrary feed forward for upwards motion
      // We could have arb ff for both up and down, but seems
      // easier (and good enough) to tune PID for down motion
      // and add an arb FF correction for up

      // if(dynamic_arm_joint_.getPosition() >= config_.stage_2_height && last_position_ <= config_.stage_2_height) {
      //   dynamic_arm_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
      //   dynamic_arm_joint_.setDemand1Value(config_.arb_feed_forward_up_high);
      // }
      // else if (dynamic_arm_joint_.getPosition() <= config_.stage_2_height && last_position_ >= config_.stage_2_height) {
      //   dynamic_arm_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
      //   dynamic_arm_joint_.setDemand1Value(config_.arb_feed_forward_up_low);
      // }


      //for now, up and down PID is the same, so slot 1 is used for climbing
      /*
      if(last_setpoint_ != setpoint) {
        if(setpoint > dynamic_arm_joint_.getPosition()) {
          dynamic_arm_joint_.setPIDFSlot(0);
        }
        else {
          dynamic_arm_joint_.setPIDFSlot(1);
        }
      }
      last_setpoint_ = setpoint;
      */
    }
    else //climbing
    {
      dynamic_arm_joint_.setMotionAcceleration(config_.motion_magic_acceleration_slow);
      dynamic_arm_joint_.setMotionCruiseVelocity(config_.motion_magic_velocity_slow);
      // dynamic_arm_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
      // dynamic_arm_joint_.setDemand1Value(config_.arb_feed_forward_down);
      //dynamic_arm_joint_.setPeakOutputForward(0.0);
      dynamic_arm_joint_.setPIDFSlot(1);
    }
  }
  else
  {
    dynamic_arm_joint_.setMode(hardware_interface::TalonMode_PercentOutput);
    if ((ros::Time::now() - last_time_down_).toSec() < config_.dynamic_arm_zeroing_timeout)
    {
      // Not yet zeroed. Run the dynamic_arm down slowly until the limit switch is set.
      ROS_INFO_STREAM_THROTTLE(0.25, "Zeroing dynamic arm with percent output: "
                   << config_.dynamic_arm_zeroing_percent_output);
      dynamic_arm_joint_.setCommand(config_.dynamic_arm_zeroing_percent_output);
    }
    else
    {
      // Stop moving to prevent motor from burning out
      ROS_INFO_STREAM_THROTTLE(0.25, "DynamicArm timed out");
      dynamic_arm_joint_.setCommand(0);
    }

    // If not zeroed but enabled, check if the arm is moving down
    if ((dynamic_arm_joint_.getMode() == hardware_interface::TalonMode_Disabled) ||
      (dynamic_arm_joint_.getSpeed() < 0)) // TODO : param
    {
      // If moving down, or disabled and thus not expected to move down, reset the timer
      last_time_down_ = ros::Time::now();
    }
  }
}

void DynamicArmController::stopping(const ros::Time &/*time*/)
{
}

//Command Service Function
bool DynamicArmController::cmdService(controllers_2022_msgs::DynamicArmSrv::Request  &req,
                  controllers_2022_msgs::DynamicArmSrv::Response &/*response*/)
{
  if(isRunning())
  {
    command_buffer_.writeFromNonRT(DynamicArmCommand(req.data, req.use_percent_output, req.go_slow));
  }
  else
  {
    ROS_ERROR_STREAM("Can't accept new commands. DynamicArmController is not running.");
    return false;
  }
  return true;
}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(dynamic_arm_controller::DynamicArmController, controller_interface::ControllerBase)
