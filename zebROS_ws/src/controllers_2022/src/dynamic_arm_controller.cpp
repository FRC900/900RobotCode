#include "controllers_2022/dynamic_arm_controller.h"

// #define SENSE_CURRENT // comment out to disable current sensing

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

  if (!controller_nh.getParam("motion_magic_velocity_veryfast", config_.motion_magic_velocity_veryfast))
  {
    ROS_ERROR("dynamic_arm_controller : Could not find motion_magic_velocity_veryfast");
    return false;
  }

  if (!controller_nh.getParam("motion_magic_velocity_traversal", config_.motion_magic_velocity_traversal))
  {
    ROS_ERROR("dynamic_arm_controller : Could not find motion_magic_velocity_traversal");
    return false;
  }

  if (!controller_nh.getParam("motion_magic_acceleration_traversal", config_.motion_magic_acceleration_traversal))
  {
    ROS_ERROR("dynamic_arm_controller : Could not find motion_magic_acceleration_traversal");
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

  if (!controller_nh.getParam("motion_magic_acceleration_veryfast", config_.motion_magic_acceleration_veryfast))
  {
    ROS_ERROR("dynamic_arm_controller : Could not find motion_magic_acceleration_veryfast");
    return false;
  }

  if (!controller_nh.getParam("dynamic_arm_current_threshold", current_threshold_))
  {
    ROS_ERROR("dynamic_arm_controller : Could not find dynamic_arm_current_threshold");
    return false;
  }

  if (!controller_nh.getParam("dynamic_arm_max_current_iterations", max_current_iterations_)) // 1 iteration = 10ms
  {
    ROS_ERROR("dynamic_arm_controller : Could not find dynamic_arm_current_threshold");
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
  dynamic_arm_zeroing_service_ = controller_nh.advertiseService("zero", &DynamicArmController::zeroService, this);
  zeroed_publisher_ = controller_nh.advertise<std_msgs::Bool>("is_zeroed", 100);

  dynamic_reconfigure_server_.init(controller_nh, config_);

  return true;
}

void DynamicArmController::starting(const ros::Time &time) {
  zeroed_ = false;
  last_zeroed_  = false;
  last_time_down_ = ros::Time::now();
  command_buffer_.writeFromNonRT(DynamicArmCommand());
  current_iterations_ = 0;
}

void DynamicArmController::update(const ros::Time &time, const ros::Duration &/*duration*/)
{
  bool current_is_too_high = false;
#ifdef SENSE_CURRENT
  if (dynamic_arm_joint_.getOutputCurrent() >= current_threshold_) {
    current_iterations_++;
    if (current_iterations_ >= max_current_iterations_) {
      ROS_WARN_STREAM_THROTTLE(0.5, "dynamic_arm_controller : motor is above current limit. stopping.");
      current_is_too_high = true;
    }
  } else {
    current_iterations_ = 0;
  }
#endif
  DynamicArmCommand setpoint = *(command_buffer_.readFromRT());

  // If we hit the limit switch, (re)zero the position.
  if (dynamic_arm_joint_.getReverseLimitSwitch() || current_is_too_high)
  {
    ROS_INFO_STREAM_THROTTLE(2, "dynamic_arm_controller : " << (!current_is_too_high ? "hit bottom limit switch" : "current too high"));
    if (!last_zeroed_)
    {
      zeroed_ = true;
      last_zeroed_ = true;
      dynamic_arm_joint_.setSelectedSensorPosition(0);
      if (dynamic_arm_joint_.getMode() == hardware_interface::TalonMode_PercentOutput) {
        setpoint.SetData(0.0); // stop motor
      }
      // dynamic_arm_joint_.setDemand1Type(hardware_interface::DemandType_ArbitraryFeedForward);
      // dynamic_arm_joint_.setDemand1Value(config_.arb_feed_forward_up_low);
    }
  }
  else
  {
    last_zeroed_ = false;
  }

  std_msgs::Bool zeroed;
  zeroed.data = zeroed_;
  zeroed_publisher_.publish(zeroed);

  if (zeroed_) // run normally, seeking to various positions
  {

    if (setpoint.GetUsingPercentOutput()) {
      dynamic_arm_joint_.setMode(hardware_interface::TalonMode_PercentOutput);
    } else {
      dynamic_arm_joint_.setMode(hardware_interface::TalonMode_MotionMagic);
    }

    dynamic_arm_joint_.setCommand(setpoint.GetData());

    switch (setpoint.GetProfile()) {
      case controllers_2022_msgs::DynamicArmSrv::Request::RETRACT:
        dynamic_arm_joint_.setMotionAcceleration(config_.motion_magic_acceleration_fast);
        dynamic_arm_joint_.setMotionCruiseVelocity(config_.motion_magic_velocity_fast);
        break;
      case controllers_2022_msgs::DynamicArmSrv::Request::EXTEND:
        dynamic_arm_joint_.setMotionAcceleration(config_.motion_magic_acceleration_veryfast);
        dynamic_arm_joint_.setMotionCruiseVelocity(config_.motion_magic_velocity_veryfast);
        break;
      case controllers_2022_msgs::DynamicArmSrv::Request::TRANSITION:
        dynamic_arm_joint_.setMotionAcceleration(config_.motion_magic_acceleration_slow);
        dynamic_arm_joint_.setMotionCruiseVelocity(config_.motion_magic_velocity_slow);
        break;
      case controllers_2022_msgs::DynamicArmSrv::Request::TRAVERSAL:
        dynamic_arm_joint_.setMotionAcceleration(config_.motion_magic_acceleration_traversal);
        dynamic_arm_joint_.setMotionCruiseVelocity(config_.motion_magic_velocity_traversal);
        break;
      default:
        break;
    }
  }
  else if (do_zero_)
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
    if (!zeroed_) {
      ROS_ERROR_STREAM("dynamic_arm_controller : this command WILL NOT BE RUN until the arm is zeroed");
      ROS_INFO_STREAM("dynamic_arm_controller : If you want to zero now, call the /frcrobot_jetson/dynamic_arm_controller/zero service");
    }
    command_buffer_.writeFromNonRT(DynamicArmCommand(req.data, req.use_percent_output, req.profile));
  }
  else
  {
    ROS_ERROR_STREAM("Can't accept new commands. DynamicArmController is not running.");
    return false;
  }
  return true;
}

//Command Service Function
bool DynamicArmController::zeroService(std_srvs::Trigger::Request  &req,
                  std_srvs::Trigger::Response &/*response*/)
{
  do_zero_ = true;
  return true;
}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(dynamic_arm_controller::DynamicArmController, controller_interface::ControllerBase)
