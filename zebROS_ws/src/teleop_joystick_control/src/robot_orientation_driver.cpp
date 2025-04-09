// TODO - consider tying pid enable pub to robot enabled?
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "pid_velocity_msg/PIDVelocity.h"
#include "teleop_orientation_msgs/TeleopOrientation.h"

#include "teleop_joystick_control/RobotOrientationDriver.h"

RobotOrientationDriver::RobotOrientationDriver(const ros::NodeHandle &nh)
	: nh_(nh)
	, orientation_command_sub_{nh_.subscribe("orientation_command", 1, &RobotOrientationDriver::orientationCmdCallback, this, ros::TransportHints().tcpNoDelay())}
	, velocity_orientation_command_sub_{nh_.subscribe("velocity_orientation_command", 1, &RobotOrientationDriver::velocityOrientationCmdCallback, this, ros::TransportHints().tcpNoDelay())}
	, pid_enable_pub_{nh_.advertise<std_msgs::Bool>("orient_strafing/pid_enable", 1, true)} // latching
	, pid_state_pub_{nh_.advertise<std_msgs::Float64>("orient_strafing/state", 1)}
	, pid_setpoint_pub_{nh_.advertise<pid_velocity_msg::PIDVelocity>("orient_strafing/setpoint", 1)}
	, pid_control_effort_sub_{nh_.subscribe("orient_strafing/control_effort", 1, &RobotOrientationDriver::controlEffortCallback, this, ros::TransportHints().tcpNoDelay())}
	, imu_sub_{nh_.subscribe("/imu/zeroed_imu", 1, &RobotOrientationDriver::imuCallback, this, ros::TransportHints().tcpNoDelay())}
	, match_data_sub_{nh_.subscribe("/frcrobot_rio/match_data", 1, &RobotOrientationDriver::matchStateCallback, this)}
    , robot_orient_service_{nh_.advertiseService("set_teleop_orient", &RobotOrientationDriver::holdTargetOrientation, this)}
	, orientation_profile_{nh_}
	// one_shot = true, auto_start = false
	// inversting that
	, most_recent_teleop_timer_{nh_.createTimer(RESET_TO_TELEOP_CMDVEL_TIMEOUT, &RobotOrientationDriver::checkFromTeleopTimeout, this, false, true)}
	, pid_publish_timer_{nh_.createTimer(PID_PUBLISH_RATE, &RobotOrientationDriver::publishPIDSetpoint, this, false, true)}
{
	// Make sure the PID node is enabled
	std_msgs::Bool enable_pub_msg;
	enable_pub_msg.data = true;
	pid_enable_pub_.publish(enable_pub_msg);
}

void RobotOrientationDriver::publishPIDSetpoint(const ros::TimerEvent & /*event*/)
{
	// Publish desired robot orientation to the PID node
	pid_velocity_msg::PIDVelocity pid_setpoint_msg;
	const auto orientation_state = orientation_profile_.getOrientationState();
	pid_setpoint_msg.position = orientation_state.position;
	pid_setpoint_msg.velocity = orientation_state.velocity;
	pid_setpoint_pub_.publish(pid_setpoint_msg);
}

// Interface to teleop code to set the desired orientation of the robot
// Expose the from_teleop flag to allow the teleop code to determine if
// the teleop evaluateCommands function is driving the robot (used only
// in the case of green button driving, which has a separate cmd_vel publisher)
void RobotOrientationDriver::setTargetOrientation(const double angle, const bool from_teleop, const double velocity)
{
	setTargetOrientation(angle, from_teleop, velocity, false);
}

// Internal common function to handle setting the desired orientation of the robot
void RobotOrientationDriver::setTargetOrientation(const double angle, const bool from_teleop, const double velocity, const bool drive_in_teleop)
{
	if (robot_enabled_)
	{
		// Don't motion profile if we're in teleop mode
		// Don't use internal motion profile if velocity is priovided
		//   by an external node already calculating the velocity as
		//   (hopefully) part of its own profiling
		// TODO - a check on delta angle - don't profile small motions?
		if (from_teleop || (velocity != 0))
		{
			orientation_profile_.setOrientationTarget(OrientationState(angle, velocity));
		}
		else
		{
			orientation_profile_.createProfile(robot_orientation_, angle);
		}
	}
	else
	{
		//ROS_ERROR_STREAM_THROTTLE(2, "=======ROBOT DISABLED=======");
		// If the robot is disabled, set the desired orientation to the
		// current orientation to prevent the robot from snapping to a
		// random angle when reenabled
		orientation_profile_.setOrientationTarget(OrientationState(robot_orientation_, 0));
	}
	most_recent_is_teleop_ = from_teleop || drive_in_teleop;

	// Reset the "non-teleop mode has timed-out" timer
	if (!from_teleop)
	{
		most_recent_teleop_timer_.stop();
		most_recent_teleop_timer_.start();
	}
}

// Set the desired orientation to the current IMU orientation
// to stop the robot at the current orientation
void RobotOrientationDriver::stopRotation(void)
{
	setTargetOrientation(robot_orientation_);
}

void RobotOrientationDriver::orientationCmdCallback(const std_msgs::Float64::ConstPtr &orient_msg)
{
	setTargetOrientation(orient_msg->data, false);
}

void RobotOrientationDriver::velocityOrientationCmdCallback(const teleop_orientation_msgs::TeleopOrientation::ConstPtr &orient_msg)
{
	// Using the drive_from_teleop flag will let the robot know to use a profile if velocity is 0.
	setTargetOrientation(orient_msg->position, false, orient_msg->velocity, orient_msg->drive_from_teleop);
}

void RobotOrientationDriver::controlEffortCallback(const std_msgs::Float64::ConstPtr &control_effort)
{
	// Store a local copy of the output of the PID node. This
	// will be the z-orientation velocity from the PID controller
	orientation_command_effort_ = control_effort->data;
}

void RobotOrientationDriver::setRobotOrientation(const double angle)
{
	robot_orientation_ = angle;
	// If the robot is disabled, set the desired orientation to the
	// current orientation to prevent the robot from snapping to a
	// random angle when reenabled
	if (!robot_enabled_)
	{
		setTargetOrientation(robot_orientation_, false);
	}
	// Update PID plant state using this most recent
	// robot orientation value
	std_msgs::Float64 pid_state_msg;
	pid_state_msg.data = robot_orientation_;
	pid_state_pub_.publish(pid_state_msg);
}

void RobotOrientationDriver::setRobotEnabled(const bool enabled)
{
	robot_enabled_ = enabled;
	// If the robot is disabled, set the desired orientation to the
	// current orientation to prevent the robot from snapping to a
	// random angle when reenabled
	if (!robot_enabled_)
	{
		setTargetOrientation(robot_orientation_, false);
	}
}

double RobotOrientationDriver::getCurrentOrientation(void) const 
{
	return robot_orientation_; 
}

double RobotOrientationDriver::getTargetOrientation(void)
{
	return orientation_profile_.getOrientationState().position;
}

bool RobotOrientationDriver::getRobotEnabled(void) const
{
	return robot_enabled_;
}

double RobotOrientationDriver::getOrientationVelocityPIDOutput(void) const
{
	return orientation_command_effort_;
}

bool RobotOrientationDriver::mostRecentCommandIsFromTeleop(void) const
{
	return most_recent_is_teleop_;
}

void RobotOrientationDriver::imuCallback(const sensor_msgs::Imu &imuState)
{
	const tf2::Quaternion imuQuat(imuState.orientation.x, imuState.orientation.y, imuState.orientation.z, imuState.orientation.w);
	double roll;
	double pitch;
	double yaw;
	tf2::Matrix3x3(imuQuat).getRPY(roll, pitch, yaw);

	if (std::isfinite(yaw)) // ignore NaN results
	{
		setRobotOrientation(yaw);
	}
}

void RobotOrientationDriver::matchStateCallback(const frc_msgs::MatchSpecificData &msg)
{
	// TODO : if in diagnostic mode, zero all outputs on the
	// transition from enabled to disabled
	setRobotEnabled(msg.Enabled);
}

// Used to time-out control from external sources and return
// it to teleop after messages from the external sources stop
void RobotOrientationDriver::checkFromTeleopTimeout(const ros::TimerEvent & /*event*/)
{
	most_recent_is_teleop_ = true;
}

bool RobotOrientationDriver::holdTargetOrientation(teleop_joystick_control::AlignToOrientation::Request &req,
												   teleop_joystick_control::AlignToOrientation::Response & /* res*/)
{
	setTargetOrientation(req.angle, true /* from telop */);
	return true;
}

bool RobotOrientationDriver::isJoystickOverridden() const {
	return joystick_overridden_;
}

void RobotOrientationDriver::setJoystickOverride(bool should_override) {
	joystick_overridden_ = should_override;
}