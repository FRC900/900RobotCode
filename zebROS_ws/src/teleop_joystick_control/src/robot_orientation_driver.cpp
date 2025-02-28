// TODO - consider tying pid enable pub to robot enabled?
#include <angles/angles.h>
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
	// one_shot = true, auto_start = false
	// inversting that
	, most_recent_teleop_timer_{nh_.createTimer(RESET_TO_TELEOP_CMDVEL_TIMEOUT, &RobotOrientationDriver::checkFromTeleopTimeout, this, false, true)}
{
	std_msgs::Bool enable_pub_msg;
	enable_pub_msg.data = true;
	pid_enable_pub_.publish(enable_pub_msg);
}


void RobotOrientationDriver::setTargetOrientation(const double angle, const bool from_teleop, const double velocity)
{
	//ROS_INFO_STREAM("Setting orientation with from teleop =" << from_teleop);
	if (robot_enabled_)
	{
		target_orientation_ = angle;
	}
	else
	{
		//ROS_ERROR_STREAM_THROTTLE(2, "=======ROBOT DISABLED=======");
		// If the robot is disabled, set the desired orientation to the
		// current orientation to prevent the robot from snapping to a
		// random angle when reenabled
		target_orientation_ = robot_orientation_;
	}
	most_recent_is_teleop_ = from_teleop;
	// Publish desired robot orientation to the PID node
	pid_velocity_msg::PIDVelocity pid_setpoint_msg;
	target_orientation_ = angles::normalize_angle(target_orientation_);
	pid_setpoint_msg.position = target_orientation_;
	pid_setpoint_msg.velocity = velocity;
	//ROS_INFO_STREAM_THROTTLE(2, "Publishing pid setpoid with value " << pid_setpoint_msg);
	pid_setpoint_pub_.publish(pid_setpoint_msg);

	//ROS_INFO_STREAM(__FUNCTION__ << "pub setpoint = " << pid_setpoint_msg.data );
	// Make sure the PID node is enabled

	// Reset the "non-teleop mode has timed-out" timer
	if (!from_teleop)
	{
		most_recent_teleop_timer_.stop();
		most_recent_teleop_timer_.start();
	}

	//ROS_INFO_STREAM("Target orientation = " << target_orientation_ << " state = " << robot_orientation_);
	//ROS_INFO_STREAM(__FUNCTION__ << "pub enable = " << (int)enable_pub_msg.data );
}

// Set the desired orientation to the current IMU orientation
// to stop the robot at the current orientation
void RobotOrientationDriver::stopRotation(void)
{
	setTargetOrientation(robot_orientation_);
}

void RobotOrientationDriver::orientationCmdCallback(const std_msgs::Float64::ConstPtr &orient_msg)
{
	//ROS_INFO_STREAM(__FUNCTION__ << " angle = " << orient_msg->data);
	setTargetOrientation(orient_msg->data, false);
}

void RobotOrientationDriver::velocityOrientationCmdCallback(const teleop_orientation_msgs::TeleopOrientation::ConstPtr &orient_msg)
{
	//ROS_INFO_STREAM(__FUNCTION__ << " angle = " << orient_msg->data);
	setTargetOrientation(orient_msg->position, orient_msg->drive_from_teleop, orient_msg->velocity);
}

void RobotOrientationDriver::controlEffortCallback(const std_msgs::Float64::ConstPtr &control_effort)
{
	// Store a local copy of the output of the PID node. This
	// will be the z-orientation velocity from the PID controller
	orientation_command_effort_ = control_effort->data;
}

void RobotOrientationDriver::setRobotOrientation(const double angle)
{
	//ROS_INFO_STREAM(__FUNCTION__ << " angle = " << angle);
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
	//ROS_INFO_STREAM(__FUNCTION__ << " robot_enabled = " << enabled);
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

double RobotOrientationDriver::getTargetOrientation(void) const
{
	return target_orientation_;
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