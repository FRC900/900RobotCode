// Single node which holds the targer orientation
// of the robot drive base and runs a velocity
// PID loop to turn that desired orientation into
// a Z-orientation velocity which can be fed to the
// drive base
//
// Orientation commands can come from two sources
//  - the teleop node, in response to joystick commands
//  - from various automated actions
//
// This code is built into the teleop node, so joystick
// inputs take the form of direct calls to either
// setTargetOrientation or incrementTargetOrientation.
// The first is used to snap the robot to a specific angle,
// the second is used to rotate the robot an amount from the
// current position.
// The teleop code can likewise just call
// getOrientationVelocityPIDOutput to get the z rotational
// velocity to build a twist message
// Automated actions can publish messages to the command
// topic this node advertises.  They can then read back
// the commanded velocity published from the attached
// PID node and combine that with whatever they're using
// for X&Y linear velocity
// Since the code paths for teleop vs. automated actions
// are different, this class can keep track of the source
// of the most recent orientation command. That way, the teleop
// node can decide if it should publish results or if the
// results will be handled by the automated action which
// requested them.
#ifndef INC_ROBOT_ORIENTATION_DRIVER_H
#define INC_ROBOT_ORIENTATION_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#include "frc_msgs/MatchSpecificData.h"

constexpr double INITIAL_ROBOT_ORIENTATION = M_PI / 2.0;
class RobotOrientationDriver
{
public:
	RobotOrientationDriver(const ros::NodeHandle &nh);

	void setTargetOrientation(double angle, bool from_teleop = true);
	void stopRotation(void);
	// subscriber to read setTargetOrientation as needed

	// Called from callbacks in teleop code
	void setRobotOrientation(double angle);
	void setRobotEnabled(bool enabled);

	// For debugging?
	double getTargetOrientation(void) const;

	// Used to generate cmd_vel message in teleop code
	double getOrientationVelocityPIDOutput(void) const;

	double getCurrentOrientation(void) const;
	bool   getRobotEnabled(void) const;

	// Publisher to publish orientation? Or can this be read from PID node?
	// Timer to publish to PID nodes? Or only in callbacks from odom yaw or in response to set/inc orientation?
	// match data subscriber
	//
	bool mostRecentCommandIsFromTeleop(void) const;

private:
	ros::NodeHandle nh_;
	ros::Subscriber orientation_command_sub_;
	ros::Publisher  pid_enable_pub_;
	ros::Publisher  pid_state_pub_;    // current IMU orientation
	ros::Publisher  pid_setpoint_pub_; // desired robot orientation
	ros::Publisher  cmd_vel_pub_;      // for when we are in teleop mode and want normal robot control
	ros::Subscriber pid_control_effort_sub_; // PID output - angular Z velocity
	ros::Subscriber imu_sub_; // subscriber to zereod yaw output
	ros::Subscriber match_data_sub_; // subscriber to match data, used to get enable/disable

	// The current orientation (angular-Z) setpoint for the drive base
	double target_orientation_{INITIAL_ROBOT_ORIENTATION};

	// The current orientation of the robot itself
	double robot_orientation_{INITIAL_ROBOT_ORIENTATION};

	// Current angular-Z velocity setting from PID node output
	double orientation_command_effort_{0.0};

	// Robot enabled/disabled state from DS
	bool robot_enabled_{false};

	// True if most recent target sent to PID is from teleop commands
	bool most_recent_is_teleop_{false};

	void orientationCmdCallback(const std_msgs::Float64::ConstPtr &orientation_cmd);
	void controlEffortCallback(const std_msgs::Float64::ConstPtr &control_effort);
	void imuCallback(const sensor_msgs::Imu &imuState);
	void matchStateCallback(const frc_msgs::MatchSpecificData &msg);
};

#endif