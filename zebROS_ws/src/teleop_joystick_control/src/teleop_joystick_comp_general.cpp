// Toggle between rotation using rightStickX and rightTrigger - leftTrigger
#define ROTATION_WITH_STICK

#include "ros/ros.h"
#include "frc_msgs/JoystickState.h"
//#define NEED_JOINT_STATES
#ifdef NEED_JOINT_STATES
#include "sensor_msgs/JointState.h"
#endif
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include <string>
#include <cmath>
#include <boost/bind.hpp>

#include "std_srvs/Empty.h"

#include <vector>
#include "teleop_joystick_control/RobotOrient.h"

#include "frc_msgs/MatchSpecificData.h"

#include "actionlib/client/simple_action_client.h"

#include "ddynamic_reconfigure/ddynamic_reconfigure.h"

#include "teleop_joystick_control/RobotOrientationDriver.h"
#include <std_srvs/SetBool.h>

#include "teleop_joystick_control/teleop_joystick_comp_general.h"

#include "imu_zero_msgs/ImuZeroAngle.h"
#include "talon_swerve_drive_controller_msgs/SetXY.h"

struct DynamicReconfigVars config;

bool diagnostics_mode = false;

ros::ServiceClient ParkSrv;
ros::ServiceClient IMUZeroSrv;
ros::ServiceClient SwerveOdomZeroSrv;

bool joystick1_left_trigger_pressed = false;
bool joystick1_right_trigger_pressed = false;

std::unique_ptr<Driver> driver;

uint8_t alliance_color{};
bool called_park_endgame = false;

void matchStateCallback(const frc_msgs::MatchSpecificData &msg)
{
	// TODO : if in diagnostic mode, zero all outputs on the
	// transition from enabled to disabled
	alliance_color = msg.allianceColor;
	if (!called_park_endgame && msg.matchTimeRemaining < config.match_time_to_park && msg.Autonomous == false && msg.Enabled == true && msg.matchTimeRemaining > 0) {
		// check for enabled and time != 0 so we don't trigger when the node starts (time defaults to 0, auto defaults to false)
		std_srvs::SetBool setBool;
		setBool.request.data = true;
		if (!ParkSrv.call(setBool))
		{
			ROS_ERROR("ParkSrv call failed in matchStateCallback");
		} else {
			called_park_endgame = true;
			ROS_INFO("ParkSrv called");
		}
	}
}

ros::ServiceClient setCenterSrv;

Driver::Driver(ros::NodeHandle n, DynamicReconfigVars config) 
	: teleop_cmd_vel_{TeleopCmdVel<DynamicReconfigVars>(config)}
	, JoystickRobotVel_{n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1)}
	, BrakeSrv_{n.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, {{"tcp_nodelay", "1"}})}
	, robot_orientation_driver_{RobotOrientationDriver(n)}
{
	teleop_cmd_vel_.setRobotOrient(false, 0.0);
	teleop_cmd_vel_.resetCaps();
}

bool Driver::orientCallback(teleop_joystick_control::RobotOrient::Request& req,
		teleop_joystick_control::RobotOrient::Response&/* res*/)
{
	// Used to switch between robot orient and field orient driving
	teleop_cmd_vel_.setRobotOrient(req.robot_orient, req.offset_angle);
	ROS_WARN_STREAM("Robot Orient = " << req.robot_orient << ", Offset Angle = " << req.offset_angle);
	return true;
}

bool Driver::waitForBrakeSrv(ros::Duration startup_wait_time) {
	return BrakeSrv_.waitForExistence(startup_wait_time);
}

bool Driver::getNoDriverInput() {
	return no_driver_input_;
}

void Driver::moveDirection(int x, int y, int z, double button_move_speed) {
	geometry_msgs::Twist cmd_vel;
	direction_x_ += x;
	direction_y_ += y;
	direction_z_ += z;

	sendDirection(button_move_speed);
}

void Driver::sendDirection(double button_move_speed) {
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = direction_x_ * button_move_speed;
	cmd_vel.linear.y = direction_y_ * button_move_speed;
	cmd_vel.linear.z = direction_z_ * button_move_speed;
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;
	cmd_vel.angular.z = 0.0;

	// avoid z output (ends up overriding this cmd_vel and we stutter)
	robot_orientation_driver_.setTargetOrientation(robot_orientation_driver_.getCurrentOrientation(), false);
	JoystickRobotVel_.publish(cmd_vel);
}

void Driver::setTargetOrientation(const double angle, const bool from_teleop, const double velocity) {
	robot_orientation_driver_.setTargetOrientation(angle, from_teleop, velocity);
}

void Driver::setJoystickOverride(bool override) {
	robot_orientation_driver_.setJoystickOverride(override);
}

ros::Time Driver::evalateDriverCommands(const frc_msgs::JoystickState &joy_state, const DynamicReconfigVars& config) {

	teleop_cmd_vel_.updateRateLimit(config);
	// TODO : make swerve invert the yaw so we can deal in ccw-positive angles
	// everywhere outside of that code
	geometry_msgs::Twist cmd_vel = teleop_cmd_vel_.generateCmdVel(joy_state, -robot_orientation_driver_.getCurrentOrientation(), config);

	if (robot_orientation_driver_.mostRecentCommandIsFromTeleop() || cmd_vel.linear.x != 0.0 || cmd_vel.linear.y != 0.0 || cmd_vel.angular.z != 0.0) {
		double original_angular_z = cmd_vel.angular.z;

		if (original_angular_z == 0.0 && old_angular_z_ != 0.0) {
			ROS_WARN_STREAM_THROTTLE(1, "Send set angle = false");
			sendSetAngle_ = false;
		}

		if (original_angular_z == 0.0 && !sendSetAngle_) {
			ROS_INFO_STREAM("Old angular z " << old_angular_z_ << " signbit " << signbit(old_angular_z_));
			double multiplier = 1;
			if (signbit(old_angular_z_)) {
				multiplier = -1;
			}
			if (old_angular_z_ == 0.0) {
				ROS_WARN_STREAM("Old angular z is zero, wierd");
			}
			ROS_INFO_STREAM("Locking to current orientation!");
			robot_orientation_driver_.setTargetOrientation(robot_orientation_driver_.getCurrentOrientation(), true /* from telop */);
			sendSetAngle_ = true;
		}
		ROS_INFO_STREAM_THROTTLE(1, "CMD_VEL angular z" << cmd_vel.angular.z);

		if (cmd_vel.angular.z == 0.0 || robot_orientation_driver_.isJoystickOverridden())
		{
			cmd_vel.angular.z = robot_orientation_driver_.getOrientationVelocityPIDOutput();
			if (fabs(cmd_vel.angular.z) < config.rotation_epsilon) {
				// COAST MODE
				//cmd_vel.angular.z = 0.001 * (cmd_vel.angular.z > 0 ? 1 : -1);
				cmd_vel.angular.z = 0.0;
			}
		}

		/*original_angular_z == 0.0*/ 
		if((cmd_vel.linear.x == 0.0) && (cmd_vel.linear.y == 0.0) && ( cmd_vel.angular.z == 0.0 ) && !sendRobotZero_)
		{
			no_driver_input_ = true;
			if (std_srvs::Empty empty; !BrakeSrv_.call(empty))
			{
				ROS_ERROR("BrakeSrv call failed in sendRobotZero_");
			}
			ROS_INFO("BrakeSrv called");

			JoystickRobotVel_.publish(cmd_vel);
			sendRobotZero_ = true;
		}
		// 0.002 is slightly more than the 0.001 we set for coast mode
		else if((cmd_vel.linear.x != 0.0) || (cmd_vel.linear.y != 0.0) || (fabs(cmd_vel.angular.z) >= 0.002))
		{
			if (alliance_color == frc_msgs::MatchSpecificData::ALLIANCE_COLOR_RED) {
				// because map-relative forward (which we now use) is from blue -> red, we need to invert cmd_vel for red driving.
				cmd_vel.linear.x *= -1;
				cmd_vel.linear.y *= -1;
			}

			JoystickRobotVel_.publish(cmd_vel);
			sendRobotZero_ = false;
			no_driver_input_ = false;
			// if the original command was not zero, then teleop was controlling rotation

		}
		old_angular_z_ = original_angular_z;
	}
	return joy_state.header.stamp;
}

void publish_diag_cmds(void)
{
	ROS_WARN("Called unimplemented function \"publish_diag_cmds\"");
	// should publish commands to the diagnostic mode nodes
}

void zero_all_diag_commands(void)
{
	ROS_WARN("Called unimplemented function \"zero_all_diag_commands\"");
	// should zero out all diagnostic mode commands
}

void preemptActionlibServers(void)
{
	ROS_WARN_STREAM("Preempting ALL actionlib servers!");
	// dont forget to add new actionlib servers here
}

void buttonBoxCallback(const ros::MessageEvent<std_msgs::Bool const>& event)
{
	ROS_WARN("Called unimplemented function \"buttonBoxCallback\"");
}

#ifdef NEED_JOINT_STATES
void jointStateCallback(const sensor_msgs::JointState &joint_state)
{
	ROS_WARN("Called unimplemented function \"jointStateCallback\" ");
	// TODO - remove this if not used
}
#endif


void TeleopInitializer::add_custom_var(DDRVariable var) {
	custom_vars.push_back(var);
}

void TeleopInitializer::set_n_params(ros::NodeHandle n_params) {
	n_params_ = n_params;
}

void TeleopInitializer::set_n(ros::NodeHandle n) {
	n_= n;
}

void TeleopInitializer::init() {
	int num_joysticks = 1;
	if(!n_params_.getParam("num_joysticks", num_joysticks))
	{
		ROS_ERROR("Could not read num_joysticks in teleop_joystick_comp");
	}
	if(!n_params_.getParam("joystick_deadzone", config.joystick_deadzone))
	{
		ROS_ERROR("Could not read joystick_deadzone in teleop_joystick_comp");
	}
	if(!n_params_.getParam("radial_deadzone", config.radial_deadzone))
	{
		ROS_ERROR("Could not read radial_deadzone in teleop_joystick_comp");
	}
	if(!n_params_.getParam("joystick_pow", config.joystick_pow))
	{
		ROS_ERROR("Could not read joystick_pow in teleop_joystick_comp");
	}
	if(!n_params_.getParam("rotation_pow", config.rotation_pow))
	{
		ROS_ERROR("Could not read rotation_pow in teleop_joystick_comp");
	}
	if(!n_params_.getParam("min_speed", config.min_speed))
	{
		ROS_ERROR("Could not read min_speed in teleop_joystick_comp");
	}
	if(!n_params_.getParam("max_speed", config.max_speed))
	{
		ROS_ERROR("Could not read max_speed in teleop_joystick_comp");
	}
	if(!n_params_.getParam("max_speed_slow", config.max_speed_slow))
	{
		ROS_ERROR("Could not read max_speed_slow in teleop_joystick_comp");
	}
	if(!n_params_.getParam("button_move_speed", config.button_move_speed))
	{
		ROS_ERROR("Could not read button_move_speed in teleop_joystick_comp");
	}
	if(!n_params_.getParam("max_rot", config.max_rot))
	{
		ROS_ERROR("Could not read max_rot in teleop_joystick_comp");
	}
	if(!n_params_.getParam("max_rot_slow", config.max_rot_slow))
	{
		ROS_ERROR("Could not read max_rot_slow in teleop_joystick_comp");
	}
	if(!n_params_.getParam("drive_rate_limit_time", config.drive_rate_limit_time))
	{
		ROS_ERROR("Could not read drive_rate_limit_time in teleop_joystick_comp");
	}
	if(!n_params_.getParam("rotate_rate_limit_time", config.rotate_rate_limit_time))
	{
		ROS_ERROR("Could not read rotate_rate_limit_time in teleop_joystick_comp");
	}
	if(!n_params_.getParam("trigger_threshold", config.trigger_threshold))
	{
		ROS_ERROR("Could not read trigger_threshold in teleop_joystick_comp");
	}
	if(!n_params_.getParam("stick_threshold", config.stick_threshold))
	{
		ROS_ERROR("Could not read stick_threshold in teleop_joystick_comp");
	}
	if(!n_params_.getParam("imu_zero_angle", config.imu_zero_angle))
	{
		ROS_ERROR("Could not read imu_zero_angle in teleop_joystick_comp");
	}
	if(!n_params_.getParam("rotation_axis_scale", config.rotation_axis_scale))
	{
		ROS_ERROR("Could not read rotation_axis_scale in teleop_joystick_comp");
	}
	if(!n_params_.getParam("rotation_epsilon", config.rotation_epsilon))
	{
		ROS_ERROR("Could not read rotation_epsilon in teleop_joystick_comp");
	}
	if(!n_params_.getParam("match_time_to_park", config.match_time_to_park))
	{
		ROS_ERROR("Could not read match_time_to_park in teleop_joystick_comp");
	}

	ddynamic_reconfigure::DDynamicReconfigure ddr(n_params_);

	ddr.registerVariable<double>("joystick_deadzone", &config.joystick_deadzone, "Joystick deadzone, in percent", 0., 1.);
	ddr.registerVariable<double>("radial_deadzone", &config.radial_deadzone, "Radial deadzone, in radians", 0., M_PI/4);
	ddr.registerVariable<double>("min_speed", &config.min_speed, "Min linear speed to get robot to overcome friction, in m/s", 0, 1);
	ddr.registerVariable<double>("max_speed", &config.max_speed, "Max linear speed, in m/s", 0, 10.);
	ddr.registerVariable<double>("max_speed_slow", &config.max_speed_slow, "Max linear speed in slow mode, in m/s", 0., 5.);
	ddr.registerVariable<double>("max_rot", &config.max_rot, "Max angular speed", 0., 13.);
	ddr.registerVariable<double>("max_rot_slow", &config.max_rot_slow, "Max angular speed in slow mode", 0., 3.);
	ddr.registerVariable<double>("button_move_speed", &config.button_move_speed, "Linear speed when move buttons are pressed, in m/s", 0, 5);
	ddr.registerVariable<double>("joystick_pow", &config.joystick_pow, "Joystick Scaling Power, linear", 0., 5.);
	ddr.registerVariable<double>("rotation_pow", &config.rotation_pow, "Joystick Scaling Power, rotation", 0., 5.);
	ddr.registerVariable<double>("drive_rate_limit_time", &config.drive_rate_limit_time, "msec to go from full back to full forward", 0., 2000.);
	ddr.registerVariable<double>("rotate_rate_limit_time", &config.rotate_rate_limit_time, "msec to go from full counterclockwise to full clockwise", 0., 2000.);
	ddr.registerVariable<double>("trigger_threshold", &config.trigger_threshold, "Amount trigger has to be pressed to trigger action", 0., 1.);
	ddr.registerVariable<double>("stick_threshold", &config.stick_threshold, "Amount stick has to be moved to trigger diag mode action", 0., 1.);
	ddr.registerVariable<double>("imu_zero_angle", &config.imu_zero_angle, "Value to pass to imu/set_zero when zeroing", -360., 360.);
	ddr.registerVariable<double>("rotation_epsilon", &config.rotation_epsilon, "rotation_epsilon", 0.0, 1.0);
	ddr.registerVariable<double>("match_time_to_park", &config.match_time_to_park, "match_time_to_park", 0.0, 60.0);

	// register custom variables
	for (DDRVariable& var : this->custom_vars) {
		ddr.registerVariable<double>(var.name, &var.value, var.description, var.min, var.max);
	}

	ddr.publishServicesTopics();

	driver = std::make_unique<Driver>(n_, config);

	const std::map<std::string, std::string> service_connection_header{{"tcp_nodelay", "1"}};

	ParkSrv = n_.serviceClient<std_srvs::SetBool>("/frcrobot_jetson/swerve_drive_controller/toggle_park", false, service_connection_header);
	IMUZeroSrv = n_.serviceClient<imu_zero_msgs::ImuZeroAngle>("/imu/set_imu_zero", false, service_connection_header);
	setCenterSrv = n_.serviceClient<talon_swerve_drive_controller_msgs::SetXY>("/frcrobot_jetson/swerve_drive_controller/change_center_of_rotation", false, service_connection_header);	
	SwerveOdomZeroSrv = n_.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/reset_odom", false, service_connection_header);
#ifdef NEED_JOINT_STATES
	ros::Subscriber joint_states_sub = n_.subscribe("/frcrobot_jetson/joint_states", 1, &jointStateCallback);
#endif
	//ros::Subscriber talon_states_sub = n.subscribe("/frcrobot_jetson/talonfxpro_states", 1, &talonFXProStateCallback);
	ros::Subscriber match_state_sub = n_.subscribe("/frcrobot_rio/match_data", 1, matchStateCallback);
	ros::ServiceServer robot_orient_service = n_.advertiseService("robot_orient", &Driver::orientCallback, driver.get());

	const ros::Duration startup_wait_time_secs(15);
	const ros::Time startup_start_time = ros::Time::now();
	ros::Duration startup_wait_time;
	startup_wait_time = std::max(startup_wait_time_secs - (ros::Time::now() - startup_start_time), ros::Duration(0.1));
	if(!driver->waitForBrakeSrv(startup_wait_time))
	{
		ROS_ERROR("Wait (15 sec) timed out, for Brake Service in teleop_joystick_comp.cpp");
	}

	startup_wait_time = std::max(startup_wait_time_secs - (ros::Time::now() - startup_start_time), ros::Duration(0.1));
	if(!IMUZeroSrv.waitForExistence(startup_wait_time))
	{
		ROS_ERROR("Wait (1 sec) timed out, for IMU Zero Service in teleop_joystick_comp.cpp");
	}

	//Read from _num_joysticks joysticks
	// Set up this callback last, since it might use all of the various stuff
	// initialized above here. Setting it up first risks the chance that a callback
	// happens immediately and tries to use them before they have valid values

	std::vector <ros::Subscriber> subscriber_array;
	for(int j = 0; j < num_joysticks; j++)
	{
		std::stringstream s;
		s << "/frcrobot_rio/joystick_states";
		s << (j+1);
		subscriber_array.push_back(n_.subscribe<frc_msgs::JoystickState>(s.str(), 1, boost::bind(evaluateCommands, _1, j)));
	}

	ROS_WARN("joy_init");

	ros::spin();
}