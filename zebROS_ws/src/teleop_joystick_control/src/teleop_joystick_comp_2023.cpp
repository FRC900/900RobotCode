// Toggle between rotation using rightStickX and rightTrigger - leftTrigger
#define ROTATION_WITH_STICK


#include "ros/ros.h"
#include "frc_msgs/JoystickState.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <cmath>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

#include "std_srvs/Empty.h"

#include <vector>
#include "teleop_joystick_control/RobotOrient.h"
#include "teleop_joystick_control/OrientStrafingAngle.h"

#include "frc_msgs/ButtonBoxState.h"
#include "frc_msgs/MatchSpecificData.h"

#include "actionlib/client/simple_action_client.h"

#include "dynamic_reconfigure_wrapper/dynamic_reconfigure_wrapper.h"
#include "teleop_joystick_control/TeleopJoystickComp2023Config.h"
#include "teleop_joystick_control/TeleopJoystickCompDiagnostics2023Config.h"

#include "teleop_joystick_control/TeleopCmdVel.h"
#include "behavior_actions/AutoMode.h"

#include "path_follower_msgs/holdPositionAction.h"

#include <imu_zero/ImuZeroAngle.h>

#include <math.h>

std::unique_ptr<TeleopCmdVel<teleop_joystick_control::TeleopJoystickComp2023Config>> teleop_cmd_vel;

bool diagnostics_mode = false;

double orient_strafing_angle = 0; // set from service call anyway

frc_msgs::ButtonBoxState button_box;

// array of joystick_states messages for multiple joysticks
std::vector <frc_msgs::JoystickState> joystick_states_array;
std::vector <std::string> topic_array;

ros::Publisher orient_strafing_enable_pub;
ros::Publisher orient_strafing_setpoint_pub;
ros::Publisher orient_strafing_state_pub;

teleop_joystick_control::TeleopJoystickComp2023Config config;
teleop_joystick_control::TeleopJoystickCompDiagnostics2023Config diagnostics_config;

ros::Publisher JoystickRobotVel;

ros::ServiceClient BrakeSrv;
ros::ServiceClient IMUZeroSrv;

double imu_angle;

bool robot_is_disabled{false};
ros::Publisher auto_mode_select_pub;

bool joystick1_left_trigger_pressed = false;
bool joystick1_right_trigger_pressed = false;

// Diagnostic mode controls
imu_zero::ImuZeroAngle imu_cmd;


bool imu_service_needed = true;

ros::Publisher dynamic_arm_piston_;
double last_offset;
bool last_robot_orient;

// Diagnostic mode controls


int direction_x{};
int direction_y{};
int direction_z{};

void moveDirection(int x, int y, int z) {
	geometry_msgs::Twist cmd_vel;
	direction_x += x;
	direction_y += y;
	direction_z += z;
	cmd_vel.linear.x = direction_x * config.button_move_speed;
	cmd_vel.linear.y = direction_y * config.button_move_speed;
	cmd_vel.linear.z = direction_z * config.button_move_speed;
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;
	cmd_vel.angular.z = 0.0;

	JoystickRobotVel.publish(cmd_vel);
}

void sendDirection() {
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = direction_x * config.button_move_speed;
	cmd_vel.linear.y = direction_y * config.button_move_speed;
	cmd_vel.linear.z = direction_z * config.button_move_speed;
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;
	cmd_vel.angular.z = 0.0;

	JoystickRobotVel.publish(cmd_vel);
}

void publish_diag_cmds(void)
{
	// should publish commands to the diagnostic mode nodes
}

void zero_all_diag_commands(void)
{
	// should zero out all diagnostic mode commands
}

std::shared_ptr<actionlib::SimpleActionClient<path_follower_msgs::holdPositionAction>> distance_ac;


void imuCallback(const sensor_msgs::Imu &imuState)
{
	const tf2::Quaternion imuQuat(imuState.orientation.x, imuState.orientation.y, imuState.orientation.z, imuState.orientation.w);
	double roll;
	double pitch;
	double yaw;
	tf2::Matrix3x3(imuQuat).getRPY(roll, pitch, yaw);

	if (std::isfinite(yaw)) // ignore NaN results
	{
		imu_angle = -yaw;

		// Pass along yaw state to orient PID node
		std_msgs::Float64 imu_angle_msg;
		imu_angle_msg.data = yaw; // TBD - invert or not?
		orient_strafing_state_pub.publish(imu_angle_msg);
	}
}

void preemptActionlibServers(void)
{
	ROS_WARN_STREAM("Preempting ALL actionlib servers!");
	// dont forget to add new actionlib servers here
}

bool orientCallback(teleop_joystick_control::RobotOrient::Request& req,
		teleop_joystick_control::RobotOrient::Response&/* res*/)
{
	last_offset = req.offset_angle;
	last_robot_orient = req.robot_orient;
	// Used to switch between robot orient and field orient driving
	teleop_cmd_vel->setRobotOrient(req.robot_orient, req.offset_angle);
	ROS_WARN_STREAM("Robot Orient = " << req.robot_orient << ", Offset Angle = " << req.offset_angle);
	return true;
}

bool orientStrafingAngleCallback(teleop_joystick_control::OrientStrafingAngle::Request& req,
		teleop_joystick_control::OrientStrafingAngle::Response&/* res*/)
{
	orient_strafing_angle = req.angle;
	return true;
}

bool sendRobotZero = false;
bool snappingToAngle = false;

void buttonBoxCallback(const ros::MessageEvent<frc_msgs::ButtonBoxState const>& event)
{
	//ROS_INFO_STREAM("Button Box callback running!");

	button_box = *(event.getMessage());

	static ros::Time last_header_stamp = button_box.header.stamp;

	if(button_box.lockingSwitchPress)
	{
		// // Clear out pressed state when switching modes
		// // so that the press will be seen by the new mode
		// joystick1_left_trigger_pressed = false;
		// joystick1_right_trigger_pressed = false;
		// zero_all_diag_commands();
		//
		// // Disable snap-to-angle in diagnostics mode
		// std_msgs::Bool enable_align_msg;
		// enable_align_msg.data = false;
		// orient_strafing_enable_pub.publish(enable_align_msg);
		// diagnostics_mode = true;
		// ROS_WARN_STREAM("Enabling diagnostics mode!");
		teleop_cmd_vel->setRobotOrient(true, 0.0);
		ROS_WARN_STREAM("Robot relative mode!");
	}

	if(button_box.lockingSwitchButton)
	{
	}
	else
	{
	}

	if(button_box.lockingSwitchRelease)
	{
		// // Clear out pressed state when switching modes
		// // so that the press will be seen by the new mode
		// joystick1_left_trigger_pressed = false;
		// joystick1_right_trigger_pressed = false;
		// // Force a publish 0 to all the diag mode
		// // controllers here before switching out
		// // of diagnostics mode
		// zero_all_diag_commands();
		// publish_diag_cmds();
		// diagnostics_mode = false;
		// ROS_WARN_STREAM("Disabling diagnostics mode!");
		teleop_cmd_vel->setRobotOrient(false, 0.0);
		ROS_WARN_STREAM("Field relative mode!");
	}

	if(button_box.topRedPress)
	{
			preemptActionlibServers();
	}
	if(button_box.topRedButton)
	{
	}
	if(button_box.topRedRelease)
	{
	}

	if(button_box.leftRedPress)
	{

	}

	if(button_box.leftRedButton)
	{

	}
	if(button_box.leftRedRelease)
	{
	}

	if(button_box.rightRedPress)
	{


	}
	if(button_box.rightRedButton)
	{
	}
	if(button_box.rightRedRelease)
	{
	}

	if(button_box.leftSwitchUpPress)
	{
	}
	if(button_box.leftSwitchUpButton)
	{
	}
	if(button_box.leftSwitchUpRelease)
	{

	}

	if(button_box.leftSwitchDownPress)
	{
	}
	if(button_box.leftSwitchDownButton)
	{
	}
	if(button_box.leftSwitchDownRelease)
	{

	}

	if(button_box.rightSwitchUpPress)
	{
	}
	if(button_box.rightSwitchUpButton)
	{

	}
	if(button_box.rightSwitchDownButton)
	{

	}
	if(button_box.rightSwitchUpRelease)
	{

	}

	if(button_box.rightSwitchDownPress)
	{
	}
	if(button_box.rightSwitchDownButton)
	{

	}
	if(button_box.rightSwitchDownRelease)
	{

	}

	if(button_box.leftBluePress)
	{
	}
	if(button_box.leftBlueButton)
	{
	}
	if(button_box.leftBlueRelease)
	{
	}

	if(button_box.rightBluePress)
	{
	}
	if(button_box.rightBlueButton)
	{
	}
	if(button_box.rightBlueRelease)
	{
	}

	if(button_box.yellowPress)
	{
		/* 
		ROS_INFO_STREAM("Driving to fender at correct angle!");
		path_follower_msgs::holdPositionGoal goal;
		goal.pose.position.x = 0.3;
		goal.pose.position.y = 0.3;
		goal.pose.position.z = 0.0;
		tf2::Quaternion q;
		q.setRPY(0, 0, hub_angle);
		goal.pose.orientation.x = q.x();
		goal.pose.orientation.y = q.y();
		goal.pose.orientation.z = q.z();
		goal.pose.orientation.w = q.w();
		goal.isAbsoluteCoord = true;
		distance_ac->sendGoal(goal);
		*/
	}
	if(button_box.yellowButton)
	{
	}
	if(button_box.yellowRelease)
	{	
		/* 
		distance_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		sendRobotZero = false;
		*/
	}

	if(button_box.leftGreenPress)
	{
		moveDirection(0, 1, 0);
	}
	if(button_box.leftGreenButton)
	{
		sendDirection();
	}
	if(button_box.leftGreenRelease)
	{
		moveDirection(0, -1, 0);
	}

	if(button_box.rightGreenPress)
	{
		moveDirection(0, -1, 0);
	}
	if(button_box.rightGreenButton)
	{
		sendDirection();
	}
	if(button_box.rightGreenRelease)
	{
		moveDirection(0, 1, 0);
	}

	if(button_box.topGreenPress)
	{
		moveDirection(1, 0, 0);
	}
	if(button_box.topGreenButton)
	{
		sendDirection();
	}
	if(button_box.topGreenRelease)
	{
		moveDirection(-1, 0, 0);
	}

	if(button_box.bottomGreenPress)
	{
		moveDirection(-1, 0, 0);
	}
	if(button_box.bottomGreenButton)
	{
		sendDirection();
	}
	if(button_box.bottomGreenRelease)
	{
		moveDirection(1, 0, 0);
	}

	// Auto-mode select?
	if(button_box.bottomSwitchUpPress)
	{
		imu_service_needed = true;
	}
	if(button_box.bottomSwitchUpButton)
	{
		if (robot_is_disabled)
		{
			behavior_actions::AutoMode auto_mode_msg;
			auto_mode_msg.header.stamp = ros::Time::now();
			auto_mode_msg.auto_mode = 1;
			auto_mode_select_pub.publish(auto_mode_msg);
		}
		if(imu_service_needed){
			imu_cmd.request.angle = config.top_position_angle;
			IMUZeroSrv.call(imu_cmd);
			imu_service_needed = false;
		}
	}
	if(button_box.bottomSwitchUpRelease)
	{
		imu_service_needed = true;
	}

	if(button_box.bottomSwitchDownPress)
	{
		imu_service_needed = true;
	}
	if(button_box.bottomSwitchDownButton)
	{
		if (robot_is_disabled)
		{
			behavior_actions::AutoMode auto_mode_msg;
			auto_mode_msg.header.stamp = ros::Time::now();
			auto_mode_msg.auto_mode = 2;
			auto_mode_select_pub.publish(auto_mode_msg);
		}
		if(imu_service_needed){
			imu_cmd.request.angle = config.bottom_position_angle;
			IMUZeroSrv.call(imu_cmd);
			imu_service_needed = false;
			}
	}
	if(button_box.bottomSwitchDownRelease)
	{
		imu_service_needed = true;
	}
	if(!button_box.bottomSwitchUpButton && !button_box.bottomSwitchDownButton){ //The switch is in the middle position
		if(imu_service_needed){
			imu_cmd.request.angle = config.middle_position_angle;
			IMUZeroSrv.call(imu_cmd);
			imu_service_needed = false;
		}
	}

	last_header_stamp = button_box.header.stamp;
	if (diagnostics_mode)
	{
		publish_diag_cmds();
	}
}

void evaluateCommands(const ros::MessageEvent<frc_msgs::JoystickState const>& event)
{
	//So the code can use specific joysticks
	int joystick_id = -1;

	const ros::M_string &header = event.getConnectionHeader();
	const std::string topic = header.at("topic");

	//Identifies the incoming message as the correct joystick based on the topic the message was recieved from
	for(size_t i = 0; (i < topic_array.size()); i++)
	{
		if(topic == topic_array[i])
		{
			joystick_states_array[i] = *(event.getMessage());
			joystick_id = i;
			break;
		}
	}

	if(joystick_id == -1)
	{
		ROS_ERROR("Joystick message topic not identified. Teleop callback failed.");
		return;
	}

	//Only do this for the first joystick
	if(joystick_id == 0)
	{
		//ROS_INFO_STREAM("js0 callback running!");

		static ros::Time last_header_stamp = joystick_states_array[0].header.stamp;

		//ROS_INFO_STREAM("leftStick = " << joystick_states_array[0].leftStickX << " " << joystick_states_array[0].leftStickY);
		teleop_cmd_vel->updateRateLimit(config);
		geometry_msgs::Twist cmd_vel = teleop_cmd_vel->generateCmdVel(joystick_states_array[0], imu_angle, config);
		//ROS_INFO_STREAM("cmd_vel out " << cmd_vel.linear.x << " " << cmd_vel.linear.y << " " << cmd_vel.linear.z);

		if (cmd_vel.angular.z != 0.0) {
			if (snappingToAngle) {
				// Disable angle snapping, if enabled
				std_msgs::Bool enable_align_msg;
				enable_align_msg.data = false;
				orient_strafing_enable_pub.publish(enable_align_msg);
			}
		} else {
			if (snappingToAngle) {
				// Re-enable angle snapping, if enabled
				std_msgs::Bool enable_align_msg;
				enable_align_msg.data = true;
				orient_strafing_enable_pub.publish(enable_align_msg);
			}
		}

		if((cmd_vel.linear.x == 0.0) && (cmd_vel.linear.y == 0.0) && (cmd_vel.angular.z == 0.0) && !sendRobotZero)
		{
			std_srvs::Empty empty;
			if (!BrakeSrv.call(empty))
			{
				ROS_ERROR("BrakeSrv call failed in sendRobotZero_");
			}
			ROS_INFO("BrakeSrv called");

			JoystickRobotVel.publish(cmd_vel);
			sendRobotZero = true;
		}
		else if((cmd_vel.linear.x != 0.0) || (cmd_vel.linear.y != 0.0) || (cmd_vel.angular.z != 0.0))
		{
			//ROS_INFO_STREAM("Publishing " << cmd_vel.linear.x << " " << cmd_vel.linear.y << " " << cmd_vel.linear.z);
			JoystickRobotVel.publish(cmd_vel);
			sendRobotZero = false;
		}

		if(!diagnostics_mode)
		{
			//Joystick1: buttonA
			if(joystick_states_array[0].buttonAPress)
			{

			}
			if(joystick_states_array[0].buttonAButton)
			{
			}
			if(joystick_states_array[0].buttonARelease)
			{

			}

			//Joystick1: buttonB
			if(joystick_states_array[0].buttonBPress)
			{

			}

			if(joystick_states_array[0].buttonBButton)
			{
			}
			if(joystick_states_array[0].buttonBRelease)
			{

			}

			//Joystick1: buttonX
			if(joystick_states_array[0].buttonXPress)
			{

			}
			if(joystick_states_array[0].buttonXButton)
			{
			}
			if(joystick_states_array[0].buttonXRelease)
			{
			}

			//Joystick1: buttonY
			if(joystick_states_array[0].buttonYPress)
			{

			}
			if(joystick_states_array[0].buttonYButton)
			{
			}
			if(joystick_states_array[0].buttonYRelease)
			{

			}

			//Joystick1: bumperLeft
			if(joystick_states_array[0].bumperLeftPress)
			{
				teleop_cmd_vel->setSlowMode(true);
			}
			if(joystick_states_array[0].bumperLeftButton)
			{
			}
			if(joystick_states_array[0].bumperLeftRelease)
			{
				teleop_cmd_vel->setSlowMode(false);
			}

			//Joystick1: bumperRight
			if(joystick_states_array[0].bumperRightPress)
			{
			}
			if(joystick_states_array[0].bumperRightButton)
			{
			}
			if(joystick_states_array[0].bumperRightRelease)
			{
			}

			//Joystick1: directionLeft
			if(joystick_states_array[0].directionLeftPress)
			{

			}
			if(joystick_states_array[0].directionLeftButton)
			{

			}
			else
			{
			}
			if(joystick_states_array[0].directionLeftRelease)
			{

			}

			//Joystick1: directionRight
			if(joystick_states_array[0].directionRightPress)
			{
			}
			if(joystick_states_array[0].directionRightButton)
			{
			}
			if(joystick_states_array[0].directionRightRelease)
			{
			}

			//Joystick1: directionUp
			if(joystick_states_array[0].directionUpPress)
			{
			}
			if(joystick_states_array[0].directionUpButton)
			{
			}
			if(joystick_states_array[0].directionUpRelease)
			{
			}

			//Joystick1: directionDown
			if(joystick_states_array[0].directionDownPress)
			{
			}
			if(joystick_states_array[0].directionDownButton)
			{
			}
			if(joystick_states_array[0].directionDownRelease)
			{
			}

			//Joystick1: stickLeft
			if(joystick_states_array[0].stickLeftPress)
			{
			}
			if(joystick_states_array[0].stickLeftButton)
			{
			}
			else
			{
			}
			if(joystick_states_array[0].stickLeftRelease)
			{
			}

#ifdef ROTATION_WITH_STICK
			if(joystick_states_array[0].leftTrigger > config.trigger_threshold)
			{
				if(!joystick1_left_trigger_pressed)
				{
				}

				joystick1_left_trigger_pressed = true;
			}
			else
			{
				if(joystick1_left_trigger_pressed)
				{
				}

				joystick1_left_trigger_pressed = false;
			}

			//Joystick1: rightTrigger
			if(joystick_states_array[0].rightTrigger > config.trigger_threshold)
			{
				// RIP aligned shooting... worked awesome on the practice field
			}
			else
			{	
				if(joystick1_right_trigger_pressed){
				}

				// teleop_cmd_vel->setSlowMode(false);
				joystick1_right_trigger_pressed = false;
			}
#endif
		}
		else
		{
			// Drive in diagnostic mode unconditionally
	#if 0
			//Joystick1 Diagnostics: leftStickY
			if(abs(joystick_states_array[0].leftStickY) > config.stick_threshold)
			{
			}

			//Joystick1 Diagnostics: leftStickX
			if(abs(joystick_states_array[0].leftStickX) > config.stick_threshold)
			{
			}

			//Joystick1 Diagnostics: rightStickY
			if(abs(joystick_states_array[0].rightStickY) > config.stick_threshold)
			{
			}

			//Joystick1 Diagnostics: rightStickX
			if(abs(joystick_states_array[0].rightStickX) > config.stick_threshold)
			{
			}
#endif

			//Joystick1 Diagnostics: stickLeft
			if(joystick_states_array[0].stickLeftPress)
			{
			}
			if(joystick_states_array[0].stickLeftButton)
			{
			}
			if(joystick_states_array[0].stickLeftRelease)
			{
			}

			//Joystick1 Diagnostics: stickRight
			if(joystick_states_array[0].stickRightPress)
			{
			}
			if(joystick_states_array[0].stickRightButton)
			{
			}
			if(joystick_states_array[0].stickRightRelease)
			{
			}

			//Joystick1 Diagnostics: buttonA
			if(joystick_states_array[0].buttonAPress)
			{
			}
			if(joystick_states_array[0].buttonAButton)
			{
			}
			if(joystick_states_array[0].buttonARelease)
			{
			}

			//Joystick1 Diagnostics: buttonB
			if(joystick_states_array[0].buttonBPress)
			{
			}
			if(joystick_states_array[0].buttonBButton)
			{
			}
			if(joystick_states_array[0].buttonBRelease)
			{
			}

			//Joystick1 Diagnostics: buttonX
			if(joystick_states_array[0].buttonXPress)
			{
			}
			if(joystick_states_array[0].buttonXButton)
			{
			}
			if(joystick_states_array[0].buttonXRelease)
			{
			}

			//Joystick1 Diagnostics: buttonY
			if(joystick_states_array[0].buttonYPress)
			{
			}
			if(joystick_states_array[0].buttonYButton)
			{
			}
			if(joystick_states_array[0].buttonYRelease)
			{
			}

			//Joystick1: buttonBack
			if(joystick_states_array[0].buttonBackPress)
			{
			}
			if(joystick_states_array[0].buttonBackButton)
			{
			}
			if(joystick_states_array[0].buttonBackRelease)
			{
			}

			//Joystick1: buttonStart
			if(joystick_states_array[0].buttonStartPress)
			{
			}
			if(joystick_states_array[0].buttonStartButton)
			{
			}
			if(joystick_states_array[0].buttonStartRelease)
			{
			}

			//Joystick1 Diagnostics: bumperLeft
			if(joystick_states_array[0].bumperLeftPress)
			{
			}
			if(joystick_states_array[0].bumperLeftButton)
			{
			}
			if(joystick_states_array[0].bumperLeftRelease)
			{
			}

			//Joystick1 Diagnostics: bumperRight
			if(joystick_states_array[0].bumperRightPress)
			{
			}
			if(joystick_states_array[0].bumperRightButton)
			{
			}
			if(joystick_states_array[0].bumperRightRelease)
			{
			}

			//Joystick1 Diagnostics: leftTrigger
			if(joystick_states_array[0].leftTrigger > config.trigger_threshold)
			{
				if(!joystick1_left_trigger_pressed)
				{
					//zero_all_diag_commands();
				}

				joystick1_left_trigger_pressed = true;
			}
			else
			{
				if(joystick1_left_trigger_pressed)
				{
				}

				joystick1_left_trigger_pressed = false;
			}
			//Joystick1 Diagnostics: rightTrigger
			if(joystick_states_array[0].rightTrigger > config.trigger_threshold)
			{
				if(!joystick1_right_trigger_pressed)
				{
					//zero_all_diag_commands();
				}

				joystick1_right_trigger_pressed = true;
			}
			else
			{
				if(joystick1_right_trigger_pressed)
				{
				}

				joystick1_right_trigger_pressed = false;
			}

			//Joystick1 Diagnostics: directionLeft
			if(joystick_states_array[0].directionLeftPress)
			{
			}
			if(joystick_states_array[0].directionLeftButton)
			{
			}
			if(joystick_states_array[0].directionLeftRelease)
			{
			}

			//Joystick1 Diagnostics: directionRight
			if(joystick_states_array[0].directionRightPress)
			{
			}
			if(joystick_states_array[0].directionRightButton)
			{
			}
			if(joystick_states_array[0].directionRightRelease)
			{
			}

			//Joystick1 Diagnostics: directionUp
			if(joystick_states_array[0].directionUpPress)
			{
			}
			if(joystick_states_array[0].directionUpButton)
			{
			}
			if(joystick_states_array[0].directionUpRelease)
			{
			}

			//Joystick1 Diagnostics: directionDown
			if(joystick_states_array[0].directionDownPress)
			{
			}
			if(joystick_states_array[0].directionDownButton)
			{
			}
			if(joystick_states_array[0].directionDownRelease)
			{
			}
		}

		last_header_stamp = joystick_states_array[0].header.stamp;
	}
	else if(joystick_id == 1)
	{
		// TODO Add empty button mappings here.
	}
	if (diagnostics_mode)
	{
		publish_diag_cmds();
	}
}

void jointStateCallback(const sensor_msgs::JointState &joint_state)
{
	// TODO - remove this if not used
}

void matchStateCallback(const frc_msgs::MatchSpecificData &msg)
{
	// TODO : if in diagnostic mode, zero all outputs on the
	// transition from enabled to disabled
	robot_is_disabled = msg.Disabled;
}

/*
void hubAngleCallback(const std_msgs::Float64 &msg) {
	hub_angle = msg.data;
}
*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Joystick_controller");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "teleop_params");
	ros::NodeHandle n_diagnostics_params(n, "teleop_diagnostics_params");
	ros::NodeHandle n_swerve_params(n, "/frcrobot_jetson/swerve_drive_controller");

	int num_joysticks = 1;
	if(!n_params.getParam("num_joysticks", num_joysticks))
	{
		ROS_ERROR("Could not read num_joysticks in teleop_joystick_comp");
	}
	if(!n_params.getParam("joystick_deadzone", config.joystick_deadzone))
	{
		ROS_ERROR("Could not read joystick_deadzone in teleop_joystick_comp");
	}
	if(!n_params.getParam("joystick_pow", config.joystick_pow))
	{
		ROS_ERROR("Could not read joystick_pow in teleop_joystick_comp");
	}
	if(!n_params.getParam("rotation_pow", config.rotation_pow))
	{
		ROS_ERROR("Could not read rotation_pow in teleop_joystick_comp");
	}
	if(!n_params.getParam("min_speed", config.min_speed))
	{
		ROS_ERROR("Could not read min_speed in teleop_joystick_comp");
	}
	if(!n_params.getParam("max_speed", config.max_speed))
	{
		ROS_ERROR("Could not read max_speed in teleop_joystick_comp");
	}
	if(!n_params.getParam("max_speed_slow", config.max_speed_slow))
	{
		ROS_ERROR("Could not read max_speed_slow in teleop_joystick_comp");
	}
	if(!n_params.getParam("button_move_speed", config.button_move_speed))
	{
		ROS_ERROR("Could not read button_move_speed in teleop_joystick_comp");
	}
	if(!n_params.getParam("max_rot", config.max_rot))
	{
		ROS_ERROR("Could not read max_rot in teleop_joystick_comp");
	}
	if(!n_params.getParam("max_rot_slow", config.max_rot_slow))
	{
		ROS_ERROR("Could not read max_rot_slow in teleop_joystick_comp");
	}
	if(!n_params.getParam("drive_rate_limit_time", config.drive_rate_limit_time))
	{
		ROS_ERROR("Could not read drive_rate_limit_time in teleop_joystick_comp");
	}
	if(!n_params.getParam("rotate_rate_limit_time", config.rotate_rate_limit_time))
	{
		ROS_ERROR("Could not read rotate_rate_limit_time in teleop_joystick_comp");
	}
	if(!n_params.getParam("trigger_threshold", config.trigger_threshold))
	{
		ROS_ERROR("Could not read trigger_threshold in teleop_joystick_comp");
	}
	if(!n_params.getParam("stick_threshold", config.stick_threshold))
	{
		ROS_ERROR("Could not read stick_threshold in teleop_joystick_comp");
	}
	if(!n_params.getParam("bottom_position_angle", config.bottom_position_angle))
	{
		ROS_ERROR("Could not read bottom_position_angle in teleop_joystick_comp");
	}
	if(!n_params.getParam("middle_position_angle", config.middle_position_angle))
	{
		ROS_ERROR("Could not read middle_position_angle in teleop_joystick_comp");
	}
	if(!n_params.getParam("top_position_angle", config.top_position_angle))
	{
		ROS_ERROR("Could not read top_position_angle in teleop_joystick_comp");
	}

	teleop_cmd_vel = std::make_unique<TeleopCmdVel<teleop_joystick_control::TeleopJoystickComp2023Config>>(config);

	imu_angle = M_PI / 2.;

	const std::map<std::string, std::string> service_connection_header{{"tcp_nodelay", "1"}};

	BrakeSrv = n.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, service_connection_header);
	IMUZeroSrv = n.serviceClient<imu_zero::ImuZeroAngle>("/imu/set_imu_zero", false, service_connection_header);

	orient_strafing_enable_pub = n.advertise<std_msgs::Bool>("orient_strafing/pid_enable", 1);
	orient_strafing_setpoint_pub = n.advertise<std_msgs::Float64>("orient_strafing/setpoint", 1);
	orient_strafing_state_pub = n.advertise<std_msgs::Float64>("orient_strafing/state", 1);
	JoystickRobotVel = n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);
	ros::Subscriber imu_heading = n.subscribe("/imu/zeroed_imu", 1, &imuCallback);
	ros::Subscriber joint_states_sub = n.subscribe("/frcrobot_jetson/joint_states", 1, &jointStateCallback);

	ros::Subscriber match_state_sub = n.subscribe("/frcrobot_rio/match_data", 1, matchStateCallback);
	ros::ServiceServer robot_orient_service = n.advertiseService("robot_orient", orientCallback);

	auto_mode_select_pub = n.advertise<behavior_actions::AutoMode>("/auto/auto_mode", 1, true);

	const ros::Duration startup_wait_time_secs(15);
	const ros::Time startup_start_time = ros::Time::now();
	ros::Duration startup_wait_time;
	startup_wait_time = std::max(startup_wait_time_secs - (ros::Time::now() - startup_start_time), ros::Duration(0.1));
	if(!BrakeSrv.waitForExistence(startup_wait_time))
	{
		ROS_ERROR("Wait (15 sec) timed out, for Brake Service in teleop_joystick_comp.cpp");
	}

	startup_wait_time = std::max(startup_wait_time_secs - (ros::Time::now() - startup_start_time), ros::Duration(0.1));
	if(!IMUZeroSrv.waitForExistence(startup_wait_time))
	{
		ROS_ERROR("Wait (1 sec) timed out, for IMU Zero Service in teleop_joystick_comp.cpp");
	}

	ros::ServiceServer orient_strafing_angle_service = n.advertiseService("orient_strafing_angle", orientStrafingAngleCallback);

	DynamicReconfigureWrapper<teleop_joystick_control::TeleopJoystickComp2023Config> drw(n_params, config);
	DynamicReconfigureWrapper<teleop_joystick_control::TeleopJoystickCompDiagnostics2023Config> diagnostics_drw(n_diagnostics_params, diagnostics_config);

	//Read from _num_joysticks joysticks
	// Set up this callback last, since it might use all of the various stuff
	// initialized above here. Setting it up first risks the chance that a callback
	// happens immediately and tries to use them before they have valid values
	std::vector <ros::Subscriber> subscriber_array;
	joystick_states_array.resize(num_joysticks);
	for(int j = 0; j < num_joysticks; j++)
	{
		std::stringstream s;
		s << "/frcrobot_rio/joystick_states";
		s << (j+1);
		topic_array.push_back(s.str());
		subscriber_array.push_back(n.subscribe(topic_array[j], 1, &evaluateCommands));
	}

	ros::Subscriber button_box_sub = n.subscribe("/frcrobot_rio/button_box_states", 1, &buttonBoxCallback);

	ROS_WARN("joy_init");

	ros::spin();
	return 0;
}
