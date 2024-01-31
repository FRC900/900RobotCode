// Toggle between rotation using rightStickX and rightTrigger - leftTrigger
#define ROTATION_WITH_STICK

#include "ros/ros.h"
#include "frc_msgs/JoystickState.h"
//#define NEED_JOINT_STATES
#ifdef NEED_JOINT_STATES
#include "sensor_msgs/JointState.h"
#endif
#include "geometry_msgs/Twist.h"
#include <string>
#include <cmath>

#include "std_srvs/Empty.h"

#include <vector>
#include "teleop_joystick_control/RobotOrient.h"

#include "frc_msgs/ButtonBoxState2023.h"
#include "frc_msgs/MatchSpecificData.h"

#include "actionlib/client/simple_action_client.h"

#include "ddynamic_reconfigure/ddynamic_reconfigure.h"

#include "teleop_joystick_control/TeleopCmdVel2023.h"
#include "behavior_actions/AutoMode.h"

#include "path_follower_msgs/holdPositionAction.h"

#include <imu_zero_msgs/ImuZeroAngle.h>
#include <angles/angles.h>
#include "teleop_joystick_control/RobotOrientationDriver.h"
#include <teleop_joystick_control/SnapConeCube.h>
#include <behavior_actions/Intaking2023Action.h>
#include <behavior_actions/Placing2023Action.h>
#include <behavior_actions/FourbarElevatorPath2023Action.h>
#include <talon_swerve_drive_controller_msgs/SetXY.h>
#include <behavior_actions/AlignAndPlaceGrid2023Action.h>
#include <talon_state_msgs/TalonFXProState.h>
#include <std_srvs/SetBool.h>

struct DynamicReconfigVars
{
	double joystick_deadzone{0};          // "Joystick deadzone, in percent",
	double min_speed{0};                  // "Min linear speed to get robot to overcome friction, in m/s"
	double max_speed{2.0};                // "Max linear speed, in m/s"
	double max_speed_slow{0.75};          // "Max linear speed in slow mode, in m/s"
	double max_rot{6.0};                  // "Max angular speed"
	double max_rot_slow{2.0};             // "Max angular speed in slow mode"
	double button_move_speed{0.5};        // "Linear speed when move buttons are pressed, in m/s"
	double joystick_pow{1.5};             // "Joystick Scaling Power, linear"
	double rotation_pow{1.0};             // "Joystick Scaling Power, rotation"
	double drive_rate_limit_time{200};    // "msec to go from full back to full forward"
	double rotate_rate_limit_time{500};   // "msec to go from full counterclockwise to full clockwise"
	double trigger_threshold{0.5};        // "Amount trigger has to be pressed to trigger action"
	double stick_threshold{0.5};          // "Amount stick has to be moved to trigger diag mode action"
	double imu_zero_angle{0.0};           // "Value to pass to imu/set_zero when zeroing"
	double rotation_epsilon{0.01};		  // "Threshold Z-speed deciding if the robot is stopped"
	double rotation_axis_scale{1.0};      // "Scale factor for rotation axis stick input"
	double angle_to_add{0.135};
	double angle_threshold{angles::from_degrees(1)};
	double match_time_to_park{20}; // enable auto-parking after the 0.75 second timeout if the match time left < this value
} config;

std::unique_ptr<TeleopCmdVel<DynamicReconfigVars>> teleop_cmd_vel;

std::unique_ptr<RobotOrientationDriver> robot_orientation_driver;

bool diagnostics_mode = false;

frc_msgs::ButtonBoxState2023 button_box;

// array of joystick_states messages for multiple joysticks
std::vector <frc_msgs::JoystickState> joystick_states_array;
std::vector <std::string> topic_array;


ros::Publisher JoystickRobotVel;

ros::ServiceClient BrakeSrv;
ros::ServiceClient ParkSrv;
ros::ServiceClient IMUZeroSrv;
ros::ServiceClient SwerveOdomZeroSrv;
ros::ServiceClient FourbarRezeroSrv;

ros::Publisher auto_mode_select_pub;

bool joystick1_left_trigger_pressed = false;
bool joystick1_right_trigger_pressed = false;

bool up_down_switch_mid = false;
bool left_right_switch_mid = false;

double last_offset;
bool last_robot_orient;

// Diagnostic mode controls
int direction_x{};
int direction_y{};
int direction_z{};

bool robot_is_disabled{false};
bool elevator_up = false;
bool no_driver_input{false};

uint8_t grid;
uint8_t game_piece;
uint8_t node;

uint8_t auto_starting_pos = 1; // 1 indexed
uint8_t auto_mode = 0; // 0 indexed

bool sendRobotZero = false;
bool sendSetAngle = true;
double old_angular_z = 0.0;
bool use_pathing = false;
uint8_t grid_position = 0;
bool moved = false;
bool pathed = false;
bool last_no_driver_input = false;

uint8_t autoMode() {
	// if ignoring starting positions, set the same auto modes for the three listed next to the switch position
	//        L  M      R
	// up =   1, 2, and 3
	// mid =  4, 5, and 6
	// down = 7, 8, and 9
	ROS_INFO_STREAM("teleop_joystick_comp_2023 : auto_mode = " << std::to_string(auto_mode * 3 + auto_starting_pos));
	return auto_mode * 3 + auto_starting_pos;
}

uint8_t alliance_color{};
bool called_park_endgame = false;

void matchStateCallback(const frc_msgs::MatchSpecificData &msg)
{
	// TODO : if in diagnostic mode, zero all outputs on the
	// transition from enabled to disabled
	robot_is_disabled = msg.Disabled;
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

ros::ServiceClient snapConeCubeSrv;
ros::ServiceClient setCenterSrv;

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
std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::Intaking2023Action>> intaking_ac;
std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::Placing2023Action>> placing_ac;
std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::FourbarElevatorPath2023Action>> pathing_ac;
std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::AlignAndPlaceGrid2023Action>> align_and_place_ac;

size_t elevator_idx = std::numeric_limits<size_t>::max();
double elevator_height{0};
double elevator_setpoint{0};

void talonFXProStateCallback(const talon_state_msgs::TalonFXProState talon_state)
{           
	// fourbar_master_idx == max of size_t at the start
	if (elevator_idx == std::numeric_limits<size_t>::max()) // could maybe just check for > 0
	{
		for (size_t i = 0; i < talon_state.name.size(); i++)
		{
			if (talon_state.name[i] == "elevator_leader")
			{
				elevator_idx = i;
				break;
			}
		}
	}
	if (elevator_idx != std::numeric_limits<size_t>::max())
	{
		elevator_height = talon_state.position[elevator_idx];
		elevator_setpoint = talon_state.control_position[elevator_idx];

		pathed = (elevator_height >= config.elevator_threshold);
		// if we are currently above the height or want to go above the height
		if (elevator_height > config.elevator_threshold || elevator_setpoint > config.elevator_threshold) {
			teleop_cmd_vel->setSuperSlowMode(true);
		}
		else {
			teleop_cmd_vel->setSuperSlowMode(false);
		}
	}
	else {
		ROS_ERROR_STREAM_THROTTLE(0.1, "teleop_joystick_comp_2023 : Can not find talon with name = elevator_leader");
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



void place() {
	behavior_actions::Placing2023Goal goal;
	goal.node = node;
	goal.piece = game_piece;
	goal.override_game_piece = true;
	goal.step = moved ? goal.PLACE_RETRACT : goal.MOVE;
	placing_ac->sendGoal(goal);
	moved = !moved;
}

// start at waiting to align
// when driver transition becomes true, and we are at waiting to align, then align
// if aligning and you see press callback, place
// if on placing and see a press event, go to waiting to align  
enum AutoPlaceState {
	WAITING_TO_ALIGN = 0,
	ALIGNING = 1,
	PLACING = 2,
};

AutoPlaceState auto_place_state = AutoPlaceState::WAITING_TO_ALIGN; 
uint8_t intake_piece = 0;

void buttonBoxCallback(const ros::MessageEvent<frc_msgs::ButtonBoxState2023 const>& event)
{
	//ROS_INFO_STREAM("Button Box callback running!");

	button_box = *(event.getMessage());

	static ros::Time last_header_stamp = button_box.header.stamp;
	
	bool driver_input_changed = false;
	if (last_no_driver_input != no_driver_input && no_driver_input == true) {
		driver_input_changed = true; 
	}
	ROS_INFO_STREAM_THROTTLE(1, "Auto place state = " << std::to_string(auto_place_state));
	
	if(button_box.lockingSwitchPress)
	{
	}

	if(button_box.lockingSwitchButton)
	{
		ROS_INFO_STREAM_THROTTLE(1, "Use pathing = true");
		use_pathing = true;
	}
	else
	{
		ROS_INFO_STREAM_THROTTLE(1, "Use pathing = false");
		use_pathing = false;
	}
	ROS_INFO_STREAM_THROTTLE(2, "Use pathing = " << use_pathing);
	ROS_INFO_STREAM_THROTTLE(2, "Alliance color = " << alliance_color);

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
		// robot_orientation_driver->stopRotation();
		// ROS_WARN_STREAM("Disabling diagnostics mode!");

		teleop_cmd_vel->setRobotOrient(false, 0.0);
		ROS_WARN_STREAM("Field relative mode!");
	}

	if(button_box.redButton) {
	}
	if(button_box.redPress) {
		ROS_WARN_STREAM("Preempting all actions!");
		placing_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		intaking_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		pathing_ac->cancelAllGoals();
		align_and_place_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		pathed = false;
	}
	if(button_box.redRelease) {
	}

	if(button_box.topLeftConeButton) {
	}
	if(button_box.topLeftConePress) {
		ROS_INFO_STREAM("teleop : rezero fourbar!");
		std_srvs::Empty rezero_cmd;
		FourbarRezeroSrv.call(rezero_cmd);
	}
	if(button_box.topLeftConeRelease) {
	}

	if(button_box.topMiddleConeButton) {
	}
	if(button_box.topMiddleConePress) {
		intake_piece = behavior_actions::Intaking2023Goal::BASE_TOWARDS_US_CONE;
	}
	if(button_box.topMiddleConeRelease) {
	}

	if(button_box.topRightCubeButton) {
	}
	if(button_box.topRightCubePress) {
		intake_piece = behavior_actions::Intaking2023Goal::CUBE;
	}
	if(button_box.topRightCubeRelease) {
	}

	// HAS TO BE ABOVE BUTTON
	if(button_box.gridSelectConeLeftPress || button_box.gridSelectConeRightPress) {
		game_piece = behavior_actions::Placing2023Goal::VERTICAL_CONE;
		last_no_driver_input = false;
		if (use_pathing) {
			if (auto_place_state == AutoPlaceState::PLACING) {
				auto_place_state = AutoPlaceState::WAITING_TO_ALIGN;
				ROS_INFO_STREAM("Auto place state == WAITING_TO_ALIGN"); 
			}
			else if (auto_place_state == AutoPlaceState::ALIGNING && elevator_height > config.elevator_threshold) {
				place();
				auto_place_state = AutoPlaceState::PLACING;
				ROS_INFO_STREAM("Auto place state == PLACING"); 
			}
			else if (elevator_height <= config.elevator_threshold && auto_place_state == AutoPlaceState::ALIGNING) {
				auto_place_state = AutoPlaceState::WAITING_TO_ALIGN;
				ROS_INFO_STREAM("AutoPlaceState set back to waiting to align ");
			}
		}
		if (!use_pathing) {
			ROS_INFO_STREAM("Placing a cone!");
			place();
			pathed = false;
		}
	}

	if(button_box.gridSelectConeLeftButton && driver_input_changed && auto_place_state == AutoPlaceState::WAITING_TO_ALIGN && use_pathing) {
		game_piece = behavior_actions::Placing2023Goal::VERTICAL_CONE;
		behavior_actions::AlignAndPlaceGrid2023Goal align_goal;
		align_goal.alliance = alliance_color;
		moved = true;
		pathed = true;
		align_goal.tolerance = config.cone_tolerance;
		align_goal.tolerance_for_extend = 0.25;
		align_goal.auto_place = false;
		align_goal.grid_id = 1 + grid_position;
		ROS_INFO_STREAM("Sending align to goal with id " << std::to_string(align_goal.grid_id));
		align_goal.node = node;
		align_goal.piece = game_piece;
		align_goal.override_game_piece = false;
		align_goal.from_Trex = false; // maybe should be true since that is what we do in auto?
		align_and_place_ac->sendGoal(align_goal);
		auto_place_state = AutoPlaceState::ALIGNING; 
		ROS_INFO_STREAM("AutoPlaceState == ALIGNING");
	}
	if(button_box.gridSelectConeLeftRelease) {
	}

	// PRESS MUST BE BEFORE BUTTON
	if(button_box.gridSelectCubePress) {
		last_no_driver_input = false;
		if (use_pathing) {
			if (auto_place_state == AutoPlaceState::PLACING) {
				auto_place_state = AutoPlaceState::WAITING_TO_ALIGN;
				ROS_INFO_STREAM("Auto place state == WAITING_TO_ALIGN"); 
			}
			else if (elevator_height <= config.elevator_threshold && auto_place_state == AutoPlaceState::ALIGNING) {
				auto_place_state = AutoPlaceState::WAITING_TO_ALIGN;
				ROS_INFO_STREAM("AutoPlaceState set back to waiting to align");
			}
		}
		if (!use_pathing) {
			ROS_INFO_STREAM("Placing a cube!");
			place();
		}
	}

	if(button_box.gridSelectCubeButton && driver_input_changed && auto_place_state == AutoPlaceState::WAITING_TO_ALIGN && use_pathing) {
		game_piece = behavior_actions::Placing2023Goal::CUBE;
		behavior_actions::AlignAndPlaceGrid2023Goal align_goal;
		align_goal.alliance = alliance_color;
		moved = true;
		pathed = true;
		align_goal.tolerance = config.cube_tolerance;
		align_goal.tolerance_for_extend = 2.0;
		align_goal.auto_place = true;
		align_goal.grid_id = 2 + grid_position;
		ROS_INFO_STREAM("Sending align to goal with id " << std::to_string(align_goal.grid_id));
		align_goal.node = node;
		align_goal.piece = game_piece;
		align_goal.override_game_piece = false;
		align_goal.from_Trex = false; // maybe should be true since that is what we do in auto?
		align_and_place_ac->sendGoal(align_goal);
		auto_place_state = AutoPlaceState::PLACING;
		ROS_INFO_STREAM("Auto Place STATE == PLACING");
	}
	if(button_box.gridSelectCubeRelease) {
	}

	if(button_box.gridSelectConeRightButton && driver_input_changed && auto_place_state == AutoPlaceState::WAITING_TO_ALIGN && use_pathing) {
		game_piece = behavior_actions::Placing2023Goal::VERTICAL_CONE; // type doesn't matter for placing
		behavior_actions::AlignAndPlaceGrid2023Goal align_goal;
		align_goal.alliance = alliance_color;
		moved = true;
		pathed = true;
		align_goal.tolerance = config.cone_tolerance;
		align_goal.tolerance_for_extend = 0.25;
		align_goal.auto_place = false;
		align_goal.grid_id = 3 + grid_position;
		ROS_INFO_STREAM("Sending align to goal with id " << std::to_string(align_goal.grid_id));
		align_goal.node = node;
		align_goal.piece = game_piece;
		align_goal.override_game_piece = false;
		align_goal.from_Trex = false; // maybe should be true since that is what we do in auto?
		
		align_and_place_ac->sendGoal(align_goal);
		auto_place_state = AutoPlaceState::ALIGNING; 
		ROS_INFO_STREAM("AutoPlaceState == ALIGNING");
	}
	if(button_box.gridSelectConeRightPress) {
		// already handled above
	}
	if(button_box.gridSelectConeRightRelease) {
	}

	if(button_box.heightSelectSwitchUpButton) {
		node = behavior_actions::Placing2023Goal::HIGH;
	}
	if(button_box.heightSelectSwitchUpPress) {
		if (robot_is_disabled)
		{
			auto_mode = 0;
			// should we be worried about messages being dropped here?
			behavior_actions::AutoMode auto_mode_msg;
			auto_mode_msg.header.stamp = ros::Time::now();
			auto_mode_msg.auto_mode = autoMode();
			auto_mode_select_pub.publish(auto_mode_msg);
		}
		node = behavior_actions::Placing2023Goal::HIGH;
	}
	if(button_box.heightSelectSwitchUpRelease) {
	}

	if(!(button_box.heightSelectSwitchUpButton || button_box.heightSelectSwitchDownButton)) {
		if (!up_down_switch_mid) {
			node = behavior_actions::Placing2023Goal::MID;
			if (robot_is_disabled)
			{
				auto_mode = 1;
				// should we be worried about messages being dropped here?
				behavior_actions::AutoMode auto_mode_msg;
				auto_mode_msg.header.stamp = ros::Time::now();
				auto_mode_msg.auto_mode = autoMode();
				auto_mode_select_pub.publish(auto_mode_msg);
			}
		}

		up_down_switch_mid = true;
	} else {
		up_down_switch_mid = false;
	}

	if(button_box.heightSelectSwitchDownButton) {
		node = behavior_actions::Placing2023Goal::HYBRID;
	}
	if(button_box.heightSelectSwitchDownPress) {
		node = behavior_actions::Placing2023Goal::HYBRID;
		if (robot_is_disabled)
		{
			auto_mode = 2;
			// should we be worried about messages being dropped here?
			behavior_actions::AutoMode auto_mode_msg;
			auto_mode_msg.header.stamp = ros::Time::now();
			auto_mode_msg.auto_mode = autoMode();
			auto_mode_select_pub.publish(auto_mode_msg);
		}
	}
	if(button_box.heightSelectSwitchDownRelease) {
	}

	if(button_box.heightSelectSwitchLeftButton) {
	}
	if(button_box.heightSelectSwitchLeftPress) {
		if (robot_is_disabled)
		{
			auto_starting_pos = 1;
			// should we be worried about messages being dropped here?
			behavior_actions::AutoMode auto_mode_msg;
			auto_mode_msg.header.stamp = ros::Time::now();
			auto_mode_msg.auto_mode = autoMode();
			auto_mode_select_pub.publish(auto_mode_msg);
		}
		grid_position = 0;
	}
	if(button_box.heightSelectSwitchLeftRelease) {
	}

	if(!(button_box.heightSelectSwitchLeftButton || button_box.heightSelectSwitchRightButton)) {
		if (!left_right_switch_mid) {
			node = behavior_actions::Placing2023Goal::MID;
			if (robot_is_disabled)
			{
				auto_starting_pos = 2;
				// should we be worried about messages being dropped here?
				behavior_actions::AutoMode auto_mode_msg;
				auto_mode_msg.header.stamp = ros::Time::now();
				auto_mode_msg.auto_mode = autoMode();
				auto_mode_select_pub.publish(auto_mode_msg);
			}
		}
		grid_position = 3;
		left_right_switch_mid = true;
	} else {
		left_right_switch_mid = false;
	}

	if(button_box.heightSelectSwitchRightButton) {
	}
	if(button_box.heightSelectSwitchRightPress) {
		if (robot_is_disabled)
		{
			auto_starting_pos = 3;
			// should we be worried about messages being dropped here?
			behavior_actions::AutoMode auto_mode_msg;
			auto_mode_msg.header.stamp = ros::Time::now();
			auto_mode_msg.auto_mode = autoMode();
			auto_mode_select_pub.publish(auto_mode_msg);
		}
		grid_position = 6;
	}
	if(button_box.heightSelectSwitchRightRelease) {
	}

	if(button_box.centralYellowButton) {
	}
	if(button_box.centralYellowPress) {
		robot_orientation_driver->setTargetOrientation(0.0, true);
	}
	if(button_box.centralYellowRelease) {
	}

	if(button_box.bottomLeftYellowButton) {
	}
	if(button_box.bottomLeftYellowPress) {
		robot_orientation_driver->setTargetOrientation(M_PI, true);
	}
	if(button_box.bottomLeftYellowRelease) {
	}

	if(button_box.bottomRightWhiteButton) {
	}
	if(button_box.bottomRightWhitePress) {
		ROS_INFO_STREAM("teleop_joystick_comp_2023 : zeroing IMU to 180");
		imu_zero_msgs::ImuZeroAngle imu_cmd;
		imu_cmd.request.angle = 180.0;
		IMUZeroSrv.call(imu_cmd);
		ROS_INFO_STREAM("teleop_joystick_comp_2023 : zeroing swerve odom");
		std_srvs::Empty odom_cmd;
		SwerveOdomZeroSrv.call(odom_cmd);
	}
	if(button_box.bottomRightWhiteRelease) {
	}

	if(button_box.rightGreenPress)
	{
		moveDirection(0, 1, 0);
	}
	if(button_box.rightGreenButton)
	{
		sendDirection();
	}
	if(button_box.rightGreenRelease)
	{
		moveDirection(0, -1, 0);
	}


	if(button_box.leftGreenPress)
	{
		moveDirection(0, -1, 0);
	}
	if(button_box.leftGreenButton)
	{
		sendDirection();
	}
	if(button_box.leftGreenRelease)
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
}

bool aligned_to_game_piece = false;

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
		//ROS_INFO_STREAM_THROTTLE(3, "2023 js0 callback running!");

		static ros::Time last_header_stamp = joystick_states_array[0].header.stamp;

		teleop_cmd_vel->updateRateLimit(config);
		// TODO : make swerve invert the yaw so we can deal in ccw-positive angles
		// everywhere outside of that code
		geometry_msgs::Twist cmd_vel = teleop_cmd_vel->generateCmdVel(joystick_states_array[0], -robot_orientation_driver->getCurrentOrientation(), config);

		/*
		if (!rotation_increment) {
			robot_orientation_driver->stopRotation();
		}
		*/
		//ROS_INFO_STREAM_THROTTLE(1, "Angular z " << cmd_vel.angular.z);

		//ROS_INFO_STREAM_THROTTLE(1, "From teleop=" << robot_orientation_driver->mostRecentCommandIsFromTeleop());
		if (robot_orientation_driver->mostRecentCommandIsFromTeleop() || cmd_vel.linear.x != 0.0 || cmd_vel.linear.y != 0.0 || cmd_vel.angular.z != 0.0) {
			double original_angular_z = cmd_vel.angular.z;

			if (original_angular_z == 0.0 && old_angular_z != 0.0) {
				ROS_WARN_STREAM_THROTTLE(1, "Send set angle = false");
				sendSetAngle = false;
			}

			if (original_angular_z == 0.0 && !sendSetAngle) {
				ROS_INFO_STREAM("Old angular z " << old_angular_z << " signbit " << signbit(old_angular_z));
				double multiplier = 1;
				if (signbit(old_angular_z)) {
					multiplier = -1;
				}
				if (old_angular_z == 0.0) {
					ROS_WARN_STREAM("Old angular z is zero, wierd");
				}
				ROS_INFO_STREAM("Locking to current orientation!");
				robot_orientation_driver->setTargetOrientation(robot_orientation_driver->getCurrentOrientation() + multiplier * config.angle_to_add , true /* from telop */);
				sendSetAngle = true;
			}

			if (cmd_vel.angular.z == 0.0)
			{
				cmd_vel.angular.z = robot_orientation_driver->getOrientationVelocityPIDOutput();
				if (fabs(cmd_vel.angular.z) < config.rotation_epsilon) {
					cmd_vel.angular.z = 0.0;
				}
			}

			if((cmd_vel.linear.x == 0.0) && (cmd_vel.linear.y == 0.0) && (original_angular_z == 0.0) && !sendRobotZero)
			{
				no_driver_input = true;
				if (std_srvs::Empty empty; !BrakeSrv.call(empty))
				{
					ROS_ERROR("BrakeSrv call failed in sendRobotZero_");
				}
				ROS_INFO("BrakeSrv called");

				JoystickRobotVel.publish(cmd_vel);
				sendRobotZero = true;
			}
			else if((cmd_vel.linear.x != 0.0) || (cmd_vel.linear.y != 0.0) || (cmd_vel.angular.z != 0.0))
			{
				//ROS_INFO_STREAM("2023-Publishing " << cmd_vel.linear.x << " " << cmd_vel.linear.y << " " << cmd_vel.linear.z);
				JoystickRobotVel.publish(cmd_vel);
				sendRobotZero = false;
				no_driver_input = false;
				// if the original command was not zero, then teleop was controlling rotation

			}
			old_angular_z = original_angular_z;
		}

		if(!diagnostics_mode)
		{
			//Joystick1: buttonA
			//Joystick1: buttonA
			if(joystick_states_array[0].buttonAPress)
			{
				ROS_INFO_STREAM("teleop_joystick_comp_2023 : snapping to nearest cone and enabling robot relative driving mode!");
				if (teleop_joystick_control::SnapConeCube srv; snapConeCubeSrv.call(srv))
				{
					if (srv.response.nearest_cone_angle > -900) {
						ROS_INFO_STREAM("Using angle of " << srv.response.nearest_cone_angle);
						robot_orientation_driver->setTargetOrientation(srv.response.nearest_cone_angle, true /*from teleop*/);
						teleop_cmd_vel->setRobotOrient(true, 0);
					}
				}
			}
			if(joystick_states_array[0].buttonAButton)
			{
				
			}
			if(joystick_states_array[0].buttonARelease)
			{
				teleop_cmd_vel->setRobotOrient(false, 0);
			}

			//Joystick1: buttonB
			if(joystick_states_array[0].buttonBPress)
			{
				ROS_INFO_STREAM("teleop_joystick_comp_2023 : snapping to nearest cube and enabling robot relative driving mode!");
				if (teleop_joystick_control::SnapConeCube srv; snapConeCubeSrv.call(srv))
				{
					if (srv.response.nearest_cube_angle > -900) {
						ROS_INFO_STREAM_THROTTLE(1, "Using angle of " << srv.response.nearest_cube_angle);
						robot_orientation_driver->setTargetOrientation(srv.response.nearest_cube_angle, true /*from teleop*/);
						teleop_cmd_vel->setRobotOrient(true, 0);
					}
				}
			}
			if(joystick_states_array[0].buttonBButton)
			{	
			}
			if(joystick_states_array[0].buttonBRelease)
			{
				teleop_cmd_vel->setRobotOrient(false, 0);
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
				behavior_actions::Intaking2023Goal goal;
				goal.outtake = goal.OUTTAKE_CONE;
				intaking_ac->sendGoal(goal);
			}
			if(joystick_states_array[0].bumperRightButton)
			{
			}
			if(joystick_states_array[0].bumperRightRelease)
			{
				intaking_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
			}


			// Should be the dpad right here

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

			// end dpad


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
					behavior_actions::Intaking2023Goal goal;
					goal.piece = intake_piece;
					intaking_ac->sendGoal(goal);
				}

				joystick1_left_trigger_pressed = true;
			}
			else
			{
				if(joystick1_left_trigger_pressed)
				{
					intaking_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
				}

				joystick1_left_trigger_pressed = false;
			}

			//Joystick1: rightTrigger
			if(joystick_states_array[0].rightTrigger > config.trigger_threshold)
			{
				if(!joystick1_right_trigger_pressed)
				{
					behavior_actions::Intaking2023Goal goal;
					goal.piece = goal.VERTICAL_CONE;
					intaking_ac->sendGoal(goal);
				}

				joystick1_right_trigger_pressed = true;
			}
			else
			{
				if(joystick1_right_trigger_pressed)
				{
					intaking_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
				}

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

#ifdef NEED_JOINT_STATES
void jointStateCallback(const sensor_msgs::JointState &joint_state)
{
	// TODO - remove this if not used
}
#endif

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
	if(!n_params.getParam("max_speed_elevator_extended", config.max_speed_elevator_extended))
	{
		ROS_ERROR("Could not read max_speed_elevator_extended in teleop_joystick_comp");
	}
	if(!n_params.getParam("max_rot_elevator_extended", config.max_rot_elevator_extended))
	{
		ROS_ERROR("Could not read max_rot_elevator_extended in teleop_joystick_comp");
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
	if(!n_params.getParam("imu_zero_angle", config.imu_zero_angle))
	{
		ROS_ERROR("Could not read imu_zero_angle in teleop_joystick_comp");
	}
	if(!n_params.getParam("rotation_axis_scale", config.rotation_axis_scale))
	{
		ROS_ERROR("Could not read rotation_axis_scale in teleop_joystick_comp");
	}
	if(!n_params.getParam("rotation_epsilon", config.rotation_epsilon))
	{
		ROS_ERROR("Could not read rotation_epsilon in teleop_joystick_comp");
	}

	if(!n_params.getParam("angle_to_add", config.angle_to_add))
	{
		ROS_ERROR("Could not read angle_to_add in teleop_joystick_comp");
	}

	if(!n_params.getParam("cone_tolerance", config.cone_tolerance))
	{
		ROS_ERROR("Could not read cone_tolerance in teleop_joystick_comp");
	}

	if(!n_params.getParam("cube_tolerance", config.cube_tolerance))
	{
		ROS_ERROR("Could not read cube_tolerance in teleop_joystick_comp");
	}

	if(!n_params.getParam("match_time_to_park", config.match_time_to_park))
	{
		ROS_ERROR("Could not read match_time_to_park in teleop_joystick_comp");
	}

	ddynamic_reconfigure::DDynamicReconfigure ddr(n_params);

	ddr.registerVariable<double>("joystick_deadzone", &config.joystick_deadzone, "Joystick deadzone, in percent", 0., 1.);
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
	ddr.registerVariable<double>("max_speed_elevator_extended", &config.max_speed_elevator_extended, "Max linear speed in elevator extended mode, in m/s", 0., 2);
	ddr.registerVariable<double>("max_rot_elevator_extended", &config.max_rot_elevator_extended, "Max angular speed in elevator extended mode", 0., 1.);
	ddr.registerVariable<double>("rotation_epsilon", &config.rotation_epsilon, "rotation_epsilon", 0.0, 1.0);
	ddr.registerVariable<double>("angle_to_add", &config.angle_to_add, "angle_to_add", 0.0, 10);
	ddr.registerVariable<double>("cone_tolerance", &config.cone_tolerance, "cone_tolerance", 0.0, 0.5);
	ddr.registerVariable<double>("cube_tolerance", &config.cube_tolerance, "cube_tolerance", 0.0, 0.5);
	ddr.registerVariable<double>("match_time_to_park", &config.match_time_to_park, "match_time_to_park", 0.0, 60.0);

	ddr.publishServicesTopics();

	teleop_cmd_vel = std::make_unique<TeleopCmdVel<DynamicReconfigVars>>(config);
	robot_orientation_driver = std::make_unique<RobotOrientationDriver>(n);

	const std::map<std::string, std::string> service_connection_header{{"tcp_nodelay", "1"}};

	BrakeSrv = n.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, service_connection_header);
	ParkSrv = n.serviceClient<std_srvs::SetBool>("/frcrobot_jetson/swerve_drive_controller/toggle_park", false, service_connection_header);
	IMUZeroSrv = n.serviceClient<imu_zero_msgs::ImuZeroAngle>("/imu/set_imu_zero", false, service_connection_header);
	snapConeCubeSrv = n.serviceClient<teleop_joystick_control::SnapConeCube>("/snap_to_angle/snap_cone_cube", false, service_connection_header);
	setCenterSrv = n.serviceClient<talon_swerve_drive_controller_msgs::SetXY>("/frcrobot_jetson/swerve_drive_controller/change_center_of_rotation", false, service_connection_header);	
	JoystickRobotVel = n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);
	SwerveOdomZeroSrv = n.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/reset_odom", false, service_connection_header);
	FourbarRezeroSrv = n.serviceClient<std_srvs::Empty>("/frcrobot_jetson/four_bar_controller_2023/rezero_service", false, service_connection_header);
#ifdef NEED_JOINT_STATES
	ros::Subscriber joint_states_sub = n.subscribe("/frcrobot_jetson/joint_states", 1, &jointStateCallback);
#endif
	ros::Subscriber talon_states_sub = n.subscribe("/frcrobot_jetson/talonfxpro_states", 1, &talonFXProStateCallback);
	ros::Subscriber match_state_sub = n.subscribe("/frcrobot_rio/match_data", 1, matchStateCallback);
	ros::ServiceServer robot_orient_service = n.advertiseService("robot_orient", orientCallback);

	auto_mode_select_pub = n.advertise<behavior_actions::AutoMode>("/auto/auto_mode", 1, true);

	intaking_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::Intaking2023Action>>("/intaking/intaking_server_2023", true);
	placing_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::Placing2023Action>>("/placing/placing_server_2023", true);
	pathing_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::FourbarElevatorPath2023Action>>("/fourbar_elevator_path/fourbar_elevator_path_server_2023", true);
	align_and_place_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::AlignAndPlaceGrid2023Action>>("/align_and_place_grid", true);

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

	teleop_cmd_vel->setRobotOrient(false, 0.0);
	teleop_cmd_vel->setSlowMode(false);

	ros::spin();
	return 0;
}
