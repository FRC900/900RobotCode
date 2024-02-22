// Toggle between rotation using rightStickX and rightTrigger - leftTrigger
#define ROTATION_WITH_STICK

#include "ros/ros.h"
#include "frc_msgs/JoystickState.h"
//#define NEED_JOINT_STATES
#ifdef NEED_JOINT_STATES
#include "sensor_msgs/JointState.h"
#endif
#include <string>
#include <cmath>

#include "std_srvs/Empty.h"

#include <vector>

#include "frc_msgs/ButtonBoxState2023.h"

#include "actionlib/client/simple_action_client.h"

#include <imu_zero_msgs/ImuZeroAngle.h>
#include <teleop_joystick_control/SnapConeCube.h>
#include <behavior_actions/Intaking2023Action.h>
#include <behavior_actions/Placing2023Action.h>
#include <behavior_actions/FourbarElevatorPath2023Action.h>
#include <behavior_actions/AlignAndPlaceGrid2023Action.h>
#include <talon_state_msgs/TalonFXProState.h>
#include "behavior_actions/DriveToObjectAction.h"

#include "teleop_joystick_control/teleop_joystick_comp_general.h"

class AutoModeCalculator2023 : public AutoModeCalculator {
public:
	AutoModeCalculator2023() = default;
	uint8_t calculateAutoMode() override {
		return auto_mode_ * 3 + auto_starting_pos_;
	}
	void set_auto_mode(const uint8_t auto_mode) {
		auto_mode_ = auto_mode;
	}
	void set_auto_starting_pos(const uint8_t auto_starting_pos) {
		auto_starting_pos_ = auto_starting_pos;
	}
private:
	uint8_t auto_mode_{0};
	uint8_t auto_starting_pos_{1};
};

AutoModeCalculator2023 auto_calculator;

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

bool moved = false;
bool up_down_switch_mid{false};
bool left_right_switch_mid{false};
bool robot_is_disabled{true};
uint8_t game_piece;
uint8_t node;
bool use_pathing = false;
uint8_t grid_position = 0;
bool pathed = false;
bool last_no_driver_input = false;

// Figure out a better name for this
struct DynamicReconfigVars2023
{
	double max_speed_elevator_extended{1.5};
	double max_rot_elevator_extended{0.2}; 
	double cone_length{0.3302/2};
	double cube_length{0.2032/2};
	double elevator_threshold{0.5};
	double cone_tolerance{0.1};
	double cube_tolerance{0.1};
} config2023;

ros::ServiceClient FourbarRezeroSrv;

ros::ServiceClient snapConeCubeSrv;

std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::DriveToObjectAction>> auto_note_pickup_ac;

std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::Intaking2023Action>> intaking_ac;
std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::Placing2023Action>> placing_ac;
std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::FourbarElevatorPath2023Action>> pathing_ac;
std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::AlignAndPlaceGrid2023Action>> align_and_place_ac;

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

		pathed = (elevator_height >= config2023.elevator_threshold);
		// if we are currently above the height or want to go above the height
		if (elevator_height > config2023.elevator_threshold || elevator_setpoint > config2023.elevator_threshold) {
			driver->teleop_cmd_vel_.setCaps(config2023.max_speed_elevator_extended, config2023.max_rot_elevator_extended);

		}
		else {
			driver->teleop_cmd_vel_.resetCaps();
		}
	}
	else {
		ROS_ERROR_STREAM_THROTTLE(0.1, "teleop_joystick_comp_2023 : Can not find talon with name = elevator_leader");
	}
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

uint8_t intake_piece = 0;

void buttonBoxCallback(const frc_msgs::ButtonBoxState2023ConstPtr &button_box)
{
	//ROS_INFO_STREAM("Button Box callback running!");

	bool driver_input_changed = false;
	if (last_no_driver_input != driver->getNoDriverInput() && driver->getNoDriverInput() == true) {
		driver_input_changed = true; 
	}
	ROS_INFO_STREAM_THROTTLE(1, "Auto place state = " << std::to_string(auto_place_state));
	
	if(button_box->lockingSwitchPress)
	{
	}

	if(button_box->lockingSwitchButton)
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

	if(button_box->lockingSwitchRelease)
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

		driver->teleop_cmd_vel_.setRobotOrient(false, 0.0);
		ROS_WARN_STREAM("Field relative mode!");
	}

	if(button_box->redButton) {
	}
	if(button_box->redPress) {
		ROS_WARN_STREAM("Preempting all actions!");
		placing_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		intaking_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		pathing_ac->cancelAllGoals();
		align_and_place_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		auto_note_pickup_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		pathed = false;
	}
	if(button_box->redRelease) {
	}

	if(button_box->topLeftConeButton) {
	}
	if(button_box->topLeftConePress) {
		ROS_INFO_STREAM("teleop : rezero fourbar!");
		std_srvs::Empty rezero_cmd;
		FourbarRezeroSrv.call(rezero_cmd);
	}
	if(button_box->topLeftConeRelease) {
	}

	if(button_box->topMiddleConeButton) {
	}
	if(button_box->topMiddleConePress) {
		intake_piece = behavior_actions::Intaking2023Goal::BASE_TOWARDS_US_CONE;
	}
	if(button_box->topMiddleConeRelease) {
	}

	if(button_box->topRightCubeButton) {
	}
	if(button_box->topRightCubePress) {
		intake_piece = behavior_actions::Intaking2023Goal::CUBE;
	}
	if(button_box->topRightCubeRelease) {
	}

	// HAS TO BE ABOVE BUTTON
	if(button_box->gridSelectConeLeftPress || button_box->gridSelectConeRightPress) {
		game_piece = behavior_actions::Placing2023Goal::VERTICAL_CONE;
		last_no_driver_input = false;
		if (use_pathing) {
			if (auto_place_state == AutoPlaceState::PLACING) {
				auto_place_state = AutoPlaceState::WAITING_TO_ALIGN;
				ROS_INFO_STREAM("Auto place state == WAITING_TO_ALIGN"); 
			}
			else if (auto_place_state == AutoPlaceState::ALIGNING && elevator_height > config2023.elevator_threshold) {
				place();
				auto_place_state = AutoPlaceState::PLACING;
				ROS_INFO_STREAM("Auto place state == PLACING"); 
			}
			else if (elevator_height <= config2023.elevator_threshold && auto_place_state == AutoPlaceState::ALIGNING) {
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

	if(button_box->gridSelectConeLeftButton && driver_input_changed && auto_place_state == AutoPlaceState::WAITING_TO_ALIGN && use_pathing) {
		game_piece = behavior_actions::Placing2023Goal::VERTICAL_CONE;
		behavior_actions::AlignAndPlaceGrid2023Goal align_goal;
		align_goal.alliance = alliance_color;
		moved = true;
		pathed = true;
		align_goal.tolerance = config2023	.cone_tolerance;
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
	if(button_box->gridSelectConeLeftRelease) {
	}

	// PRESS MUST BE BEFORE BUTTON
	if(button_box->gridSelectCubePress) {
		last_no_driver_input = false;
		if (use_pathing) {
			if (auto_place_state == AutoPlaceState::PLACING) {
				auto_place_state = AutoPlaceState::WAITING_TO_ALIGN;
				ROS_INFO_STREAM("Auto place state == WAITING_TO_ALIGN"); 
			}
			else if (elevator_height <= config2023.elevator_threshold && auto_place_state == AutoPlaceState::ALIGNING) {
				auto_place_state = AutoPlaceState::WAITING_TO_ALIGN;
				ROS_INFO_STREAM("AutoPlaceState set back to waiting to align");
			}
		}
		if (!use_pathing) {
			ROS_INFO_STREAM("Placing a cube!");
			place();
		}
	}

	if(button_box->gridSelectCubeButton && driver_input_changed && auto_place_state == AutoPlaceState::WAITING_TO_ALIGN && use_pathing) {
		game_piece = behavior_actions::Placing2023Goal::CUBE;
		behavior_actions::AlignAndPlaceGrid2023Goal align_goal;
		align_goal.alliance = alliance_color;
		moved = true;
		pathed = true;
		align_goal.tolerance = config2023.cube_tolerance;
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
	if(button_box->gridSelectCubeRelease) {
	}

	if(button_box->gridSelectConeRightButton && driver_input_changed && auto_place_state == AutoPlaceState::WAITING_TO_ALIGN && use_pathing) {
		game_piece = behavior_actions::Placing2023Goal::VERTICAL_CONE; // type doesn't matter for placing
		behavior_actions::AlignAndPlaceGrid2023Goal align_goal;
		align_goal.alliance = alliance_color;
		moved = true;
		pathed = true;
		align_goal.tolerance = config2023.cone_tolerance;
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
	if(button_box->gridSelectConeRightPress) {
		// already handled above
	}
	if(button_box->gridSelectConeRightRelease) {
	}

	if(button_box->heightSelectSwitchUpButton) {
		node = behavior_actions::Placing2023Goal::HIGH;
	}
	if(button_box->heightSelectSwitchUpPress) {
		if (robot_is_disabled)
		{
			auto_calculator.set_auto_mode(0);
		}
		node = behavior_actions::Placing2023Goal::HIGH;
	}
	if(button_box->heightSelectSwitchUpRelease) {
	}

	if(!(button_box->heightSelectSwitchUpButton || button_box->heightSelectSwitchDownButton)) {
		if (!up_down_switch_mid) {
			node = behavior_actions::Placing2023Goal::MID;
			if (robot_is_disabled)
			{
				auto_calculator.set_auto_mode(1);
			}
		}

		up_down_switch_mid = true;
	} else {
		up_down_switch_mid = false;
	}

	if(button_box->heightSelectSwitchDownButton) {
		node = behavior_actions::Placing2023Goal::HYBRID;
	}
	if(button_box->heightSelectSwitchDownPress) {
		node = behavior_actions::Placing2023Goal::HYBRID;
		if (robot_is_disabled)
		{
			auto_calculator.set_auto_mode(2);
		}
	}
	if(button_box->heightSelectSwitchDownRelease) {
	}

	if(button_box->heightSelectSwitchLeftButton) {
	}
	if(button_box->heightSelectSwitchLeftPress) {
		if (robot_is_disabled)
		{
			auto_calculator.set_auto_starting_pos(1);
		}
		grid_position = 0;
	}
	if(button_box->heightSelectSwitchLeftRelease) {
	}

	if(!(button_box->heightSelectSwitchLeftButton || button_box->heightSelectSwitchRightButton)) {
		if (!left_right_switch_mid) {
			node = behavior_actions::Placing2023Goal::MID;
			if (robot_is_disabled)
			{
				auto_calculator.set_auto_starting_pos(2);
			}
		}
		grid_position = 3;
		left_right_switch_mid = true;
	} else {
		left_right_switch_mid = false;
	}

	if(button_box->heightSelectSwitchRightButton) {
	}
	if(button_box->heightSelectSwitchRightPress) {
		if (robot_is_disabled)
		{
			auto_calculator.set_auto_starting_pos(3);
		}
		grid_position = 6;
	}
	if(button_box->heightSelectSwitchRightRelease) {
	}

	if(button_box->centralYellowButton) {
	}
	if(button_box->centralYellowPress) {
		driver->setTargetOrientation(0.0, true);
	}
	if(button_box->centralYellowRelease) {
	}

	if(button_box->bottomLeftYellowButton) {
	}
	if(button_box->bottomLeftYellowPress) {
		driver->setTargetOrientation(M_PI, true);
	}
	if(button_box->bottomLeftYellowRelease) {
	}

	if(button_box->bottomRightWhiteButton) {
	}
	if(button_box->bottomRightWhitePress) {
		// for zeroing, assuming the robot starts facing away from the speaker (yes this is 2024 but we need to test it)
		imu_zero_msgs::ImuZeroAngle imu_cmd;
		if (alliance_color == frc_msgs::MatchSpecificData::ALLIANCE_COLOR_RED) {
			ROS_INFO_STREAM("teleop_joystick_comp_2023 : red alliance");
			imu_cmd.request.angle = 180.0;
		} else {
			ROS_INFO_STREAM("teleop_joystick_comp_2023 : blue or unknown alliance");
			imu_cmd.request.angle = 0.0;
		}
		ROS_INFO_STREAM("teleop_joystick_comp_2023 : zeroing IMU to " << imu_cmd.request.angle);
		IMUZeroSrv.call(imu_cmd);
		ROS_INFO_STREAM("teleop_joystick_comp_2023 : zeroing swerve odom");
		std_srvs::Empty odom_cmd;
		SwerveOdomZeroSrv.call(odom_cmd);
	}
	if(button_box->bottomRightWhiteRelease) {
	}

	if(button_box->rightGreenPress)
	{
		driver->moveDirection(0, 1, 0, config.button_move_speed);
	}
	if(button_box->rightGreenButton)
	{
		driver->sendDirection(config.button_move_speed);
	}
	if(button_box->rightGreenRelease)
	{
		driver->moveDirection(0, -1, 0, config.button_move_speed);
	}


	if(button_box->leftGreenPress)
	{
		driver->moveDirection(0, -1, 0, config.button_move_speed);
	}
	if(button_box->leftGreenButton)
	{
		driver->sendDirection(config.button_move_speed);
	}
	if(button_box->leftGreenRelease)
	{
		driver->moveDirection(0, 1, 0, config.button_move_speed);
	}


	if(button_box->topGreenPress)
	{
		driver->moveDirection(1, 0, 0, config.button_move_speed);
	}
	if(button_box->topGreenButton)
	{
		driver->sendDirection(config.button_move_speed);
	}
	if(button_box->topGreenRelease)
	{
		driver->moveDirection(-1, 0, 0, config.button_move_speed);
	}

	if(button_box->bottomGreenPress)
	{
		driver->moveDirection(-1, 0, 0, config.button_move_speed);
	}
	if(button_box->bottomGreenButton)
	{
		driver->sendDirection(config.button_move_speed);
	}
	if(button_box->bottomGreenRelease)
	{
		driver->moveDirection(1, 0, 0, config.button_move_speed);
	}
}

void evaluateCommands(const frc_msgs::JoystickStateConstPtr& joystick_state, int joystick_id) {

	//Only do this for the first joystick
	if(joystick_id == 0)
	{
		static ros::Time last_header_stamp = ros::Time(0);
		last_header_stamp = driver->evalateDriverCommands(*joystick_state, config);

		if(!diagnostics_mode)
		{
			//Joystick1: buttonA
			//Joystick1: buttonA
			if(joystick_state->buttonAPress)
			{
				ROS_INFO_STREAM("teleop_joystick_comp_2023 : snapping to nearest cone and enabling robot relative driving mode!");
				if (teleop_joystick_control::SnapConeCube srv; snapConeCubeSrv.call(srv))
				{
					if (srv.response.nearest_cone_angle > -900) {
						ROS_INFO_STREAM("Using angle of " << srv.response.nearest_cone_angle);
						driver->setTargetOrientation(srv.response.nearest_cone_angle, true /*from teleop*/);
						driver->teleop_cmd_vel_.setRobotOrient(true, 0);
					}
				}
			}
			if(joystick_state->buttonAButton)
			{
				
			}
			if(joystick_state->buttonARelease)
			{
				driver->teleop_cmd_vel_.setRobotOrient(false, 0);
			}

			//Joystick1: buttonB
			if(joystick_state->buttonBPress)
			{
				ROS_INFO_STREAM("teleop_joystick_comp_2023 : snapping to nearest cube and enabling robot relative driving mode!");
				if (teleop_joystick_control::SnapConeCube srv; snapConeCubeSrv.call(srv))
				{
					if (srv.response.nearest_cube_angle > -900) {
						ROS_INFO_STREAM_THROTTLE(1, "Using angle of " << srv.response.nearest_cube_angle);
						driver->setTargetOrientation(srv.response.nearest_cube_angle, true /*from teleop*/);
						driver->teleop_cmd_vel_.setRobotOrient(true, 0);
					}
				}
			}
			if(joystick_state->buttonBButton)
			{	
			}
			if(joystick_state->buttonBRelease)
			{
				driver->teleop_cmd_vel_.setRobotOrient(false, 0);
			}

			//Joystick1: buttonX
			if(joystick_state->buttonXPress)
			{

			}
			if(joystick_state->buttonXButton)
			{
			}
			if(joystick_state->buttonXRelease)
			{
			}

			//Joystick1: buttonY
			if(joystick_state->buttonYPress)
			{

			}
			if(joystick_state->buttonYButton)
			{
			}
			if(joystick_state->buttonYRelease)
			{

			}

			//Joystick1: bumperLeft
			if(joystick_state->bumperLeftPress)
			{
				driver->teleop_cmd_vel_.setCaps(config.max_speed_slow, config.max_rot_slow);
			}
			if(joystick_state->bumperLeftButton)
			{
			}
			if(joystick_state->bumperLeftRelease)
			{
				driver->teleop_cmd_vel_.resetCaps();
			}

			//Joystick1: bumperRight
			if(joystick_state->bumperRightPress)
			{
				behavior_actions::Intaking2023Goal goal;
				goal.outtake = goal.OUTTAKE_CONE;
				intaking_ac->sendGoal(goal);
			}
			if(joystick_state->bumperRightButton)
			{
			}
			if(joystick_state->bumperRightRelease)
			{
				intaking_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
			}


			// Should be the dpad right here

			//Joystick1: directionLeft
			if(joystick_state->directionLeftPress)
			{
				
			}
			if(joystick_state->directionLeftButton)
			{

			}
			else
			{
			}
			if(joystick_state->directionLeftRelease)
			{

			}

			//Joystick1: directionRight
			if(joystick_state->directionRightPress)
			{
			}
			if(joystick_state->directionRightButton)
			{
			}
			if(joystick_state->directionRightRelease)
			{
			}

			//Joystick1: directionUp
			if(joystick_state->directionUpPress)
			{
			}
			if(joystick_state->directionUpButton)
			{
			}
			if(joystick_state->directionUpRelease)
			{
			}

			//Joystick1: directionDown
			if(joystick_state->directionDownPress)
			{
			}
			if(joystick_state->directionDownButton)
			{
			}
			if(joystick_state->directionDownRelease)
			{
			}

			// end dpad


			//Joystick1: stickLeft
			if(joystick_state->stickLeftPress)
			{
			}
			if(joystick_state->stickLeftButton)
			{
			}
			else
			{
			}
			if(joystick_state->stickLeftRelease)
			{
			}

#ifdef ROTATION_WITH_STICK
			if(joystick_state->leftTrigger > config.trigger_threshold)
			{
				ROS_INFO_STREAM("LEFT TRIGGER");
				if(!joystick1_left_trigger_pressed)
				{
					behavior_actions::DriveToObjectGoal goal;
					goal.id = "note";
					goal.distance_away = 0.6;
					goal.tolerance = 0.1; // todo make configurable
					auto_note_pickup_ac->sendGoal(goal);
					ROS_WARN_STREAM("Auto picking up notes");
				}

				joystick1_left_trigger_pressed = true;
			}
			else
			{
				if(joystick1_left_trigger_pressed)
				{
					auto_note_pickup_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
				}

				joystick1_left_trigger_pressed = false;
			}

			//Joystick1: rightTrigger
			if(joystick_state->rightTrigger > config.trigger_threshold)
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
			if(abs(joystick_state->leftStickY) > config.stick_threshold)
			{
			}

			//Joystick1 Diagnostics: leftStickX
			if(abs(joystick_state->leftStickX) > config.stick_threshold)
			{
			}

			//Joystick1 Diagnostics: rightStickY
			if(abs(joystick_state->rightStickY) > config.stick_threshold)
			{
			}

			//Joystick1 Diagnostics: rightStickX
			if(abs(joystick_state->rightStickX) > config.stick_threshold)
			{
			}
#endif

			//Joystick1 Diagnostics: stickLeft
			if(joystick_state->stickLeftPress)
			{
			}
			if(joystick_state->stickLeftButton)
			{
			}
			if(joystick_state->stickLeftRelease)
			{
			}

			//Joystick1 Diagnostics: stickRight
			if(joystick_state->stickRightPress)
			{
			}
			if(joystick_state->stickRightButton)
			{
			}
			if(joystick_state->stickRightRelease)
			{
			}

			//Joystick1 Diagnostics: buttonA
			if(joystick_state->buttonAPress)
			{
			}
			if(joystick_state->buttonAButton)
			{
			}
			if(joystick_state->buttonARelease)
			{
			}

			//Joystick1 Diagnostics: buttonB
			if(joystick_state->buttonBPress)
			{
			}
			if(joystick_state->buttonBButton)
			{
			}
			if(joystick_state->buttonBRelease)
			{
			}

			//Joystick1 Diagnostics: buttonX
			if(joystick_state->buttonXPress)
			{
			}
			if(joystick_state->buttonXButton)
			{
			}
			if(joystick_state->buttonXRelease)
			{
			}

			//Joystick1 Diagnostics: buttonY
			if(joystick_state->buttonYPress)
			{
			}
			if(joystick_state->buttonYButton)
			{
			}
			if(joystick_state->buttonYRelease)
			{
			}

			//Joystick1: buttonBack
			if(joystick_state->buttonBackPress)
			{
			}
			if(joystick_state->buttonBackButton)
			{
			}
			if(joystick_state->buttonBackRelease)
			{
			}

			//Joystick1: buttonStart
			if(joystick_state->buttonStartPress)
			{
			}
			if(joystick_state->buttonStartButton)
			{
			}
			if(joystick_state->buttonStartRelease)
			{
			}

			//Joystick1 Diagnostics: bumperLeft
			if(joystick_state->bumperLeftPress)
			{
			}
			if(joystick_state->bumperLeftButton)
			{
			}
			if(joystick_state->bumperLeftRelease)
			{
			}

			//Joystick1 Diagnostics: bumperRight
			if(joystick_state->bumperRightPress)
			{
			}
			if(joystick_state->bumperRightButton)
			{
			}
			if(joystick_state->bumperRightRelease)
			{
			}

			//Joystick1 Diagnostics: leftTrigger
			if(joystick_state->leftTrigger > config.trigger_threshold)
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
			if(joystick_state->rightTrigger > config.trigger_threshold)
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
			if(joystick_state->directionLeftPress)
			{
			}
			if(joystick_state->directionLeftButton)
			{
			}
			if(joystick_state->directionLeftRelease)
			{
			}

			//Joystick1 Diagnostics: directionRight
			if(joystick_state->directionRightPress)
			{
			}
			if(joystick_state->directionRightButton)
			{
			}
			if(joystick_state->directionRightRelease)
			{
			}

			//Joystick1 Diagnostics: directionUp
			if(joystick_state->directionUpPress)
			{
			}
			if(joystick_state->directionUpButton)
			{
			}
			if(joystick_state->directionUpRelease)
			{
			}

			//Joystick1 Diagnostics: directionDown
			if(joystick_state->directionDownPress)
			{
			}
			if(joystick_state->directionDownButton)
			{
			}
			if(joystick_state->directionDownRelease)
			{
			}
		}

		last_header_stamp = joystick_state->header.stamp;
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
	// TODO: Abstract this code as well?
	ros::init(argc, argv, "Joystick_controller");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "teleop_params");
	ros::NodeHandle n_diagnostics_params(n, "teleop_diagnostics_params");
	ros::NodeHandle n_swerve_params(n, "/frcrobot_jetson/swerve_drive_controller");

	auto_calculator.register_publisher(n);

	if(!n_params.getParam("max_speed_elevator_extended", config2023.max_speed_elevator_extended))
	{
		ROS_ERROR("Could not read max_speed_elevator_extended in teleop_joystick_comp");
	}
	if(!n_params.getParam("max_rot_elevator_extended", config2023.max_rot_elevator_extended))
	{
		ROS_ERROR("Could not read max_rot_elevator_extended in teleop_joystick_comp");
	}	
	if(!n_params.getParam("cone_tolerance", config2023.cone_tolerance))
	{
		ROS_ERROR("Could not read cone_tolerance in teleop_joystick_comp");
	}
	if(!n_params.getParam("cube_tolerance", config2023.cube_tolerance))
	{
		ROS_ERROR("Could not read cube_tolerance in teleop_joystick_comp");
	}

	TeleopInitializer initializer;
	initializer.set_n_params(n_params);
	
	initializer.add_custom_var( DDRVariable {"max_speed_elevator_extended", config2023.max_speed_elevator_extended, "Max linear speed in elevator extended mode, in m/s", 0., 2} );
	initializer.add_custom_var( DDRVariable {"max_rot_elevator_extended", config2023.max_rot_elevator_extended, "Max angular speed in elevator extended mode", 0., 1.} );
	initializer.add_custom_var( DDRVariable {"cone_tolerance", config2023.cone_tolerance, "cone_tolerance", 0.0, 0.5} );
	initializer.add_custom_var( DDRVariable {"cube_tolerance", config2023.cube_tolerance, "cube_tolerance", 0.0, 0.5} );

	FourbarRezeroSrv = n.serviceClient<std_srvs::Empty>("/frcrobot_jetson/four_bar_controller_2023/rezero_service", false, {{"tcp_nodelay", "1"}});

	ros::Subscriber talon_states_sub = n.subscribe("/frcrobot_jetson/talonfxpro_states", 1, &talonFXProStateCallback);

	auto_note_pickup_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::DriveToObjectAction>>("/drive_to_object/drive_to_object", true);

	intaking_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::Intaking2023Action>>("/intaking/intaking_server_2023", true);
	placing_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::Placing2023Action>>("/placing/placing_server_2023", true);
	pathing_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::FourbarElevatorPath2023Action>>("/fourbar_elevator_path/fourbar_elevator_path_server_2023", true);
	align_and_place_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::AlignAndPlaceGrid2023Action>>("/align_and_place_grid", true);

	ros::Subscriber button_box_sub = n.subscribe("/frcrobot_rio/button_box_states", 1, &buttonBoxCallback);

	initializer.init();

	return 0;
}