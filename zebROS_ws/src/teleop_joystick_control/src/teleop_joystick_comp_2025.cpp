// Toggle between rotation using rightStickX and rightTrigger - leftTrigger
#define ROTATION_WITH_STICK

#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Empty.h"

#include "frc_msgs/ButtonBoxState2025.h"
#include "frc_msgs/JoystickState.h"
#include "imu_zero_msgs/ImuZeroAngle.h"

//#define NEED_JOINT_STATES
#ifdef NEED_JOINT_STATES
#include "sensor_msgs/JointState.h"
#endif
#include "actionlib/client/simple_action_client.h"

// #include <talon_state_msgs/TalonFXProState.h>

#include "teleop_joystick_control/teleop_joystick_comp_general.h"

#include <path_follower_msgs/PathAction.h>

#include <behavior_actions/AlignAndPlace2025Action.h>
#include <behavior_actions/Elevater2025Action.h>
#include <behavior_actions/Intaking2025Action.h>
#include <behavior_actions/Placing2025Action.h>
#include <behavior_actions/Roller2025Action.h>
#include <behavior_actions/PulseOuttake2025Action.h>

#include "talon_controller_msgs/Command.h"

class AutoModeCalculator2025 : public AutoModeCalculator {
public:
	explicit AutoModeCalculator2025(ros::NodeHandle &n)
		: AutoModeCalculator(n)
	{
	}
	void set_auto_mode(const uint8_t auto_mode) {
		auto_mode_ = auto_mode;
	}
private:
	uint8_t calculateAutoMode() override {
		return auto_mode_;
	}
	uint8_t auto_mode_{1};
};

auto current_level = behavior_actions::AlignAndPlace2025Goal::L4;

std::unique_ptr<AutoModeCalculator2025> auto_calculator;

// TODO: Add 2025 versions, initialize in main before calling generic inititalizer
std::unique_ptr<actionlib::SimpleActionClient<path_follower_msgs::PathAction>> path_follower_ac; // just in case
std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::AlignAndPlace2025Action>> align_and_place_ac; // usual method of placing with alignment
std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::Elevater2025Action>> elevater_ac; // for manual elevator movements to get unstuck
std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::Intaking2025Action>> intaking_ac; // manual intaking
std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::Placing2025Action>> placing_ac; // manual placement (very sad)
std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::Roller2025Action>> roller_ac; // manual eject (very sad)

std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::PulseOuttake2025Action>> pulse_outtake_ac; // manual eject (very sad)



ros::ServiceClient toggle_auto_rotate_client;
ros::ServiceClient outtaking_client;
ros::ServiceClient toggle_no_motion_calibration_client;

bool currently_outtaking = false;
bool not_safe = true;
bool placing_in_safe = false;
// void talonFXProStateCallback(const talon_state_msgs::TalonFXProStateConstPtr &talon_state)
// {    
// 	ROS_WARN("Calling unimplemented function \"talonFXProStateCallback()\" in teleop_joystick_comp_2025.cpp ");
// }

void evaluateCommands(const frc_msgs::JoystickStateConstPtr& joystick_state, int joystick_id)	
{
	//Only do this for the first joystick
	if(joystick_id == 0) {
		static ros::Time last_header_stamp = ros::Time(0);
		last_header_stamp = driver->evaluateDriverCommands(*joystick_state, config);

		if(!diagnostics_mode)
		{
			//Joystick1: buttonA (B on apex controller / right button)
			if(joystick_state->buttonAPress)
			{	
			}
			if(joystick_state->buttonAButton)
			{
				
			}
			if(joystick_state->buttonARelease)
			{
			}

			//Joystick1: buttonB (actually A on apex controller / bottom button)
			if(joystick_state->buttonBPress)
			{
				current_level = behavior_actions::AlignAndPlace2025Goal::L2;
			}
			if(joystick_state->buttonBButton)
			{	
			}
			if(joystick_state->buttonBRelease)
			{
			}

			//Joystick1: buttonX (button Y on controller with replaced white buttons)
			// this is the top button on the ABXY set
			if(joystick_state->buttonXPress)
			{
				current_level = behavior_actions::AlignAndPlace2025Goal::L4;
			}
			if(joystick_state->buttonXButton)
			{
			}
			if(joystick_state->buttonXRelease)
			{
			}

			//Joystick1: buttonY (X on apex / left button)
			if(joystick_state->buttonYPress)
			{
				current_level = behavior_actions::AlignAndPlace2025Goal::L3;
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
			}
			if(joystick_state->bumperLeftButton)
			{
			}
			if(joystick_state->bumperLeftRelease)
			{
			}

			//Joystick1: bumperRight
			if(joystick_state->bumperRightPress)
			{
				ROS_INFO_STREAM("Sending pulse outtake goal");
				behavior_actions::PulseOuttake2025Goal outtake_goal_;
				pulse_outtake_ac->sendGoal(outtake_goal_);
				//driver->teleop_cmd_vel_.setCaps(config.max_speed_slow, config.max_rot_slow);
			}
			if(joystick_state->bumperRightButton)
			{
			}
			if(joystick_state->bumperRightRelease)
			{
				//driver->teleop_cmd_vel_.resetCaps();
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
				if(!joystick1_left_trigger_pressed)
				{
					if (not_safe) { // not safe :)
						ROS_INFO_STREAM("Sending align and place goal LEFT");
						behavior_actions::AlignAndPlace2025Goal align_goal_;
						align_goal_.pipe = align_goal_.LEFT_PIPE;
						align_goal_.level = current_level;
						align_and_place_ac->sendGoal(align_goal_);
					}
					else { // safe :(
						// elevator up, want to score
						if (placing_in_safe) {
							ROS_INFO_STREAM("Placing in safe mode and moving elevator down!");
							// send placing goal
							behavior_actions::Placing2025Goal place_goal;
							place_goal.level = current_level;
							place_goal.setup_only = false; // shoot the coral
							placing_ac->sendGoal(place_goal);
							// set placing in safe to false
							placing_in_safe = false;
						}
						else {
							// elevator not up, want to bring it up 
							ROS_INFO_STREAM("Sending just place goal (setup only)! LEFT");
							behavior_actions::Placing2025Goal place_goal;
							place_goal.level = current_level;
							place_goal.setup_only = true; // need time to manually align
							placing_ac->sendGoal(place_goal);
							// set placing in safe to false
							placing_in_safe = true;
						}
					}
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
			if(joystick_state->rightTrigger > config.trigger_threshold)
			{
				if(!joystick1_right_trigger_pressed)
				{	
					if (not_safe) {
						ROS_INFO_STREAM("Sending align and place goal RIGHT");
						behavior_actions::AlignAndPlace2025Goal align_goal_;
						align_goal_.pipe = align_goal_.RIGHT_PIPE;
						align_goal_.level = current_level;
						align_and_place_ac->sendGoal(align_goal_);
					}
					else { // safe :(
						// elevator up, want to score
						if (placing_in_safe) {
							ROS_INFO_STREAM("Placing in safe mode and moving elevator down!");
							// send placing goal
							behavior_actions::Placing2025Goal place_goal;
							place_goal.level = current_level;
							place_goal.setup_only = false; // shoot the coral
							placing_ac->sendGoal(place_goal);
							// set placing in safe to false
							placing_in_safe = false;
						}
						else {
							// elevator not up, want to bring it up 
							ROS_INFO_STREAM("Sending just place goal (setup only)! RIGHT");
							behavior_actions::Placing2025Goal place_goal;
							place_goal.level = current_level;
							place_goal.setup_only = true; // need time to manually align
							placing_ac->sendGoal(place_goal);
							// set placing in safe to false
							placing_in_safe = true;
						}
					}
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

			// //Joystick1 Diagnostics: stickLeft
			// if(joystick_state->stickLeftPress)
			// {
			// }
			// if(joystick_state->stickLeftButton)
			// {
			// }
			// if(joystick_state->stickLeftRelease)
			// {
			// }

			// //Joystick1 Diagnostics: stickRight
			// if(joystick_state->stickRightPress)
			// {
			// }
			// if(joystick_state->stickRightButton)
			// {
			// }
			// if(joystick_state->stickRightRelease)
			// {
			// }

			// //Joystick1 Diagnostics: buttonA
			// if(joystick_state->buttonAPress)
			// {
			// }
			// if(joystick_state->buttonAButton)
			// {
			// }
			// if(joystick_state->buttonARelease)
			// {
			// }

			// //Joystick1 Diagnostics: buttonB
			// if(joystick_state->buttonBPress)
			// {
			// }
			// if(joystick_state->buttonBButton)
			// {
			// }
			// if(joystick_state->buttonBRelease)
			// {
			// }

			// //Joystick1 Diagnostics: buttonX
			// if(joystick_state->buttonXPress)
			// {
			// }
			// if(joystick_state->buttonXButton)
			// {
			// }
			// if(joystick_state->buttonXRelease)
			// {
			// }

			// //Joystick1 Diagnostics: buttonY
			// if(joystick_state->buttonYPress)
			// {
			// }
			// if(joystick_state->buttonYButton)
			// {
			// }
			// if(joystick_state->buttonYRelease)
			// {
			// }

			// //Joystick1: buttonBack
			// if(joystick_state->buttonBackPress)
			// {
			// }
			// if(joystick_state->buttonBackButton)
			// {
			// }
			// if(joystick_state->buttonBackRelease)
			// {
			// }

			// //Joystick1: buttonStart
			// if(joystick_state->buttonStartPress)
			// {
			// }
			// if(joystick_state->buttonStartButton)
			// {
			// }
			// if(joystick_state->buttonStartRelease)
			// {
			// }

			// //Joystick1 Diagnostics: bumperLeft
			// if(joystick_state->bumperLeftPress)
			// {
			// }
			// if(joystick_state->bumperLeftButton)
			// {
			// }
			// if(joystick_state->bumperLeftRelease)
			// {
			// }

			// //Joystick1 Diagnostics: bumperRight
			// if(joystick_state->bumperRightPress)
			// {
			// }
			// if(joystick_state->bumperRightButton)
			// {
			// }
			// if(joystick_state->bumperRightRelease)
			// {
			// }

			//Joystick1 Diagnostics: leftTrigger
			if(joystick_state->leftTrigger > config.trigger_threshold)
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
			//Joystick1 Diagnostics: rightTrigger
			if(joystick_state->rightTrigger > config.trigger_threshold)
			{
				if(!joystick1_right_trigger_pressed)
				{
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

			// //Joystick1 Diagnostics: directionLeft
			// if(joystick_state->directionLeftPress)
			// {
			// }
			// if(joystick_state->directionLeftButton)
			// {
			// }
			// if(joystick_state->directionLeftRelease)
			// {
			// }

			// //Joystick1 Diagnostics: directionRight
			// if(joystick_state->directionRightPress)
			// {
			// }
			// if(joystick_state->directionRightButton)
			// {
			// }
			// if(joystick_state->directionRightRelease)
			// {
			// }

			// //Joystick1 Diagnostics: directionUp
			// if(joystick_state->directionUpPress)
			// {
			// }
			// if(joystick_state->directionUpButton)
			// {
			// }
			// if(joystick_state->directionUpRelease)
			// {
			// }

			// //Joystick1 Diagnostics: directionDown
			// if(joystick_state->directionDownPress)
			// {
			// }
			// if(joystick_state->directionDownButton)
			// {
			// }
			// if(joystick_state->directionDownRelease)
			// {
			// }
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

bool aligning = false;

void buttonBoxCallback(const frc_msgs::ButtonBoxState2025ConstPtr &button_box)
{
	if (button_box->notSafeModeLockingSwitchButton)
	{
	}
	else
	{
	}
	if (button_box->notSafeModeLockingSwitchPress)
	{
		std_srvs::SetBool auto_rotate_enable_srv;
		auto_rotate_enable_srv.request.data = true;
		toggle_auto_rotate_client.call(auto_rotate_enable_srv);
		not_safe = true;
		ROS_WARN_STREAM("NOT SAFE MODE ACTIVATED ! :)");
	}
	if (button_box->notSafeModeLockingSwitchRelease)
	{
		std_srvs::SetBool auto_rotate_disable_srv;
		auto_rotate_disable_srv.request.data = false;
		toggle_auto_rotate_client.call(auto_rotate_disable_srv);
		not_safe = false;
		ROS_WARN_STREAM("SAFE MODE ACTIVATED :(");
	}

	// TODO We'll probably want to check the actual value here
	auto_calculator->set_auto_mode(button_box->auto_mode);

	if(button_box->zeroButton) {
	}
	if(button_box->zeroPress) {
		// for zeroing, assuming the robot starts facing away from the speaker
		imu_zero_msgs::ImuZeroAngle imu_cmd;
		if (alliance_color == frc_msgs::MatchSpecificData::ALLIANCE_COLOR_RED) {
			ROS_INFO_STREAM("teleop_joystick_comp_2025 : red alliance");
			imu_cmd.request.angle = 180.0;
		} else {
			ROS_INFO_STREAM("teleop_joystick_comp_2025 : blue or unknown alliance");
			imu_cmd.request.angle = 0.0;
		}
		ROS_INFO_STREAM("teleop_joystick_comp_2025 : zeroing IMU to " << imu_cmd.request.angle);
		IMUZeroSrv.call(imu_cmd);

		ROS_INFO_STREAM("teleop_joystick_comp_2025 : DISABLE NO MOTION CALIBRATION = TRUE");
		std::srvs::SetBool disable_no_motion_calibration_srv;
		disable_no_motion_calibration_srv.request.data = true;
		toggle_no_motion_calibration_client.call(disable_no_motion_calibration_srv);
		// ROS_INFO_STREAM("teleop_joystick_comp_2025 : zeroing swerve odom");
		// std_srvs::Empty odom_cmd;
		// SwerveOdomZeroSrv.call(odom_cmd);
	}
	if(button_box->zeroRelease) {
	}

	if (button_box->redButton)
	{
	}
	if (button_box->redPress)
	{
		ROS_WARN_STREAM("teleop_joystick_comp_2025: PREEMPTING all actions");
		path_follower_ac->cancelAllGoals();
		elevater_ac->cancelAllGoals();
		intaking_ac->cancelAllGoals();
		align_and_place_ac->cancelAllGoals();
		placing_ac->cancelAllGoals();
		driver->setJoystickOverride(false);
	}
	if (button_box->redRelease)
	{
	}

	if (button_box->outtakeButton)
	{
	}
	if (button_box->outtakePress)
	{
		talon_controller_msgs::Command outtake_srv;
		outtake_srv.request.command = (!currently_outtaking) ? -3.0 : 0.0; // hardcoded voltage yippee
		outtaking_client.call(outtake_srv);

		currently_outtaking = !currently_outtaking;
	}
	if (button_box->outtakeRelease)
	{
	}

	if (button_box->backupButton2Button)
	{

	}
	if (button_box->backupButton2Press)
	{
		behavior_actions::Roller2025Goal roller_goal;
		roller_goal.mode = roller_goal.OUTTAKE;
		roller_ac->sendGoal(roller_goal);
	}
	if (button_box->backupButton2Release)
	{
	}

	if (button_box->elevatorL2Button)
	{
	}
	if (button_box->elevatorL2Press)
	{
		ROS_INFO_STREAM("Sending elevater goal UP to L2");
		behavior_actions::Elevater2025Goal elevater_goal_;
		elevater_goal_.mode = behavior_actions::Elevater2025Goal::L2;
		elevater_ac->sendGoal(elevater_goal_);
	}
	if (button_box->elevatorL2Release)
	{
	}

	if (button_box->elevatorRetractButton)
	{
	}
	if (button_box->elevatorRetractPress)
	{

		ROS_INFO_STREAM("Sending elevater goal DOWN to INTAKE");
		behavior_actions::Elevater2025Goal elevater_goal_;
		elevater_goal_.mode = behavior_actions::Elevater2025Goal::INTAKE;
		elevater_ac->sendGoal(elevater_goal_);
	}
	if (button_box->elevatorRetractRelease)
	{
	}

	if (button_box->intakeButton)
	{
	}
	if (button_box->intakePress)
	{
		ROS_INFO_STREAM("Sending intaking manual goal in teleop");
		behavior_actions::Intaking2025Goal intaking_goal_;
		intaking_ac->sendGoal(intaking_goal_);
	}
	if (button_box->intakeRelease)
	{
	}

	if (button_box->rightSwitchUpButton)
	{
	}
	if (button_box->rightSwitchUpPress)
	{
	}
	if (button_box->rightSwitchUpRelease)
	{
	}

	if (button_box->rightSwitchDownButton)
	{
	}
	if (button_box->rightSwitchDownPress)
	{
	}
	if (button_box->rightSwitchDownRelease)
	{
	}

	// Switch in middle position
	if (!(button_box->rightSwitchDownButton || button_box->rightSwitchUpButton))
	{
	}

	if (button_box->leftSwitchUpButton)
	{
	}
	if (button_box->leftSwitchUpPress)
	{
	}
	if (button_box->leftSwitchUpRelease)
	{
	}

	if (button_box->leftSwitchDownButton)
	{
	}
	if (button_box->leftSwitchDownPress)
	{
	}
	if (button_box->leftSwitchDownRelease)
	{
	}

	// Switch in middle position
	if (!(button_box->leftSwitchDownButton || button_box->leftSwitchUpButton))
	{
	}


	if (button_box->rightGreenPress)
	{
		driver->moveDirection(0, 1, 0, config.button_move_speed);
	}
	if (button_box->rightGreenButton)
	{
		driver->sendDirection(config.button_move_speed);
	}
	if (button_box->rightGreenRelease)
	{
		driver->moveDirection(0, -1, 0, config.button_move_speed);
	}

	if (button_box->leftGreenPress)
	{
		driver->moveDirection(0, -1, 0, config.button_move_speed);
	}
	if (button_box->leftGreenButton)
	{
		driver->sendDirection(config.button_move_speed);
	}
	if (button_box->leftGreenRelease)
	{
		driver->moveDirection(0, 1, 0, config.button_move_speed);
	}

	if (button_box->topGreenPress)
	{
		driver->moveDirection(1, 0, 0, config.button_move_speed);
	}
	if (button_box->topGreenButton)
	{
		driver->sendDirection(config.button_move_speed);
	}
	if (button_box->topGreenRelease)
	{
		driver->moveDirection(-1, 0, 0, config.button_move_speed);
	}

	if (button_box->bottomGreenPress)
	{
		driver->moveDirection(-1, 0, 0, config.button_move_speed);
	}
	if (button_box->bottomGreenButton)
	{
		driver->sendDirection(config.button_move_speed);
	}
	if (button_box->bottomGreenRelease)
	{
		driver->moveDirection(1, 0, 0, config.button_move_speed);
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

	auto_calculator = std::make_unique<AutoModeCalculator2025>(n);
	path_follower_ac = std::make_unique<actionlib::SimpleActionClient<path_follower_msgs::PathAction>>("/path_follower/path_follower_server", true);
	align_and_place_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::AlignAndPlace2025Action>>("/align_and_place/alignandplaceing_server", true);
	elevater_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::Elevater2025Action>>("/elevater/elevater_server_2025", true);
	intaking_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::Intaking2025Action>>("/intaking/intaking_server_2025", true);
	placing_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::Placing2025Action>>("/placing/placing_server_2025", true);
	roller_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::Roller2025Action>>("/roller/roller_server_2025", true);
	pulse_outtake_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::PulseOuttake2025Action>>("/pulse_outtake/pulse_server_2025", true);

	ros::Subscriber button_box_sub = n.subscribe("/frcrobot_rio/button_box_states", 1, &buttonBoxCallback);

	outtaking_client = n.serviceClient<talon_controller_msgs::Command>("/frcrobot_jetson/intake_controller/command", false, {{"tcp_nodelay", "1"}});
	toggle_auto_rotate_client = n.serviceClient<std_srvs::SetBool>("/auto_rotating/toggle_auto_rotate", false, {{"tcp_nodelay", "1"}});

	toggle_no_motion_calibration_client = n.serviceClient<std_srvs::SetBool>("/frcrobot_jetson/pigeon2_controller/disable_no_motion_calibration", false, {{"tcp_nodelay", "1"}});

	TeleopInitializer initializer;
	initializer.set_n_params(n_params);
	initializer.init(); // THIS SPINS
	return 0;
}
