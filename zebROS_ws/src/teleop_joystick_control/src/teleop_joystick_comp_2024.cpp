// Toggle between rotation using rightStickX and rightTrigger - leftTrigger
#define ROTATION_WITH_STICK

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"

#include "frc_msgs/ButtonBoxState2024.h"
#include "frc_msgs/JoystickState.h"
#include "imu_zero_msgs/ImuZeroAngle.h"

#include "behavior_actions/Intaking2024Action.h"
#include "behavior_actions/DriveObjectIntake2024Action.h"
#include "behavior_actions/DriveToObjectAction.h"

#include "behavior_actions/Shooting2024Action.h"
#include "behavior_actions/AlignToSpeaker2024Action.h"
//#define NEED_JOINT_STATES
#ifdef NEED_JOINT_STATES
#include "sensor_msgs/JointState.h"
#endif
#include "actionlib/client/simple_action_client.h"

#include <talon_state_msgs/TalonFXProState.h>

#include "teleop_joystick_control/teleop_joystick_comp_general.h"

#include "behavior_actions/AlignAndShoot2024Action.h"
#include "behavior_actions/AlignToTrap2024Action.h"
#include "behavior_actions/DriveAndScore2024Action.h"


#include "behavior_actions/Climb2024Action.h"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

class AutoModeCalculator2024 : public AutoModeCalculator {
public:
	explicit AutoModeCalculator2024(ros::NodeHandle &n)
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

std::unique_ptr<AutoModeCalculator2024> auto_calculator;

// TODO: Add 2024 versions, initialize in main before calling generic inititalizer
std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::Intaking2024Action>> intaking_ac;
std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::Shooting2024Action>> shooting_ac;
std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::AlignAndShoot2024Action>> aligned_shooting_ac;
std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::DriveObjectIntake2024Action>> drive_and_intake_ac;
std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::AlignToSpeaker2024Action>> align_to_speaker_ac;
std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::AlignToTrap2024Action>> align_to_trap_ac;
std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::Climb2024Action>> climb_ac;
std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::DriveAndScore2024Action>> drive_and_score_ac;
std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::DriveAndScore2024Action>> drive_to_object_ac;


bool reset_climb = true;

ros::ServiceClient enable_continuous_autoalign_client;

void talonFXProStateCallback(const talon_state_msgs::TalonFXProStateConstPtr &talon_state)
{    
	ROS_WARN("Calling unimplemented function \"talonFXProStateCallback()\" in teleop_joystick_comp_2024.cpp ");
}

void evaluateCommands(const frc_msgs::JoystickStateConstPtr& joystick_state, int joystick_id)	
{
	//Only do this for the first joystick
	if(joystick_id == 0) {
		static ros::Time last_header_stamp = ros::Time(0);
		last_header_stamp = driver->evalateDriverCommands(*joystick_state, config);

		if(!diagnostics_mode)
		{
			//Joystick1: buttonA
			//Joystick1: buttonA
			if(joystick_state->buttonAPress)
			{	
				// now just auto score 
				// behavior_actions::Shooting2024Goal goal;
				// goal.mode = goal.AMP;
				// shooting_ac->sendGoal(goal);
				behavior_actions::DriveAndScore2024Goal goal;
				goal.destination = goal.AMP;
				drive_and_score_ac->sendGoal(goal);
			}
			if(joystick_state->buttonAButton)
			{
				
			}
			if(joystick_state->buttonARelease)
			{
				//shooting_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
				drive_and_score_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
			}

			//Joystick1: buttonB
			if(joystick_state->buttonBPress)
			{

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
				behavior_actions::Shooting2024Goal goal;
				goal.mode = goal.SLIDE;
				shooting_ac->sendGoal(goal);
			}
			if(joystick_state->buttonXButton)
			{
			}
			if(joystick_state->buttonXRelease)
			{
				shooting_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
			}

			//Joystick1: buttonY
			if(joystick_state->buttonYPress)
			{
				// behavior_actions::Shooting2024Goal goal;
				// goal.mode = goal.PODIUM;
				// shooting_ac->sendGoal(goal);
			}
			if(joystick_state->buttonYButton)
			{
			}
			if(joystick_state->buttonYRelease)
			{
				// shooting_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());	
			}

			//Joystick1: bumperLeft
			if(joystick_state->bumperLeftPress)
			{
				behavior_actions::Intaking2024Goal intaking_goal;
				intaking_goal.destination = intaking_goal.SHOOTER;
				intaking_ac->sendGoal(intaking_goal);
			}
			if(joystick_state->bumperLeftButton)
			{
			}
			if(joystick_state->bumperLeftRelease)
			{
				//intaking_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
			}

			//Joystick1: bumperRight
			if(joystick_state->bumperRightPress)
			{
				driver->teleop_cmd_vel_.setCaps(config.max_speed_slow, config.max_rot_slow);
				//behavior_actions::Shooting2024Goal goal;
				//goal.mode = goal.AMP;
				//shooting_ac->sendGoal(goal);
			}
			if(joystick_state->bumperRightButton)
			{
			}
			if(joystick_state->bumperRightRelease)
			{
				driver->teleop_cmd_vel_.resetCaps();
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
					behavior_actions::DriveObjectIntake2024Goal goal;
					goal.destination = goal.SHOOTER;
					drive_and_intake_ac->sendGoal(goal);
				}

				joystick1_left_trigger_pressed = true;
			}
			else
			{
				if(joystick1_left_trigger_pressed)
				{
					drive_and_intake_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
				}

				joystick1_left_trigger_pressed = false;
			}

			//Joystick1: rightTrigger
			if(joystick_state->rightTrigger > config.trigger_threshold)
			{
				if(!joystick1_right_trigger_pressed)
				{
					behavior_actions::AlignAndShoot2024Goal goal;
					aligned_shooting_ac->sendGoal(goal);
				}

				joystick1_right_trigger_pressed = true;
			}
			else
			{
				if(joystick1_right_trigger_pressed)
				{
					aligned_shooting_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
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
				behavior_actions::Shooting2024Goal goal;
				goal.mode = goal.PODIUM;
				shooting_ac->sendGoal(goal);
			}
			if(joystick_state->buttonYButton)
			{
			}
			if(joystick_state->buttonYRelease)
			{
				shooting_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
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

bool aligning = false;

void buttonBoxCallback(const frc_msgs::ButtonBoxState2024ConstPtr &button_box)
{
	if (button_box->lockingSwitchButton)
	{
	}
	if (button_box->lockingSwitchPress)
	{
		std_srvs::SetBool srv;
		srv.request.data = true;
		enable_continuous_autoalign_client.call(srv);
		ROS_INFO_STREAM("teleop_joystick_comp_2024 : enabling continuous autoalign");
	}
	if (button_box->lockingSwitchRelease)
	{
		std_srvs::SetBool srv;
		srv.request.data = false;
		enable_continuous_autoalign_client.call(srv);
		ROS_INFO_STREAM("teleop_joystick_comp_2024 : disabling continuous autoalign");
	}

	// TODO We'll probably want to check the actual value here
	auto_calculator->set_auto_mode(button_box->auto_mode);

 
	if(button_box->zeroButton) {
	}
	if(button_box->zeroPress) {
		// for zeroing, assuming the robot starts facing away from the speaker (yes this is 2024 but we need to test it)
		imu_zero_msgs::ImuZeroAngle imu_cmd;
		if (alliance_color == frc_msgs::MatchSpecificData::ALLIANCE_COLOR_RED) {
			ROS_INFO_STREAM("teleop_joystick_comp_2024 : red alliance");
			imu_cmd.request.angle = 180.0;
		} else {
			ROS_INFO_STREAM("teleop_joystick_comp_2024 : blue or unknown alliance");
			imu_cmd.request.angle = 0.0;
		}
		ROS_INFO_STREAM("teleop_joystick_comp_2024 : zeroing IMU to " << imu_cmd.request.angle);
		IMUZeroSrv.call(imu_cmd);
		ROS_INFO_STREAM("teleop_joystick_comp_2024 : zeroing swerve odom");
		std_srvs::Empty odom_cmd;
		SwerveOdomZeroSrv.call(odom_cmd);
	}
	if(button_box->zeroRelease) {
	}

	if (button_box->redButton)
	{
	}
	if (button_box->redPress)
	{
		ROS_WARN_STREAM("teleop_joystick_comp_2024: PREEMPTING all actions");
		drive_and_intake_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		intaking_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		// send shooting goal with cancel_movement=true
		behavior_actions::Shooting2024Goal goal;
		goal.cancel_movement = true;
		shooting_ac->sendGoal(goal);
		aligned_shooting_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		align_to_speaker_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		align_to_trap_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		climb_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		driver->setJoystickOverride(false);
		std_srvs::SetBool toggle_relocalize;
		toggle_relocalize.request.data = true;
		if (!toggle_relocalize_srv_.call(toggle_relocalize)) {
			ROS_ERROR_STREAM("FAILED TO ENABLE RELOCALIZING WITH CMD_VEL");
		}
	}
	if (button_box->redRelease)
	{
	}

	if (button_box->backupButton1Button)
	{
		driver->sendDirection(config.button_move_speed);
	}
	if (button_box->backupButton1Press)
	{
		intaking_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		driver->moveDirection(-1, 0, 0, config.button_move_speed);
		behavior_actions::Intaking2024Goal intaking_goal;
		intaking_goal.destination = intaking_goal.OUTTAKE;
		intaking_ac->sendGoal(intaking_goal);
	}
	if (button_box->backupButton1Release)
	{
		driver->moveDirection(1, 0, 0, config.button_move_speed);
		intaking_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
	}

	if (button_box->backupButton2Button)
	{
	}
	if (button_box->backupButton2Press)
	{
		behavior_actions::Shooting2024Goal goal;
		goal.mode = goal.SHOT_PASS;
		shooting_ac->sendGoal(goal);
	}
	if (button_box->backupButton2Release)
	{
		shooting_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
	}

	if (button_box->trapButton)
	{
	}
	if (button_box->trapPress)
	{
		// align to trap
		// MAKE ENTIRE ALIGN AND TRAP
		// behavior_actions::AlignToTrap2024Goal goal;
		// goal.destination = goal.TRAP;
		// align_to_trap_ac->sendGoal(goal);
		behavior_actions::DriveAndScore2024Goal goal;
		goal.destination = goal.TRAP;
		drive_and_score_ac->sendGoal(goal);
	}
	
	if (button_box->trapRelease)
	{
		drive_and_score_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
	}

	if (button_box->climbButton)
	{
	}
	if (button_box->climbPress)
	{
		ROS_INFO_STREAM("teleop 2024: climbing!");
		behavior_actions::Climb2024Goal climb_goal;
		climb_goal.reset = reset_climb;
		climb_ac->sendGoal(climb_goal);
		reset_climb = false;
	}
	if (button_box->climbRelease)
	{
	}

	if (button_box->subwooferShootButton)
	{
		driver->setJoystickOverride(true);
		driver->setTargetOrientation(angles::from_degrees(270), true);
	}
	if (button_box->subwooferShootPress)
	{
		// actually align to amp
		behavior_actions::Shooting2024Goal goal;
		goal.mode = goal.AMP;
		goal.leave_spinning = true;
		goal.setup_only = true;
		shooting_ac->sendGoal(goal);

		driver->setTargetOrientation(angles::from_degrees(270), true);
	}
	if (button_box->subwooferShootRelease)
	{
		driver->setJoystickOverride(false);
	}

	if (button_box->speedSwitchUpButton)
	{
	}
	if (button_box->speedSwitchUpPress)
	{
	}
	if (button_box->speedSwitchUpRelease)
	{
	}

	if (button_box->speedSwitchDownButton)
	{
	}
	if (button_box->speedSwitchDownPress)
	{
	}
	if (button_box->speedSwitchDownRelease)
	{
	}

	// Switch in middle position
	if (!(button_box->speedSwitchDownButton || button_box->speedSwitchUpButton))
	{
	}

	if (button_box->shooterArmUpButton)
	{
	}
	if (button_box->shooterArmUpPress)
	{
	}
	if (button_box->shooterArmUpRelease)
	{
	}

	if (button_box->shooterArmDownButton)
	{
	}
	if (button_box->shooterArmDownPress)
	{
	}
	if (button_box->shooterArmDownRelease)
	{
	}

	// Switch in middle position
	if (!(button_box->shooterArmDownButton || button_box->shooterArmUpButton))
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

	auto_calculator = std::make_unique<AutoModeCalculator2024>(n);
	intaking_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::Intaking2024Action>>("/intaking/intaking_server_2024", true);
	shooting_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::Shooting2024Action>>("/shooting/shooting_server_2024", true);
	drive_and_intake_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::DriveObjectIntake2024Action>>("/intaking/drive_object_intake", true);
	aligned_shooting_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::AlignAndShoot2024Action>>("/align_and_shoot/align_and_shoot_2024", true);
	align_to_speaker_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::AlignToSpeaker2024Action>>("/align_to_speaker/align_to_speaker_2024", true);
	align_to_trap_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::AlignToTrap2024Action>>("/align_to_trap/align_to_trap_2024", true);
	climb_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::Climb2024Action>>("/climbing/climbing_server_2024", true);
	drive_and_score_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::DriveAndScore2024Action>>("/drive_and_score/drive_and_score_2024", true);
	// drive_to_object_ac = std::make_unique<actionlib::SimpleActionClient<behavior_actions::DriveToObjectAction>>("/drive_and_score/drive_and_score_2024", true);

	ros::Subscriber button_box_sub = n.subscribe("/frcrobot_rio/button_box_states", 1, &buttonBoxCallback);

	enable_continuous_autoalign_client = n.serviceClient<std_srvs::SetBool>("/align_and_shoot/enable_autoalign", false, {{"tcp_nodelay", "1"}});

	TeleopInitializer initializer;
	initializer.set_n_params(n_params);
	initializer.init(); // THIS SPINS
	return 0;
}
