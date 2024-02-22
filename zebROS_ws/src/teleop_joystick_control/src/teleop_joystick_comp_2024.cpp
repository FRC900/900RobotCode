// Toggle between rotation using rightStickX and rightTrigger - leftTrigger
#define ROTATION_WITH_STICK

#include "ros/ros.h"
#include "frc_msgs/JoystickState.h"
//#define NEED_JOINT_STATES
#ifdef NEED_JOINT_STATES
#include "sensor_msgs/JointState.h"
#endif
#include "actionlib/client/simple_action_client.h"

#include <talon_state_msgs/TalonFXProState.h>

#include "teleop_joystick_control/teleop_joystick_comp_general.h"

class AutoModeCalculator2024 : public AutoModeCalculator {
public:
	AutoModeCalculator2024() = default;
	uint8_t calculateAutoMode() override {
		return auto_mode_;
	}
	void set_auto_mode(const uint8_t auto_mode) {
		auto_mode_ = auto_mode;
	}
private:
	uint8_t auto_mode_{0};
};

AutoModeCalculator2024 auto_calculator;

// TODO: Add 2024 versions, initialize in main before calling generic inititalizer
//std::unique_ptr<actionlib::SimpleActionClient<behavior_actions::Intaking2023Action>> intaking_ac;

void talonFXProStateCallback(const talon_state_msgs::TalonFXProState talon_state)
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

			}
			if(joystick_state->buttonAButton)
			{
				
			}
			if(joystick_state->buttonARelease)
			{
				
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
				
			}
			if(joystick_state->bumperRightButton)
			{
			}
			if(joystick_state->bumperRightRelease)
			{
				
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

#if 0
void buttonBoxCallback(const frc_msgs::ButtonBoxState2024ConstPtr &button_box)
{
}
#endif

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

	TeleopInitializer initializer;
	initializer.set_n_params(n_params);
	initializer.init();
	// ros::Subscriber button_box_sub = n.subscribe("/frcrobot_rio/button_box_states", 1, &buttonBoxCallback);
	return 0;
}
