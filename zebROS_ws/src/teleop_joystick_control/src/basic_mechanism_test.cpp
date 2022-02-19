#include "ros/ros.h"
#include <cmath>
#include <string>

#include "frc_msgs/ButtonBoxState.h"
#include "frc_msgs/JoystickState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

#include <vector>

bool diagnostics_mode = false;

frc_msgs::ButtonBoxState button_box;

// array of joystick_states messages for multiple joysticks
std::vector <frc_msgs::JoystickState> joystick_states_array;
std::vector <std::string> topic_array;

std_msgs::Float64 intake_arc_cmd;
std_msgs::Float64 intake_straight_cmd;
std_msgs::Float64 shooter_cmd;
ros::Publisher intake_straight_pub;
ros::Publisher intake_arc_pub;
ros::Publisher shooter_pub;

ros::Time last_header_stamp;
double trigger_threshold = 0.5;

void buttonBoxCallback(const ros::MessageEvent<frc_msgs::ButtonBoxState const>& event)
{
	//ROS_INFO_STREAM("Button Box callback running!");

	button_box = *(event.getMessage());

	if(button_box.lockingSwitchPress)
	{
		ROS_WARN_STREAM("Enabling diagnostics mode!");
	}
	if(button_box.lockingSwitchButton)
	{
		diagnostics_mode = true;
	}
	else
	{
		diagnostics_mode = false;
	}
	if(button_box.lockingSwitchRelease)
	{
		ROS_WARN_STREAM("Disabling diagnostics mode!");
	}

	if(button_box.topRedPress)
	{
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
	else if(!button_box.rightSwitchDownButton)
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
	}
	if(button_box.yellowButton)
	{
	}
	if(button_box.yellowRelease)
	{
	}

	if(button_box.leftGreenPress)
	{
	}
	if(button_box.leftGreenButton)
	{
	}
	if(button_box.leftGreenRelease)
	{
	}

	if(button_box.rightGreenPress)
	{
	}
	if(button_box.rightGreenButton)
	{
	}
	if(button_box.rightGreenRelease)
	{
	}

	if(button_box.topGreenPress)
	{
	}
	if(button_box.topGreenButton)
	{
	}
	if(button_box.topGreenRelease)
	{
	}

	if(button_box.bottomGreenPress)
	{
	}
	if(button_box.bottomGreenButton)
	{
	}
	if(button_box.bottomGreenRelease)
	{
	}

	if(button_box.bottomSwitchUpPress)
	{
	}
	if(button_box.bottomSwitchUpButton)
	{
	}
	else if(!button_box.bottomSwitchDownButton)
	{
	}
	if(button_box.bottomSwitchUpRelease)
	{
	}

	if(button_box.bottomSwitchDownPress)
	{
	}
	if(button_box.bottomSwitchDownButton)
	{
	}
	if(button_box.bottomSwitchDownRelease)
	{
	}

	last_header_stamp = button_box.header.stamp;
}

void evaluateCommands(const ros::MessageEvent<frc_msgs::JoystickState const>& event)
{
	const auto zero_all_commands = [](void)
	{
		intake_arc_cmd.data = 0.0;
		intake_straight_cmd.data = 0.0;
		shooter_cmd.data = 0.0;
		ROS_INFO_STREAM("Set intake_arc_cmd.data to " << intake_arc_cmd.data);
		ROS_INFO_STREAM("Set intake_straight_cmd.data to " << intake_straight_cmd.data);
		ROS_INFO_STREAM("Set shooter_cmd.data to " << shooter_cmd.data);
	};
	//So the code can use specific joysticks
	int joystick_id = -1;

	const ros::M_string &header = event.getConnectionHeader();
	const std::string &topic = header.at("topic");

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

		if(!diagnostics_mode)
		{
			//Joystick1: buttonA
			if(joystick_states_array[0].buttonAPress)
			{
				intake_arc_cmd.data = std::max(0.0, intake_arc_cmd.data - 0.1);
				ROS_INFO_STREAM("Set intake_arc_cmd.data to " << intake_arc_cmd.data);
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
				intake_straight_cmd.data = std::min(1.0, intake_straight_cmd.data + 0.1);
				ROS_INFO_STREAM("Set intake_straight_cmd.data to " << intake_straight_cmd.data);
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
				intake_straight_cmd.data = std::max(0.0, intake_straight_cmd.data - 0.1);
				ROS_INFO_STREAM("Set intake_straight_cmd.data to " << intake_straight_cmd.data);
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
				intake_arc_cmd.data = std::min(1.0, intake_arc_cmd.data + 0.1);
				ROS_INFO_STREAM("Set intake_arc_cmd.data to " << intake_arc_cmd.data);
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
				zero_all_commands();
			}
			if(joystick_states_array[0].bumperLeftButton)
			{
			}
			if(joystick_states_array[0].bumperLeftRelease)
			{
			}

			//Joystick1: bumperRight
			if(joystick_states_array[0].bumperRightPress)
			{
				zero_all_commands();
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
				shooter_cmd.data = shooter_cmd.data + 1;
				ROS_INFO_STREAM("Set shooter_cmd.data to " << shooter_cmd.data);
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
				shooter_cmd.data = std::max(0.0, shooter_cmd.data - 1);
				ROS_INFO_STREAM("Set shooter_cmd.data to " << shooter_cmd.data);
			}
			if(joystick_states_array[0].directionDownButton)
			{
			}
			else
			{
			}
			if(joystick_states_array[0].directionDownRelease)
			{
			}
			else
			{
			}

			//Joystick1: stickLeft
			if(joystick_states_array[0].stickLeftPress)
			{
			}
			else
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
			else
			{
			}

			if(joystick_states_array[0].leftTrigger > trigger_threshold)
			{
				zero_all_commands();
			}
			else
			{
			}

			//Joystick1: rightTrigger
			if(joystick_states_array[0].rightTrigger > trigger_threshold)
			{
				zero_all_commands();
			}
			else
			{
			}
		}
	}
	intake_straight_pub.publish(intake_straight_cmd);
	intake_arc_pub.publish(intake_arc_cmd);
	shooter_pub.publish(shooter_cmd);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Joystick_controller");
	ros::NodeHandle n;

	int num_joysticks = 1;

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";

	intake_straight_pub = n.advertise<std_msgs::Float64>("/frcrobot_jetson/intake_straight_controller/command", 1, true);
	intake_arc_pub = n.advertise<std_msgs::Float64>("/frcrobot_jetson/intake_arc_controller/command", 1, true);
	shooter_pub = n.advertise<std_msgs::Float64>("/frcrobot_jetson/shooter_controller/command", 1, true);

	intake_arc_cmd.data = 0.0;
	intake_straight_cmd.data = 0.0;
	shooter_cmd.data = 0.0;

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
		ROS_INFO_STREAM("Subscribing to " << s.str());
	}

	ros::Subscriber button_box_sub = n.subscribe("/frcrobot_rio/button_box_states", 1, &buttonBoxCallback);

	ROS_WARN("joy_init");

	ros::spin();
	return 0;
}
