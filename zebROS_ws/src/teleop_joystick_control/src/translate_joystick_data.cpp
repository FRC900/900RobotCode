#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ros_control_boilerplate/JoystickState.h>

ros_control_boilerplate::JoystickState processed_msg_last;
ros_control_boilerplate::JoystickState processed_msg;

bool initialized = false;

// Joy_node only published when there is a change in the state of the joystick, not constantly, meaning that this function only runs when there is a change in joystick state. This is the reason for the added lines on press/release after the line that publishes below.
void rawDataCB(const sensor_msgs::Joy::ConstPtr &msg)
{
	// Translating sticks and triggers
	processed_msg.leftStickX = msg->axes[0];
	processed_msg.leftStickY = msg->axes[1];
	processed_msg.leftTrigger = msg->axes[2];
	processed_msg.rightStickX = msg->axes[3];
	processed_msg.rightStickY = msg->axes[4];
	processed_msg.rightTrigger = msg->axes[5];

	// Translating Dpad (from two axes into the four buttons our code uses)
	if(msg->axes[6] > 0)
	{
		processed_msg.directionLeftButton = true;
	}

	else
	{
		processed_msg.directionLeftButton = false;
	}

	if(msg->axes[6] < 0)
	{
		processed_msg.directionRightButton = true;
	}

	else
	{
		processed_msg.directionRightButton = false;
	}

	if(msg->axes[7] > 0)
	{
		processed_msg.directionUpButton = true;
	}

	else
	{
		processed_msg.directionUpButton = false;
	}

	if(msg->axes[7] < 0)
	{
		processed_msg.directionDownButton = true;
	}

	else
	{
			processed_msg.directionDownButton = false;
	}

	// Translating all buttons other than the Dpad
	processed_msg.buttonAButton = msg->buttons[0];
	processed_msg.buttonBButton = msg->buttons[1];
	processed_msg.buttonXButton = msg->buttons[2];
	processed_msg.buttonYButton = msg->buttons[3];
	processed_msg.bumperLeftButton = msg->buttons[4];
	processed_msg.bumperRightButton = msg->buttons[5];
	processed_msg.buttonBackButton = msg->buttons[6];
	processed_msg.buttonStartButton = msg->buttons[7];
	processed_msg.stickLeftButton = msg->buttons[9];
	processed_msg.stickRightButton = msg->buttons[10];

	if(initialized == false)
	{
		// Such that processed_msg_last has content for the first time through press/release translating
		processed_msg_last = processed_msg;
		initialized = true;
	}

	// Creating press booleans by comparing the last publish to the current one
	processed_msg.buttonAPress = !processed_msg_last.buttonAButton && processed_msg.buttonAButton;
	processed_msg.buttonBPress = !processed_msg_last.buttonBButton && processed_msg.buttonBButton;
	processed_msg.buttonXPress = !processed_msg_last.buttonXButton && processed_msg.buttonXButton;
	processed_msg.buttonYPress = !processed_msg_last.buttonYButton && processed_msg.buttonYButton;
	processed_msg.bumperLeftPress = !processed_msg_last.bumperLeftButton && processed_msg.bumperLeftButton;
	processed_msg.bumperRightPress = !processed_msg_last.bumperRightButton && processed_msg.bumperRightButton;
	processed_msg.buttonBackPress = !processed_msg_last.buttonBackButton && processed_msg.buttonBackButton;
	processed_msg.buttonStartPress = !processed_msg_last.buttonStartButton && processed_msg.buttonStartButton;
	processed_msg.stickLeftPress = !processed_msg_last.stickLeftButton && processed_msg.stickLeftButton;
	processed_msg.stickRightPress = !processed_msg_last.stickRightButton && processed_msg.stickRightButton;
	processed_msg.directionLeftPress = !processed_msg_last.directionLeftButton && processed_msg.directionLeftButton;
	processed_msg.directionRightPress = !processed_msg_last.directionRightButton && processed_msg.directionRightButton;
	processed_msg.directionUpPress = !processed_msg_last.directionUpButton && processed_msg.directionUpButton;
	processed_msg.directionUpPress = !processed_msg_last.directionDownButton && processed_msg.directionDownButton;

	// Creating release booleans by comparing the last publish to the current one
	processed_msg.buttonARelease = processed_msg_last.buttonAButton && !processed_msg.buttonAButton;
	processed_msg.buttonBRelease = processed_msg_last.buttonBButton && !processed_msg.buttonBButton;
	processed_msg.buttonXRelease = processed_msg_last.buttonXButton && !processed_msg.buttonXButton;
	processed_msg.buttonYRelease = processed_msg_last.buttonYButton && !processed_msg.buttonYButton;
	processed_msg.bumperLeftRelease = processed_msg_last.bumperLeftButton && !processed_msg.bumperLeftButton;
	processed_msg.bumperRightRelease = processed_msg_last.bumperRightButton && !processed_msg.bumperRightButton;
	processed_msg.buttonBackRelease = processed_msg_last.buttonBackButton && !processed_msg.buttonBackButton;
	processed_msg.buttonStartRelease = processed_msg_last.buttonStartButton && !processed_msg.buttonStartButton;
	processed_msg.stickLeftRelease = processed_msg_last.stickLeftButton && !processed_msg.stickLeftButton;
	processed_msg.stickRightRelease = processed_msg_last.stickRightButton && !processed_msg.stickRightButton;
	processed_msg.directionLeftRelease = processed_msg_last.directionLeftButton && !processed_msg.directionLeftButton;
	processed_msg.directionRightRelease = processed_msg_last.directionRightButton && !processed_msg.directionRightButton;
	processed_msg.directionUpRelease = processed_msg_last.directionUpButton && !processed_msg.directionUpButton;
	processed_msg.directionDownRelease = processed_msg_last.directionDownButton && !processed_msg.directionDownButton;

	// Set processed_msg_last to be correct the next time through
	processed_msg_last = processed_msg;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "translate_joystick_data");
    ros::NodeHandle n;

    ros::Subscriber raw_data_sub = n.subscribe("/frcrobot_jetson/joystick_states_raw", 1000, rawDataCB);
    ros::Publisher processed_data_pub = n.advertise<ros_control_boilerplate::JoystickState>("/frcrobot_jetson/joystick_states", 1000);

	ros::Rate loop_rate(10);

    while(ros::ok())
    {
        processed_data_pub.publish(processed_msg);

		// So that press is only true for one publish
		processed_msg.buttonAPress = false;
		processed_msg.buttonBPress = false;
		processed_msg.buttonXPress = false;
		processed_msg.buttonYPress = false;
		processed_msg.bumperLeftPress = false;
		processed_msg.bumperRightPress = false;
		processed_msg.buttonBackPress = false;
		processed_msg.buttonStartPress = false;
		processed_msg.stickLeftPress = false;
		processed_msg.stickRightPress = false;
		processed_msg.directionLeftPress = false;
		processed_msg.directionRightPress = false;
		processed_msg.directionUpPress = false;
		processed_msg.directionDownPress = false;

		// So that release is only true for one publish
		processed_msg.buttonARelease = false;
		processed_msg.buttonBRelease = false;
		processed_msg.buttonXRelease = false;
		processed_msg.buttonYRelease = false;
		processed_msg.bumperLeftRelease = false;
		processed_msg.bumperRightRelease = false;
		processed_msg.buttonBackRelease = false;
		processed_msg.buttonStartRelease = false;
		processed_msg.stickLeftRelease = false;
		processed_msg.stickRightRelease = false;
		processed_msg.directionLeftRelease = false;
		processed_msg.directionRightRelease = false;
		processed_msg.directionUpRelease = false;
		processed_msg.directionDownRelease = false;

		ros::spinOnce();
		loop_rate.sleep();
    }
    return 0;
}
