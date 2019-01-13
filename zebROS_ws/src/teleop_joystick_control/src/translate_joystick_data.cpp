#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ros_control_boilerplate/JoystickState.h>

sensor_msgs::Joy msg_last;
ros_control_boilerplate::JoystickState processed_msg;

bool initialized = false;

void rawDataCB(const sensor_msgs::Joy::ConstPtr &msg)
{
	if(initialized == false)
	{
		msg_last = *msg;
		initialized = true;
	}

	processed_msg.rightStickY = msg->axes[4];
	processed_msg.rightStickX = msg->axes[3];
	processed_msg.leftStickY = msg->axes[1];
	processed_msg.leftStickX = msg->axes[0];
	processed_msg.leftTrigger = msg->axes[2];
	processed_msg.rightTrigger = msg->axes[5];

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

	processed_msg.buttonXButton = msg->buttons[2];
	processed_msg.buttonYButton = msg->buttons[3];
	processed_msg.bumperLeftButton = msg->buttons[4];
	processed_msg.bumperRightButton = msg->buttons[5];
	processed_msg.stickLeftButton = msg->buttons[9];
	processed_msg.stickRightButton = msg->buttons[10];
	processed_msg.buttonAButton = msg->buttons[0];
	processed_msg.buttonBButton = msg->buttons[1];
	processed_msg.buttonBackButton = msg->buttons[6];
	processed_msg.buttonStartButton = msg->buttons[7];

	processed_msg.buttonAPress = !msg_last.buttons[0] && msg->buttons[0];
	processed_msg.buttonBPress = !msg_last.buttons[1] && msg->buttons[1];
	processed_msg.buttonXPress = !msg_last.buttons[2] && msg->buttons[2];
	processed_msg.buttonYPress = !msg_last.buttons[3] && msg->buttons[3];
	processed_msg.bumperLeftPress = !msg_last.buttons[4] && msg->buttons[4];
	processed_msg.bumperRightPress = !msg_last.buttons[5] && msg->buttons[5];
	processed_msg.buttonBackPress = !msg_last.buttons[6] && msg->buttons[6];
	processed_msg.buttonStartPress = !msg_last.buttons[7] && msg->buttons[7];
	processed_msg.stickLeftPress = !msg_last.buttons[9] && msg->buttons[9];
	processed_msg.stickRightPress = !msg_last.buttons[10] && msg->buttons[10];
	
	processed_msg.buttonARelease = msg_last.buttons[0] && !msg->buttons[0];
	processed_msg.buttonBRelease = msg_last.buttons[1] && !msg->buttons[1];
	processed_msg.buttonXRelease = msg_last.buttons[2] && !msg->buttons[2];
	processed_msg.buttonYRelease = msg_last.buttons[3] && !msg->buttons[3];
	processed_msg.bumperLeftRelease = msg_last.buttons[4] && !msg->buttons[4];
	processed_msg.bumperRightRelease = msg_last.buttons[5] && !msg->buttons[5];
	processed_msg.buttonBackRelease = msg_last.buttons[6] && !msg->buttons[6];
	processed_msg.buttonStartRelease = msg_last.buttons[7] && !msg->buttons[7];
	processed_msg.stickLeftRelease = msg_last.buttons[9] && !msg->buttons[9];
	processed_msg.stickRightRelease = msg_last.buttons[10] && !msg->buttons[10];
	
	msg_last = *msg;
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
		ros::spinOnce();
		loop_rate.sleep();
    }
    return 0;
}
