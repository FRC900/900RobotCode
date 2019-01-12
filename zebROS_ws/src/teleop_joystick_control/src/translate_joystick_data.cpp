#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ros_control_boilerplate/JoystickState.h>

ros_control_boilerplate::JoystickState processed_msg;

void rawDataCB(const sensor_msgs::Joy::ConstPtr &msg)
{
	processed_msg.rightStickY = msg->axes[4];
	processed_msg.rightStickX = msg->axes[3];
	processed_msg.leftStickY = msg->axes[1];
	processed_msg.leftStickX = msg->axes[0];
	processed_msg.leftTrigger = msg->axes[2];
	processed_msg.rightTrigger = msg->axes[5];
	processed_msg.dPadX = msg->axes[6];
	processed_msg.dpadY = msg->axes[7];

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
