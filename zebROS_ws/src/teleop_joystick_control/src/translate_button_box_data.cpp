#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <frc_msgs/ButtonBoxState.h>

frc_msgs::ButtonBoxState processed_msg_last;
frc_msgs::ButtonBoxState processed_msg;

ros::Publisher processed_data_pub;

void rawDataCB(const sensor_msgs::Joy::ConstPtr &raw_msg)
{
	// preserve time stamp
	processed_msg.header.stamp = raw_msg->header.stamp;

	// Copy over button values
	processed_msg.lockingSwitchButton		= raw_msg->buttons.size() > 0	? raw_msg->buttons[0]	: false;
	processed_msg.topRedButton				= raw_msg->buttons.size() > 1	? raw_msg->buttons[1]	: false;
	processed_msg.leftRedButton				= raw_msg->buttons.size() > 2	? raw_msg->buttons[2]	: false;
	processed_msg.rightRedButton			= raw_msg->buttons.size() > 3	? raw_msg->buttons[3]	: false;
	processed_msg.leftSwitchUpButton		= raw_msg->buttons.size() > 4	? raw_msg->buttons[4]	: false;
	processed_msg.leftSwitchDownButton		= raw_msg->buttons.size() > 5	? raw_msg->buttons[5]	: false;
	processed_msg.rightSwitchUpButton		= raw_msg->buttons.size() > 6	? raw_msg->buttons[6]	: false;
	processed_msg.rightSwitchDownButton		= raw_msg->buttons.size() > 7	? raw_msg->buttons[7]	: false;
	processed_msg.leftBlueButton			= raw_msg->buttons.size() > 8	? raw_msg->buttons[8]	: false;
	processed_msg.rightBlueButton			= raw_msg->buttons.size() > 9	? raw_msg->buttons[9]	: false;
	processed_msg.yellowButton				= raw_msg->buttons.size() > 10	? raw_msg->buttons[10]	: false;
	processed_msg.leftGreenButton			= raw_msg->buttons.size() > 11	? raw_msg->buttons[11]	: false;
	processed_msg.rightGreenButton			= raw_msg->buttons.size() > 12	? raw_msg->buttons[12]	: false;
	processed_msg.topGreenButton			= raw_msg->buttons.size() > 13	? raw_msg->buttons[13]	: false;
	processed_msg.bottomGreenButton			= raw_msg->buttons.size() > 14	? raw_msg->buttons[14]	: false;
	processed_msg.bottomSwitchUpButton		= raw_msg->buttons.size() > 15	? raw_msg->buttons[15]	: false;
	processed_msg.bottomSwitchDownButton	= raw_msg->buttons.size() > 16	? raw_msg->buttons[16]	: false;

	// Creating press booleans by comparing the last publish to the current one
	processed_msg.lockingSwitchPress	= !processed_msg_last.lockingSwitchButton		&& processed_msg.lockingSwitchButton;
	processed_msg.topRedPress			= !processed_msg_last.topRedButton				&& processed_msg.topRedButton;
	processed_msg.leftRedPress			= !processed_msg_last.leftRedButton				&& processed_msg.leftRedButton;
	processed_msg.rightRedPress			= !processed_msg_last.rightRedButton			&& processed_msg.rightRedButton;
	processed_msg.leftSwitchUpPress		= !processed_msg_last.leftSwitchUpButton		&& processed_msg.leftSwitchUpButton;
	processed_msg.leftSwitchDownPress	= !processed_msg_last.leftSwitchDownButton		&& processed_msg.leftSwitchDownButton;
	processed_msg.rightSwitchUpPress	= !processed_msg_last.rightSwitchUpButton		&& processed_msg.rightSwitchUpButton;
	processed_msg.rightSwitchDownPress	= !processed_msg_last.rightSwitchDownButton		&& processed_msg.rightSwitchDownButton;
	processed_msg.leftBluePress			= !processed_msg_last.leftBlueButton			&& processed_msg.leftBlueButton;
	processed_msg.rightBluePress		= !processed_msg_last.rightBlueButton			&& processed_msg.rightBlueButton;
	processed_msg.yellowPress			= !processed_msg_last.yellowButton				&& processed_msg.yellowButton;
	processed_msg.leftGreenPress		= !processed_msg_last.leftGreenButton			&& processed_msg.leftGreenButton;
	processed_msg.rightGreenPress		= !processed_msg_last.rightGreenButton			&& processed_msg.rightGreenButton;
	processed_msg.topGreenPress			= !processed_msg_last.topGreenButton			&& processed_msg.topGreenButton;
	processed_msg.bottomGreenPress		= !processed_msg_last.bottomGreenButton			&& processed_msg.bottomGreenButton;
	processed_msg.bottomSwitchUpPress	= !processed_msg_last.bottomSwitchUpButton		&& processed_msg.bottomSwitchUpButton;
	processed_msg.bottomSwitchDownPress	= !processed_msg_last.bottomSwitchDownButton	&& processed_msg.bottomSwitchDownButton;

	// Creating release booleans by comparing the last publish to the current one
	processed_msg.lockingSwitchRelease		= processed_msg_last.lockingSwitchButton	&& !processed_msg.lockingSwitchButton;
	processed_msg.topRedRelease				= processed_msg_last.topRedButton			&& !processed_msg.topRedButton;
	processed_msg.leftRedRelease			= processed_msg_last.leftRedButton			&& !processed_msg.leftRedButton;
	processed_msg.rightRedRelease			= processed_msg_last.rightRedButton			&& !processed_msg.rightRedButton;
	processed_msg.leftSwitchUpRelease		= processed_msg_last.leftSwitchUpButton		&& !processed_msg.leftSwitchUpButton;
	processed_msg.leftSwitchDownRelease		= processed_msg_last.leftSwitchDownButton	&& !processed_msg.leftSwitchDownButton;
	processed_msg.rightSwitchUpRelease		= processed_msg_last.rightSwitchUpButton	&& !processed_msg.rightSwitchUpButton;
	processed_msg.rightSwitchDownRelease	= processed_msg_last.rightSwitchDownButton	&& !processed_msg.rightSwitchDownButton;
	processed_msg.leftBlueRelease			= processed_msg_last.leftBlueButton			&& !processed_msg.leftBlueButton;
	processed_msg.rightBlueRelease			= processed_msg_last.rightBlueButton		&& !processed_msg.rightBlueButton;
	processed_msg.yellowRelease				= processed_msg_last.yellowButton			&& !processed_msg.yellowButton;
	processed_msg.leftGreenRelease			= processed_msg_last.leftGreenButton		&& !processed_msg.leftGreenButton;
	processed_msg.rightGreenRelease			= processed_msg_last.rightGreenButton		&& !processed_msg.rightGreenButton;
	processed_msg.topGreenRelease			= processed_msg_last.topGreenButton			&& !processed_msg.topGreenButton;
	processed_msg.bottomGreenRelease		= processed_msg_last.bottomGreenButton		&& !processed_msg.bottomGreenButton;
	processed_msg.bottomSwitchUpRelease		= processed_msg_last.bottomSwitchUpButton	&& !processed_msg.bottomSwitchUpButton;
	processed_msg.bottomSwitchDownRelease	= processed_msg_last.bottomSwitchDownButton	&& !processed_msg.bottomSwitchDownButton;

	// So that processed_msg is published every time a new raw message is translated
	processed_data_pub.publish(processed_msg);

	// Set processed_msg_last to be correct the next time through
	processed_msg_last = processed_msg;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "translate_joystick_data");
    ros::NodeHandle n;

    ros::Subscriber raw_data_sub = n.subscribe("/frcrobot_rio/button_box_states_raw", 5, rawDataCB);
    processed_data_pub = n.advertise<frc_msgs::ButtonBoxState>("button_box_states", 5);

	processed_msg_last = processed_msg;

	ros::spin();

    return 0;
}
