#include "ros/ros.h"
#include <string>

#include "frc_msgs/ButtonBoxState.h"
#include "frc_msgs/JoystickState.h"
#include "std_msgs/Float64.h"

#include "std_srvs/Empty.h"

#include "teleop_joystick_control/TeleopJoystickComp2022Config.h"
#include "teleop_joystick_control/TeleopJoystickCompDiagnostics2022Config.h"

#include "teleop_joystick_control/TeleopCmdVel.h"

#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>

bool diagnostics_mode = false;

frc_msgs::ButtonBoxState button_box;

// array of joystick_states messages for multiple joysticks
std::vector <frc_msgs::JoystickState> joystick_states_array;
std::vector <std::string> topic_array;

std_msgs::Float64 indexer_arc_cmd;
std_msgs::Float64 indexer_straight_cmd;
std_msgs::Float64 shooter_cmd;
std_msgs::Float64 climber_cmd;
std_msgs::Float64 intake_cmd;
std_msgs::Float64 intake_solenoid_cmd;
ros::Publisher indexer_straight_pub;
ros::Publisher indexer_arc_pub;
ros::Publisher shooter_pub;
ros::Publisher climber_pub;
ros::Publisher intake_pub;
ros::Publisher intake_solenoid_pub;

ros::Time last_header_stamp;
double trigger_threshold = 0.5;

// Teleop stuff
std::unique_ptr<TeleopCmdVel<teleop_joystick_control::TeleopJoystickComp2022Config>> teleop_cmd_vel;
teleop_joystick_control::TeleopJoystickComp2022Config config;
teleop_joystick_control::TeleopJoystickCompDiagnostics2022Config diagnostics_config;

ros::ServiceClient BrakeSrv;
ros::Publisher JoystickRobotVel;
double imu_angle;
void imuCallback(const sensor_msgs::Imu &imuState)
{
	const tf2::Quaternion imuQuat(imuState.orientation.x, imuState.orientation.y, imuState.orientation.z, imuState.orientation.w);
	double roll;
	double pitch;
	double yaw;
	tf2::Matrix3x3(imuQuat).getRPY(roll, pitch, yaw);

	if (yaw == yaw) // ignore NaN results
		imu_angle = -yaw;
}

#define SHOOTER_VELOCITY_MODE
#ifdef SHOOTER_VELOCITY_MODE
const std::string shooter_topic_name = "/frcrobot_jetson/shooter_controller/command";
constexpr double shooter_increment = 10.0;
constexpr double shooter_max_value = std::numeric_limits<double>::max(); // kinda optimistic
constexpr double shooter_min_value = 0.0;
#else // Shooter running in % out mode
const std::string shooter_topic_name = "/frcrobot_jetson/shooter_percent_out_controller/command";
constexpr double shooter_increment = 0.1;
constexpr double shooter_max_value = 1.0;
constexpr double shooter_min_value = -1.0;
#endif

void decIndexerArc(void)
{
	indexer_arc_cmd.data = std::max(-1.0, indexer_arc_cmd.data - 0.1);
	ROS_INFO_STREAM("Set indexer_arc_cmd.data to " << indexer_arc_cmd.data);
}
void incIndexerArc(void)
{
	indexer_arc_cmd.data = std::min(1.0, indexer_arc_cmd.data + 0.1);
	ROS_INFO_STREAM("Set indexer_arc_cmd.data to " << indexer_arc_cmd.data);
}
void decIndexerStraight(void)
{
	indexer_straight_cmd.data = std::max(-1.0, indexer_straight_cmd.data - 0.1);
	ROS_INFO_STREAM("Set indexer_straight_cmd.data to " << indexer_straight_cmd.data);
}
void incIndexerStraight(void)
{
	indexer_straight_cmd.data = std::min(1.0, indexer_straight_cmd.data + 0.1);
	ROS_INFO_STREAM("Set indexer_straight_cmd.data to " << indexer_straight_cmd.data);
}

void incShooter(void)
{
	shooter_cmd.data = std::min(shooter_max_value, shooter_cmd.data + shooter_increment);
	ROS_INFO_STREAM("Set shooter_cmd.data to " << shooter_cmd.data);
}

void decShooter(void)
{
	shooter_cmd.data = std::max(shooter_min_value, shooter_cmd.data - shooter_increment);
	ROS_INFO_STREAM("Set shooter_cmd.data to " << shooter_cmd.data);
}

void incIntake(void)
{
	intake_cmd.data = std::min(1.0, intake_cmd.data + 0.1);
	ROS_INFO_STREAM("Set intake_cmd.data to " << intake_cmd.data);
}

void decIntake(void)
{
	intake_cmd.data = std::max(-1.0, intake_cmd.data - 0.1);
	ROS_INFO_STREAM("Set intake_cmd.data to " << intake_cmd.data);
}

void extendIntakeSolenoid(void)
{
	intake_solenoid_cmd.data = 1;
	ROS_INFO_STREAM("Set intake_solenoid_cmd.data to " << intake_solenoid_cmd.data);
}
void retractIntakeSolenoid(void)
{
	intake_solenoid_cmd.data = 0;
	ROS_INFO_STREAM("Set intake_solenoid_cmd.data to " << intake_solenoid_cmd.data);
}

void publish_cmds(void)
{
	indexer_straight_pub.publish(indexer_straight_cmd);
	indexer_arc_pub.publish(indexer_arc_cmd);
	shooter_pub.publish(shooter_cmd);
	climber_pub.publish(climber_cmd);
	intake_pub.publish(intake_cmd);
	intake_solenoid_pub.publish(intake_solenoid_cmd);
}

void zero_all_commands(void)
{
	// TODO - should we leave the intake solenoid alone?
	indexer_arc_cmd.data = 0.0;
	indexer_straight_cmd.data = 0.0;
	shooter_cmd.data = 0.0;
	climber_cmd.data = 0.0;
	intake_cmd.data = 0.0;
	ROS_INFO_STREAM("Set indexer_arc_cmd.data to " << indexer_arc_cmd.data);
	ROS_INFO_STREAM("Set indexer_straight_cmd.data to " << indexer_straight_cmd.data);
	ROS_INFO_STREAM("Set shooter_cmd.data to " << shooter_cmd.data);
	ROS_INFO_STREAM("Set climber_cmd.data to " << climber_cmd.data);
	ROS_INFO_STREAM("Set intake_cmd.data to " << intake_cmd.data);
}

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
		decShooter();
	}
	if(button_box.leftRedButton)
	{
	}
	if(button_box.leftRedRelease)
	{
	}

	if(button_box.rightRedPress)
	{
		incShooter();
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
		decIntake();
	}
	if(button_box.leftBlueButton)
	{
	}
	if(button_box.leftBlueRelease)
	{
	}

	if(button_box.rightBluePress)
	{
		incIntake();
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
		decIndexerStraight();
	}
	if(button_box.leftGreenButton)
	{
	}
	if(button_box.leftGreenRelease)
	{
	}

	if(button_box.rightGreenPress)
	{
		incIndexerStraight();
	}
	if(button_box.rightGreenButton)
	{
	}
	if(button_box.rightGreenRelease)
	{
	}

	if(button_box.topGreenPress)
	{
		incIndexerArc();
	}
	if(button_box.topGreenButton)
	{
	}
	if(button_box.topGreenRelease)
	{
	}

	if(button_box.bottomGreenPress)
	{
		decIndexerArc();
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
	publish_cmds();
}


void evaluateCommands(const ros::MessageEvent<frc_msgs::JoystickState const>& event)
{
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
			static bool sendRobotZero = false;

			geometry_msgs::Twist cmd_vel = teleop_cmd_vel->generateCmdVel(joystick_states_array[0], imu_angle, config);

			if((cmd_vel.linear.x == 0.0) && (cmd_vel.linear.y == 0.0) && (cmd_vel.angular.z == 0.0) && !sendRobotZero)
			{
				std_srvs::Empty empty;
				if (!BrakeSrv.call(empty))
				{
					ROS_ERROR("BrakeSrv call failed in sendRobotZero_");
				}
				ROS_INFO("BrakeSrv called");

				JoystickRobotVel.publish(cmd_vel);
				sendRobotZero = true;
			}
			else if((cmd_vel.linear.x != 0.0) || (cmd_vel.linear.y != 0.0) || (cmd_vel.angular.z != 0.0))
			{
				JoystickRobotVel.publish(cmd_vel);
				sendRobotZero = false;
			}

			if(joystick_states_array[0].buttonAPress)
			{
				decIndexerArc();
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
				incIndexerStraight();
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
				decIndexerStraight();
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
				incIndexerArc();
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
				decIntake();
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
				incIntake();
			}
			if(joystick_states_array[0].buttonStartButton)
			{
			}
			if(joystick_states_array[0].buttonStartRelease)
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
				incShooter();
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
				decShooter();
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
				retractIntakeSolenoid();
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

			//Joystick1: stickRight
			if(joystick_states_array[0].stickRightPress)
			{
				extendIntakeSolenoid();
			}
			else
			{
			}
			if(joystick_states_array[0].stickRightButton)
			{
			}
			else
			{
			}
			if(joystick_states_array[0].stickRightRelease)
			{
			}
			else
			{
			}
#if 0
			static bool rightStickCentered{true};
			const auto rightStickY = joystick_states_array[0].rightStickY;
			if(rightStickY < -0.5)
			{
				if (rightStickCentered)
				{
					climber_cmd.data = std::min(1.0, climber_cmd.data + 0.1);
					ROS_INFO_STREAM("Set climber_cmd.data to " << climber_cmd.data);
					rightStickCentered = false;
				}
			}
			else if(rightStickY > 0.5)
			{
				if (rightStickCentered)
				{
					climber_cmd.data = std::max(-1.0, climber_cmd.data - 0.1);
					ROS_INFO_STREAM("Set climber_cmd.data to " << climber_cmd.data);
					rightStickCentered = false;
				}
			}
			else if((rightStickY > -0.05) && (rightStickY < 0.05))
			{
				rightStickCentered = true;
			}

			static bool leftStickCentered{true};
			const auto leftStickY = joystick_states_array[0].leftStickY;
			if(leftStickY < -0.5)
			{
				if (leftStickCentered)
				{
					incIntake();
					leftStickCentered = false;
				}
			}
			else if(leftStickY > 0.5)
			{
				if (leftStickCentered)
				{
					decIntake();
					leftStickCentered = false;
				}
			}
			else if((leftStickY > -0.05) && (leftStickY < 0.05))
			{
				leftStickCentered = true;
			}
#endif

			//Joystick1: leftTrigger
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
	publish_cmds();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Joystick_controller");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "teleop_params");
	ros::NodeHandle n_swerve_params(n, "/frcrobot_jetson/swerve_drive_controller");

	int num_joysticks = 2;

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
	if(!n_params.getParam("max_speed", config.max_speed))
	{
		ROS_ERROR("Could not read max_speed in teleop_joystick_comp");
	}
	if(!n_params.getParam("max_speed_slow", config.max_speed_slow))
	{
		ROS_ERROR("Could not read max_speed_slow in teleop_joystick_comp");
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

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";
	teleop_cmd_vel = std::make_unique<TeleopCmdVel<teleop_joystick_control::TeleopJoystickComp2022Config>>(config);
	BrakeSrv = n.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, service_connection_header);
	if(!BrakeSrv.waitForExistence(ros::Duration(15)))
	{
		ROS_ERROR("Wait (15 sec) timed out, for Brake Service in teleop_joystick_comp.cpp");
	}

	imu_angle = M_PI / 2.;
	JoystickRobotVel = n.advertise<geometry_msgs::Twist>("/frcrobot_jetson/swerve_drive_controller/cmd_vel", 1);

	indexer_straight_pub = n.advertise<std_msgs::Float64>("/frcrobot_jetson/indexer_straight_controller/command", 1, true);
	indexer_arc_pub = n.advertise<std_msgs::Float64>("/frcrobot_jetson/indexer_arc_controller/command", 1, true);
	shooter_pub = n.advertise<std_msgs::Float64>(shooter_topic_name, 1, true);
	climber_pub = n.advertise<std_msgs::Float64>("/frcrobot_jetson/climber_percent_out_controller/command", 1, true);
	intake_pub = n.advertise<std_msgs::Float64>("/frcrobot_jetson/intake_motor_controller/command", 1, true);
	intake_solenoid_pub = n.advertise<std_msgs::Float64>("/frcrobot_jetson/intake_solenoid_controller/command", 1, true);

	zero_all_commands();
	intake_solenoid_cmd.data = 0.0;

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
