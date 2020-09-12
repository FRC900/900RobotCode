#include "ros/ros.h"
#include "frc_msgs/JoystickState.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <cmath>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

#include "std_srvs/Empty.h"

#include <vector>
#include "teleop_joystick_control/RobotOrient.h"
#include "teleop_joystick_control/OrientStrafingAngle.h"

#include "frc_msgs/ButtonBoxState.h"
#include "frc_msgs/MatchSpecificData.h"

#include <controllers_2020_msgs/ClimberSrv.h>
#include <controllers_2020_msgs/ControlPanelSrv.h>
#include <controllers_2020_msgs/IndexerSrv.h>
#include <controllers_2020_msgs/IntakeArmSrv.h>
#include <controllers_2020_msgs/IntakeRollerSrv.h>
#include <controllers_2020_msgs/ShooterSrv.h>
#include <controllers_2020_msgs/TurretSrv.h>

#include "actionlib/client/simple_action_client.h"
#include "behavior_actions/AlignToShootAction.h"
#include "behavior_actions/EjectAction.h"
#include "behavior_actions/IndexerAction.h"
#include "behavior_actions/IntakeAction.h"
#include "behavior_actions/PathAction.h"
#include "behavior_actions/ShooterAction.h"

#include "behavior_actions/ShooterOffset.h"

#include "dynamic_reconfigure_wrapper/dynamic_reconfigure_wrapper.h"
#include "teleop_joystick_control/TeleopJoystickCompConfig.h"
#include "teleop_joystick_control/TeleopJoystickCompDiagnosticsConfig.h"

#include "teleop_joystick_control/TeleopCmdVel.h"

std::unique_ptr<TeleopCmdVel> teleop_cmd_vel;

bool diagnostics_mode = false;
bool green_led_on = true;
bool can_climb = false;

double orient_strafing_angle;

enum ControlPanelMode{ rotation, increment, position };
ControlPanelMode control_panel_mode = increment;

enum ShooterMode{ high, automatic, low };
ShooterMode shooter_mode = automatic;

frc_msgs::ButtonBoxState button_box;

// array of joystick_states messages for multiple joysticks
std::vector <frc_msgs::JoystickState> joystick_states_array;
std::vector <std::string> topic_array;

ros::Publisher orient_strafing_enable_pub;
ros::Publisher orient_strafing_setpoint_pub;
ros::Publisher orient_strafing_state_pub;

ros::Publisher green_led_pub;

ros::Publisher shooter_offset_pub;

teleop_joystick_control::TeleopJoystickCompConfig config;
teleop_joystick_control::TeleopJoystickCompDiagnosticsConfig diagnostics_config;

ros::Publisher JoystickRobotVel;

ros::ServiceClient BrakeSrv;

ros::ServiceClient climber_controller_client;
ros::ServiceClient control_panel_controller_client;
ros::ServiceClient indexer_controller_client;
ros::ServiceClient intake_arm_controller_client;
ros::ServiceClient intake_roller_controller_client;
ros::ServiceClient shooter_controller_client;
ros::ServiceClient turret_controller_client;

controllers_2020_msgs::ClimberSrv climber_controller_cmd;
controllers_2020_msgs::ControlPanelSrv control_panel_controller_cmd;
controllers_2020_msgs::IndexerSrv indexer_controller_cmd;
controllers_2020_msgs::IntakeArmSrv intake_arm_controller_cmd;
controllers_2020_msgs::IntakeRollerSrv intake_roller_controller_cmd;
controllers_2020_msgs::ShooterSrv shooter_controller_cmd;
controllers_2020_msgs::TurretSrv turret_controller_cmd;

behavior_actions::ShooterOffset shooter_offset;

std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::AlignToShootAction>> align_shooter_ac;
std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::EjectAction>> eject_ac;
std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::IndexerAction>> indexer_ac;
std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::IntakeAction>> intake_ac;
std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::PathAction>> path_ac;
std::shared_ptr<actionlib::SimpleActionClient<behavior_actions::ShooterAction>> shooter_ac;

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

void preemptActionlibServers(void)
{
	ROS_WARN_STREAM("Preempting actionlib servers!");
	align_shooter_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
	eject_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
	intake_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
	path_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
	shooter_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
}

bool orientCallback(teleop_joystick_control::RobotOrient::Request& req,
		teleop_joystick_control::RobotOrient::Response&/* res*/)
{
	// Used to switch between robot orient and field orient driving
	teleop_cmd_vel->setRobotOrient(req.robot_orient, req.offset_angle);
	ROS_WARN_STREAM("Robot Orient = " << req.robot_orient << ", Offset Angle = " << req.offset_angle);
	return true;
}

bool orientStrafingAngleCallback(teleop_joystick_control::OrientStrafingAngle::Request& req,
		teleop_joystick_control::OrientStrafingAngle::Response&/* res*/)
{
	orient_strafing_angle = req.angle;
	return true;
}

void buttonBoxCallback(const ros::MessageEvent<frc_msgs::ButtonBoxState const>& event)
{
	//ROS_INFO_STREAM("Button Box callback running!");

	button_box = *(event.getMessage());

	static ros::Time last_header_stamp = button_box.header.stamp;

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
		//Panic, preempt everything, retract everything
		preemptActionlibServers();

		indexer_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());

		intake_arm_controller_cmd.request.intake_arm_extend = false;
		ROS_WARN_STREAM("Calling intake server with intake_arm_extend = " << std::to_string(intake_arm_controller_cmd.request.intake_arm_extend));
		intake_arm_controller_client.call(intake_arm_controller_cmd);
	}
	if(button_box.topRedButton)
	{
	}
	if(button_box.topRedRelease)
	{
	}

	if(button_box.leftRedPress)
	{
		//Eject with only intake
		preemptActionlibServers();
		ROS_WARN_STREAM("Ejecting with intake!");
		behavior_actions::EjectGoal goal;
		goal.run_indexer_backwards = false;
		goal.run_intake_backwards = true;
		eject_ac->sendGoal(goal);
	}
	if(button_box.leftRedButton)
	{
	}
	if(button_box.leftRedRelease)
	{
	}

	if(button_box.rightRedPress)
	{
		//Eject with intake and indexer
		preemptActionlibServers();
		ROS_WARN_STREAM("Ejecting with intake and indexer!");
		behavior_actions::EjectGoal goal;
		goal.run_indexer_backwards = true;
		goal.run_intake_backwards = true;
		eject_ac->sendGoal(goal);
	}
	if(button_box.rightRedButton)
	{
	}
	if(button_box.rightRedRelease)
	{
	}

	if(button_box.leftSwitchUpPress)
	{
		if(can_climb)
		{
			//climber up (don't if not deployed)
			if(climber_controller_cmd.request.climber_deploy)
			{
				preemptActionlibServers();

				//Unbrake climber
				climber_controller_cmd.request.climber_elevator_brake = false;
				ROS_WARN_STREAM("Calling climber controller to release brake!");

				//Set percent out
				climber_controller_cmd.request.winch_percent_out = config.climber_percent_out_up;
				ROS_WARN_STREAM("Calling climber controller with winch_percent_out = " << climber_controller_cmd.request.winch_percent_out);
				climber_controller_client.call(climber_controller_cmd);
			}
			else
			{
				ROS_WARN_STREAM("Cannot run climber winch unless climber is deployed!");
			}
		} else {
			ROS_WARN_STREAM("Cannot climb unless match is in end game!");
		}
	}
	if(button_box.leftSwitchUpButton)
	{
	}
	if(button_box.leftSwitchUpRelease)
	{
		//Brake climber
		climber_controller_cmd.request.climber_elevator_brake = true;
		ROS_WARN_STREAM("Calling climber controller to engage brake!");

		//Set percent out to zero
		climber_controller_cmd.request.winch_percent_out = 0.0;
		ROS_WARN_STREAM("Calling climber controller with winch_percent_out = " << climber_controller_cmd.request.winch_percent_out);
		climber_controller_client.call(climber_controller_cmd);
	}

	if(button_box.leftSwitchDownPress)
	{
		if(can_climb) {
			//climber down (don't if not deployed)
			if(climber_controller_cmd.request.climber_deploy)
			{
				preemptActionlibServers();

				//Unbrake climber
				climber_controller_cmd.request.climber_elevator_brake = false;
				ROS_WARN_STREAM("Calling climber controller to release brake!");

				//Set percent out
				climber_controller_cmd.request.winch_percent_out = config.climber_percent_out_down;
				ROS_WARN_STREAM("Calling climber controller with winch_percent_out = " << climber_controller_cmd.request.winch_percent_out);
				climber_controller_client.call(climber_controller_cmd);
			}
			else
			{
				ROS_WARN_STREAM("Cannot run climber winch unless climber is deployed!");
			}
		} else {
			ROS_WARN_STREAM("Cannot climb unless match is in endgame!");
		}
	}
	if(button_box.leftSwitchDownButton)
	{
	}
	if(button_box.leftSwitchDownRelease)
	{
		//Brake climber
		climber_controller_cmd.request.climber_elevator_brake = true;
		ROS_WARN_STREAM("Calling climber controller to engage brake!");

		//Set percent out to zero
		climber_controller_cmd.request.winch_percent_out = 0.0;
		ROS_WARN_STREAM("Calling climber controller with winch_percent_out = " << climber_controller_cmd.request.winch_percent_out);
		climber_controller_client.call(climber_controller_cmd);
	}

	if(button_box.rightSwitchUpPress)
	{
		ROS_WARN_STREAM("Setting control_panel_mode = rotation");
	}
	if(button_box.rightSwitchUpButton)
	{
		control_panel_mode = rotation;
	}
	else if(!button_box.rightSwitchDownButton)
	{
		control_panel_mode = increment;
	}
	if(button_box.rightSwitchUpRelease)
	{
		ROS_WARN_STREAM("Setting control_panel_mode = increment");
	}

	if(button_box.rightSwitchDownPress)
	{
		ROS_WARN_STREAM("Setting control_panel_mode = position");
	}
	if(button_box.rightSwitchDownButton)
	{
		control_panel_mode = position;
	}
	if(button_box.rightSwitchDownRelease)
	{
		ROS_WARN_STREAM("Setting control_panel_mode = increment");
	}

	if(button_box.leftBluePress)
	{
		preemptActionlibServers();

		//Toggle climber deploy
		climber_controller_cmd.request.climber_deploy = !climber_controller_cmd.request.climber_deploy;
		ROS_WARN_STREAM("Calling climber controller with climber_deploy = " << std::to_string(climber_controller_cmd.request.climber_deploy));
		climber_controller_client.call(climber_controller_cmd);
	}
	if(button_box.leftBlueButton)
	{
	}
	if(button_box.leftBlueRelease)
	{
	}

	if(button_box.rightBluePress)
	{
		//Call control panel depending on control_panel_mode
		switch(control_panel_mode)
		{
			case rotation :
				//Call server with rotation
				break;
			case increment :

				preemptActionlibServers();

				control_panel_controller_cmd.request.control_panel_rotations = config.control_panel_increment;
				ROS_WARN_STREAM("Calling control panel controller with control_panel_rotations = " << control_panel_controller_cmd.request.control_panel_rotations);
				control_panel_controller_client.call(control_panel_controller_cmd);
				control_panel_controller_cmd.request.control_panel_rotations = 0.0;
				break;
			case position :
				//Call server with position
				break;
		}
	}
	if(button_box.rightBlueButton)
	{
	}
	if(button_box.rightBlueRelease)
	{
	}

	if(button_box.yellowPress)
	{
		preemptActionlibServers();

		behavior_actions::ShooterGoal goal;

		//Start shooting depending on shooter_mode
		switch(shooter_mode)
		{
			case high :
				//Call server with mode high
				goal.mode = 2;
				break;
			case automatic :
				//Call server with mode automatic
				goal.mode = 0;
				break;
			case low :
				//Call server with mode low
				goal.mode = 1;
				break;
		}

		ROS_WARN_STREAM("Calling shooter_ac with goal.mode = " << std::to_string(goal.mode));
		shooter_ac->sendGoal(goal);
	}
	if(button_box.yellowButton)
	{
	}
	if(button_box.yellowRelease)
	{
		//Stop shooting (preempt)
		shooter_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
	}

	static bool zeroing_turret_offset = false;

	if(button_box.leftGreenPress)
	{
	}
	if(button_box.leftGreenButton)
	{
		if(!zeroing_turret_offset)
		{
			if(button_box.rightGreenButton)
			{
				//Zero turret offset
				zeroing_turret_offset = true;
				shooter_offset.turret_offset = 0.0;
				//ROS_INFO_STREAM("Shooter turret offset = " << shooter_offset.turret_offset);
				shooter_offset_pub.publish(shooter_offset);
			}
			else
			{
				//Fine tune shooter left
				shooter_offset.turret_offset += config.shooter_turret_offset_rate*(button_box.header.stamp - last_header_stamp).toSec();
				//ROS_INFO_STREAM("Shooter turret offset = " << shooter_offset.turret_offset);
				shooter_offset_pub.publish(shooter_offset);
			}
		}
	}
	if(button_box.leftGreenRelease)
	{
		if(!button_box.rightGreenButton)
			zeroing_turret_offset = false;
	}

	if(button_box.rightGreenPress)
	{
	}
	if(button_box.rightGreenButton)
	{
		if(!zeroing_turret_offset && !button_box.leftGreenButton)
		{
			//Fine tune shooter right
			shooter_offset.turret_offset -= config.shooter_turret_offset_rate*(button_box.header.stamp - last_header_stamp).toSec();
			//ROS_INFO_STREAM("Shooter turret offset = " << shooter_offset.turret_offset);
			shooter_offset_pub.publish(shooter_offset);
		}
	}
	if(button_box.rightGreenRelease)
	{
		if(!button_box.leftGreenButton)
			zeroing_turret_offset = false;
	}

	static bool zeroing_speed_offset = false;

	if(button_box.topGreenPress)
	{
	}
	if(button_box.topGreenButton)
	{
		if(!zeroing_speed_offset)
		{
			if(button_box.bottomGreenButton)
			{
				//Zero speed offset
				zeroing_speed_offset = true;
				shooter_offset.speed_offset = 0.0;
				//ROS_INFO_STREAM("Shooter speed offset = " << shooter_offset.speed_offset);
				shooter_offset_pub.publish(shooter_offset);
			}
			else
			{
				//Fine tune shooter faster
				shooter_offset.speed_offset += config.shooter_speed_offset_rate*(button_box.header.stamp - last_header_stamp).toSec();
				//ROS_INFO_STREAM("Shooter speed offset = " << shooter_offset.speed_offset);
				shooter_offset_pub.publish(shooter_offset);
			}
		}
	}
	if(button_box.topGreenRelease)
	{
		if(!button_box.bottomGreenButton)
			zeroing_speed_offset = false;
	}

	if(button_box.bottomGreenPress)
	{
	}
	if(button_box.bottomGreenButton)
	{
		if(!zeroing_speed_offset && !button_box.rightGreenButton)
		{
			//Fine tune shooter slower
			shooter_offset.speed_offset -= config.shooter_speed_offset_rate*(button_box.header.stamp - last_header_stamp).toSec();
			//ROS_INFO_STREAM("Shooter speed offset = " << shooter_offset.speed_offset);
			shooter_offset_pub.publish(shooter_offset);
		}
	}
	if(button_box.bottomGreenRelease)
	{
		if(!button_box.topGreenButton)
			zeroing_speed_offset = false;
	}

	if(button_box.bottomSwitchUpPress)
	{
		ROS_WARN_STREAM("Setting shootet_mode = high");
	}
	if(button_box.bottomSwitchUpButton)
	{
		shooter_mode = high;
	}
	else if(!button_box.bottomSwitchDownButton)
	{
		shooter_mode = automatic;
	}
	if(button_box.bottomSwitchUpRelease)
	{
		ROS_WARN_STREAM("Setting shootet_mode = automatic");
	}

	if(button_box.bottomSwitchDownPress)
	{
		ROS_WARN_STREAM("Setting shootet_mode = low");
	}
	if(button_box.bottomSwitchDownButton)
	{
		shooter_mode = low;
	}
	if(button_box.bottomSwitchDownRelease)
	{
		ROS_WARN_STREAM("Setting shootet_mode = automatic");
	}

	last_header_stamp = button_box.header.stamp;
}

void evaluateCommands(const ros::MessageEvent<frc_msgs::JoystickState const>& event)
{
	//So the code can use specific joysticks
	int joystick_id = -1;

	const ros::M_string &header = event.getConnectionHeader();
	const std::string topic = header.at("topic");

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

		static ros::Time last_header_stamp = joystick_states_array[0].header.stamp;

		if(!diagnostics_mode)
		{
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

			//Joystick1: buttonA
			if(joystick_states_array[0].buttonAPress)
			{
				//Call control panel depending on control_panel_mode
				switch(control_panel_mode)
				{
					case rotation :
						//Call server with rotation
						break;
					case increment :
						preemptActionlibServers();

						control_panel_controller_cmd.request.control_panel_rotations = config.control_panel_increment;
						ROS_WARN_STREAM("Calling control panel controller with control_panel_rotations = " << control_panel_controller_cmd.request.control_panel_rotations);
						control_panel_controller_client.call(control_panel_controller_cmd);
						control_panel_controller_cmd.request.control_panel_rotations = 0.0;
						break;
					case position :
						//Call server with position
						break;
				}
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
				preemptActionlibServers();

				behavior_actions::ShooterGoal goal;

				//Start shooting depending on shooter_mode
				switch(shooter_mode)
				{
					case high :
						//Call server with mode high
						goal.mode = 2;
						break;
					case automatic :
						//Call server with mode automatic
						goal.mode = 0;
						break;
					case low :
						//Call server with mode low
						goal.mode = 1;
						break;
				}

				ROS_WARN_STREAM("Calling shooter_ac with goal.mode = " << std::to_string(goal.mode));
				shooter_ac->sendGoal(goal);
			}
			if(joystick_states_array[0].buttonBButton)
			{
			}
			if(joystick_states_array[0].buttonBRelease)
			{
				//Stop shooting (preempt)
				shooter_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
			}

			//Joystick1: buttonX
			if(joystick_states_array[0].buttonXPress)
			{
				//Panic, preempt everything, retract everything
				preemptActionlibServers();

				indexer_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());

				intake_arm_controller_cmd.request.intake_arm_extend = false;
				ROS_WARN_STREAM("Calling intake server with intake_arm_extend = " << std::to_string(intake_arm_controller_cmd.request.intake_arm_extend));
				intake_arm_controller_client.call(intake_arm_controller_cmd);
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
			}
			if(joystick_states_array[0].bumperRightButton)
			{
			}
			if(joystick_states_array[0].bumperRightRelease)
			{
			}

			std_msgs::Bool enable_pub_msg;

			//Joystick1: directionLeft
			if(joystick_states_array[0].directionLeftPress)
			{
				ROS_WARN_STREAM("Snapping to angle for climb!");
			}
			if(joystick_states_array[0].directionLeftButton)
			{
				// Align for climbing
				enable_pub_msg.data = true;
			}
			else
			{
				enable_pub_msg.data = false;
			}
			if(joystick_states_array[0].directionLeftRelease)
			{
			}

			orient_strafing_enable_pub.publish(enable_pub_msg);

			std_msgs::Float64 orient_strafing_angle_msg;
			orient_strafing_angle_msg.data = orient_strafing_angle;
			orient_strafing_setpoint_pub.publish(orient_strafing_angle_msg);

			std_msgs::Float64 imu_angle_msg;
			imu_angle_msg.data = imu_angle;
			orient_strafing_state_pub.publish(imu_angle_msg);

			//Joystick1: directionRight
			if(joystick_states_array[0].directionRightPress)
			{
				preemptActionlibServers();

				//Toggle climber deploy
				climber_controller_cmd.request.climber_deploy = !climber_controller_cmd.request.climber_deploy;
				ROS_WARN_STREAM("Calling climber controller with climber_deploy = " << std::to_string(climber_controller_cmd.request.climber_deploy));
				climber_controller_client.call(climber_controller_cmd);
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
				if(can_climb)
				{
					//climber up (don't if not deployed)
					if(climber_controller_cmd.request.climber_deploy)
					{
						preemptActionlibServers();

						//Unbrake climber
						climber_controller_cmd.request.climber_elevator_brake = false;
						ROS_WARN_STREAM("Calling climber controller to release brake!");

						//Set percent out
						climber_controller_cmd.request.winch_percent_out = config.climber_percent_out_up;
						ROS_WARN_STREAM("Calling climber controller with winch_percent_out = " << climber_controller_cmd.request.winch_percent_out);
						climber_controller_client.call(climber_controller_cmd);
					}
				} else {
					ROS_WARN_STREAM("Cannot climb unless match is in endgame!");
				}
			}
			if(joystick_states_array[0].directionUpButton)
			{
			}
			if(joystick_states_array[0].directionUpRelease)
			{
				//Brake climber
				climber_controller_cmd.request.climber_elevator_brake = true;
				ROS_WARN_STREAM("Calling climber controller to engage brake!");

				//Set percent out to zero
				climber_controller_cmd.request.winch_percent_out = 0.0;
				ROS_WARN_STREAM("Calling climber controller with winch_percent_out = " << climber_controller_cmd.request.winch_percent_out);
				climber_controller_client.call(climber_controller_cmd);
			}

			//Joystick1: directionDown
			if(joystick_states_array[0].directionDownPress)
			{
				if(can_climb)
				{
					//climber down (don't if not deployed)
					if(climber_controller_cmd.request.climber_deploy)
					{
						preemptActionlibServers();

						//Unbrake climber
						climber_controller_cmd.request.climber_elevator_brake = false;
						ROS_WARN_STREAM("Calling climber controller to release brake!");

						//Set percent out
						climber_controller_cmd.request.winch_percent_out = config.climber_percent_out_down;
						ROS_WARN_STREAM("Calling climber controller with winch_percent_out = " << climber_controller_cmd.request.winch_percent_out);
						climber_controller_client.call(climber_controller_cmd);
					}
				} else {
					ROS_WARN_STREAM("Cannot climb unless match is in endgame!");
				}
			}
			if(joystick_states_array[0].directionDownButton)
			{
			}
			if(joystick_states_array[0].directionDownRelease)
			{
				//Brake climber
				climber_controller_cmd.request.climber_elevator_brake = true;
				ROS_WARN_STREAM("Calling climber controller to engage brake!");

				//Set percent out to zero
				climber_controller_cmd.request.winch_percent_out = 0.0;
				ROS_WARN_STREAM("Calling climber controller with winch_percent_out = " << climber_controller_cmd.request.winch_percent_out);
				climber_controller_client.call(climber_controller_cmd);
			}

			//Joystick1: stickLeft
			if(joystick_states_array[0].stickLeftPress)
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

			static bool left_trigger_pressed = false;

			if(joystick_states_array[0].leftTrigger > config.trigger_threshold)
			{
				//Call intake server, but only once
				if(!left_trigger_pressed)
				{
					preemptActionlibServers();
					ROS_WARN_STREAM("Calling intake server!");
					behavior_actions::IntakeGoal goal;
					intake_ac->sendGoal(goal);
				}

				left_trigger_pressed = true;
			}
			else
			{
				//Preempt intake server, but only once
				if(left_trigger_pressed)
				{
					ROS_WARN_STREAM("Preempting intake server!");
					intake_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
				}

				left_trigger_pressed = false;
			}

			//Joystick1: rightTrigger
			if(joystick_states_array[0].rightTrigger > config.trigger_threshold)
			{
				teleop_cmd_vel->setSlowMode(true);
			}
			else
			{
				teleop_cmd_vel->setSlowMode(false);
			}
		}
		else
		{
			//Joystick1 Diagnostics: leftStickY
			if(abs(joystick_states_array[0].leftStickY) > config.stick_threshold)
			{
				shooter_controller_cmd.request.set_velocity += (std::copysign(diagnostics_config.shooter_setpoint_rate, joystick_states_array[0].leftStickY)
						*(joystick_states_array[0].header.stamp - last_header_stamp).toSec());
				ROS_WARN_STREAM("Calling shooter controller with set_velocity = " << shooter_controller_cmd.request.set_velocity
						<< " (" << (shooter_controller_cmd.request.set_velocity*30/M_PI) << " rpm)");
				shooter_controller_client.call(shooter_controller_cmd);
			}

			//Joystick1 Diagnostics: leftStickX
			if(abs(joystick_states_array[0].leftStickX) > config.stick_threshold)
			{
				turret_controller_cmd.request.set_point += (std::copysign(diagnostics_config.turret_setpoint_rate, joystick_states_array[0].leftStickX)
						*(joystick_states_array[0].header.stamp - last_header_stamp).toSec());
				turret_controller_cmd.request.set_point = std::clamp(turret_controller_cmd.request.set_point, -diagnostics_config.turret_angle_limit, diagnostics_config.turret_angle_limit);
				ROS_WARN_STREAM("Calling turret controller with set_point = " << turret_controller_cmd.request.set_point);
				turret_controller_client.call(turret_controller_cmd);
			}

			//Joystick1 Diagnostics: rightStickY
			if(abs(joystick_states_array[0].rightStickY) > config.stick_threshold)
			{
				indexer_controller_cmd.request.indexer_velocity += (std::copysign(diagnostics_config.indexer_setpoint_rate, joystick_states_array[0].leftStickY)
						*(joystick_states_array[0].header.stamp - last_header_stamp).toSec());
				ROS_WARN_STREAM("Calling indexer controller with indexer_velocity = " << indexer_controller_cmd.request.indexer_velocity
						<< " (" << (indexer_controller_cmd.request.indexer_velocity*30/M_PI) << " rpm)");
				indexer_controller_client.call(indexer_controller_cmd);
			}

			//Joystick1 Diagnostics: rightStickX
			if(abs(joystick_states_array[0].rightStickX) > config.stick_threshold)
			{
			}

			//Joystick1 Diagnostics: stickLeft
			if(joystick_states_array[0].stickLeftPress)
			{
				shooter_controller_cmd.request.set_velocity = 0.0;
				ROS_WARN_STREAM("Calling shooter controller with set_velocity = 0.0 Stopping shooter!");
				shooter_controller_client.call(shooter_controller_cmd);
			}
			if(joystick_states_array[0].stickLeftButton)
			{
			}
			if(joystick_states_array[0].stickLeftRelease)
			{
			}

			//Joystick1 Diagnostics: stickRight
			if(joystick_states_array[0].stickRightPress)
			{
				indexer_controller_cmd.request.indexer_velocity = 0.0;
				ROS_WARN_STREAM("Calling indexer controller with indexer_velocity = 0.0 Stopping indexer!");
				indexer_controller_client.call(indexer_controller_cmd);
			}
			if(joystick_states_array[0].stickRightButton)
			{
			}
			if(joystick_states_array[0].stickRightRelease)
			{
			}

			//Joystick1 Diagnostics: buttonA
			if(joystick_states_array[0].buttonAPress)
			{
				control_panel_controller_cmd.request.control_panel_rotations = config.control_panel_increment;
				ROS_WARN_STREAM("Calling control panel controller with control_panel_rotations = " << control_panel_controller_cmd.request.control_panel_rotations);
				control_panel_controller_client.call(control_panel_controller_cmd);
				control_panel_controller_cmd.request.control_panel_rotations = 0.0;
			}
			if(joystick_states_array[0].buttonAButton)
			{
			}
			if(joystick_states_array[0].buttonARelease)
			{
			}

			//Joystick1 Diagnostics: buttonB
			if(joystick_states_array[0].buttonBPress)
			{
				intake_roller_controller_cmd.request.percent_out = 0.0;
				ROS_WARN_STREAM("Calling intake controller with percent_out = 0.0 Stopping intake!");
				intake_roller_controller_client.call(intake_roller_controller_cmd);
			}
			if(joystick_states_array[0].buttonBButton)
			{
			}
			if(joystick_states_array[0].buttonBRelease)
			{
			}

			//Joystick1 Diagnostics: buttonX
			if(joystick_states_array[0].buttonXPress)
			{
				green_led_on = !green_led_on;

				std_msgs::Float64 green_led_position;
				green_led_position.data = green_led_on ? 1.0 : 0.0;

				green_led_pub.publish(green_led_position);

				ROS_WARN_STREAM("Calling green LED controller with green_led_on = " << std::to_string(green_led_on));
			}
			if(joystick_states_array[0].buttonXButton)
			{
			}
			if(joystick_states_array[0].buttonXRelease)
			{
			}

			//Joystick1 Diagnostics: buttonY
			if(joystick_states_array[0].buttonYPress)
			{
				intake_arm_controller_cmd.request.intake_arm_extend = !intake_arm_controller_cmd.request.intake_arm_extend;
				ROS_WARN_STREAM("Calling intake server with intake_arm_extend = " << std::to_string(intake_arm_controller_cmd.request.intake_arm_extend));
				intake_arm_controller_client.call(intake_arm_controller_cmd);
			}
			if(joystick_states_array[0].buttonYButton)
			{
			}
			if(joystick_states_array[0].buttonYRelease)
			{
			}

			//Joystick1 Diagnostics: bumperLeft
			if(joystick_states_array[0].bumperLeftPress)
			{
				shooter_controller_cmd.request.shooter_hood_raise = !shooter_controller_cmd.request.shooter_hood_raise;
				ROS_WARN_STREAM("Calling shooter controller with shooter_hood_raise = " << std::to_string(shooter_controller_cmd.request.shooter_hood_raise));
				shooter_controller_client.call(shooter_controller_cmd);
			}
			if(joystick_states_array[0].bumperLeftButton)
			{
			}
			if(joystick_states_array[0].bumperLeftRelease)
			{
			}

			//Joystick1 Diagnostics: bumperRight
			if(joystick_states_array[0].bumperRightPress)
			{
			}
			if(joystick_states_array[0].bumperRightButton)
			{
				intake_roller_controller_cmd.request.percent_out -= diagnostics_config.intake_setpoint_rate*(joystick_states_array[0].header.stamp - last_header_stamp).toSec();
				intake_roller_controller_cmd.request.percent_out = std::clamp(intake_roller_controller_cmd.request.percent_out, -1.0, 1.0);
				ROS_WARN_STREAM("Calling intake controller with percent_out = " << intake_roller_controller_cmd.request.percent_out);
				intake_roller_controller_client.call(intake_roller_controller_cmd);
			}
			if(joystick_states_array[0].bumperRightRelease)
			{
			}

			//Joystick1 Diagnostics: leftTrigger
			if(joystick_states_array[0].leftTrigger > config.trigger_threshold)
			{
			}
			else
			{
			}

			//Joystick1 Diagnostics: rightTrigger
			if(joystick_states_array[0].rightTrigger > config.trigger_threshold)
			{
				intake_roller_controller_cmd.request.percent_out += diagnostics_config.intake_setpoint_rate*(joystick_states_array[0].header.stamp - last_header_stamp).toSec();
				intake_roller_controller_cmd.request.percent_out = std::clamp(intake_roller_controller_cmd.request.percent_out, -1.0, 1.0);
				ROS_WARN_STREAM("Calling intake controller with percent_out = " << intake_roller_controller_cmd.request.percent_out);
				intake_roller_controller_client.call(intake_roller_controller_cmd);
			}
			else
			{
			}

			//Joystick1 Diagnostics: directionLeft
			if(joystick_states_array[0].directionLeftPress)
			{
				climber_controller_cmd.request.climber_elevator_brake = !climber_controller_cmd.request.climber_elevator_brake;
				ROS_WARN_STREAM("Calling climber controller with climber_elevator_brake = " << std::to_string(climber_controller_cmd.request.climber_elevator_brake));
				climber_controller_client.call(climber_controller_cmd);
			}
			if(joystick_states_array[0].directionLeftButton)
			{
			}
			if(joystick_states_array[0].directionLeftRelease)
			{
			}

			//Joystick1 Diagnostics: directionRight
			if(joystick_states_array[0].directionRightPress)
			{
				climber_controller_cmd.request.climber_deploy = !climber_controller_cmd.request.climber_deploy;
				ROS_WARN_STREAM("Calling climber controller with climber_deploy = " << std::to_string(climber_controller_cmd.request.climber_deploy));
				climber_controller_client.call(climber_controller_cmd);
			}
			if(joystick_states_array[0].directionRightButton)
			{
			}
			if(joystick_states_array[0].directionRightRelease)
			{
			}

			//Joystick1 Diagnostics: directionUp
			if(joystick_states_array[0].directionUpPress)
			{
				//Set percent out
				climber_controller_cmd.request.winch_percent_out = config.climber_percent_out_up;
				ROS_WARN_STREAM("Calling climber controller with winch_percent_out = " << climber_controller_cmd.request.winch_percent_out);
				climber_controller_client.call(climber_controller_cmd);
			}
			if(joystick_states_array[0].directionUpButton)
			{
			}
			if(joystick_states_array[0].directionUpRelease)
			{
				//Set percent out to zero
				climber_controller_cmd.request.winch_percent_out = 0.0;
				ROS_WARN_STREAM("Calling climber controller with winch_percent_out = " << climber_controller_cmd.request.winch_percent_out);
				climber_controller_client.call(climber_controller_cmd);
			}

			//Joystick1 Diagnostics: directionDown
			if(joystick_states_array[0].directionDownPress)
			{
				//Set percent out
				climber_controller_cmd.request.winch_percent_out = config.climber_percent_out_down;
				ROS_WARN_STREAM("Calling climber controller with winch_percent_out = " << climber_controller_cmd.request.winch_percent_out);
				climber_controller_client.call(climber_controller_cmd);
			}
			if(joystick_states_array[0].directionDownButton)
			{
			}
			if(joystick_states_array[0].directionDownRelease)
			{
				//Set percent out to zero
				climber_controller_cmd.request.winch_percent_out = 0.0;
				ROS_WARN_STREAM("Calling climber controller with winch_percent_out = " << climber_controller_cmd.request.winch_percent_out);
				climber_controller_client.call(climber_controller_cmd);
			}
		}

		last_header_stamp = joystick_states_array[0].header.stamp;
	}
	else if(joystick_id == 1)
	{
		// TODO Add empty button mappings here.
	}
}

void jointStateCallback(const sensor_msgs::JointState &joint_state)
{
}

void matchStateCallback(const frc_msgs::MatchSpecificData &msg)
{
	can_climb = msg.matchTimeRemaining <= config.climber_time_lock && !msg.Autonomous;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Joystick_controller");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "teleop_params");
	ros::NodeHandle n_diagnostics_params(n, "teleop_diagnostics_params");
	ros::NodeHandle n_swerve_params(n, "/frcrobot_jetson/swerve_drive_controller");



	int num_joysticks = 1;
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
	if(!n_params.getParam("limit_switch_debounce_iterations", config.limit_switch_debounce_iterations))
	{
		ROS_ERROR("Could not read limit_switch_debounce_iterations in teleop_joystick_comp");
	}
	if(!n_params.getParam("linebreak_debounce_iterations", config.linebreak_debounce_iterations))
	{
		ROS_ERROR("Could not read linebreak_debounce_iterations in teleop_joystick_comp");
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
	if(!n_params.getParam("climber_align_angle", config.climber_align_angle))
	{
		ROS_ERROR("Could not read climber_align_angle in teleop_joystick_comp");
	}
	if(!n_params.getParam("trigger_threshold", config.trigger_threshold))
	{
		ROS_ERROR("Could not read trigger_threshold in teleop_joystick_comp");
	}
	if(!n_params.getParam("stick_threshold", config.stick_threshold))
	{
		ROS_ERROR("Could not read stick_threshold in teleop_joystick_comp");
	}
	if(!n_params.getParam("climber_time_lock", config.climber_time_lock))
	{
		ROS_ERROR("Could not read climber_time_lock in teleop_joystick_comp");
	}
	if(!n_params.getParam("control_panel_increment", config.control_panel_increment))
	{
		ROS_ERROR("Could not read control_panel_increment in teleop_joystick_comp");
	}
	if(!n_params.getParam("climber_percent_out_up", config.climber_percent_out_up))
	{
		ROS_ERROR("Could not read climber_percent_out_up in teleop_joystick_comp");
	}
	if(!n_params.getParam("climber_percent_out_down", config.climber_percent_out_down))
	{
		ROS_ERROR("Could not read climber_percent_out_down in teleop_joystick_comp");
	}
	if(!n_params.getParam("shooter_turret_offset_rate", config.shooter_turret_offset_rate))
	{
		ROS_ERROR("Could not read shooter_turret_offset_rate in teleop_joystick_comp");
	}
	if(!n_params.getParam("shooter_speed_offset_rate", config.shooter_speed_offset_rate))
	{
		ROS_ERROR("Could not read shooter_speed_offset_rate in teleop_joystick_comp");
	}

	if(!n_diagnostics_params.getParam("shooter_setpoint_rate", diagnostics_config.shooter_setpoint_rate))
	{
		ROS_ERROR("Could not read shooter_setpoint_rate in teleop_joystick_comp");
	}
	if(!n_diagnostics_params.getParam("turret_setpoint_rate", diagnostics_config.turret_setpoint_rate))
	{
		ROS_ERROR("Could not read turret_setpoint_rate in teleop_joystick_comp");
	}
	if(!n_diagnostics_params.getParam("turret_angle_limit", diagnostics_config.turret_angle_limit))
	{
		ROS_ERROR("Could not read turret_angle_limit in teleop_joystick_comp");
	}
	if(!n_diagnostics_params.getParam("intake_setpoint_rate", diagnostics_config.intake_setpoint_rate))
	{
		ROS_ERROR("Could not read intake_setpoint_rate in teleop_joystick_comp");
	}
	if(!n_diagnostics_params.getParam("indexer_setpoint_rate", diagnostics_config.indexer_setpoint_rate))
	{
		ROS_ERROR("Could not read indexer_setpoint_rate in teleop_joystick_comp");
	}

	orient_strafing_angle = config.climber_align_angle;

	//Initialize the climber command
	climber_controller_cmd.request.winch_percent_out = 0.0;
	climber_controller_cmd.request.climber_deploy = false;
	climber_controller_cmd.request.climber_elevator_brake = true;

	//Initialize the control panel command
	control_panel_controller_cmd.request.control_panel_rotations = 0.0;

	//Initialize the indexer command
	indexer_controller_cmd.request.indexer_velocity = 0.0;

	//Initialize the intake command
	intake_arm_controller_cmd.request.intake_arm_extend = false;
	intake_roller_controller_cmd.request.percent_out = 0.0;

	//Initialize the shooter command
	shooter_controller_cmd.request.shooter_hood_raise = false;
	shooter_controller_cmd.request.set_velocity = 0.0;

	//Initialize the turret command
	turret_controller_cmd.request.set_point = 0.0;

	//Initialize the shooter offset
	shooter_offset.speed_offset = 0.0;
	shooter_offset.turret_offset = 0.0;

	teleop_cmd_vel = std::make_unique<TeleopCmdVel>(config);

	imu_angle = M_PI / 2.;

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";

	BrakeSrv = n.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, service_connection_header);
	if(!BrakeSrv.waitForExistence(ros::Duration(15)))
	{
		ROS_ERROR("Wait (15 sec) timed out, for Brake Service in teleop_joystick_comp.cpp");
	}

	green_led_pub = n.advertise<std_msgs::Float64>("/frcrobot_rio/green_led_controller/command", 1, true);
	std_msgs::Float64 green_led_position;
	green_led_position.data = green_led_on ? 1.0 : 0.0;
	green_led_pub.publish(green_led_position);

	shooter_offset_pub = n.advertise<behavior_actions::ShooterOffset>("teleop_shooter_offsets", 1, true);
	shooter_offset_pub.publish(shooter_offset);

	orient_strafing_enable_pub = n.advertise<std_msgs::Bool>("orient_strafing/pid_enable", 1);
	orient_strafing_setpoint_pub = n.advertise<std_msgs::Float64>("orient_strafing/setpoint", 1);
	orient_strafing_state_pub = n.advertise<std_msgs::Float64>("orient_strafing/state", 1);
	JoystickRobotVel = n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);
	ros::Subscriber imu_heading = n.subscribe("/imu/zeroed_imu", 1, &imuCallback);
	ros::Subscriber joint_states_sub = n.subscribe("/frcrobot_jetson/joint_states", 1, &jointStateCallback);

	ros::Subscriber match_state_sub = n.subscribe("/frcrobot_rio/match_data", 1, matchStateCallback);
	ros::ServiceServer robot_orient_service = n.advertiseService("robot_orient", orientCallback);

	ros::ServiceServer orient_strafing_angle_service = n.advertiseService("orient_strafing_angle", orientStrafingAngleCallback);

	align_shooter_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::AlignToShootAction>>("/align_to_shoot/align_to_shoot_server", true);
	eject_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::EjectAction>>("/eject/eject_server", true);
	indexer_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::IndexerAction>>("/indexer/indexer_server", true);
	intake_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::IntakeAction>>("/powercell_intake/powercell_intake_server", true);
	path_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::PathAction>>("/path/path_server", true);
	shooter_ac = std::make_shared<actionlib::SimpleActionClient<behavior_actions::ShooterAction>>("/shooter/shooter_server", true);

	climber_controller_client = n.serviceClient<controllers_2020_msgs::ClimberSrv>("/frcrobot_jetson/climber_controller/climber_command");
	indexer_controller_client = n.serviceClient<controllers_2020_msgs::IndexerSrv>("/frcrobot_jetson/indexer_controller/indexer_command");
	intake_arm_controller_client = n.serviceClient<controllers_2020_msgs::IntakeArmSrv>("/frcrobot_jetson/intake_controller/intake_arm_command");
	intake_roller_controller_client = n.serviceClient<controllers_2020_msgs::IntakeRollerSrv>("/frcrobot_jetson/intake_controller/intake_roller_command");
	shooter_controller_client = n.serviceClient<controllers_2020_msgs::ShooterSrv>("/frcrobot_jetson/shooter_controller/shooter_command");
	turret_controller_client = n.serviceClient<controllers_2020_msgs::TurretSrv>("/frcrobot_jetson/turret_controller/shooter_command");

	DynamicReconfigureWrapper<teleop_joystick_control::TeleopJoystickCompConfig> drw(n_params, config);
	DynamicReconfigureWrapper<teleop_joystick_control::TeleopJoystickCompDiagnosticsConfig> diagnostics_drw(n_diagnostics_params, diagnostics_config);

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
	}

	ros::Subscriber button_box_sub = n.subscribe("/frcrobot_rio/button_box_states", 1, &buttonBoxCallback);

	ROS_WARN("joy_init");

	ros::spin();
	return 0;
}
