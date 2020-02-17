#include "teleop_joystick_control/teleop_joystick_comp.h"
#include "std_srvs/Empty.h"

#include "std_srvs/Trigger.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"

#include "actionlib/client/simple_action_client.h"

#include "std_srvs/SetBool.h"
#include "std_srvs/Empty.h"
#include <vector>
#include "teleop_joystick_control/RobotOrient.h"
#include "teleop_joystick_control/OrientStrafingAngle.h"

#include <controllers_2020_msgs/ClimberSrv.h>
#include <controllers_2020_msgs/ControlPanelSrv.h>
#include <controllers_2020_msgs/IndexerSrv.h>
#include <controllers_2020_msgs/IntakeArmSrv.h>
#include <controllers_2020_msgs/IntakeRollerSrv.h>
#include <controllers_2020_msgs/ShooterSrv.h>
#include <controllers_2020_msgs/TurretSrv.h>

#include "dynamic_reconfigure_wrapper/dynamic_reconfigure_wrapper.h"
#include "teleop_joystick_control/TeleopJoystickCompConfig.h"
#include "teleop_joystick_control/TeleopJoystickCompDiagnosticsConfig.h"

#include "teleop_joystick_control/TeleopCmdVel.h"

std::unique_ptr<TeleopCmdVel> teleop_cmd_vel;

double orient_strafing_angle = 0.0;

// array of joystick_states messages for multiple joysticks
std::vector <frc_msgs::JoystickState> joystick_states_array;
std::vector <std::string> topic_array;

ros::Publisher orient_strafing_enable_pub;
ros::Publisher orient_strafing_setpoint_pub;
ros::Publisher orient_strafing_state_pub;

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

double imu_angle;

void imuCallback(const sensor_msgs::Imu &imuState)
{
	const tf2::Quaternion imuQuat(imuState.orientation.x, imuState.orientation.y, imuState.orientation.z, imuState.orientation.w);
	double roll;
	double pitch;
	double yaw;
	tf2::Matrix3x3(imuQuat).getRPY(roll, pitch, yaw);

	if (yaw == yaw) // ignore NaN results
		imu_angle = yaw;
}

void preemptActionlibServers()
{
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
		}
		if(joystick_states_array[0].buttonBRelease)
		{
		}

		//Joystick1: buttonX
		if(joystick_states_array[0].buttonXPress)
		{
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

		//Joystick1: leftTrigger
		if(joystick_states_array[0].leftTrigger >= 0.5)
		{
		}
		else
		{
		}

		std_msgs::Bool enable_pub_msg;

		if((joystick_states_array[0].leftTrigger >= 0.5) && (cmd_vel.angular.z == 0.0)) //TODO Make a trigger point config value
		{
			enable_pub_msg.data = true;
		}
		else
		{
			enable_pub_msg.data = false;
		}

		orient_strafing_enable_pub.publish(enable_pub_msg);

		std_msgs::Float64 orient_strafing_angle_msg;
		orient_strafing_angle_msg.data = orient_strafing_angle;
		orient_strafing_setpoint_pub.publish(orient_strafing_angle_msg);

		std_msgs::Float64 imu_angle_msg;
		imu_angle_msg.data = imu_angle;
		orient_strafing_state_pub.publish(imu_angle_msg);

		//Joystick1: rightTrigger
		if(joystick_states_array[0].rightTrigger >= 0.5) //TODO Make a trigger point config value
		{
			teleop_cmd_vel->setSlowMode(true);
		}
		else
		{
			teleop_cmd_vel->setSlowMode(false);
		}

		//Joystick1: directionLeft
		if(joystick_states_array[0].directionLeftPress)
		{
		}
		if(joystick_states_array[0].directionLeftButton)
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
		}
		if(joystick_states_array[0].directionDownButton)
		{
		}
		if(joystick_states_array[0].directionDownRelease)
		{
		}
	}
	else if(joystick_id == 1)
	{
		static ros::Time last_header_stamp = joystick_states_array[1].header.stamp;

		static controllers_2020_msgs::ClimberSrv climber_diagnostics;
		static controllers_2020_msgs::ControlPanelSrv control_panel_diagnostics;
		static controllers_2020_msgs::IndexerSrv indexer_diagnostics;
		static controllers_2020_msgs::IntakeArmSrv intake_arm_diagnostics;
		static controllers_2020_msgs::IntakeRollerSrv intake_roller_diagnostics;
		static controllers_2020_msgs::ShooterSrv shooter_diagnostics;
		static controllers_2020_msgs::TurretSrv turret_diagnostics;

		static bool diagnostics_initialized = false;

		if(!diagnostics_initialized)
		{
			//Initialize the climber command
			climber_diagnostics.request.winch_set_point = 0.0;
			climber_diagnostics.request.climber_deploy = true;
			climber_diagnostics.request.climber_elevator_brake = true;

			//Initialize the control panel command
			control_panel_diagnostics.request.control_panel_rotations = 0.0;

			//Initialize the indexer command
			indexer_diagnostics.request.indexer_velocity = 0.0;

			//Initialize the intake command
			intake_arm_diagnostics.request.intake_arm_extend = false;
			intake_roller_diagnostics.request.percent_out = 0.0;

			//Initialize the shooter command
			shooter_diagnostics.request.shooter_hood_raise = false;
			shooter_diagnostics.request.set_velocity = 0.0;

			//Initialize the turret command
			turret_diagnostics.request.set_point = 0.0;

			diagnostics_initialized = true;
		}

		//Joystick2: leftStickY
		if(abs(joystick_states_array[1].leftStickY) > diagnostics_config.left_stick_trigger_point)
		{
			shooter_diagnostics.request.set_velocity += (std::copysign(diagnostics_config.shooter_setpoint_rate, joystick_states_array[1].leftStickY)
					*(joystick_states_array[1].header.stamp - last_header_stamp).toSec());
			ROS_WARN_STREAM("Calling shooter controller with set_velocity = " << shooter_diagnostics.request.set_velocity
					<< " (" << (shooter_diagnostics.request.set_velocity*30/M_PI) << " rpm)");
			shooter_controller_client.call(shooter_diagnostics);
		}

		//Joystick2: leftStickX
		if(abs(joystick_states_array[1].leftStickX) > diagnostics_config.left_stick_trigger_point)
		{
			turret_diagnostics.request.set_point += (std::copysign(diagnostics_config.turret_setpoint_rate, joystick_states_array[1].leftStickX)
					*(joystick_states_array[1].header.stamp - last_header_stamp).toSec());
			turret_diagnostics.request.set_point = std::clamp(turret_diagnostics.request.set_point, -diagnostics_config.turret_angle_limit, diagnostics_config.turret_angle_limit);
			ROS_WARN_STREAM("Calling turret controller with set_point = " << turret_diagnostics.request.set_point);
			turret_controller_client.call(turret_diagnostics);
		}

		//Joystick2: rightStickY
		if(abs(joystick_states_array[1].rightStickY) > 0.5) //TODO Make a trigger point config value
		{
			indexer_diagnostics.request.indexer_velocity += (std::copysign(diagnostics_config.indexer_setpoint_rate, joystick_states_array[1].leftStickY)
					*(joystick_states_array[1].header.stamp - last_header_stamp).toSec());
			ROS_WARN_STREAM("Calling indexer controller with indexer_velocity = " << indexer_diagnostics.request.indexer_velocity
					<< " (" << (indexer_diagnostics.request.indexer_velocity*30/M_PI) << " rpm)");
			indexer_controller_client.call(indexer_diagnostics);
		}

		//Joystick2: rightStickX
		if(abs(joystick_states_array[1].rightStickX) > 0.5) //TODO Make a trigger point config value
		{
		}

		//Joystick2: stickLeft
		if(joystick_states_array[1].stickLeftPress)
		{
			shooter_diagnostics.request.set_velocity = 0.0;
			ROS_WARN_STREAM("Calling shooter controller with set_velocity = 0.0 Stopping shooter!");
			shooter_controller_client.call(shooter_diagnostics);
		}
		if(joystick_states_array[1].stickLeftButton)
		{
		}
		if(joystick_states_array[1].stickLeftRelease)
		{
		}

		//Joystick2: stickRight
		if(joystick_states_array[1].stickRightPress)
		{
			indexer_diagnostics.request.indexer_velocity = 0.0;
			ROS_WARN_STREAM("Calling indexer controller with indexer_velocity = 0.0 Stopping indexer!");
			indexer_controller_client.call(indexer_diagnostics);
		}
		if(joystick_states_array[1].stickRightButton)
		{
		}
		if(joystick_states_array[1].stickRightRelease)
		{
		}

		//Joystick2: buttonA
		if(joystick_states_array[1].buttonAPress)
		{
			control_panel_diagnostics.request.control_panel_rotations = diagnostics_config.control_panel_increment;
			ROS_WARN_STREAM("Calling control panel controller with control_panel_rotations = " << control_panel_diagnostics.request.control_panel_rotations);
			control_panel_controller_client.call(control_panel_diagnostics);
			control_panel_diagnostics.request.control_panel_rotations = 0.0;
		}
		if(joystick_states_array[1].buttonAButton)
		{
		}
		if(joystick_states_array[1].buttonARelease)
		{
		}

		//Joystick2: buttonB
		if(joystick_states_array[1].buttonBPress)
		{
			intake_roller_diagnostics.request.percent_out = 0.0;
			ROS_WARN_STREAM("Calling intake controller with percent_out = 0.0 Stopping intake!");
			intake_roller_controller_client.call(intake_roller_diagnostics);
		}
		if(joystick_states_array[1].buttonBButton)
		{
		}
		if(joystick_states_array[1].buttonBRelease)
		{
		}

		//Joystick2: buttonX
		if(joystick_states_array[1].buttonXPress)
		{
		}
		if(joystick_states_array[1].buttonXButton)
		{
		}
		if(joystick_states_array[1].buttonXRelease)
		{
		}

		//Joystick2: buttonY
		if(joystick_states_array[1].buttonYPress)
		{
			intake_arm_diagnostics.request.intake_arm_extend = !intake_arm_diagnostics.request.intake_arm_extend;
			ROS_WARN_STREAM("Calling intake server with intake_arm_extend = " << intake_arm_diagnostics.request.intake_arm_extend);
			intake_arm_controller_client.call(intake_arm_diagnostics);
		}
		if(joystick_states_array[1].buttonYButton)
		{
		}
		if(joystick_states_array[1].buttonYRelease)
		{
		}

		//Joystick2: bumperLeft
		if(joystick_states_array[1].bumperLeftPress)
		{
			shooter_diagnostics.request.shooter_hood_raise = !shooter_diagnostics.request.shooter_hood_raise;
			ROS_WARN_STREAM("Calling shooter controller with shooter_hood_raise = " << shooter_diagnostics.request.shooter_hood_raise);
			shooter_controller_client.call(shooter_diagnostics);
		}
		if(joystick_states_array[1].bumperLeftButton)
		{
		}
		if(joystick_states_array[1].bumperLeftRelease)
		{
		}

		//Joystick2: bumperRight
		if(joystick_states_array[1].bumperRightPress)
		{
		}
		if(joystick_states_array[1].bumperRightButton)
		{
			intake_roller_diagnostics.request.percent_out	+= diagnostics_config.intake_setpoint_rate*(joystick_states_array[1].header.stamp - last_header_stamp).toSec();
			intake_roller_diagnostics.request.percent_out = std::clamp(intake_roller_diagnostics.request.percent_out, -1.0, 1.0);
			ROS_WARN_STREAM("Calling intake controller with percent_out = " << intake_roller_diagnostics.request.percent_out);
			intake_roller_controller_client.call(intake_roller_diagnostics);
		}
		if(joystick_states_array[1].bumperRightRelease)
		{
		}

		//Joystick2: leftTrigger
		if(joystick_states_array[1].leftTrigger >= 0.5) //TODO Make a trigger point config value
		{
		}
		else
		{
		}

		//Joystick2: rightTrigger
		if(joystick_states_array[1].rightTrigger >= 0.5) //TODO Make a trigger point config value
		{
			intake_roller_diagnostics.request.percent_out	-= diagnostics_config.intake_setpoint_rate*(joystick_states_array[1].header.stamp - last_header_stamp).toSec();
			intake_roller_diagnostics.request.percent_out = std::clamp(intake_roller_diagnostics.request.percent_out, -1.0, 1.0);
			ROS_WARN_STREAM("Calling intake controller with percent_out = " << intake_roller_diagnostics.request.percent_out);
			intake_roller_controller_client.call(intake_roller_diagnostics);
		}
		else
		{
		}

		//Joystick2: directionLeft
		if(joystick_states_array[1].directionLeftPress)
		{
			climber_diagnostics.request.climber_deploy = !climber_diagnostics.request.climber_deploy;
			ROS_WARN_STREAM("Calling climber controller with climber_deploy = " << climber_diagnostics.request.climber_deploy);
			climber_controller_client.call(climber_diagnostics);
		}
		if(joystick_states_array[1].directionLeftButton)
		{
		}
		if(joystick_states_array[1].directionLeftRelease)
		{
		}

		//Joystick2: directionRight
		if(joystick_states_array[1].directionRightPress)
		{
			climber_diagnostics.request.climber_elevator_brake = !climber_diagnostics.request.climber_elevator_brake;
			ROS_WARN_STREAM("Calling climber controller with climber_elevator_brake = " << climber_diagnostics.request.climber_elevator_brake);
			climber_controller_client.call(climber_diagnostics);
		}
		if(joystick_states_array[1].directionRightButton)
		{
		}
		if(joystick_states_array[1].directionRightRelease)
		{
		}

		//Joystick2: directionUp
		if(joystick_states_array[1].directionUpPress)
		{
		}
		if(joystick_states_array[1].directionUpButton)
		{
			climber_diagnostics.request.winch_set_point += diagnostics_config.climber_setpoint_rate*((joystick_states_array[1].header.stamp - last_header_stamp).toSec());
			climber_diagnostics.request.winch_set_point = std::clamp(climber_diagnostics.request.winch_set_point, diagnostics_config.climber_low_bound, diagnostics_config.climber_high_bound);
			ROS_WARN_STREAM("Calling climber controller with winch_set_point = " << climber_diagnostics.request.winch_set_point);
			climber_controller_client.call(climber_diagnostics);
		}
		if(joystick_states_array[1].directionUpRelease)
		{
		}

		//Joystick2: directionDown
		if(joystick_states_array[1].directionDownPress)
		{
		}
		if(joystick_states_array[1].directionDownButton)
		{
			climber_diagnostics.request.winch_set_point -= diagnostics_config.climber_setpoint_rate*((joystick_states_array[1].header.stamp - last_header_stamp).toSec());
			climber_diagnostics.request.winch_set_point = std::clamp(climber_diagnostics.request.winch_set_point, diagnostics_config.climber_low_bound, diagnostics_config.climber_high_bound);
			ROS_WARN_STREAM("Calling climber controller with winch_set_point = " << climber_diagnostics.request.winch_set_point);
			climber_controller_client.call(climber_diagnostics);
		}
		if(joystick_states_array[1].directionDownRelease)
		{
		}

		last_header_stamp = joystick_states_array[1].header.stamp;
	}
}

void jointStateCallback(const sensor_msgs::JointState &joint_state)
{
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

	if(!n_diagnostics_params.getParam("climber_setpoint_rate", diagnostics_config.climber_setpoint_rate))
	{
		ROS_ERROR("Could not read climber_setpoint_rate in teleop_joystick_comp");
	}
	if(!n_diagnostics_params.getParam("climber_low_bound", diagnostics_config.climber_low_bound))
	{
		ROS_ERROR("Could not read climber_low_bound in teleop_joystick_comp");
	}
	if(!n_diagnostics_params.getParam("climber_high_bound", diagnostics_config.climber_high_bound))
	{
		ROS_ERROR("Could not read climber_high_bound in teleop_joystick_comp");
	}
	if(!n_diagnostics_params.getParam("shooter_setpoint_rate", diagnostics_config.shooter_setpoint_rate))
	{
		ROS_ERROR("Could not read shooter_setpoint_rate in teleop_joystick_comp");
	}
	if(!n_diagnostics_params.getParam("left_stick_trigger_point", diagnostics_config.left_stick_trigger_point))
	{
		ROS_ERROR("Could not read left_stick_trigger_point in teleop_joystick_comp");
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
		ROS_ERROR("Could not read intake_setpoint_rate in teleop_joystick_comp");
	}
	if(!n_diagnostics_params.getParam("control_panel_increment", diagnostics_config.control_panel_increment))
	{
		ROS_ERROR("Could not read control_panel_increment in teleop_joystick_comp");
	}

	teleop_cmd_vel = std::make_unique<TeleopCmdVel>(config);

	imu_angle = M_PI / 2.;

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";

	BrakeSrv = n.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, service_connection_header);
	if(!BrakeSrv.waitForExistence(ros::Duration(15)))
	{
		ROS_ERROR("Wait (15 sec) timed out, for Brake Service in teleop_joystick_comp.cpp");
	}

	orient_strafing_enable_pub = n.advertise<std_msgs::Bool>("orient_strafing/pid_enable", 1);
	orient_strafing_setpoint_pub = n.advertise<std_msgs::Float64>("orient_strafing/setpoint", 1);
	orient_strafing_state_pub = n.advertise<std_msgs::Float64>("orient_strafing/state", 1);
	JoystickRobotVel = n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);
	ros::Subscriber imu_heading = n.subscribe("/imu/zeroed_imu", 1, &imuCallback);
	ros::Subscriber joint_states_sub = n.subscribe("/frcrobot_jetson/joint_states", 1, &jointStateCallback);

	ros::ServiceServer robot_orient_service = n.advertiseService("robot_orient", orientCallback);

	ros::ServiceServer orient_strafing_angle_service = n.advertiseService("orient_strafing_angle", orientStrafingAngleCallback);

	climber_controller_client = n.serviceClient<controllers_2020_msgs::ClimberSrv>("/frcrobot_jetson/climber_controller_2020/climber_command");
	indexer_controller_client = n.serviceClient<controllers_2020_msgs::IndexerSrv>("/frcrobot_jetson/indexer_controller/indexer_command");
	intake_arm_controller_client = n.serviceClient<controllers_2020_msgs::IntakeArmSrv>("/frcrobot_jetson/intake_controller/intake_arm_command");
	intake_roller_controller_client = n.serviceClient<controllers_2020_msgs::IntakeRollerSrv>("/frcrobot_jetson/intake_controller/intake_roller_command");
	shooter_controller_client = n.serviceClient<controllers_2020_msgs::ShooterSrv>("/frcrobot_jetson/shooter_controller/shooter_command");
	turret_controller_client = n.serviceClient<controllers_2020_msgs::TurretSrv>("/frcrobot_jetson/turret_controller/shooter_command");

	DynamicReconfigureWrapper<teleop_joystick_control::TeleopJoystickCompConfig> drw(n_params, config);
	DynamicReconfigureWrapper<teleop_joystick_control::TeleopJoystickCompDiagnosticsConfig> diagnostics_drw(n_diagnostics_params, diagnostics_config);

	//Read from _num_joysticks_ joysticks
	// Set up this callback last, since it might use all of the various stuff
	// initialized above here. Setting it up first risks the chance that a callback
	// happens immediately and tries to use them before they have valid values
	std::vector <ros::Subscriber> subscriber_array;
	joystick_states_array.resize(num_joysticks);
	for(int j = 0; j < num_joysticks; j++)
	{
		std::stringstream s;
		s << "/teleop/translator";
		s << j;
		s << "/joystick_states";
		topic_array.push_back(s.str());
		subscriber_array.push_back(n.subscribe(topic_array[j], 1, &evaluateCommands));
	}

	ROS_WARN("joy_init");

	ros::spin();
	return 0;
}
