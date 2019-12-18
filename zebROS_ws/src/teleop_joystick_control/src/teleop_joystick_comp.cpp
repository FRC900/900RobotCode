#include <realtime_tools/realtime_buffer.h>
#include "teleop_joystick_control/teleop_joystick_comp.h"
#include "teleop_joystick_control/rate_limiter.h"
#include "std_srvs/Empty.h"

#include "std_srvs/Trigger.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"

#include "behaviors/IntakeAction.h"
#include "behaviors/IntakeGoal.h"
#include "behaviors/PlaceAction.h"
#include "behaviors/PlaceGoal.h"
#include "behaviors/ElevatorAction.h"
#include "behaviors/ElevatorGoal.h"
#include "behaviors/ClimbAction.h"
#include "behaviors/ClimbGoal.h"
#include "behaviors/AlignAction.h"
#include "behaviors/AlignGoal.h"
#include "actionlib/client/simple_action_client.h"
#include "behaviors/enumerated_elevator_indices.h"

#include "std_srvs/SetBool.h"
#include "std_srvs/Empty.h"
#include <vector>
#include "teleop_joystick_control/RobotOrient.h"

#include "controllers_2019/PanelIntakeSrv.h"
#include "controllers_2019/CargoIntakeSrv.h"

#include "dynamic_reconfigure_wrapper/dynamic_reconfigure_wrapper.h"
#include "teleop_joystick_control/TeleopJoystickCompConfig.h"

int elevator_cur_setpoint_idx;
int climber_cur_step;

int cargo_limit_switch_true_count = 0;
int panel_limit_switch_true_count = 0;
int cargo_limit_switch_false_count = 0;
int panel_limit_switch_false_count = 0;
bool panel_push_extend = false;

const int climber_num_steps = 4;
const int elevator_num_setpoints = 4;

bool robot_orient = false;
double offset_angle = 0;

// array of joystick_states messages for multiple joysticks
std::vector <frc_msgs::JoystickState> joystick_states_array;
std::vector <std::string> topic_array;

teleop_joystick_control::TeleopJoystickCompConfig config;

// Set to either fast or slow max speeds based on mode
double max_speed;
double max_rot;

// 500 msec to go from full back to full forward
constexpr double drive_rate_limit_time = 100.;

ros::Publisher elevator_setpoint;
ros::Publisher JoystickRobotVel;
ros::Publisher cargo_pid;
ros::Publisher terabee_pid;
ros::Publisher distance_pid;
ros::Publisher navX_pid;
ros::Publisher enable_align;

ros::ServiceClient BrakeSrv;
ros::ServiceClient run_align;

ros::ServiceClient manual_server_panelIn;
ros::ServiceClient manual_server_cargoOut;
ros::ServiceClient manual_server_cargoIn;

ros::ServiceClient continue_outtake_client;

ros::ServiceClient align_with_terabee;

//use shared pointers to make the clients global
std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeAction>> intake_cargo_ac;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::PlaceAction>> outtake_cargo_ac;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeAction>> intake_hatch_panel_ac;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::PlaceAction>> outtake_hatch_panel_ac;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::ElevatorAction>> elevator_ac;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::ClimbAction>> climber_ac;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::AlignAction>> align_hatch_ac;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::AlignAction>> align_cargo_ac;
double navX_angle;

bool ManualToggleClamp = false;
bool ManualTogglePush = false;
bool ManualToggleKicker = false;
bool ManualToggleArm = false;

double dead_zone_check(double test_axis, double dead_zone)
{
	// Less than dead_zone? Return 0
	if (fabs(test_axis) <= fabs(dead_zone))
		return 0;

	// Scale the remaining values - just above dead_zone
	// is 0, scaling up to 1.0 as the input value goes
	// to 1.0. This avoids a jerk from 0->0.x as the input
	// transitions from dead_zone to non-dead_zone
	return (fabs(test_axis) - dead_zone) / (1.0 - dead_zone);
}

void navXCallback(const sensor_msgs::Imu &navXState)
{
	const tf2::Quaternion navQuat(navXState.orientation.x, navXState.orientation.y, navXState.orientation.z, navXState.orientation.w);
	double roll;
	double pitch;
	double yaw;
	tf2::Matrix3x3(navQuat).getRPY(roll, pitch, yaw);

	if (yaw == yaw) // ignore NaN results
		navX_angle = yaw;
}

void preemptActionlibServers()
{
	intake_cargo_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
	outtake_cargo_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
	intake_hatch_panel_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
	outtake_hatch_panel_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
	elevator_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
	climber_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
	align_hatch_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
	align_cargo_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
}

bool orientCallback(teleop_joystick_control::RobotOrient::Request& req,
					teleop_joystick_control::RobotOrient::Response&/* res*/)
{
	// Used to switch between robot orient and field orient driving
	robot_orient = req.robot_orient;
	offset_angle = req.offset_angle;
	ROS_WARN_STREAM("Robot Orient = " << (robot_orient) << ", Offset Angle = " << offset_angle);
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
		// Raw joystick values for X & Y translation
		const double leftStickX = joystick_states_array[0].leftStickX;
		const double leftStickY = joystick_states_array[0].leftStickY;
		//ROS_INFO_STREAM(__LINE__ << " "  << leftStickX << " " << leftStickY);

		// Convert to polar coordinates
		double direction = atan2(leftStickY, leftStickX);

		// Do a dead zone check on the magnitude of the velocity,
		// then scale it by a power function to increase resolution
		// of low-speed inputs. This will give an output range from
		// 0-100%
		// Finally, scale it so that 0% corresponds to the minimum
		// output needed to move the robot and 100% to the max
		// configured speed
		double magnitude = dead_zone_check(hypot(leftStickX, leftStickY), config.joystick_deadzone);
		//ROS_INFO_STREAM(__LINE__ << " "  << magnitude);
		if (magnitude != 0)
		{
			magnitude = pow(magnitude, config.joystick_pow);
			//ROS_INFO_STREAM(__LINE__ << " "  << magnitude << " " << direction);
			magnitude *= max_speed - config.min_speed;
			//ROS_INFO_STREAM(__LINE__ << " "  << magnitude);
			magnitude += config.min_speed;
			//ROS_INFO_STREAM(__LINE__ << " "  << magnitude);
		}

		// This applies a ramp to the output - it limits the amount of change per
		// unit time that's allowed.  This helps limit extreme current draw in
		// cases where the robot is changing direction rapidly.
		static rate_limiter::RateLimiter x_rate_limit(-max_speed, max_speed, drive_rate_limit_time);
		static rate_limiter::RateLimiter y_rate_limit(-max_speed, max_speed, drive_rate_limit_time);

		// Convert back to rectangular coordinates for the x and Y velocity
		const double xSpeed = x_rate_limit.applyLimit(magnitude * cos(direction), joystick_states_array[0].header.stamp);
		const double ySpeed = y_rate_limit.applyLimit(magnitude * sin(direction), joystick_states_array[0].header.stamp);
		//ROS_INFO_STREAM(__LINE__ << " "  << xSpeed << " " << ySpeed);

		// Rotation is a bit simpler since it is just one independent axis
		const double rightStickX = dead_zone_check(joystick_states_array[0].rightStickX, config.joystick_deadzone);

		// Scale the input by a power function to increase resolution
		// of the slower settings. Use copysign to preserve the sign
		// of the original input (keeps the direction correct)
		double rotation = pow(rightStickX, config.rotation_pow);
		rotation  = copysign(rotation, joystick_states_array[0].rightStickX);
		rotation *= max_rot;

		// Rate-limit changes in rotation
		static rate_limiter::RateLimiter rotation_rate_limit(-max_rot, max_rot, drive_rate_limit_time);
		rotation = rotation_rate_limit.applyLimit(rotation, joystick_states_array[0].header.stamp);

		static bool sendRobotZero = false;
		if (xSpeed == 0.0 && ySpeed == 0.0 && rotation == 0.0)
		{
			if (!sendRobotZero)
			{
				geometry_msgs::Twist vel;
				vel.linear.x = 0;
				vel.linear.y = 0;
				vel.linear.z = 0;

				vel.angular.x = 0;
				vel.angular.y = 0;
				vel.angular.z = 0;

				JoystickRobotVel.publish(vel);
				std_srvs::Empty empty;
				if (!BrakeSrv.call(empty))
				{
					ROS_ERROR("BrakeSrv call failed in sendRobotZero");
				}
				ROS_INFO("BrakeSrv called");
				sendRobotZero = true;
			}
		}
		else // X or Y or rotation != 0 so tell the drive base to move
		{
			//Publish drivetrain messages and call servers
			Eigen::Vector2d joyVector;
			joyVector[0] = -xSpeed; //intentionally flipped
			joyVector[1] = -ySpeed;

			const Eigen::Rotation2Dd rotate(robot_orient ? -offset_angle : -navX_angle);
			const Eigen::Vector2d rotatedJoyVector = rotate.toRotationMatrix() * joyVector;

			geometry_msgs::Twist vel;
			vel.linear.x = rotatedJoyVector[1];
			vel.linear.y = rotatedJoyVector[0];
			vel.linear.z = 0;

			vel.angular.x = 0;
			vel.angular.y = 0;
			vel.angular.z = -rotation;

			JoystickRobotVel.publish(vel);
			sendRobotZero = false;
		}

		//Joystick1: buttonA
		if(joystick_states_array[0].buttonAPress)
		{
			//Align the robot
			ROS_WARN("Joystick1: buttonAPress - Auto Align");

			preemptActionlibServers();
			behaviors::AlignGoal goal;
			goal.trigger = true;
			if(cargo_limit_switch_true_count > config.limit_switch_debounce_iterations) {
				goal.has_cargo = true;
				align_cargo_ac->sendGoal(goal);
			}
			else {
				goal.has_cargo = false;
				align_hatch_ac->sendGoal(goal);
			}
		}
		if(joystick_states_array[0].buttonAButton)
		{
			/*
			ROS_INFO_THROTTLE(1, "buttonAButton");
            std_msgs::Bool enable_pid;
			enable_pid.data = true;
            terabee_pid.publish(enable_pid);
			enable_align.publish(enable_pid);
			*/

			//behaviors::AlignGoal goal;
			//goal.trigger = true;
			//align_hatch_ac->sendGoal(goal);
		}
		if(joystick_states_array[0].buttonARelease)
		{
			/*
            std_msgs::Bool enable_pid;
			enable_pid.data = false;
            terabee_pid.publish(enable_pid);
			enable_align.publish(enable_pid);
			*/
			ROS_INFO_STREAM("Joystick1: buttonARelease");
		}

		//Joystick1: buttonB
		if(joystick_states_array[0].buttonBPress)
		{
			ROS_INFO_STREAM("Joystick1: bumperLeftPress");
			preemptActionlibServers();
			ROS_INFO_STREAM("Joystick1: Place Panel");
			behaviors::PlaceGoal goal;
			goal.setpoint_index = elevator_cur_setpoint_idx;
            goal.end_setpoint_index = INTAKE;
			outtake_hatch_panel_ac->sendGoal(goal);
			elevator_cur_setpoint_idx = 0;
			/*
			preemptActionlibServers();
			behaviors::AlignGoal goal;
			goal.has_cargo = false;
			align_hatch_ac->sendGoal(goal);
			*/
		}
		if(joystick_states_array[0].buttonBRelease) {
            std_srvs::Empty empty_srv;
            continue_outtake_client.call(empty_srv);
        }

		//Joystick1: buttonX
		if(joystick_states_array[0].buttonXPress)
		{
			//Determines where elevator will go when called to outtake or move to a setpoint
			ROS_INFO_STREAM("Joystick1: buttonXPress - Increment Elevator");
			elevator_cur_setpoint_idx = (elevator_cur_setpoint_idx + 1) % elevator_num_setpoints;
			ROS_WARN("elevator current setpoint index %d", elevator_cur_setpoint_idx);
		}
		if(joystick_states_array[0].buttonXButton)
		{
			/*
			ROS_INFO_THROTTLE(1, "buttonXButton");
            std_msgs::Bool enable_pid;
			enable_pid.data = true;
            distance_pid.publish(enable_pid);
			enable_align.publish(enable_pid);
			*/
		}
		if(joystick_states_array[0].buttonXRelease)
		{
			/*
			ROS_INFO_STREAM("Joystick1: buttonXRelease");
            std_msgs::Bool enable_pid;
			enable_pid.data = false;
            distance_pid.publish(enable_pid);
			enable_align.publish(enable_pid);
			*/
		}
		//Joystick1: buttonY
		if(joystick_states_array[0].buttonYPress)
		{
			ROS_INFO_STREAM("joystick1: buttonYPress");
			preemptActionlibServers();

			behaviors::IntakeGoal goal;
			intake_hatch_panel_ac->sendGoal(goal);
		}
		//Joystick1: bumperLeft
	  /*
		if(joystick_states_array[0].bumperLeftPress)
		{
			ROS_INFO_STREAM("Joystick1: bumperLeftPress");
			preemptActionlibServers();
			if(cargo_limit_switch_true_count > config.limit_switch_debounce_iterations)
			{
				//If we have a cargo, outtake it
				ROS_INFO_STREAM("Joystick1: Place Cargo");
				behaviors::PlaceGoal goal;
				goal.setpoint_index = elevator_cur_setpoint_idx;
				outtake_cargo_ac->sendGoal(goal);
				elevator_cur_setpoint_idx = 0;
				ROS_WARN("elevator current setpoint index %d", elevator_cur_setpoint_idx);
			}
			else
			{
				//If we don't have a cargo, intake one
				ROS_INFO_STREAM("Joystick1: Intake Cargo");
				behaviors::IntakeGoal goal;
				intake_cargo_ac->sendGoal(goal);
			}
		}
		*/
		if(joystick_states_array[0].bumperLeftPress)
		{
            //TODO get rid of this testing code
            //If we have a cargo, outtake it
			preemptActionlibServers();
            ROS_INFO_STREAM("Joystick1: Place Cargo");
            behaviors::PlaceGoal goal;
            goal.setpoint_index = elevator_cur_setpoint_idx;
			goal.end_setpoint_index = INTAKE;
            outtake_cargo_ac->sendGoal(goal);
            elevator_cur_setpoint_idx = 0;
            ROS_WARN("elevator current setpoint index %d", elevator_cur_setpoint_idx);
            /* NOOT NOOT TOGGLE
			if (panel_push_extend)
			{
				ROS_INFO_STREAM("Toggling to clamped and not extended");
				controllers_2019::PanelIntakeSrv srv;
				srv.request.claw_release = false;
				srv.request.push_extend = false;
				if (!manual_server_panelIn.call(srv))
					ROS_ERROR("teleop call to manual_server_panelIn failed for bumperLeftPress");
			}
			else
			{
				ROS_INFO_STREAM("Toggling to unclamped and extended");
				controllers_2019::PanelIntakeSrv srv;
				srv.request.claw_release = true;
				srv.request.push_extend = true;
				if (!manual_server_panelIn.call(srv))
					ROS_ERROR("teleop call to manual_server_panelIn failed for bumperLeftPress");
			}
            */
		}
		if(joystick_states_array[0].bumperLeftButton)
		{
		}
		if(joystick_states_array[0].bumperLeftRelease)
		{
			ROS_INFO_STREAM("Joystick1: bumperLeftRelease");
			outtake_cargo_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
		}
		//Joystick1: bumperRight
		if(joystick_states_array[0].bumperRightPress)
		{
            //TODO get rid of this testing cde
            //If we don't have a cargo, intake one
			preemptActionlibServers();
            ROS_INFO_STREAM("Joystick1: Intake Cargo");
            behaviors::IntakeGoal goal;
            intake_cargo_ac->sendGoal(goal);
            /*
			if (intake_arm_down)
			{
				ROS_INFO_STREAM("Toggling to roller not extended");
				cargo_intake_controller::CargoIntakeSrv srv;
				srv.request.roller_extend = false;
				if (!manual_server_cargoIn.call(srv))
					ROS_ERROR("teleop call to manual_server_cargoIn failed for bumperRightPress");
			}
			else
			{
				ROS_INFO_STREAM("Toggling to roller extended");
				cargo_intake_controller::CargoIntakeSrv srv;
				srv.request.roller_extend = true;
				if (!manual_server_cargoIn.call(srv))
					ROS_ERROR("teleop call to manual_server_cargoIn failed for bumperRightPress");
			}
            */
		}
		if(joystick_states_array[0].bumperRightButton)
		{
		}
		if(joystick_states_array[0].bumperRightRelease)
		{
			//forces right bumper to be held for intaking to continue
			//TODO test this brings arm up instantly
			intake_cargo_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
			ROS_INFO_STREAM("Joystick1: bumperRightRelease");

		}
		static bool slow_mode      = false;
		static bool prev_slow_mode = false;
		if(joystick_states_array[0].rightTrigger >= 0.5)
		{
			max_speed = config.max_speed_slow;
			max_rot = config.max_rot_slow;
			slow_mode = true;
		}
		else
		{
			max_speed = config.max_speed;
			max_rot = config.max_rot;
			slow_mode = false;
		}
		if (prev_slow_mode != slow_mode)
		{
			x_rate_limit.updateMinMax(-max_speed, max_speed);
			y_rate_limit.updateMinMax(-max_speed, max_speed);
			rotation_rate_limit.updateMinMax(-max_rot, max_rot);
			prev_slow_mode = slow_mode;
		}
		if(joystick_states_array[0].leftTrigger >= 0.5)
		{
		}
		//Joystick1: directionLeft
		if(joystick_states_array[0].directionLeftPress)
		{
			//Move the elevator to the current setpoint
			ROS_WARN("Calling elevator server; move to setpoint %d", elevator_cur_setpoint_idx);
			preemptActionlibServers();
			behaviors::ElevatorGoal goal;
			goal.setpoint_index = elevator_cur_setpoint_idx;
			if(cargo_limit_switch_true_count > config.limit_switch_debounce_iterations)
			{
				goal.place_cargo = true;
			}
			else
			{
				goal.place_cargo = false;
			}
			elevator_ac->sendGoal(goal);
			ROS_WARN("elevator current setpoint index %d", elevator_cur_setpoint_idx);
		}
		if(joystick_states_array[0].directionLeftButton)
		{
			ROS_INFO_THROTTLE(1, "directionLeftButton");
		}
		if(joystick_states_array[0].directionLeftRelease)
		{
			ROS_INFO_STREAM("Joystick1: directionLeftRelease");
		}
		//Joystick1: directionRight
		if(joystick_states_array[0].directionRightPress)
		{
			//Preempt every server running; for emergencies or testing only
			ROS_WARN("Preempting All Servers");
			preemptActionlibServers();
		}
		if(joystick_states_array[0].directionRightButton)
		{
			ROS_INFO_THROTTLE(1, "directionRightButton");
		}
		if(joystick_states_array[0].directionRightRelease)
		{
			ROS_INFO_STREAM("Joystick1: directionRightRelease");
		}
		//Joystick1: directionUp
		if(joystick_states_array[0].directionUpPress)
		{
			//Start or move to the next step of the climb
			ROS_INFO_STREAM("Joystick1: Calling Climber Server");
			preemptActionlibServers();
			ROS_WARN("Climber current step = %d", climber_cur_step);
			behaviors::ClimbGoal goal;
			goal.step = climber_cur_step;
			climber_ac->sendGoal(goal);
			climber_cur_step = (climber_cur_step + 1) % climber_num_steps;
		}
		if(joystick_states_array[0].directionUpButton)
		{
			ROS_INFO_THROTTLE(1, "directionUpButton");
		}
		if(joystick_states_array[0].directionUpRelease)
		{
			ROS_INFO_STREAM("Joystick1: directionUpRelease");
		}
		//Joystick1: directionDown
		if(joystick_states_array[0].directionDownPress)
		{
			//Abort the climb and lower back down
			ROS_WARN("Joystick1: Preempting Climber Server");
			climber_ac->cancelAllGoals();
			climber_cur_step = 0;
		}
		if(joystick_states_array[0].directionDownButton)
		{
			ROS_INFO_THROTTLE(1, "directionDownButton");
		}
		if(joystick_states_array[0].directionDownRelease)
		{
			ROS_INFO_STREAM("Joystick1: directionDownRelease");
		}
	}

	else if(joystick_id == 1)
	{
		//Joystick2: buttonA
		if(joystick_states_array[1].buttonAPress) //Clamp
		{
			ManualToggleClamp = !ManualToggleClamp;
		    ROS_INFO_STREAM("Joystick2: buttonAPress");
			controllers_2019::PanelIntakeSrv msg;
		    msg.request.claw_release = ManualToggleClamp;
		    msg.request.push_extend = ManualTogglePush;
			if (!manual_server_panelIn.call(msg))
				ROS_ERROR("teleop call to manual_server_panelIn failed for buttonAPress");
			//cargo_outtake_controller::CargoOuttakeSrv msg2;
			//msg2.request.kicker_in = ManualToggleKicker;
			//msg2.request.clamp_release = ManualToggleClamp;
			//if (!manual_server_cargoOut.call(msg2))
			//	ROS_ERROR("teleop call to manual_server_cargoOut failed for buttonAPress");
		}
		/*  if(joystick_states_array[1].buttonAButton)
		  {
		  ROS_INFO_THROTTLE(1, "buttonAButton");
		  std_srvs::SetBool msg;
		  msg.request.data = true;
		  run_align.call(msg);
		  }
		  if(joystick_states_array[1].buttonARelease)
		  {
		  ROS_INFO_STREAM("Joystick2: buttonARelease");
		  std_srvs::SetBool msg;
		  msg.request.data = false;
		  run_align.call(msg);
		  }*/
		//Joystick2: buttonB
		/* TODO this may be causing weird issues
		if(joystick_states_array[1].buttonBPress)
		{
			ManualTogglePush = !ManualTogglePush;
			ROS_INFO_STREAM("Joystick2: buttonBPress");
			controllers_2019::PanelIntakeSrv msg;
			msg.request.claw_release = ManualToggleClamp;
			msg.request.push_extend = ManualTogglePush;
			if (!manual_server_panelIn.call(msg))
				ROS_ERROR("teleop call to manual_server_panelIn failed for buttonBPress");
		} */
		/*if(joystick_states_array[1].buttonBButton)
		{
		ROS_INFO_THROTTLE(1, "buttonBButton");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
		}
		if(joystick_states_array[1].buttonBRelease)
		{
		ROS_INFO_STREAM("Joystick2: buttonBRelease");
		std_srvs::SetBool msg;
		msg.request.data = false;
		run_align.call(msg);
		}*/
		//Joystick2: buttonX
		if(joystick_states_array[1].buttonXPress)
		{
			//ManualToggleKicker = !ManualToggleKicker;
			//ROS_INFO_STREAM("Joystick2: buttonXPress");
			//cargo_outtake_controller::CargoOuttakeSrv msg;
			//msg.request.kicker_in = ManualToggleKicker;
			//msg.request.clamp_release = ManualToggleClamp;
			//if (!manual_server_cargoOut.call(msg))
			//	ROS_ERROR("teleop call to manual_server_cargoOut failed for buttonXPress");
		}
		/*if(joystick_states_array[1].buttonXButton)
		{
		ROS_INFO_THROTTLE(1, "buttonXButton");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
		}
		if(joystick_states_array[1].buttonXRelease)
		{
		ROS_INFO_STREAM("Joystick2: buttonXRelease");
		std_srvs::SetBool msg;
		msg.request.data = false;
		run_align.call(msg);
		}*/
		//Joystick2: buttonY
		/* TODO this may be causing weird issues
		if(joystick_states_array[1].buttonYPress)
		{
			ManualToggleArm = !ManualToggleArm;
			ROS_INFO_STREAM("Joystick2: buttonYPress");
			cargo_intake_controller::CargoIntakeSrv msg;
			msg.request.intake_arm = ManualToggleArm;
			msg.request.power = 0.0;
			if (!manual_server_cargoIn.call(msg))
				ROS_ERROR("teleop call to manual_server_cargoIn failed for buttonYPress");
		} */
/*	if(joystick_states_array[1].buttonYButton)
	{
		ROS_INFO_THROTTLE(1, "buttonYButton");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
	}
	if(joystick_states_array[1].buttonYRelease)
	{
		ROS_INFO_STREAM("Joystick2: buttonYRelease");
		std_srvs::SetBool msg;
		msg.request.data = false;
		run_align.call(msg);
	}
	//Joystick2: bumperLeft
	if(joystick_states_array[1].bumperLeftPress)
	{
		ROS_INFO_STREAM("Joystick2: bumperLeftPress");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
	}
	if(joystick_states_array[1].bumperLeftButton)
	{
		ROS_INFO_THROTTLE(1, "bumperLeftButton");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
	}
	if(joystick_states_array[1].bumperLeftRelease)
	{
		ROS_INFO_STREAM("Joystick2: bumperLeftRelease");
		std_srvs::SetBool msg;
		msg.request.data = false;
		run_align.call(msg);
	}
	//Joystick2: bumperRight
	if(joystick_states_array[1].bumperRightPress)
	{
		ROS_INFO_STREAM("Joystick2: bumperRightPress");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
	}
	if(joystick_states_array[1].bumperRightButton)
	{
		ROS_INFO_THROTTLE(1, "bumperRightButton");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
	}
	if(joystick_states_array[1].bumperRightRelease)
	{
		ROS_INFO_STREAM("Joystick2: bumperRightRelease");
		std_srvs::SetBool msg;
		msg.request.data = false;
		run_align.call(msg);
	}
	//Joystick2: directionLeft
	if(joystick_states_array[1].directionLeftPress)
	{
		ROS_INFO_STREAM("Joystick2: directionLeftPress");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
	}
	if(joystick_states_array[1].directionLeftButton)
	{
		ROS_INFO_THROTTLE(1, "directionLeftButton");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
	}
	if(joystick_states_array[1].directionLeftRelease)
	{
		ROS_INFO_STREAM("Joystick2: directionLeftRelease");
		std_srvs::SetBool msg;
		msg.request.data = false;
		run_align.call(msg);
	}
	//Joystick2: directionRight
	if(joystick_states_array[1].directionRightPress)
	{
		ROS_INFO_STREAM("Joystick2: directionRightPress");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
	}
	if(joystick_states_array[1].directionRightButton)
	{
		ROS_INFO_THROTTLE(1, "directionRightButton");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
	}
	if(joystick_states_array[1].directionRightRelease)
	{
		ROS_INFO_STREAM("Joystick2: directionRightRelease");
		std_srvs::SetBool msg;
		msg.request.data = false;
		run_align.call(msg);
	}
	//Joystick2: directionUp
	if(joystick_states_array[1].directionUpPress)
	{
		ROS_INFO_STREAM("Joystick2: directionUpPress");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
	}
	if(joystick_states_array[1].directionUpButton)
	{
		ROS_INFO_THROTTLE(1, "directionUpButton");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
	}
	if(joystick_states_array[1].directionUpRelease)
	{
		ROS_INFO_STREAM("Joystick2: directionUpRelease");
		std_srvs::SetBool msg;
		msg.request.data = false;
		run_align.call(msg);
	}
	//Joystick2: directionDown
	if(joystick_states_array[1].directionDownPress)
	{
		ROS_INFO_STREAM("Joystick2: directionDownPress");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
	}
	if(joystick_states_array[1].directionDownButton)
	{
		ROS_INFO_THROTTLE(1, "directionDownButton");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
	}
	if(joystick_states_array[1].directionDownRelease)
	{
		ROS_INFO_STREAM("Joystick2: directionDownRelease");
		std_srvs::SetBool msg;
		msg.request.data = false;
		run_align.call(msg);
	}*/
	}

    //Publish elevator setpoinut
    std_msgs::Int8 elevator_setpoint_msg;
    elevator_setpoint_msg.data = elevator_cur_setpoint_idx;
    elevator_setpoint.publish(elevator_setpoint_msg);
}

void jointStateCallback(const sensor_msgs::JointState &joint_state)
{
	//get index of limit_switch sensor for this actionlib server
	static size_t cargo_linebreak_idx = std::numeric_limits<size_t>::max();
	static size_t panel_limit_switch_1_idx = std::numeric_limits<size_t>::max();
	static size_t panel_limit_switch_2_idx = std::numeric_limits<size_t>::max();
	static size_t panel_push_extend_idx = std::numeric_limits<size_t>::max();
	if (cargo_linebreak_idx >= joint_state.name.size() || panel_limit_switch_1_idx >= joint_state.name.size() || panel_limit_switch_2_idx >= joint_state.name.size())
	{
		for (size_t i = 0; i < joint_state.name.size(); i++)
		{
			if (joint_state.name[i] == "cargo_intake_linebreak_1")
				cargo_linebreak_idx = i;
			if (joint_state.name[i] == "panel_intake_limit_switch_1")
				panel_limit_switch_1_idx = i;
			if (joint_state.name[i] == "panel_intake_limit_switch_2")
				panel_limit_switch_2_idx = i;
			if (joint_state.name[i] == "panel_push_extend")
				panel_push_extend_idx = i;
		}
	}

	//update limit_switch counts based on the value of the limit_switch sensor
	if (cargo_linebreak_idx < joint_state.position.size())
	{
		bool cargo_limit_switch_true = (joint_state.position[cargo_linebreak_idx] != 0);
		if(cargo_limit_switch_true)
		{
			cargo_limit_switch_true_count += 1;
			cargo_limit_switch_false_count = 0;
		}
		else
		{
			cargo_limit_switch_true_count = 0;
			cargo_limit_switch_false_count += 1;
		}
	}
	else
	{
		ROS_WARN_THROTTLE(1, "outtake line break sensor not found in joint_states");
		cargo_limit_switch_false_count = 0;
		cargo_limit_switch_true_count = 0;
	}

	if (panel_limit_switch_1_idx < joint_state.position.size() && panel_limit_switch_2_idx < joint_state.position.size())
	{
		bool panel_limit_switch_true = ((joint_state.position[panel_limit_switch_1_idx] != 0) || (joint_state.position[panel_limit_switch_2_idx] != 0));
		if(panel_limit_switch_true)
		{
			panel_limit_switch_true_count += 1;
			panel_limit_switch_false_count = 0;
		}
		else
		{
			panel_limit_switch_true_count = 0;
			panel_limit_switch_false_count += 1;
		}
	}
	else
	{
		ROS_WARN_THROTTLE(1, "teleop : intake line break sensor not found in joint_states");
		panel_limit_switch_false_count += 1;
		panel_limit_switch_true_count = 0;
	}

	if (panel_push_extend_idx < joint_state.position.size())
	{
		panel_push_extend = (joint_state.position[panel_push_extend_idx] != 0);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Joystick_controller");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "teleop_params");
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

	max_speed = config.max_speed;
	max_rot = config.max_rot;

    navX_angle = M_PI / 2.;

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";

	BrakeSrv = n.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, service_connection_header);
	if(!BrakeSrv.waitForExistence(ros::Duration(15)))
	{
		ROS_ERROR("Wait (15 sec) timed out, for Brake Service in teleop_joystick_comp.cpp");
	}
	JoystickRobotVel = n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);
	elevator_setpoint = n.advertise<std_msgs::Int8>("elevator_setpoint",1);
	ros::Subscriber navX_heading = n.subscribe("navx_mxp", 1, &navXCallback);
	ros::Subscriber joint_states_sub = n.subscribe("/frcrobot_jetson/joint_states", 1, &jointStateCallback);

	//initialize actionlib clients
	intake_cargo_ac = std::make_shared<actionlib::SimpleActionClient<behaviors::IntakeAction>>("/cargo_intake/cargo_intake_server", true);
	outtake_cargo_ac = std::make_shared<actionlib::SimpleActionClient<behaviors::PlaceAction>>("/cargo_outtake/cargo_outtake_server", true);
	intake_hatch_panel_ac = std::make_shared<actionlib::SimpleActionClient<behaviors::IntakeAction>>("/hatch_intake/intake_hatch_panel_server", true);
	outtake_hatch_panel_ac = std::make_shared<actionlib::SimpleActionClient<behaviors::PlaceAction>>("/hatch_outtake/outtake_hatch_panel_server", true);
	climber_ac = std::make_shared<actionlib::SimpleActionClient<behaviors::ClimbAction>>("/climber/climber_server", true);
	align_hatch_ac = std::make_shared<actionlib::SimpleActionClient<behaviors::AlignAction>>("/align_hatch/align_hatch_server", true);
	align_cargo_ac = std::make_shared<actionlib::SimpleActionClient<behaviors::AlignAction>>("/align_cargo/align_cargo_server", true);
	elevator_ac = std::make_shared<actionlib::SimpleActionClient<behaviors::ElevatorAction>>("/elevator/elevator_server", true);

	run_align = n.serviceClient<std_srvs::SetBool>("/align_with_terabee/run_align");

	manual_server_panelIn = n.serviceClient<controllers_2019::PanelIntakeSrv>("/frcrobot_jetson/panel_intake_controller/panel_command");
	//manual_server_cargoOut = n.serviceClient<cargo_outtake_controller::CargoOuttakeSrv>("/cargo_outtake_controller/cargo_outtake_command");
	manual_server_cargoIn = n.serviceClient<controllers_2019::CargoIntakeSrv>("/cargo_intake_controller/cargo_intake_command");

    continue_outtake_client = n.serviceClient<std_srvs::Empty>("/hatch_outtake/continue_outtake_panel");

	cargo_pid = n.advertise<std_msgs::Bool>("/align_server/cargo_pid/pid_enable", 1);
	terabee_pid = n.advertise<std_msgs::Bool>("/align_server/align_with_terabee/enable_y_pub", 1);
	distance_pid = n.advertise<std_msgs::Bool>("/align_server/distance_pid/pid_enable", 1);
	navX_pid = n.advertise<std_msgs::Bool>("/align_server/navX_pid/pid_enable", 1);
	enable_align = n.advertise<std_msgs::Bool>("/align_server/align_pid/pid_enable", 1);

	ros::ServiceServer robot_orient_service = n.advertiseService("robot_orient", orientCallback);

	DynamicReconfigureWrapper<teleop_joystick_control::TeleopJoystickCompConfig> drw(n_params, config);

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
