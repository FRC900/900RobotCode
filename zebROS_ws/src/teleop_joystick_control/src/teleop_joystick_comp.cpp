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
#include <vector>
#include "teleop_joystick_control/RobotOrient.h"

double joystick_deadzone;
double slow_mode;
double max_speed;
double max_rot;
double joystick_pow;
double rotation_pow;
int linebreak_debounce_iterations;

int elevator_cur_setpoint_idx;
int climber_cur_step;

int cargo_linebreak_true_count = 0;
int panel_linebreak_true_count = 0;
int cargo_linebreak_false_count = 0;
int panel_linebreak_false_count = 0;


const int climber_num_steps = 3;
const int elevator_num_setpoints = 4;

bool robot_orient = false;
double offset_angle = 0;

std::vector <frc_msgs::JoystickState> joystick_states_array;
std::vector <std::string> topic_array;


// 500 msec to go from full back to full forward
const double drive_rate_limit_time = 500.;
rate_limiter::RateLimiter left_stick_x_rate_limit(-1.0, 1.0, drive_rate_limit_time);
rate_limiter::RateLimiter left_stick_y_rate_limit(-1.0, 1.0, drive_rate_limit_time);
rate_limiter::RateLimiter right_stick_x_rate_limit(-1.0, 1.0, drive_rate_limit_time);
rate_limiter::RateLimiter right_stick_y_rate_limit(-1.0, 1.0, drive_rate_limit_time);
rate_limiter::RateLimiter left_trigger_rate_limit(-1.0, 1.0, drive_rate_limit_time);
rate_limiter::RateLimiter right_trigger_rate_limit(-1.0, 1.0, drive_rate_limit_time);

ros::Publisher elevator_setpoint;
ros::Publisher JoystickRobotVel;
ros::Publisher cargo_pid;
ros::Publisher terabee_pid;
ros::Publisher distance_pid;
ros::Publisher navX_pid;
ros::Publisher enable_align;

ros::ServiceClient BrakeSrv;
ros::ServiceClient run_align;
ros::ServiceClient align_with_terabee;
//use shared pointers to make the clients global
std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeAction>> intake_cargo_ac;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::PlaceAction>> outtake_cargo_ac;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::IntakeAction>> intake_hatch_panel_ac;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::PlaceAction>> outtake_hatch_panel_ac;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::ElevatorAction>> elevator_ac;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::ClimbAction>> climber_ac;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::AlignAction>> align_ac;
double navX_angle;

struct ElevatorGoal
{
	ElevatorGoal():
		index_(0)
	{
	}
	ElevatorGoal(double index):
		index_(index)
	{
	}
	double index_;

};

realtime_tools::RealtimeBuffer<ElevatorGoal> ElevatorGoal;


void dead_zone_check(double &val1, double &val2)
{
	if (fabs(val1) <= joystick_deadzone && fabs(val2) <= joystick_deadzone)
	{
		val1 = 0;
		val2 = 0;
	}
}

// Use a realtime buffer to store the odom callback data
// The main teleop code isn't technically realtime but we
// want it to be the fast part of the code, so for now
// pretend that is the realtime side of the code
/*realtime_tools::RealtimeBuffer<ElevatorPos> elevatorPos;
  realtime_tools::RealtimeBuffer<CubeState> cubeState;
  realtime_tools::RealtimeBuffer<ElevatorPos> elevatorCmd;*/
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
	align_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
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
	int i = 0;

	const ros::M_string &header = event.getConnectionHeader();

	std::string topic = header.at("topic");

	//Identifies the incoming message as the correct joystick based on the topic the message was recieved from
	for(bool msg_assign = false; msg_assign == false; i++)
	{
		if(topic == topic_array[i])
		{
			joystick_states_array[i] = *(event.getMessage());
			msg_assign = true;
		}
	}


    //Publish elevator setpoinut
    std_msgs::Int8 elevator_setpoint_msg;
    elevator_setpoint_msg.data = elevator_cur_setpoint_idx;
    elevator_setpoint.publish(elevator_setpoint_msg);

	//Only do this for the first joystick
	if(i == 1)
	{
		// TODO - experiment with rate-limiting after scaling instead?
		double leftStickX = joystick_states_array[0].leftStickX;
		double leftStickY = joystick_states_array[0].leftStickY;

		dead_zone_check(leftStickX, leftStickY);

		leftStickX = left_stick_x_rate_limit.applyLimit(leftStickX);
		leftStickY = left_stick_y_rate_limit.applyLimit(leftStickY);

		leftStickX =  pow(leftStickX, joystick_pow) * max_speed;
		leftStickY =  pow(leftStickY, joystick_pow) * max_speed;

		copysign(leftStickX, joystick_states_array[0].leftStickX);
		copysign(leftStickY, -joystick_states_array[0].leftStickY);

		double rightStickX = joystick_states_array[0].rightStickX;
		double rightStickY = joystick_states_array[0].rightStickY;

		dead_zone_check(rightStickX, rightStickY);

		rightStickX =  pow(rightStickX, joystick_pow);
		rightStickY = -pow(rightStickY, joystick_pow);

		copysign(rightStickX, joystick_states_array[0].rightStickX);
		copysign(rightStickY, -joystick_states_array[0].rightStickY);

		rightStickX = right_stick_x_rate_limit.applyLimit(rightStickX);
		rightStickY = right_stick_y_rate_limit.applyLimit(rightStickY);

		// TODO : dead-zone for rotation?
		// TODO : test rate limiting rotation rather than individual inputs, either pre or post scaling?
		//double triggerLeft = left_trigger_rate_limit.applyLimit(joystick_states_array[0].leftTrigger);
		//double triggerRight = right_trigger_rate_limit.applyLimit(joystick_states_array[0].rightTrigger);
		double rotation = pow(rightStickX, rotation_pow) * max_rot;
		copysign(rotation, rightStickX);

		static bool sendRobotZero = false;
		if (leftStickX == 0.0 && leftStickY == 0.0 && rotation == 0.0)
		{
			if (!sendRobotZero)
			{
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
			joyVector[0] = -leftStickX; //intentionally flipped
			joyVector[1] = -leftStickY;

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
			if(cargo_linebreak_true_count > linebreak_debounce_iterations) {
				goal.has_cargo = true;
			}
			else {
				goal.has_cargo = false;
			}
			align_ac->sendGoal(goal);
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
			//align_ac->sendGoal(goal);
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
			/*
			preemptActionlibServers();
			behaviors::AlignGoal goal;
			goal.has_cargo = false;
			align_ac->sendGoal(goal);
			*/
		}
			/*
		ROS_INFO_STREAM("Joystick1: buttonBPress - Cargo Outtake");
		preemptActionlibServers();
		behaviors::PlaceGoal goal;
		goal.setpoint_index = CARGO_SHIP;
		outtake_cargo_ac->sendGoal(goal);
		}
		if(joystick_states_array[0].buttonBButton)
		{
		ROS_INFO_THROTTLE(1, "buttonBButton");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
		}
		if(joystick_states_array[0].buttonBRelease)
		{
		ROS_INFO_STREAM("Joystick1: buttonBRelease");
		std_srvs::SetBool msg;
		msg.request.data = false;
		run_align.call(msg);
		}
		*/

		//Joystick1: buttonX
		if(joystick_states_array[0].buttonXPress)
		{
			//Determines where elevator will go when called to outtake or move to a setpoint
		//	ROS_INFO_STREAM("Joystick1: buttonXPress - Increment Elevator");
		//	elevator_cur_setpoint_idx = (elevator_cur_setpoint_idx + 1) % elevator_num_setpoints;
   		//	ROS_WARN("elevator current setpoint index %d", elevator_cur_setpoint_idx);
            
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
		/*if(joystick_states_array[0].buttonYPress)
		  {
		  ROS_INFO_STREAM("Joystick1: buttonYPress - Panel Outtake");
		  preemptActionlibServers();
		  behaviors::PlaceGoal goal;
		  goal.setpoint_index = CARGO_SHIP;
		  outtake_hatch_panel_ac->sendGoal(goal);
		  }
		  */
		  if(joystick_states_array[0].buttonYButton)
		  {
			  /*
			  ROS_INFO_THROTTLE(1, "buttonYButton");
			  std_msgs::Bool enable_pid;
			  enable_pid.data = true;
			  navX_pid.publish(enable_pid);
			  enable_align.publish(enable_pid);
			  */
		  }
		  if(joystick_states_array[0].buttonYRelease)
		  {
			  /*
			  ROS_INFO_STREAM("Joystick1: buttonYRelease");
			  std_msgs::Bool enable_pid;
			  enable_pid.data = false;
			  navX_pid.publish(enable_pid);
			  enable_align.publish(enable_pid);
			  */
		  }

		//Joystick1: bumperLeft
		if(joystick_states_array[0].bumperLeftPress)
		{
			ROS_INFO_STREAM("Joystick1: bumperLeftPress");
			preemptActionlibServers();
			if(cargo_linebreak_true_count > linebreak_debounce_iterations)
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
		if(joystick_states_array[0].bumperLeftButton)
		{
			ROS_INFO_THROTTLE(1, "bumperLeftButton");
		}
		if(joystick_states_array[0].bumperLeftRelease)
		{
			ROS_INFO_STREAM("Joystick1: bumperLeftRelease");
		}
		//Joystick1: bumperRight
		if(joystick_states_array[0].bumperRightPress)
		{
			ROS_INFO_STREAM("Joystick1: bumperRightPress");
			preemptActionlibServers();
			if(panel_linebreak_true_count > linebreak_debounce_iterations)
			{
				//If we have a panel, outtake it
				ROS_INFO_STREAM("Joystick1: Place Panel");
				behaviors::PlaceGoal goal;
				goal.setpoint_index = elevator_cur_setpoint_idx;
				outtake_hatch_panel_ac->sendGoal(goal);
				elevator_cur_setpoint_idx = 0;
				ROS_WARN("elevator current setpoint index %d", elevator_cur_setpoint_idx);
			}
			else
			{
				//If we don't have a panel, intake one
				ROS_INFO_STREAM("Joystick1: Intake Panel");
				behaviors::IntakeGoal goal;
				intake_hatch_panel_ac->sendGoal(goal);

			}
		}
		if(joystick_states_array[0].bumperRightButton)
		{
			ROS_INFO_THROTTLE(1, "bumperRightButton");
		}
		if(joystick_states_array[0].bumperRightRelease)
		{
			ROS_INFO_STREAM("Joystick1: bumperRightRelease");
		}
		//Joystick1: directionLeft
		if(joystick_states_array[0].directionLeftPress)
		{
			//Move the elevator to the current setpoint
			ROS_WARN("Calling elevator server; move to setpoint %d", elevator_cur_setpoint_idx);
			preemptActionlibServers();
			behaviors::ElevatorGoal goal;
			goal.setpoint_index = elevator_cur_setpoint_idx;
			if(cargo_linebreak_true_count > linebreak_debounce_iterations)
			{
				goal.place_cargo = true;
			}
			else
			{
				goal.place_cargo = false;
			}
			goal.raise_intake_after_success  = true;
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
			climber_ac->cancelGoalsAtAndBeforeTime(ros::Time::now());
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

	else if(i == 2)
	{
		//Joystick2: buttonA
		/*if(joystick_states_array[1].buttonAPress)
		  {
		  ROS_INFO_STREAM("Joystick2: buttonAPress");
		  std_srvs::SetBool msg;
		  msg.request.data = true;
		  run_align.call(msg);
		  }
		  if(joystick_states_array[1].buttonAButton)
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
		  }
		//Joystick2: buttonB
		if(joystick_states_array[1].buttonBPress)
		{
		ROS_INFO_STREAM("Joystick2: buttonBPress");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
		}
		if(joystick_states_array[1].buttonBButton)
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
		}
		//Joystick2: buttonX
		if(joystick_states_array[1].buttonXPress)
		{
		ROS_INFO_STREAM("Joystick2: buttonXPress");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
		}
		if(joystick_states_array[1].buttonXButton)
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
		}
		//Joystick2: buttonY
		if(joystick_states_array[1].buttonYPress)
		{
		ROS_INFO_STREAM("Joystick2: buttonYPress");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
	}
	if(joystick_states_array[1].buttonYButton)
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

}

void jointStateCallback(const sensor_msgs::JointState &joint_state)
{
	//get index of linebreak sensor for this actionlib server
	static size_t cargo_linebreak_idx = std::numeric_limits<size_t>::max();
	static size_t panel_linebreak_1_idx = std::numeric_limits<size_t>::max();
	static size_t panel_linebreak_2_idx = std::numeric_limits<size_t>::max();
	if (cargo_linebreak_idx >= joint_state.name.size() || panel_linebreak_1_idx >= joint_state.name.size() || panel_linebreak_2_idx >= joint_state.name.size())
	{
		for (size_t i = 0; i < joint_state.name.size(); i++)
		{
			if (joint_state.name[i] == "cargo_intake_linebreak_1")
				cargo_linebreak_idx = i;
			if (joint_state.name[i] == "panel_intake_linebreak_1")
				panel_linebreak_1_idx = i;
			if (joint_state.name[i] == "panel_intake_linebreak_2")
				panel_linebreak_2_idx = i;
		}
	}

	//update linebreak counts based on the value of the linebreak sensor
	if (cargo_linebreak_idx < joint_state.position.size())
	{
		bool cargo_linebreak_true = (joint_state.position[cargo_linebreak_idx] != 0);
		if(cargo_linebreak_true)
		{
			cargo_linebreak_true_count += 1;
			cargo_linebreak_false_count = 0;
		}
		else
		{
			cargo_linebreak_true_count = 0;
			cargo_linebreak_false_count += 1;
		}
	}
	else
	{
		ROS_WARN_THROTTLE(1, "outtake line break sensor not found in joint_states");
		cargo_linebreak_false_count = 0;
		cargo_linebreak_true_count = 0;
	}

	if (panel_linebreak_1_idx < joint_state.position.size() && panel_linebreak_2_idx < joint_state.position.size())
	{
		bool panel_linebreak_true = ((joint_state.position[panel_linebreak_1_idx] != 0) || (joint_state.position[panel_linebreak_2_idx] != 0));
		if(panel_linebreak_true)
		{
			panel_linebreak_true_count += 1;
			panel_linebreak_false_count = 0;
		}
		else
		{
			panel_linebreak_true_count = 0;
			panel_linebreak_false_count += 1;
		}
	}
	else
	{
		ROS_WARN_THROTTLE(1, "intake line break sensor not found in joint_states");
		panel_linebreak_false_count += 1;
		panel_linebreak_true_count = 0;
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
	if(!n_params.getParam("joystick_deadzone", joystick_deadzone))
	{
		ROS_ERROR("Could not read joystick_deadzone in teleop_joystick_comp");
	}
	if(!n_params.getParam("slow_mode", slow_mode))
	{
		ROS_ERROR("Could not read slow_mode in teleop_joystick_comp");
	}
	if(!n_params.getParam("joystick_pow", joystick_pow))
	{
		ROS_ERROR("Could not read joystick_pow in teleop_joystick_comp");
	}
	if(!n_params.getParam("rotation_pow", rotation_pow))
	{
		ROS_ERROR("Could not read rotation_pow in teleop_joystick_comp");
	}
	if(!n_params.getParam("linebreak_debounce_iterations", linebreak_debounce_iterations))
	{
		ROS_ERROR("Could not read linebreak_debounce_iterations in teleop_joystick_comp");
	}
	if(!n_swerve_params.getParam("max_speed", max_speed))
	{
		ROS_ERROR("Could not read max_speed in teleop_joystick_comp");
	}
	if(!n_params.getParam("max_rot", max_rot))
	{
		ROS_ERROR("Could not read max_rot in teleop_joystick_comp");
	}

	std::vector <ros::Subscriber> subscriber_array;
    navX_angle = M_PI / 2;

	//Read from _num_joysticks_ joysticks
	for(int j = 0; j < num_joysticks; j++)
	{
		std::stringstream s;
		s << "/teleop/translator";
		s << j;
		s << "/joystick_states";
		topic_array.push_back(s.str());
		subscriber_array.push_back(n.subscribe(topic_array[j], 1, &evaluateCommands));
	}


	joystick_states_array.resize(topic_array.size());

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";


	BrakeSrv = n.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, service_connection_header);
	if(!BrakeSrv.waitForExistence(ros::Duration(15)))
	{
		ROS_ERROR("Wait (15 sec) timed out, for Brake Service in teleop_joystick_comp.cpp");
	}
	JoystickRobotVel = n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);
	elevator_setpoint = n.advertise<std_msgs::Int8>("elevator_setpoint",1);
	ros::Subscriber navX_heading  = n.subscribe("navx_mxp", 1, &navXCallback);
	ros::Subscriber joint_states_sub = n.subscribe("/frcrobot_jetson/joint_states", 1, &jointStateCallback);

	//initialize actionlib clients
	intake_cargo_ac = std::make_shared<actionlib::SimpleActionClient<behaviors::IntakeAction>>("/cargo_intake/cargo_intake_server", true);
	outtake_cargo_ac = std::make_shared<actionlib::SimpleActionClient<behaviors::PlaceAction>>("/cargo_outtake/cargo_outtake_server", true);
	intake_hatch_panel_ac = std::make_shared<actionlib::SimpleActionClient<behaviors::IntakeAction>>("/hatch_intake/intake_hatch_panel_server", true);
	outtake_hatch_panel_ac = std::make_shared<actionlib::SimpleActionClient<behaviors::PlaceAction>>("/hatch_outtake/outtake_hatch_panel_server", true);
	climber_ac = std::make_shared<actionlib::SimpleActionClient<behaviors::ClimbAction>>("/climber/climber_server", true);
	align_ac = std::make_shared<actionlib::SimpleActionClient<behaviors::AlignAction>>("/align_server/align_server", true);
	elevator_ac = std::make_shared<actionlib::SimpleActionClient<behaviors::ElevatorAction>>("/elevator/elevator_server", true);


	run_align = n.serviceClient<std_srvs::SetBool>("/align_with_terabee/run_align");

	cargo_pid = n.advertise<std_msgs::Bool>("/align_server/cargo_pid/pid_enable", 1);
	terabee_pid = n.advertise<std_msgs::Bool>("/align_server/align_with_terabee/enable_y_pub", 1);
	distance_pid = n.advertise<std_msgs::Bool>("/align_server/distance_pid/pid_enable", 1);
	navX_pid = n.advertise<std_msgs::Bool>("/align_server/navX_pid/pid_enable", 1);
	enable_align = n.advertise<std_msgs::Bool>("/align_server/align_pid/pid_enable", 1);

	ros::ServiceServer robot_orient_service = n.advertiseService("robot_orient", orientCallback);

	ROS_WARN("joy_init");

	ros::spin();
	return 0;
}
