#include <atomic>
#include <realtime_tools/realtime_buffer.h>
#include "teleop_joystick_control/teleop_joystick_comp.h"
#include "std_srvs/Empty.h"

#include "behaviors/PlaceAction.h"
#include "behaviors/PlaceGoal.h"
#include "actionlib/client/simple_action_client.h"

#include "std_srvs/SetBool.h"
#include <vector>
#include <vector>

double joystick_deadzone;
double slow_mode;
double max_speed;
double max_rot;
double joystick_pow;
double rotation_pow;

std::vector <frc_msgs::JoystickState> joystick_states_array;
std::vector <std::string> topic_array;

ros::Publisher JoystickRobotVel;
ros::ServiceClient BrakeSrv;
ros::ServiceClient run_align;
std::shared_ptr<actionlib::SimpleActionClient<behaviors::PlaceAction>> outtake_cargo_ac;
std::atomic<double> navX_angle;


void deadzone_check(double &val1, double &val2)
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
        navX_angle.store(yaw, std::memory_order_relaxed);
}

void evaluateCommands(const ros::MessageEvent<frc_msgs::JoystickState const>& event)
{
	int i = 0;

	const ros::M_string &header = event.getConnectionHeader();

	std::string topic = header.at("topic");
	for(bool msg_assign = false; msg_assign == false; i++)
	{
		if(topic == topic_array[i])
		{
			joystick_states_array[i] = *(event.getMessage());
			msg_assign = true;
		}
	}
	
	//Only do this for the first joystick
	if(i == 1)
	{
		double leftStickX = joystick_states_array[0].leftStickX;
		double leftStickY = joystick_states_array[0].leftStickY;

		double rightStickX = joystick_states_array[0].rightStickX;
		double rightStickY = joystick_states_array[0].rightStickY;

		deadzone_check(leftStickX, leftStickY);
		deadzone_check(rightStickX, rightStickY);

		leftStickX =  pow(leftStickX, joystick_pow) * max_speed;
		leftStickY = -pow(leftStickY, joystick_pow) * max_speed;

		rightStickX =  pow(rightStickX, joystick_pow);
		rightStickY = -pow(rightStickY, joystick_pow);

		//rotate based off of triggers
		//const double rotation = (pow(joystick_states_array[0].leftTrigger, rotation_pow) - pow(joystick_states_array[0].rightTrigger, rotation_pow)) * max_rot;
		//
		//rotate based off of joysticks
		const double rotation = pow(rightStickX, rotation_pow) * max_rot;

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
			joyVector[0] = leftStickX; //intentionally flipped
			joyVector[1] = -leftStickY;
			const Eigen::Rotation2Dd r(-navX_angle.load(std::memory_order_relaxed) - M_PI / 2.);
			const Eigen::Vector2d rotatedJoyVector = r.toRotationMatrix() * joyVector;

			geometry_msgs::Twist vel;
			vel.linear.x = rotatedJoyVector[1];
			vel.linear.y = rotatedJoyVector[0];
			vel.linear.z = 0;

			vel.angular.x = 0;
			vel.angular.y = 0;
			vel.angular.z = rotation;

			JoystickRobotVel.publish(vel);
			sendRobotZero = false;
		}

		//buttonA
		if(joystick_states_array[0].buttonAPress)
		{
			ROS_INFO_STREAM("buttonAPress");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].buttonAButton)
		{
			ROS_INFO_THROTTLE(1, "buttonAButton");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].buttonARelease)
		{
			ROS_INFO_STREAM("buttonARelease");
			std_srvs::SetBool msg;
			msg.request.data = false;
			run_align.call(msg);
		}
		//buttonB
		if(joystick_states_array[0].buttonBPress)
		{
			ROS_INFO_STREAM("buttonBPress");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
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
			ROS_INFO_STREAM("buttonBRelease");
			std_srvs::SetBool msg;
			msg.request.data = false;
			run_align.call(msg);
		}
		//buttonX
		if(joystick_states_array[0].buttonXPress)
		{
			ROS_INFO_STREAM("buttonXPress");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].buttonXButton)
		{
			ROS_INFO_THROTTLE(1, "buttonXButton");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].buttonXRelease)
		{
			ROS_INFO_STREAM("buttonXRelease");
			std_srvs::SetBool msg;
			msg.request.data = false;
			run_align.call(msg);
		}
		//buttonY
		if(joystick_states_array[0].buttonYPress)
		{
			ROS_INFO_STREAM("buttonYPress");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].buttonYButton)
		{
			ROS_INFO_THROTTLE(1, "buttonYButton");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].buttonYRelease)
		{
			ROS_INFO_STREAM("buttonYRelease");
			std_srvs::SetBool msg;
			msg.request.data = false;
			run_align.call(msg);
		}
		//bumperLeft
		if(joystick_states_array[0].bumperLeftPress)
		{
			ROS_INFO_STREAM("bumperLeftPress");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].bumperLeftButton)
		{
			ROS_INFO_THROTTLE(1, "bumperLeftButton");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].bumperLeftRelease)
		{
			ROS_INFO_STREAM("bumperLeftRelease");
			std_srvs::SetBool msg;
			msg.request.data = false;
			run_align.call(msg);
		}
		//bumperRight
		if(joystick_states_array[0].bumperRightPress)
		{
			ROS_INFO_STREAM("bumperRightPress");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].bumperRightButton)
		{
			ROS_INFO_THROTTLE(1, "bumperRightButton");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].bumperRightRelease)
		{
			ROS_INFO_STREAM("bumperRightRelease");
			std_srvs::SetBool msg;
			msg.request.data = false;
			run_align.call(msg);
		}
		//directionLeft
		if(joystick_states_array[0].directionLeftPress)
		{
			ROS_INFO_STREAM("directionLeftPress");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].directionLeftButton)
		{
			ROS_INFO_THROTTLE(1, "directionLeftButton");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].directionLeftRelease)
		{
			ROS_INFO_STREAM("directionLeftRelease");
			std_srvs::SetBool msg;
			msg.request.data = false;
			run_align.call(msg);
		}
		//directionRight
		if(joystick_states_array[0].directionRightPress)
		{
			ROS_INFO_STREAM("directionRightPress");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].directionRightButton)
		{
			ROS_INFO_THROTTLE(1, "directionRightButton");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].directionRightRelease)
		{
			ROS_INFO_STREAM("directionRightRelease");
			std_srvs::SetBool msg;
			msg.request.data = false;
			run_align.call(msg);
		}
		//directionUp
		if(joystick_states_array[0].directionUpPress)
		{
			ROS_INFO_STREAM("directionUpPress");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].directionUpButton)
		{
			ROS_INFO_THROTTLE(1, "directionUpButton");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].directionUpRelease)
		{
			ROS_INFO_STREAM("directionUpRelease");
			std_srvs::SetBool msg;
			msg.request.data = false;
			run_align.call(msg);
		}
		//directionDown
		if(joystick_states_array[0].directionDownPress)
		{
			ROS_INFO_STREAM("directionDownPress");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].directionDownButton)
		{
			ROS_INFO_THROTTLE(1, "directionDownButton");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}
		if(joystick_states_array[0].directionDownRelease)
		{
			ROS_INFO_STREAM("directionDownRelease");
			std_srvs::SetBool msg;
			msg.request.data = false;
			run_align.call(msg);
		}
	}

	else if(i == 2)
	{
		// Stuff for the second joystick goes here
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Joystick_controller");
	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "teleop_params");

	if(!n_params.getParam("joystick_deadzone", joystick_deadzone))
	{
		ROS_ERROR("Could not read joystick_deadzone in teleop_joystick_comp");
	}
	if(!n_params.getParam("slow_mode", slow_mode))
	{
		ROS_ERROR("Could not read slow_mode in teleop_joystick_comp");
	}
	if(!n_params.getParam("max_speed", max_speed))
	{
		ROS_ERROR("Could not read max_speed in teleop_joystick_comp");
	}
	if(!n_params.getParam("max_rot", max_rot))
	{
		ROS_ERROR("Could not read max_rot in teleop_joystick_comp");
	}
	if(!n_params.getParam("joystick_pow", joystick_pow))
	{
		ROS_ERROR("Could not read joystick_pow in teleop_joystick_comp");
	}
	if(!n_params.getParam("rotation_pow", rotation_pow))
	{
		ROS_ERROR("Could not read rotation_pow in teleop_joystick_comp");
	}


	std::vector <ros::Subscriber> subscriber_array;

	navX_angle = M_PI / 2;

	//Read from two joysticks
	topic_array.push_back("/frcrobot_jetson/joystick_states");
	topic_array.push_back("/frcrobot_jetson/joystick_states1");
	for(size_t j = 0; (subscriber_array.size()) < (topic_array.size()); j++)
	{
		subscriber_array.push_back(n.subscribe((topic_array[j]), 1, &evaluateCommands));
	}

	joystick_states_array.resize(topic_array.size());

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";
	BrakeSrv = n.serviceClient<std_srvs::Empty>("swerve_drive_controller/brake", false, service_connection_header);
	JoystickRobotVel = n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);
	ros::Subscriber navX_heading  = n.subscribe("navx_mxp", 1, &navXCallback);
	outtake_cargo_ac = std::make_shared<actionlib::SimpleActionClient<behaviors::PlaceAction>>("cargo_outtake_server", true);
	run_align = n.serviceClient<std_srvs::SetBool>("run_align");

	ROS_WARN("joy_init");

	ros::spin();
	return 0;
}
