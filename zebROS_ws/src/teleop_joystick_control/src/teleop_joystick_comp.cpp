#include <atomic>
#include <realtime_tools/realtime_buffer.h>
#include "teleop_joystick_control/teleop_joystick_comp.h"
#include "std_srvs/Empty.h"

// TODO : make these parameters, possibly with dynamic reconfig
const double dead_zone = .2;
const double slow_mode = .33;
const double max_speed = 3.6;
const double max_rot = 8.8;
const double joystick_scale = 3;
const double rotation_scale = 4;

void dead_zone_check(double &val1, double &val2)
{
	if (fabs(val1) <= dead_zone && fabs(val2) <= dead_zone)
	{
		val1 = 0;
		val2 = 0;
	}
}

static ros::Publisher JoystickRobotVel;
static ros::ServiceClient BrakeSrv;

std::atomic<double> navX_angle;

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

void combineJoysticks(const ros::MessageEvent<ros_control_boilerplate::JoystickState const>& event)
{
	const ros::M_string &header = event.getConnectionHeader();
	std::string topic = header.at("topic");

	realtime_tools::RealtimeBuffer<struct> joystick_state_struct;

	const ros_control_boilerplate::JoystickState::ConstPtr &JoystickState = event.getMessage();

	if(topic == "frcrobot_jetson/joystick_states") //TODO make more generalized (read number in topic)
	{
		joystick_array[0] = *JoystickState;
	}

	else if(topic == "frcrobot_jetson/joystick_states1")
	{
		joystick_array[1] = *JoystickState;
	}
}

void evaluateCommands()
{
	while(ros::ok)
	{
		double leftStickX = JoystickState->leftStickX;
		double leftStickY = JoystickState->leftStickY;

		double rightStickX = JoystickState->rightStickX;
		double rightStickY = JoystickState->rightStickY;

		dead_zone_check(leftStickX, leftStickY);
		dead_zone_check(rightStickX, rightStickY);

		leftStickX =  pow(leftStickX, joystick_scale) * max_speed;
		leftStickY = -pow(leftStickY, joystick_scale) * max_speed;

		rightStickX =  pow(rightStickX, joystick_scale);
		rightStickY = -pow(rightStickY, joystick_scale);

		const double rotation = (pow(JoystickState->leftTrigger, rotation_scale) - pow(JoystickState->rightTrigger, rotation_scale)) * max_rot;

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
			//Publish drivetrain messages and elevator/pivot
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
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Joystick_controller");
	ros::NodeHandle n;

	navX_angle = M_PI / 2;

	ros::Subscriber joystick_sub  = n.subscribe("frcrobot_jetson/joystick_states", 1, &combineJoysticks);
	ros::Subscriber joystick_sub1  = n.subscribe("frcrobot_jetson/joystick_states1", 1, &combineJoysticks);

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";
	BrakeSrv = n.serviceClient<std_srvs::Empty>("swerve_drive_controller/brake", false, service_connection_header);
	JoystickRobotVel = n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);
	ros::Subscriber navX_heading  = n.subscribe("navx_mxp", 1, &navXCallback);

	ROS_WARN("joy_init");

	ros::spin();
	return 0;
}
