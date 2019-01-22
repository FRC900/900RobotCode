#include <atomic>
#include <realtime_tools/realtime_buffer.h>
#include "teleop_joystick_control/teleop_joystick_comp.h"
#include "teleop_joystick_control/rate_limiter.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include <vector>

// TODO : make these parameters, possibly with dynamic reconfig
const double dead_zone = .2;
const double slow_mode = .33;
const double max_speed = 3.6;
const double max_rot = 8.8;
const double joystick_scale = 3;
const double rotation_scale = 4;

std::vector <frc_msgs::JoystickState> joystick_states_array;
std::vector <std::string> topic_array;
std::vector <ros::Subscriber> subscriber_array;

// 50 msec to go from full back to full forward
const double drive_rate_limit_time = 500.;
rate_limiter::RateLimiter left_stick_x_rate_limit(-1.0, 1.0, drive_rate_limit_time);
rate_limiter::RateLimiter left_stick_y_rate_limit(-1.0, 1.0, drive_rate_limit_time);
rate_limiter::RateLimiter right_stick_x_rate_limit(-1.0, 1.0, drive_rate_limit_time);
rate_limiter::RateLimiter right_stick_y_rate_limit(-1.0, 1.0, drive_rate_limit_time);
rate_limiter::RateLimiter left_trigger_rate_limit(-1.0, 1.0, drive_rate_limit_time);
rate_limiter::RateLimiter right_trigger_rate_limit(-1.0, 1.0, drive_rate_limit_time);

void dead_zone_check(double &val1, double &val2)
	if (fabs(val1) <= dead_zone && fabs(val2) <= dead_zone)
	{
		val1 = 0;
		val2 = 0;
	}
}

static ros::Publisher JoystickRobotVel;
static ros::ServiceClient BrakeSrv;
static ros::ServiceClient run_align;

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

	if(i == 1)
	{
		// TODO - experiment with rate-limiting after scaling instead?
		double leftStickX = left_stick_x_rate_limit.applyLimit(joystick_states_array[0].leftStickX);
		double leftStickY = left_stick_y_rate_limit.applyLimit(joystick_states_array[0].leftStickY);

		double rightStickX = right_stick_x_rate_limit.applyLimit(joystick_states_array[0].rightStickX);
		double rightStickY = right_stick_y_rate_limit.applyLimit(joystick_states_array[0].rightStickY);

		dead_zone_check(leftStickX, leftStickY);
		dead_zone_check(rightStickX, rightStickY);

		leftStickX =  pow(leftStickX, joystick_scale) * max_speed;
		leftStickY = -pow(leftStickY, joystick_scale) * max_speed;

		rightStickX =  pow(rightStickX, joystick_scale);
		rightStickY = -pow(rightStickY, joystick_scale);

		// TODO : dead-zone for rotation?
		// TODO : test rate limiting rotation rather than individual inputs, either pre or post scaling?
		double triggerLeft = left_trigger_rate_limit.applyLimit(joystick_states_array[0].leftTrigger);
		double triggerRight = right_trigger_rate_limit.applyLimit(joystick_states_array[0].rightTrigger);
		const double rotation = (pow(triggerLeft, rotation_scale) - pow(triggerRight, rotation_scale)) * max_rot;

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

		if(joystick_states_array[0].buttonARelease)
		{
			ROS_WARN_STREAM("buttonAPress");
			std_srvs::SetBool msg;
			msg.request.data = true;
			run_align.call(msg);
		}

		if(joystick_states_array[0].buttonAPress)
		{
			ROS_WARN_STREAM("buttonARelease");
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

	navX_angle = M_PI / 2;

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
	run_align = n.serviceClient<std_srvs::SetBool>("run_align");

	ROS_WARN("joy_init");

	ros::spin();
	return 0;
}
