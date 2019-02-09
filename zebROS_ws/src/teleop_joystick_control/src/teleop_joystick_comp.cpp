#include <atomic>
#include <realtime_tools/realtime_buffer.h>
#include "teleop_joystick_control/teleop_joystick_comp.h"
#include "teleop_joystick_control/rate_limiter.h"
#include "std_srvs/Empty.h"
#include "std_srvs/Trigger.h"

#include "std_srvs/SetBool.h"

#include "std_srvs/Trigger.h"


// TODO : make these parameters, possibly with dynamic reconfig
const double dead_zone = .2;
const double slow_mode = .33;
const double max_speed = 3.6;
const double max_rot = 8.8;
const double joystick_scale = 3;
const double rotation_scale = 4;

// 50 msec to go from full back to full forward
const double drive_rate_limit_time = 500.;
rate_limiter::RateLimiter left_stick_x_rate_limit(-1.0, 1.0, drive_rate_limit_time);
rate_limiter::RateLimiter left_stick_y_rate_limit(-1.0, 1.0, drive_rate_limit_time);
rate_limiter::RateLimiter right_stick_x_rate_limit(-1.0, 1.0, drive_rate_limit_time);
rate_limiter::RateLimiter right_stick_y_rate_limit(-1.0, 1.0, drive_rate_limit_time);
rate_limiter::RateLimiter left_trigger_rate_limit(-1.0, 1.0, drive_rate_limit_time);
rate_limiter::RateLimiter right_trigger_rate_limit(-1.0, 1.0, drive_rate_limit_time);

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

static ros::ServiceClient cli;

void evaluateCommands(const frc_msgs::JoystickState::ConstPtr &JoystickState)
{
	// TODO - experiment with rate-limiting after scaling instead?
	double leftStickX = JoystickState->leftStickX;
	double leftStickY = JoystickState->leftStickY;



	double rightStickX = JoystickState->rightStickX;
	double rightStickY = JoystickState->rightStickY;
	ROS_INFO("JoyRecieve");
	if (JoystickState->buttonAButton) {
		ROS_INFO("A BUTTON RECIEVED");
	}
	bool aButton = JoystickState->buttonAPress;
	if (aButton) {
		ROS_INFO("PRESSED THE BUTTON");
		std_srvs::Trigger trg;
		if (cli.call(trg)) {
			ROS_INFO("SERVICE CALLED");
		} else {
			ROS_ERROR("LOL NOPE THAT SERVER CANT BE FOUND");
		}

	}





	dead_zone_check(leftStickX, leftStickY);

	leftStickX = left_stick_x_rate_limit.applyLimit(leftStickX);
	leftStickY = left_stick_y_rate_limit.applyLimit(leftStickY);

	leftStickX =  pow(leftStickX, joystick_scale) * max_speed;
	leftStickY = -pow(leftStickY, joystick_scale) * max_speed;

	double rightStickX = JoystickState->rightStickX;
	double rightStickY = JoystickState->rightStickY;

	dead_zone_check(rightStickX, rightStickY);

	rightStickX =  pow(rightStickX, joystick_scale);
	rightStickY = -pow(rightStickY, joystick_scale);

	rightStickX = right_stick_x_rate_limit.applyLimit(rightStickX);
	rightStickY = right_stick_y_rate_limit.applyLimit(rightStickY);

	// TODO : dead-zone for rotation?
	// TODO : test rate limiting rotation rather than individual inputs, either pre or post scaling?
	double triggerLeft = left_trigger_rate_limit.applyLimit(JoystickState->leftTrigger);
	double triggerRight = right_trigger_rate_limit.applyLimit(JoystickState->rightTrigger);
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

	if(JoystickState->buttonAPress)
	{
		ROS_WARN_STREAM("buttonAPress");
		std_srvs::SetBool msg;
		msg.request.data = true;
		run_align.call(msg);
	}

	if(JoystickState->buttonARelease)
	{
		ROS_WARN_STREAM("buttonARelease");
		std_srvs::SetBool msg;
		msg.request.data = false;
		run_align.call(msg);
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Joystick_controller");
	ros::NodeHandle n;

	navX_angle = M_PI / 2;

	ros::Subscriber joystick_sub  = n.subscribe("joystick_states", 1, &evaluateCommands);

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = "1";
	BrakeSrv = n.serviceClient<std_srvs::Empty>("swerve_drive_controller/brake", false, service_connection_header);
	cli = n.serviceClient<std_srvs::Trigger>("/frcrobot_jetson/align_service");
	JoystickRobotVel = n.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);


	ros::Subscriber navX_heading  = n.subscribe("navx_mxp", 1, &navXCallback);
	run_align = n.serviceClient<std_srvs::SetBool>("run_align");

	ROS_WARN("joy_init");

	ros::spin();
	return 0;
}
