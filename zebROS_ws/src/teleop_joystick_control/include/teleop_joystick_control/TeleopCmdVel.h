#include <geometry_msgs/Twist.h>
#include "teleop_joystick_control/rate_limiter.h"
#include "teleop_joystick_control/TeleopJoystickCompConfig.h"

class TeleopCmdVel
{
	public:

		TeleopCmdVel(teleop_joystick_control::TeleopJoystickCompConfig config):
			x_rate_limit_(-max_speed_, max_speed_, config.drive_rate_limit_time),
			y_rate_limit_(-max_speed_, max_speed_, config.drive_rate_limit_time),
			rotation_rate_limit_(-max_rot_, max_rot_, config.drive_rate_limit_time)
		{
			config_ = config;

			max_speed_ = config_.max_speed;
			max_rot_ = config_.max_rot;

		}

		void setRobotOrient(const bool &robot_orient, const double &offset_angle)
		{
			robot_orient_ = robot_orient;
			offset_angle_ = offset_angle;
		}

		void setSlowMode(const bool &slow_mode)
		{
			if(slow_mode_ == slow_mode)
				return;

			max_speed_ = slow_mode ? config_.max_speed_slow : config_.max_speed;
			max_rot_ = slow_mode ? config_.max_rot_slow : config_.max_rot;

			x_rate_limit_.updateMinMax(-max_speed_, max_speed_);
			y_rate_limit_.updateMinMax(-max_speed_, max_speed_);
			rotation_rate_limit_.updateMinMax(-max_rot_, max_rot_);

			slow_mode_ = slow_mode;
		}

		geometry_msgs::Twist generateCmdVel(const frc_msgs::JoystickState &event, const double &navX_angle, const teleop_joystick_control::TeleopJoystickCompConfig &config)
		{
			config_ = config;

			// Raw joystick values for X & Y translation
			const double leftStickX = event.leftStickX;
			const double leftStickY = event.leftStickY;
			//ROS_INFO_STREAM(__LINE__ << " "  << leftStickX << " " << leftStickY);

			// Convert to polar coordinates
			double direction = atan2(leftStickY, leftStickX);

			// Do a dead zone check on the magnitude of the velocity,
			// then scale it by a power function to increase resolution
			// of low-speed inputs. This will give an output range from
			// 0-100%
			// Finally, scale it so that 0% corresponds to the minimum
			// output needed to move the robot and 100% to the max
			// config_ured speed
			double magnitude = dead_zone_check(hypot(leftStickX, leftStickY), config_.joystick_deadzone);
			//ROS_INFO_STREAM(__LINE__ << " "  << magnitude);
			if (magnitude != 0)
			{
				magnitude = pow(magnitude, config_.joystick_pow);
				//ROS_INFO_STREAM(__LINE__ << " "  << magnitude << " " << direction);
				magnitude *= max_speed_ - config_.min_speed;
				//ROS_INFO_STREAM(__LINE__ << " "  << magnitude);
				magnitude += config_.min_speed;
				//ROS_INFO_STREAM(__LINE__ << " "  << magnitude);
			}

			// This applies a ramp to the output - it limits the amount of change per
			// unit time that's allowed.  This helps limit extreme current draw in
			// cases where the robot is changing direction rapidly.

			// Convert back to rectangular coordinates for the x and Y velocity
			const double xSpeed = x_rate_limit_.applyLimit(magnitude * cos(direction), event.header.stamp);
			const double ySpeed = y_rate_limit_.applyLimit(magnitude * sin(direction), event.header.stamp);
			//ROS_INFO_STREAM(__LINE__ << " "  << xSpeed << " " << ySpeed);

			// Rotation is a bit simpler since it is just one independent axis
			const double rightStickX = dead_zone_check(event.rightStickX, config_.joystick_deadzone);

			// Scale the input by a power function to increase resolution
			// of the slower settings. Use copysign to preserve the sign
			// of the original input (keeps the direction correct)
			double rotation = pow(rightStickX, config_.rotation_pow);
			rotation  = copysign(rotation, event.rightStickX);
			rotation *= max_rot_;

			// Rate-limit changes in rotation
			rotation = rotation_rate_limit_.applyLimit(rotation, event.header.stamp);

			geometry_msgs::Twist vel;

			vel.linear.z = 0;
			vel.angular.x = 0;
			vel.angular.y = 0;

			if (xSpeed == 0.0 && ySpeed == 0.0 && rotation == 0.0)
			{
				vel.linear.x = 0;
				vel.linear.y = 0;
				vel.angular.z = 0;
			}
			else // X or Y or rotation != 0 so tell the drive base to move
			{
				//Publish drivetrain messages and call servers
				Eigen::Vector2d joyVector;
				joyVector[0] = -xSpeed; //intentionally flipped
				joyVector[1] = -ySpeed;

				const Eigen::Rotation2Dd rotate(robot_orient_ ? -offset_angle_ : -navX_angle);
				const Eigen::Vector2d rotatedJoyVector = rotate.toRotationMatrix() * joyVector;

				vel.linear.x = rotatedJoyVector[1];
				vel.linear.y = rotatedJoyVector[0];
				vel.angular.z = -rotation;
			}

			return vel;
		}

	private:

		bool robot_orient_ = false;
		bool slow_mode_ = false;

		double max_speed_;
		double max_rot_;
		double offset_angle_;

		rate_limiter::RateLimiter x_rate_limit_;
		rate_limiter::RateLimiter y_rate_limit_;
		rate_limiter::RateLimiter rotation_rate_limit_;

		teleop_joystick_control::TeleopJoystickCompConfig config_;

		double dead_zone_check(const double &test_axis, const double &dead_zone)
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
};
