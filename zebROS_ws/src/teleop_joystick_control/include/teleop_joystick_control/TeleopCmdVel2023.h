#ifndef TELEOP_CMD_VEL_2023_H
#define TELEOP_CMD_VEL_2023_H

#include <optional>
#include "ros/console.h"
#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include "teleop_joystick_control/rate_limiter.h"
#include "teleop_joystick_control/TeleopJoystickCompConfig.h"
#include "teleop_joystick_control/store_xy.h"
#include "teleop_joystick_control/interpolating_map.h"

struct StrafeSpeeds
{
	double x_;
	double y_;
	StrafeSpeeds(const double x, const double y)
		: x_{x}
		, y_{y}
	{
	}
};

struct MovementCaps {
	MovementCaps(double speed_cap, double rotation_cap) {
		speed_cap_ = speed_cap;
		rotation_cap_ = rotation_cap;
	}
	double speed_cap_{};
	double rotation_cap_{};
};

template <class ConfigT>
class TeleopCmdVel
{
	public:

		explicit TeleopCmdVel(const ConfigT &config):
			x_rate_limit_(-config.max_speed, config.max_speed, config.drive_rate_limit_time),
			y_rate_limit_(-config.max_speed, config.max_speed, config.drive_rate_limit_time),
			rotation_rate_limit_(-config.max_rot, config.max_rot, config.rotate_rate_limit_time)
		{
			joystick_values_.insert(atan2(-0.5, -1.0), store_xy(-1.0, -0.5));
			joystick_values_.insert(atan2(0.5, -1.0), store_xy(-1.0, 0.5));
			joystick_values_.insert(atan2(-0.5, 1.0), store_xy(1.0, -0.5));
			joystick_values_.insert(atan2(0.5, 1.0), store_xy(1.0, 0.5));
			joystick_values_.insert(atan2(-1.0, -0.5), store_xy(-0.5, -1.0));
			joystick_values_.insert(atan2(1.0, -0.5), store_xy(-0.5, 1.0));
			joystick_values_.insert(atan2(-1.0, 0.5), store_xy(0.5, -1.0));
			joystick_values_.insert(atan2(1.0, 0.5), store_xy(0.5, 1.0));
			joystick_values_.insert(M_PI, store_xy(-1.0, 0.0));
			joystick_values_.insert(-M_PI, store_xy(-1.0, 0.0));
		}

		void setRobotOrient(const bool robot_orient, const double offset_angle, const bool save = false)
		{
			robot_orient_ = robot_orient;
			offset_angle_ = offset_angle;
			if (save)
			{
				saved_robot_orient_ = robot_orient_;
				saved_offset_angle_ = offset_angle_;
			}
		}

		void setCaps(const double speed_cap, const double rotation_cap)
		{
			movement_caps_ = MovementCaps(speed_cap, rotation_cap);
		}

		void resetCaps(void)
		{
			movement_caps_ = std::nullopt;
		}

		void restoreRobotOrient(void)
		{
			robot_orient_ = saved_robot_orient_;
			offset_angle_ = saved_offset_angle_;
		}

		// Note - updateRiseTimeInMsec() does nothing if the
		// requested time is the same as the current config
		void updateRateLimit(const ConfigT &config)	
		{
			x_rate_limit_.updateRiseTimeInMsec(config.drive_rate_limit_time);
			y_rate_limit_.updateRiseTimeInMsec(config.drive_rate_limit_time);
			rotation_rate_limit_.updateRiseTimeInMsec(config.rotate_rate_limit_time);
		}
		
		// want normal rotation in teleop
		geometry_msgs::Twist generateCmdVel(const frc_msgs::JoystickState &event, const double &navX_angle, const ConfigT &config)
		{
			double max_speed;
			double max_rot;

			if (movement_caps_) {
				max_speed = movement_caps_->speed_cap_;
				max_rot = movement_caps_->rotation_cap_;
			} else {
				max_speed = config.max_speed;
				max_rot = config.max_rot;
			}

			x_rate_limit_.updateMinMax(-max_speed, max_speed);
			y_rate_limit_.updateMinMax(-max_speed, max_speed);
			rotation_rate_limit_.updateMinMax(-max_rot, max_rot);

			// Raw joystick values for X & Y translation
			const double leftStickX = event.leftStickX;
			const double leftStickY = event.leftStickY;
			//ROS_INFO_STREAM(__LINE__ << " x:"  << leftStickX << " y:" << leftStickY);

			// Convert to polar coordinates
			double direction = atan2(leftStickY, leftStickX);
			if(fabs(angles::shortest_angular_distance(direction,0)) < config.radial_deadzone) {
				direction = 0;
			}
			else if(fabs(angles::shortest_angular_distance(direction,M_PI/2)) < config.radial_deadzone) {
				direction = M_PI/2;
			}
			else if(fabs(angles::shortest_angular_distance(direction,M_PI)) < config.radial_deadzone) {
				direction = M_PI;
			}
			else if(fabs(angles::shortest_angular_distance(direction,3*M_PI/2)) < config.radial_deadzone) {
				direction = 3*M_PI/2;
			}
			//ROS_INFO_STREAM(__LINE__ << " direction:"  << direction);

			// Do a dead zone check on the magnitude of the velocity,
			// then scale it by a power function to increase resolution
			// of low-speed inputs. This will give an output range from
			// 0-100%
			// Finally, scale it so that 0% corresponds to the minimum
			// output needed to move the robot and 100% to the max
			// configured speed
			double magnitude = dead_zone_check(hypot(leftStickX, leftStickY), config.joystick_deadzone);
			//ROS_INFO_STREAM(__LINE__ << " magnitude:"  << magnitude);
			magnitude /= joystick_values_[direction].hypot();
			//ROS_INFO_STREAM(__LINE__ << " magnitude:"  << magnitude);
			if (magnitude != 0)
			{
				magnitude = pow(magnitude, config.joystick_pow);
				//ROS_INFO_STREAM(__LINE__ << " magnitude:"  << magnitude << " direction:" << direction);
				magnitude *= max_speed - config.min_speed;
				//ROS_INFO_STREAM(__LINE__ << " magnitude:"  << magnitude);
				magnitude += config.min_speed;
				//ROS_INFO_STREAM(__LINE__ << " magnitude:"  << magnitude);
			}

			// This applies a ramp to the output - it limits the amount of change per
			// unit time that's allowed.  This helps limit extreme current draw in
			// cases where the robot is changing direction rapidly.

			// Convert back to rectangular coordinates for the x and Y velocity
			const double xSpeed = x_rate_limit_.applyLimit(magnitude * cos(direction), event.header.stamp);
			const double ySpeed = y_rate_limit_.applyLimit(magnitude * sin(direction), event.header.stamp);
			//ROS_INFO_STREAM(__LINE__ << " "  << xSpeed << " " << ySpeed);

			// Rotation is a bit simpler since it is just one independent axis
		

#ifdef ROTATION_WITH_STICK
			const double rotAxisVal = event.rightStickX;
#else
			const double rotAxisVal = event.rightTrigger - event.leftTrigger;
#endif
			const double rotAxisDeadzoned = dead_zone_check(rotAxisVal, config.joystick_deadzone);

			// Scale the input by a power function to increase resolution
			// of the slower settings. Use copysign to preserve the sign
			// of the original input (keeps the direction correct)
			double rotation = pow(rotAxisDeadzoned, config.rotation_pow);
			rotation  = copysign(rotation, rotAxisVal);
			rotation *= max_rot;

			// Rate-limit changes in rotation
			rotation = rotation_rate_limit_.applyLimit(rotation, event.header.stamp);

			geometry_msgs::Twist vel;

			vel.linear.z = 0;
			vel.angular.x = 0;
			vel.angular.y = 0;

			//ROS_INFO_STREAM(__LINE__ << " xSpeed: " << xSpeed << " ySpeed: " << ySpeed);
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
				//ROS_INFO_STREAM(__LINE__ << " vel.linear.x: " << vel.linear.x << " vel.linear.y: " << vel.linear.y);

				vel.linear.x = rotatedJoyVector[1];
				vel.linear.y = rotatedJoyVector[0];
				vel.angular.z = -rotation;
			}

			return vel;
		}


		double generateAngleIncrement(const double rotationZ, const ros::Time &stamp, ConfigT &config)
		{
			double max_rot;
			if (movement_caps_) {
				max_rot = movement_caps_->rotation_cap_;
			}
			else {
				max_rot = config.max_rot;
			}
			//rot_rate_limit_.updateMinMax(-max_rot, max_rot);
			//rot_rate_limit_.updateRiseTimeInMsec(config.rotate_rate_limit_time);

			double rotation = dead_zone_check(rotationZ, config.joystick_deadzone);

			// Scale the input by a power function to increase resolution
			// of the slower settings. Use copysign to preserve the sign
			// of the original input (keeps the direction correct)
			rotation  = pow(rotation, config.rotation_pow);
			rotation  = copysign(rotation, rotationZ);
			rotation *= max_rot;
			//rot_rate_limit_.applyLimit(rotation, stamp);

			const ros::Duration timestep = stamp - last_stamp_;
			last_stamp_ = stamp;
			// Make sure the robot doesn't rotate if there's been
			// too much time elapsed since the last joystick value
			if (timestep.toSec() > 0.2) {
				ROS_WARN_STREAM("Timestep over 0.2");
				return 0;
			}
			ROS_INFO_STREAM_THROTTLE(1, "genAngleInc config.rotation_axis_scale " << config.rotation_axis_scale << " rotationZ " << rotationZ << " final rotation =  " << rotation*timestep.toSec()*config.rotation_axis_scale);
			return angles::normalize_angle(rotation*timestep.toSec());
		}

	private:
		bool robot_orient_{false};
		double offset_angle_{M_PI / 2.0};

		std::optional<MovementCaps> movement_caps_; 

		bool saved_robot_orient_{false};
		double saved_offset_angle_{M_PI / 2.0};

		rate_limiter::RateLimiter x_rate_limit_;
		rate_limiter::RateLimiter y_rate_limit_;
		rate_limiter::RateLimiter rotation_rate_limit_;

		ros::Time last_stamp_{ros::Time::now()};
		wpi::interpolating_map<double, store_xy> joystick_values_;

		double dead_zone_check(const double test_axis, const double dead_zone)
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

#endif 
