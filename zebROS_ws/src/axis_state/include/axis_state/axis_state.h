#ifndef AXIS_STATE_INC
#define AXIS_STATE_INC

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "pid_velocity_msg/PIDVelocity.h"

class AlignActionAxisConfig
{
	public:
		AlignActionAxisConfig(const std::string &name,
							  const std::string &enable_pub_topic,
							  const std::string &command_pub_topic,
							  const std::string &state_pub_topic,
							  const std::string &error_sub_topic,
							  const std::string &timeout_param,
							  const std::string &error_threshold_param)
			: name_(name)
			, enable_pub_topic_(enable_pub_topic)
			, command_pub_topic_(command_pub_topic)
			, state_pub_topic_(state_pub_topic)
			, error_sub_topic_(error_sub_topic)
			, timeout_param_(timeout_param)
			, error_threshold_param_(error_threshold_param)
		{
		}
		std::string name_;
		std::string enable_pub_topic_;
		std::string command_pub_topic_;
		std::string state_pub_topic_;
		std::string error_sub_topic_;
		std::string timeout_param_;
		std::string error_threshold_param_;
};

template <typename T> class AlignActionAxisState {
	public:
		AlignActionAxisState(ros::NodeHandle &nh,
							 const std::string &enable_pub_topic,
							 const std::string &command_pub_topic,
							 const std::string &state_pub_topic)
			: enable_pub_(nh.advertise<std_msgs::Bool>(enable_pub_topic, 1, true))
			, command_pub_(nh.advertise<T>(command_pub_topic, 1, true))
			, state_pub_(nh.advertise<std_msgs::Float64>(state_pub_topic, 1, true))
		{
			std_msgs::Bool bool_msg;
			bool_msg.data = false;
			enable_pub_.publish(bool_msg);

			// Set defaults for PID node topics to prevent
			// spam of "Waiting for first setpoint message."
			std_msgs::Float64 float64_msg;
			float64_msg.data = 0.0;
			state_pub_.publish(float64_msg);

			T command_msg;
			command_pub_.publish(command_msg);
		}
		
		void setEnable(const bool enable_state) const
		{
			std_msgs::Bool enable_msg;
			enable_msg.data = enable_state;
			enable_pub_.publish(enable_msg);
		}

		void setState(const double state) const
		{
			std_msgs::Float64 state_msg;
			state_msg.data = state;
			state_pub_.publish(state_msg);
		}

	protected:
		void publishCommand(const T &command) const
		{
			command_pub_.publish(command);
		}

	private:
		ros::Publisher enable_pub_;
		ros::Publisher command_pub_;
		ros::Publisher state_pub_;
};

class AlignActionAxisStatePosition : public AlignActionAxisState<std_msgs::Float64> {
	public:
		AlignActionAxisStatePosition(ros::NodeHandle &nh,
									 const std::string &enable_pub_topic,
									 const std::string &command_pub_topic,
									 const std::string &state_pub_topic)
			: AlignActionAxisState(nh, enable_pub_topic, command_pub_topic, state_pub_topic)
		{
		}

		void setCommand(const double command) const
		{
			std_msgs::Float64 command_msg;
			command_msg.data = command;
			publishCommand(command_msg);
		}
};

// These messages are sent as (position, velocity)
class AlignActionAxisStatePositionVelocity : public AlignActionAxisState<pid_velocity_msg::PIDVelocity> {
	public:
		AlignActionAxisStatePositionVelocity(ros::NodeHandle &nh,
											 const std::string &enable_pub_topic,
											 const std::string &command_pub_topic,
											 const std::string &state_pub_topic)
			: AlignActionAxisState(nh, enable_pub_topic, command_pub_topic, state_pub_topic)
		{
		}

		void setCommand(const double position_command, double velocity_command) const
		{
			pid_velocity_msg::PIDVelocity command_msg;
			command_msg.position = position_command;
			command_msg.velocity = velocity_command;
			publishCommand(command_msg);
		}
};

#endif
