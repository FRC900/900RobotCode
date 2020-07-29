#pragma once

#include <controller_interface/controller.h>
#include <ros/node_handle.h>
#include <realtime_tools/realtime_buffer.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include "frc_msgs/JointMode.h"
#include "remote_joint_interface/remote_joint_interface.h"

namespace state_listener_controller
{
/**
 * \brief Controller to read from joint state message and
 *        update local copies of the read joint values
 *
 * This controller is initialized with a list of joint names
 * which are remote to the current controller manager.  This
 * controller grabs values from a JointState message and writes
 * those values to the local copy of those remote joints
 *
 * \section ROS interface
 *
 * \param type Must be "RemoteJointInterface".
 *
 * Subscribes to:
 * - \b command (sensor_msgs::JointState) : The joint state topic
 */

class JointStateListenerController :
	public controller_interface::Controller<hardware_interface::RemoteJointInterface>
{
	public:
		JointStateListenerController() {}
		~JointStateListenerController()
		{
			sub_command_.shutdown();
		}

		bool init(hardware_interface::RemoteJointInterface *hw, ros::NodeHandle &n) override
		{
			// Read list of hw, make a list, grab handles for them, plus allocate storage space
			joint_names_ = hw->getNames();
			for (auto j : joint_names_)
			{
				ROS_INFO_STREAM("Joint State Listener Controller got joint " << j);
				handles_.push_back(hw->getHandle(j));
			}

			std::string topic;

			// get topic to subscribe to
			if (!n.getParam("topic", topic))
			{
				ROS_ERROR("Parameter 'topic' not set");
				return false;
			}

			// Might wantt to make message type a template
			// parameter as well?
			sub_command_ = n.subscribe<sensor_msgs::JointState>(topic, 1, &JointStateListenerController::commandCB, this);
			return true;
		}

		void starting(const ros::Time & /*time*/) override
		{
		}
		void stopping(const ros::Time & /*time*/) override
		{
			//handles_.release();
		}

		void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
		{
			// Take the most recent set of values read from the joint_states
			// topic and write them to the local joints
			const auto vals = *command_buffer_.readFromRT();
			for (size_t i = 0; i < vals.size(); i++)
				if (vals[i])
					handles_[i].setCommand(*(vals[i]));
		}

	private:
		ros::Subscriber sub_command_;
		std::vector<std::string> joint_names_;
		std::vector<hardware_interface::JointHandle> handles_;

		// Real-time buffer holds the last command value read from the "command" topic.
		realtime_tools::RealtimeBuffer<std::vector<std::optional<double>>> command_buffer_;

		// Iterate through each desired joint state.  If it is found in
		// the message, save the value here in the realtime buffer.
		void commandCB(const sensor_msgs::JointStateConstPtr &msg)
		{
			std::vector<std::optional<double>> ret;
			for (size_t i = 0; i < joint_names_.size(); i++)
			{
				auto it = std::find(msg->name.cbegin(), msg->name.cend(), joint_names_[i]);
				if (it != msg->name.cend())
				{
					ret.push_back(msg->position[std::distance(msg->name.cbegin(), it)]);
				}
				else
				{
					ret.push_back(std::nullopt);
				}
			}
			command_buffer_.writeFromNonRT(ret);
		}
};

class JointModeListenerController :
	public controller_interface::Controller<hardware_interface::RemoteJointModeInterface>
{
	public:
		JointModeListenerController() {}
		~JointModeListenerController()
		{
			sub_command_.shutdown();
		}

		bool init(hardware_interface::RemoteJointModeInterface *hw, ros::NodeHandle &n) override
		{
			// Read list of hw, make a list, grab handles for them, plus allocate storage space
			joint_names_ = hw->getNames();
			for (auto j : joint_names_)
			{
				ROS_INFO_STREAM("Joint Mode Listener Controller got joint " << j);
				handles_.push_back(hw->getHandle(j));
			}

			std::string topic;

			// get topic to subscribe to
			if (!n.getParam("topic", topic))
			{
				ROS_ERROR("Joint Mode Listener Controller : Parameter 'topic' not set");
				return false;
			}

			// Might wantt to make message type a template
			// parameter as well?
			sub_command_ = n.subscribe<frc_msgs::JointMode>(topic, 1, &JointModeListenerController::commandCB, this);
			return true;
		}

		void starting(const ros::Time & /*time*/) override
		{
		}
		void stopping(const ros::Time & /*time*/) override
		{
		}

		void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
		{
			// Take the most recent set of values read from the joint_modes
			// topic and write them to the local joints
			const auto vals = *command_buffer_.readFromRT();
			for (size_t i = 0; i < vals.size(); i++)
				if (vals[i])
					handles_[i].setMode(static_cast<hardware_interface::JointCommandModes>(*(vals[i])));
		}

	private:
		ros::Subscriber sub_command_;
		std::vector<std::string> joint_names_;
		std::vector<hardware_interface::JointModeHandle> handles_;

		// Real-time buffer holds the last command value read from the "command" topic.
		realtime_tools::RealtimeBuffer<std::vector<std::optional<int>>> command_buffer_;

		// Iterate through each desired joint mode.  If it is found in
		// the message, save the value here in the realtime buffer.
		void commandCB(const frc_msgs::JointModeConstPtr &msg)
		{
			std::vector<std::optional<int>> ret;
			for (size_t i = 0; i < joint_names_.size(); i++)
			{
				auto it = std::find(msg->name.cbegin(), msg->name.cend(), joint_names_[i]);
				if (it != msg->name.cend())
				{
					ret.push_back(msg->mode[std::distance(msg->name.cbegin(), it)]);
				}
				else
				{
					ret.push_back(std::nullopt);
				}
			}
			command_buffer_.writeFromNonRT(ret);
		}
};

struct IMUStateData
{
	std::string           frame_id_;
	std::array<double, 4> orientation_;
	std::array<double, 9> orientation_covariance_;
	std::array<double, 3> angular_velocity_;
	std::array<double, 9> angular_velocity_covariance_;
	std::array<double, 3> linear_acceleration_;
	std::array<double, 9> linear_acceleration_covariance_;
};

class IMUStateListenerController :
	public controller_interface::Controller<hardware_interface::RemoteImuSensorInterface>
{
	public:
		IMUStateListenerController() {}
		~IMUStateListenerController()
		{
			sub_command_.shutdown();
		}

		bool init(hardware_interface::RemoteImuSensorInterface *hw, ros::NodeHandle &n) override
		{
			// Read list of hw, make a list, grab handles for them, plus allocate storage space
			auto joint_names = hw->getNames();
			if (joint_names.size() == 0)
			{
				ROS_ERROR("IMU State Listener Controller : no remote IMU joints defined");
			}
			else if (joint_names.size() > 1)
			{
				ROS_ERROR("IMU State Listener Controller : More than 1 remote IMU joints defined");
			}
			ROS_INFO_STREAM("IMU State listener got joint " << joint_names[0]);
			handle_ = hw->getHandle(joint_names[0]);

			// get topic to subscribe to
			std::string topic;
			if (!n.getParam("topic", topic))
			{
				ROS_ERROR("Parameter 'topic' not set");
				return false;
			}

			sub_command_ = n.subscribe<sensor_msgs::Imu>(topic, 1, &IMUStateListenerController::commandCB, this);
			return true;
		}

		void starting(const ros::Time & /*time*/) override
		{
		}
		void stopping(const ros::Time & /*time*/) override
		{
		}

		void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
		{
			const auto data = *command_buffer_.readFromRT();

			handle_.setFrameId(data.frame_id_);
			handle_.setOrientation(&data.orientation_[0]);
			handle_.setOrientationCovariance(&data.orientation_covariance_[0]);
			handle_.setAngularVelocity(&data.angular_velocity_[0]);
			handle_.setAngularVelocityCovariance(&data.angular_velocity_covariance_[0]);
			handle_.setLinearAcceleration(&data.linear_acceleration_[0]);
			handle_.setLinearAccelerationCovariance(&data.linear_acceleration_covariance_[0]);
		}

	private:
		ros::Subscriber sub_command_;
		hardware_interface::ImuWritableSensorHandle handle_;

		// Real-time buffer holds the last command value read from the remote IMU state topic
		// TODO : support more than 1 IMU?
		realtime_tools::RealtimeBuffer<IMUStateData> command_buffer_;

		void commandCB(const sensor_msgs::ImuConstPtr &msg)
		{
			IMUStateData data;

			data.frame_id_ = msg->header.frame_id;

			data.orientation_[0] = msg->orientation.x;
			data.orientation_[1] = msg->orientation.y;
			data.orientation_[2] = msg->orientation.z;
			data.orientation_[3] = msg->orientation.w;
			std::copy(msg->orientation_covariance.cbegin(), msg->orientation_covariance.cend(), data.orientation_covariance_.begin());
			data.angular_velocity_[0] = msg->angular_velocity.x;
			data.angular_velocity_[1] = msg->angular_velocity.y;
			data.angular_velocity_[2] = msg->angular_velocity.z;
			std::copy(msg->angular_velocity_covariance.cbegin(), msg->angular_velocity_covariance.cend(), data.angular_velocity_covariance_.begin());

			data.linear_acceleration_[0] = msg->linear_acceleration.x;
			data.linear_acceleration_[1] = msg->linear_acceleration.y;
			data.linear_acceleration_[2] = msg->linear_acceleration.z;
			std::copy(msg->linear_acceleration_covariance.cbegin(), msg->linear_acceleration_covariance.cend(), data.linear_acceleration_covariance_.begin());

			command_buffer_.writeFromNonRT(data);
		}
};

} // namespace
