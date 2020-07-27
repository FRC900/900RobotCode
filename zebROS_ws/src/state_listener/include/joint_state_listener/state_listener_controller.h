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

// since not all joint names are guaranteed to be found in the
// joint state message, keep track of which ones have using
// this class. Only write values during update if valid end up
// being true.
template <class T>
class ValueValid
{
	public:
		ValueValid() : valid_(false) { }
		ValueValid(const T &value) : value_(value), valid_(false) {}
		T      value_;
		bool   valid_;
};

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
			std::vector<ValueValid<double>> vals = *command_buffer_.readFromRT();
			for (size_t i = 0; i < vals.size(); i++)
				if (vals[i].valid_)
					handles_[i].setCommand(vals[i].value_);
		}

	private:
		ros::Subscriber sub_command_;
		std::vector<std::string> joint_names_;
		std::vector<hardware_interface::JointHandle> handles_;

		// Real-time buffer holds the last command value read from the "command" topic.
		realtime_tools::RealtimeBuffer<std::vector<ValueValid<double>>> command_buffer_;

		// Iterate through each desired joint state.  If it is found in
		// the message, save the value here in the realtime buffer.
		void commandCB(const sensor_msgs::JointStateConstPtr &msg)
		{
			std::vector<ValueValid<double>> ret;
			ret.resize(joint_names_.size());
			for (size_t i = 0; i < joint_names_.size(); i++)
			{
				auto it = std::find(msg->name.cbegin(), msg->name.cend(), joint_names_[i]);
				if (it != msg->name.cend())
				{
					const size_t loc = std::distance(msg->name.cbegin(), it);
					ret[i].value_ = msg->position[loc];
					ret[i].valid_ = true;
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
			std::vector<ValueValid<int>> vals = *command_buffer_.readFromRT();
			for (size_t i = 0; i < vals.size(); i++)
				if (vals[i].valid_)
					handles_[i].setMode(static_cast<hardware_interface::JointCommandModes>(vals[i].value_));
		}

	private:
		ros::Subscriber sub_command_;
		std::vector<std::string> joint_names_;
		std::vector<hardware_interface::JointModeHandle> handles_;

		// Real-time buffer holds the last command value read from the "command" topic.
		realtime_tools::RealtimeBuffer<std::vector<ValueValid<int>>> command_buffer_;

		// Iterate through each desired joint mode.  If it is found in
		// the message, save the value here in the realtime buffer.
		void commandCB(const frc_msgs::JointModeConstPtr &msg)
		{
			std::vector<ValueValid<int>> ret;
			ret.resize(joint_names_.size());
			for (size_t i = 0; i < joint_names_.size(); i++)
			{
				auto it = std::find(msg->name.cbegin(), msg->name.cend(), joint_names_[i]);
				if (it != msg->name.cend())
				{
					const size_t loc = std::distance(msg->name.cbegin(), it);
					ret[i].value_ = msg->mode[loc];
					ret[i].valid_ = true;
				}
			}
			command_buffer_.writeFromNonRT(ret);
		}
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
				ROS_ERROR("IMU State Listener Controller : no remote pdp joints defined");
			}
			ROS_INFO_STREAM("IMU State listener got joint " << joint_names[0]);
			handle_ = hw->getHandle(joint_names[0]);

			std::string topic;

			// get topic to subscribe to
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

			handle_.setFrameId(data.frame_id);
			handle_.setOrientation(&data_orientation_[0]);
			handle_.setOrientationCovariance(&data_orientation_covariance_[0]);
			handle_.setAngularVelocity(&data_angular_velocity_[0]);
			handle_.setAngularVelocityCovariance(&data_angular_velocity_covariance_[0]);
			handle_.setLinearAcceleration(&data_linear_acceleration_[0]);
			handle_.setLinearAccelerationCovariance(&data_linear_acceleration_covariance_[0]);
		}

	private:
		ros::Subscriber sub_command_;
		hardware_interface::ImuWritableSensorHandle handle_;

		// Real-time buffer holds the last command value read from the
		// "command" topic.
		realtime_tools::RealtimeBuffer<hardware_interface::ImuSensorHandle::Data> command_buffer_;
		std::array<double, 4> data_orientation_;
		std::array<double, 9> data_orientation_covariance_;
		std::array<double, 3> data_angular_velocity_;
		std::array<double, 9> data_angular_velocity_covariance_;
		std::array<double, 3> data_linear_acceleration_;
		std::array<double, 9> data_linear_acceleration_covariance_;

		void commandCB(const sensor_msgs::ImuConstPtr &msg)
		{
			hardware_interface::ImuSensorHandle::Data data;

			data.frame_id = msg->header.frame_id;
			data.orientation = data_orientation_.begin();
			data.orientation_covariance = data_orientation_covariance_.begin();
			data.angular_velocity = data_angular_velocity_.begin();
			data.angular_velocity_covariance = data_angular_velocity_covariance_.begin();
			data.linear_acceleration = data_linear_acceleration_.begin();
			data.linear_acceleration_covariance = data_linear_acceleration_covariance_.begin();

			data_orientation_[0] = msg->orientation.x;
			data_orientation_[1] = msg->orientation.y;
			data_orientation_[2] = msg->orientation.z;
			data_orientation_[3] = msg->orientation.w;
			std::copy(msg->orientation_covariance.cbegin(), msg->orientation_covariance.cend(), data_orientation_covariance_.begin());
			data_angular_velocity_[0] = msg->angular_velocity.x;
			data_angular_velocity_[1] = msg->angular_velocity.y;
			data_angular_velocity_[2] = msg->angular_velocity.z;
			std::copy(msg->angular_velocity_covariance.cbegin(), msg->angular_velocity_covariance.cend(), data_angular_velocity_covariance_.begin());

			data_linear_acceleration_[0] = msg->linear_acceleration.x;
			data_linear_acceleration_[1] = msg->linear_acceleration.y;
			data_linear_acceleration_[2] = msg->linear_acceleration.z;
			std::copy(msg->linear_acceleration_covariance.cbegin(), msg->linear_acceleration_covariance.cend(), data_linear_acceleration_covariance_.begin());

			command_buffer_.writeFromNonRT(data);
		}
};

} // namespace
