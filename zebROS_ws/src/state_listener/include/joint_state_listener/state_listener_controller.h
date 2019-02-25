#pragma once

#include <controller_interface/controller.h>
#include <ros/node_handle.h>
#include <realtime_tools/realtime_buffer.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include "frc_interfaces/remote_joint_interface.h"
#include "frc_msgs/PDPData.h"
#include "talon_state_controller/TalonState.h"
#include "frc_msgs/MatchSpecificData.h"

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

		virtual bool init(hardware_interface::RemoteJointInterface *hw, ros::NodeHandle &n) override
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

		virtual void starting(const ros::Time & /*time*/) override
		{
		}
		virtual void stopping(const ros::Time & /*time*/) override
		{
			//handles_.release();
		}

		virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
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

		// Real-time buffer holds the last command value read from the
		// "command" topic.
		realtime_tools::RealtimeBuffer<std::vector<ValueValid<double>>> command_buffer_;

		// Iterate through each desired joint state.  If it is found in
		// the message, save the value here in the realtime buffer.
		// // TODO : figure out how to hack this to use a ConstPtr type instead
		virtual void commandCB(const sensor_msgs::JointStateConstPtr &msg)
		{
			std::vector<ValueValid<double>> ret;
			ret.resize(joint_names_.size());
			for (size_t i = 0; i < joint_names_.size(); i++)
			{
				auto it = std::find(msg->name.cbegin(), msg->name.cend(), joint_names_[i]);
				if (it != msg->name.cend())
				{
					const size_t loc = it - msg->name.cbegin();
					ret[i].value_ = msg->position[loc];
					ret[i].valid_ = true;
				}
			}
			command_buffer_.writeFromNonRT(ret);
		}
};

class PDPStateListenerController :
	public controller_interface::Controller<hardware_interface::RemotePDPStateInterface>
{
	public:
		PDPStateListenerController() {}
		~PDPStateListenerController()
		{
			sub_command_.shutdown();
		}

		virtual bool init(hardware_interface::RemotePDPStateInterface *hw, ros::NodeHandle &n) override
		{
			// Read list of hw, make a list, grab handles for them, plus allocate storage space
			auto joint_names = hw->getNames();
			if (joint_names.size() == 0)
			{
				ROS_ERROR("PDP State Listener Controller : no remote pdp joints defined");
			}
			ROS_INFO_STREAM("PDP State Listener Controller got joint " << joint_names[0]);
			handle_ = hw->getHandle(joint_names[0]);

			std::string topic;

			// get topic to subscribe to
			if (!n.getParam("topic", topic))
			{
				ROS_ERROR("Parameter 'topic' not set");
				return false;
			}

			sub_command_ = n.subscribe<frc_msgs::PDPData>(topic, 1, &PDPStateListenerController::commandCB, this);
			return true;
		}

		virtual void starting(const ros::Time & /*time*/) override
		{
		}
		virtual void stopping(const ros::Time & /*time*/) override
		{
			//handles_.release();
		}

		virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
		{
			// Quick way to do a shallow copy of the entire HW state
			*(handle_.operator->()) = *command_buffer_.readFromRT();
		}

	private:
		ros::Subscriber sub_command_;
		hardware_interface::PDPWritableStateHandle handle_;

		// Real-time buffer holds the last command value read from the
		// "command" topic.
		realtime_tools::RealtimeBuffer<hardware_interface::PDPHWState> command_buffer_;

		// Iterate through each desired joint state.  If it is found in
		// the message, save the value here in the realtime buffer.
		// // TODO : figure out how to hack this to use a ConstPtr type instead
		virtual void commandCB(const frc_msgs::PDPDataConstPtr &msg)
		{
			hardware_interface::PDPHWState data;

			data.setVoltage(msg->voltage);
			data.setTemperature(msg->temperature);
			data.setTotalCurrent(msg->totalCurrent);
			data.setTotalPower(msg->totalPower);
			data.setTotalEnergy(msg->totalEnergy);
			for (size_t channel = 0; channel <= 15; channel++)
				data.setCurrent(msg->current[channel], channel);
			command_buffer_.writeFromNonRT(data);
		}
};

class MatchStateListenerController :
	public controller_interface::Controller<hardware_interface::RemoteMatchStateInterface>
{
	public:
		MatchStateListenerController() {}
		~MatchStateListenerController()
		{
			sub_command_.shutdown();
		}

		virtual bool init(hardware_interface::RemoteMatchStateInterface *hw, ros::NodeHandle &n) override
		{
			// Read list of hw, make a list, grab handles for them, plus allocate storage space
			auto joint_names = hw->getNames();
			if (joint_names.size() == 0)
			{
				ROS_ERROR("Match State Listener Controller : no remote match joints defined. Don't run this on a roboRio");
			}
			ROS_INFO_STREAM("Match State Listener Controller got joint " << joint_names[0]);
			handle_ = hw->getHandle(joint_names[0]);

			std::string topic;

			// get topic to subscribe to
			if (!n.getParam("topic", topic))
			{
				ROS_ERROR("Parameter 'topic' not set");
				return false;
			}

			sub_command_ = n.subscribe<frc_msgs::MatchSpecificData>(topic, 1, &MatchStateListenerController::commandCB, this);
			return true;
		}

		virtual void starting(const ros::Time & /*time*/) override
		{
		}
		virtual void stopping(const ros::Time & /*time*/) override
		{
			//handles_.release();
		}

		virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
		{
			// Quick way to do a shallow copy of the entire HW state
			*(handle_.operator->()) = *command_buffer_.readFromRT();
		}

	private:
		ros::Subscriber sub_command_;
		hardware_interface::MatchStateWritableHandle handle_;

		// Real-time buffer holds the last command value read from the
		// "command" topic.
		realtime_tools::RealtimeBuffer<hardware_interface::MatchHWState> command_buffer_;

		// Iterate through each desired joint state.  If it is found in
		// the message, save the value here in the realtime buffer.
		// // TODO : figure out how to hack this to use a ConstPtr type instead
		virtual void commandCB(const frc_msgs::MatchSpecificDataConstPtr &msg)
		{
			hardware_interface::MatchHWState data;
			data.setMatchTimeRemaining(msg->matchTimeRemaining);

			data.setGameSpecificData(msg->gameSpecificData);
			data.setEventName(msg->eventName);

			data.setAllianceColor(msg->allianceColor);
			data.setMatchType(msg->matchType);
			data.setDriverStationLocation(msg->driverStationLocation);
			data.setMatchNumber(msg->matchNumber);
			data.setReplayNumber(msg->replayNumber);

			data.setEnabled(msg->Enabled);
			data.setDisabled(msg->Disabled);
			data.setAutonomous(msg->Autonomous);
			data.setFMSAttached(msg->FMSAttached);
			data.setDSAttached(msg->DSAttached);
			data.setOperatorControl(msg->OperatorControl);
			data.setTest(msg->Test);

			data.setBatteryVoltage(msg->BatteryVoltage);
			command_buffer_.writeFromNonRT(data);
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

		virtual bool init(hardware_interface::RemoteImuSensorInterface *hw, ros::NodeHandle &n) override
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

		virtual void starting(const ros::Time & /*time*/) override
		{
		}
		virtual void stopping(const ros::Time & /*time*/) override
		{
			//handles_.release();
		}

		virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
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

		virtual void commandCB(const sensor_msgs::ImuConstPtr &msg)
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

class TalonStateListenerController :
	public controller_interface::Controller<hardware_interface::RemoteTalonStateInterface>
{
	public:
		TalonStateListenerController() {}
		~TalonStateListenerController()
		{
			sub_command_.shutdown();
		}

		virtual bool init(hardware_interface::RemoteTalonStateInterface *hw, ros::NodeHandle &n) override
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

			sub_command_ = n.subscribe<talon_state_controller::TalonState>(topic, 1, &TalonStateListenerController::commandCB, this);
			return true;
		}

		virtual void starting(const ros::Time & /*time*/) override
		{
		}
		virtual void stopping(const ros::Time & /*time*/) override
		{
			//handles_.release();
		}

		virtual void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override
		{
			// Take the most recent set of values read from the joint_states
			// topic and write them to the local joints
			auto vals = *command_buffer_.readFromRT();
			for (size_t i = 0; i < vals.size(); i++)
			{
				if (vals[i].valid_)
				{
					auto ts = vals[i].value_;
					handles_[i]->setPosition(ts.getPosition());
					handles_[i]->setSpeed(ts.getSpeed());
					handles_[i]->setOutputCurrent(ts.getOutputCurrent());
					handles_[i]->setBusVoltage(ts.getBusVoltage());
					handles_[i]->setMotorOutputPercent(ts.getMotorOutputPercent());
					handles_[i]->setOutputVoltage(ts.getOutputVoltage());
					handles_[i]->setTemperature(ts.getTemperature());
					handles_[i]->setClosedLoopError(ts.getClosedLoopError());
					handles_[i]->setIntegralAccumulator(ts.getIntegralAccumulator());
					handles_[i]->setErrorDerivative(ts.getErrorDerivative());
					handles_[i]->setClosedLoopTarget(ts.getClosedLoopTarget());
					handles_[i]->setActiveTrajectoryPosition(ts.getActiveTrajectoryPosition());
					handles_[i]->setActiveTrajectoryVelocity(ts.getActiveTrajectoryVelocity());
					handles_[i]->setActiveTrajectoryHeading(ts.getActiveTrajectoryHeading());
					handles_[i]->setMotionProfileTopLevelBufferCount(ts.getMotionProfileTopLevelBufferCount());
					handles_[i]->setFaults(ts.getFaults());
					handles_[i]->setForwardLimitSwitch(ts.getForwardLimitSwitch());
					handles_[i]->setReverseLimitSwitch(ts.getReverseLimitSwitch());
					handles_[i]->setForwardSoftlimitHit(ts.getForwardSoftlimitHit());
					handles_[i]->setReverseSoftlimitHit(ts.getReverseSoftlimitHit());
					handles_[i]->setStickyFaults(ts.getStickyFaults());
				}
			}

		}

	private:
		ros::Subscriber sub_command_;
		std::vector<std::string> joint_names_;
		std::vector<hardware_interface::TalonWritableStateHandle> handles_;

		// Real-time buffer holds the last command value read from the
		// "command" topic.
		realtime_tools::RealtimeBuffer<std::vector<ValueValid<hardware_interface::TalonHWState>>> command_buffer_;

		virtual void commandCB(const talon_state_controller::TalonStateConstPtr &msg)
		{
			std::vector<ValueValid<hardware_interface::TalonHWState>> data;
			for (size_t i = 0; i < joint_names_.size(); i++)
				data.push_back(hardware_interface::TalonHWState(msg->can_id[i]));
			for (size_t i = 0; i < joint_names_.size(); i++)
			{
				auto it = std::find(msg->name.cbegin(), msg->name.cend(), joint_names_[i]);
				if (it != msg->name.cend())
				{
					data[i].value_.setPosition(msg->position[i]);
					data[i].value_.setSpeed(msg->speed[i]);
					data[i].value_.setOutputCurrent(msg->output_voltage[i]);
					data[i].value_.setBusVoltage(msg->bus_voltage[i]);
					data[i].value_.setMotorOutputPercent(msg->motor_output_percent[i]);
					data[i].value_.setOutputVoltage(msg->output_voltage[i]);
					data[i].value_.setTemperature(msg->temperature[i]);
					data[i].value_.setClosedLoopError(msg->closed_loop_error[i]);
					data[i].value_.setIntegralAccumulator(msg->integral_accumulator[i]);
					data[i].value_.setErrorDerivative(msg->error_derivative[i]);
					data[i].value_.setClosedLoopTarget(msg->closed_loop_target[i]);
					data[i].value_.setActiveTrajectoryPosition(msg->active_trajectory_position[i]);
					data[i].value_.setActiveTrajectoryVelocity(msg->active_trajectory_velocity[i]);
					data[i].value_.setActiveTrajectoryHeading(msg->active_trajectory_heading[i]);
					data[i].value_.setMotionProfileTopLevelBufferCount(msg->motion_profile_top_level_buffer_count[i]);
					//data[i].value_.setFaults(msg->getFaults[i]);
					data[i].value_.setForwardLimitSwitch(msg->forward_limit_switch[i]);
					data[i].value_.setReverseLimitSwitch(msg->reverse_limit_switch[i]);
					data[i].value_.setForwardSoftlimitHit(msg->forward_softlimit[i]);
					data[i].value_.setReverseSoftlimitHit(msg->reverse_softlimit[i]);
					//data[i].value_.setStickyFaults(msg->getStickyFaults[i]);
					data[i].valid_ = true;
				}
			}
			command_buffer_.writeFromNonRT(data);
		}
};
} // namespace
