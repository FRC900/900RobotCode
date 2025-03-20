/*
 * Original joint_state_controller Author: Wim Meeussen
 */

#include <optional>

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <frc_interfaces/rumble_state_interface.h>
#include <frc_msgs/RumbleState.h>
#include <periodic_interval_counter/periodic_interval_counter.h>

namespace rumble_state_controller
{
/**
 * \brief Controller that publishes the state of all Rumbles in a robot.
 *
 * This controller publishes the state of all resources registered to a \c hardware_interface::RumbleStateInterface to a
 * topic of type \c rumble_state_controller/RumbleState. The following is a basic configuration of the controller.
 *
 * \code
 * rumble_state_controller:
 *   type: rumble_state_controller/RumbleStateController
 *   publish_rate: 50
 * \endcode
 *
 */
class RumbleStateController: public controller_interface::Controller<hardware_interface::RumbleStateInterface>
{
private:
	std::vector<hardware_interface::RumbleStateHandle> rumble_state_;
	std::unique_ptr<realtime_tools::RealtimePublisher<frc_msgs::RumbleState>> realtime_pub_;
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	double publish_rate_{5};
	size_t num_rumbles_{0};

public:
	bool init(hardware_interface::RumbleStateInterface *hw,
			  ros::NodeHandle                       &root_nh,
			  ros::NodeHandle                       &controller_nh) override
	{
		// get all joint names from the hardware interface
		const std::vector<std::string> &rumble_names = hw->getNames();
		num_rumbles_ = rumble_names.size();
		if (num_rumbles_ < 1)
		{
			ROS_ERROR_STREAM("Cannot initialize zero Rumbles - need to add a rumble joint def?");
			return false;
		}
		for (size_t i = 0; i < num_rumbles_; i++)
		{
			ROS_DEBUG("Got joint %s", rumble_names[i].c_str());
		}

		// get publishing period
		if (!controller_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_WARN_STREAM("Parameter 'publish_rate' not set in Rumble controller, using default " << publish_rate_);
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish_rate in Rumble controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		// realtime publisher
		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<frc_msgs::RumbleState>>(root_nh, "rumble_states", 4);

		// get joints and allocate message
		auto &m = realtime_pub_->msg_;
		for (size_t i = 0; i < num_rumbles_; i++)
		{
			m.name.push_back(rumble_names[i]);
			m.left.push_back(-1);
			m.right.push_back(-1);

			rumble_state_.push_back(hw->getHandle(rumble_names[i]));
		}

		return true;
	}

	void starting(const ros::Time &time) override
	{
		interval_counter_->reset();
	}

	void update(const ros::Time &time, const ros::Duration &period) override
	{
		// limit rate of publishing
		if (interval_counter_->update(period))
		{
			// try to publish
			if (realtime_pub_->trylock())
			{
				auto &m = realtime_pub_->msg_;
				m.header.stamp = time;
				for (unsigned i = 0; i < num_rumbles_; i++)
				{
					const auto &rumbles = rumble_state_[i];
					m.left[i] = rumbles->getLeft();
					m.right[i] = rumbles->getRight();
				}
				realtime_pub_->unlockAndPublish();
			}
			else
			{
				interval_counter_->force_publish();
			}
		}
	}

	void stopping(const ros::Time & /*time*/) override
	{}
}; // class

} //namespace

namespace state_listener_controller
{
class RumbleStateListenerController :
	public controller_interface::Controller<hardware_interface::RemoteRumbleStateInterface>
{
private:
	ros::Subscriber sub_command_;
	std::vector<std::string> joint_names_;
	std::vector<hardware_interface::RumbleWritableStateHandle> handles_;

	// Real-time buffer holds the last command value read from the "command" topic.
	realtime_tools::RealtimeBuffer<std::vector<std::optional<hardware_interface::RumbleHWState>>> command_buffer_;

	// Iterate through each desired joint state.  If it is found in
	// the message, save the value here in the realtime buffer.
	virtual void commandCB(const frc_msgs::RumbleStateConstPtr &msg)
	{
		std::vector<std::optional<hardware_interface::RumbleHWState>> ret;
		for (const auto &joint_name : joint_names_)
		{
			auto it = std::find(msg->name.cbegin(), msg->name.cend(), joint_name);
			if (it != msg->name.cend())
			{
				const size_t i = std::distance(msg->name.cbegin(), it);
				hardware_interface::RumbleHWState s;
				s.setLeft(msg->left[i]);
				s.setRight(msg->right[i]);
				ret.emplace_back(s);
			}
			else
			{
				ret.emplace_back(std::nullopt);
			}
		}
		command_buffer_.writeFromNonRT(ret);
	}


public:
	RumbleStateListenerController() = default;

	~RumbleStateListenerController() override
	{
		sub_command_.shutdown();
	}

	bool init(hardware_interface::RemoteRumbleStateInterface *hw, ros::NodeHandle &n) override
	{
		// Read list of hw, make a list, grab handles for them, plus allocate storage space
		joint_names_ = hw->getNames();
		for (const auto &j : joint_names_)
		{
			ROS_INFO_STREAM("Joint State Listener Controller got joint " << j);
			handles_.push_back(hw->getHandle(j));
		}

		std::string topic;
		// get topic to subscribe to
		if (!n.getParam("topic", topic))
		{
			ROS_ERROR("Rumble State Listener Controller parameter 'topic' not set");
			return false;
		}

		sub_command_ = n.subscribe<frc_msgs::RumbleState>(topic, 1, &RumbleStateListenerController::commandCB, this, ros::TransportHints().tcpNoDelay());
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
		const auto vals = *command_buffer_.readFromRT();
		for (size_t i = 0; i < vals.size(); i++)
		{
			if (vals[i])
			{
				handles_[i]->setLeft(vals[i]->getLeft());
				handles_[i]->setRight(vals[i]->getRight());
			}
		}
	}
}; // class

} // namespace state_listener_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rumble_state_controller::RumbleStateController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(state_listener_controller::RumbleStateListenerController, controller_interface::ControllerBase)