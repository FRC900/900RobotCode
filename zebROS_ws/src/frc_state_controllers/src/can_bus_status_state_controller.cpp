/*
 * Original joint_state_controller Author: Wim Meeussen
 */

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_interfaces/can_bus_status_interface.h>
#include <frc_msgs/CANBusStatusState.h>
#include <periodic_interval_counter/periodic_interval_counter.h>

namespace can_bus_status_state_controller
{
/**
 * \brief Controller that publishes the state of all PHs in a robot.
 *
 * This controller publishes the state of all resources registered to a \c hardware_interface::CANBusStatusStateInterface to a
 * topic of type \c can_bus_status_state_controller/CANBusStatusState. The following is a basic configuration of the controller.
 *
 * \code
 * can_bus_status_state_controller:
 *   type: can_bus_status_state_controller/CANBusStatusStateController
 *   publish_rate: 50
 * \endcode
 *
 */
class CANBusStatusStateController: public controller_interface::Controller<hardware_interface::can_bus_status::CANBusStatusStateInterface>
{
private:
	std::vector<hardware_interface::can_bus_status::CANBusStatusStateHandle> can_bus_status_state_;
	std::unique_ptr<realtime_tools::RealtimePublisher<frc_msgs::CANBusStatusState>> realtime_pub_;
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	double publish_rate_{5};
	size_t num_can_bus_status_states_{0};

public:
	bool init(hardware_interface::can_bus_status::CANBusStatusStateInterface *hw,
			  ros::NodeHandle                       						 &root_nh,
			  ros::NodeHandle                       						 &controller_nh) override
	{
		ROS_INFO_NAMED("can_bus_status_state_controller", "CANBusStatusStateController::init() called");
		// get all joint names from the hardware interface
		const std::vector<std::string> &can_bus_state_state_names = hw->getNames();
		num_can_bus_status_states_ = can_bus_state_state_names.size();
		if (num_can_bus_status_states_ < 1)
		{
			ROS_ERROR_STREAM("Cannot initialize zero CAN bus statuses - need to add a can_bus_status joint def?");
			return false;
		}
		for (size_t i = 0; i < num_can_bus_status_states_; i++)
		{
			ROS_DEBUG("Got joint %s", can_bus_state_state_names[i].c_str());
		}

		// get publishing period
		if (!controller_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_WARN_STREAM("Parameter 'publish_rate' not set in CAN bus status state controller, using default " << publish_rate_);
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish_rate in CAN bus status state controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		// realtime publisher
		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<frc_msgs::CANBusStatusState>>(root_nh, "can_bus_status_states", 4);

		// get joints and allocate message
		auto &m = realtime_pub_->msg_;
		for (size_t i = 0; i < num_can_bus_status_states_; i++)
		{
			m.name.push_back(can_bus_state_state_names[i]);
			m.bus_utilization.push_back(0.);
			m.bus_off_count.push_back(0.);
			m.tx_full_count.push_back(0.);
			m.rec.push_back(0);
			m.tec.push_back(0);
			m.is_network_fd.push_back(false);

			can_bus_status_state_.push_back(hw->getHandle(can_bus_state_state_names[i]));
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
				for (unsigned i = 0; i < num_can_bus_status_states_; i++)
				{
					const auto &can_bus_status_state = can_bus_status_state_[i];
					m.bus_utilization[i] = can_bus_status_state->getBusUtilization();
					m.bus_off_count[i] = can_bus_status_state->getBusOffCount();
					m.tx_full_count[i] = can_bus_status_state->getTXFullCount();
					m.rec[i] = can_bus_status_state->getREC();
					m.tec[i] = can_bus_status_state->getTEC();
					m.is_network_fd[i] = can_bus_status_state->getIsNetworkFD();
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

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(can_bus_status_state_controller::CANBusStatusStateController, controller_interface::ControllerBase)
