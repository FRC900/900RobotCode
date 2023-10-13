#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <ctre_interfaces/latency_compensation_state_interface.h>
#include <talon_state_msgs/LatencyCompensationState.h>
#include <periodic_interval_counter/periodic_interval_counter.h>

namespace latency_compensation_state_controller
{
class LatencyCompensationStateController: public controller_interface::Controller<hardware_interface::latency_compensation::CTRELatencyCompensationStateInterface>
{
private:
	std::vector<hardware_interface::latency_compensation::CTRELatencyCompensationStateHandle> latency_compensation_state_;
	std::unique_ptr<realtime_tools::RealtimePublisher<talon_state_msgs::LatencyCompensationState>> realtime_pub_;
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	double publish_rate_;
	size_t num_hw_joints_; ///< Number of joints present in the LatencyCompensationStateInterface

public:
	bool init(hardware_interface::latency_compensation::CTRELatencyCompensationStateInterface *hw,
			  ros::NodeHandle &root_nh,
			  ros::NodeHandle &controller_nh)
	{
		// get all joint names from the hardware interface
		const std::vector<std::string> &joint_names = hw->getNames();
		num_hw_joints_ = joint_names.size();
		for (size_t i = 0; i < num_hw_joints_; i++)
			ROS_DEBUG("Got joint %s", joint_names[i].c_str());

		// get publishing period
		if (!controller_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_ERROR("Parameter 'publish_rate' not set in latency_compensation state controller");
			return false;
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish rate in latency_compensation state controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		// realtime publisher
		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<talon_state_msgs::LatencyCompensationState>>(root_nh, "latency_compensation_states", 2);

		auto &m = realtime_pub_->msg_;

		// get joints and allocate message
		for (size_t i = 0; i < num_hw_joints_; i++)
		{
			m.name.push_back(joint_names[i]);
			latency_compensation_state_.push_back(hw->getHandle(joint_names[i]));
			m.latency_compensation_groups.push_back(talon_state_msgs::LatencyCompensationGroup());
			const auto entry_names = latency_compensation_state_.back()->getEntryNames();

			for (const auto &entry_name : entry_names)
			{
				m.latency_compensation_groups.back().name.push_back(entry_name);
                m.latency_compensation_groups.back().stamp.push_back(ros::Time{});
				m.latency_compensation_groups.back().value.push_back(0.);
				m.latency_compensation_groups.back().slope.push_back(0.);
			}
		}

		return true;
	}

	void starting(const ros::Time &time)
	{
		interval_counter_->reset();
	}

	void update(const ros::Time &time, const ros::Duration & period)
	{
		// limit rate of publishing
		if (interval_counter_->update(period))
		{
			// try to publish
			if (realtime_pub_->trylock())
			{
				auto &m = realtime_pub_->msg_;
				m.header.stamp = time;
				for (size_t i = 0; i < num_hw_joints_; i++)
				{
					const auto &cs = latency_compensation_state_[i];
					for (size_t j = 0; j < m.latency_compensation_groups[i].name.size(); j++)
					{
						cs->getEntry(m.latency_compensation_groups[i].name[j],
									 m.latency_compensation_groups[i].stamp[j],
									 m.latency_compensation_groups[i].value[j],
									 m.latency_compensation_groups[i].slope[j]);
					}
				}
				realtime_pub_->unlockAndPublish();
			}
			else
			{
				interval_counter_->force_publish();
			}
		}
	}

	void stopping(const ros::Time & /*time*/)
	{
	}

}; // class

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(latency_compensation_state_controller::LatencyCompensationStateController, controller_interface::ControllerBase)
