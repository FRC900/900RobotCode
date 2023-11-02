#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_mode_interface.h>
#include "frc_msgs/JointMode.h"
#include <pluginlib/class_list_macros.hpp>
#include "periodic_interval_counter/periodic_interval_counter.h"

namespace joint_mode_state_controller
{
class JointModeStateController: public controller_interface::Controller<hardware_interface::JointModeInterface>
{

private:
	std::vector<hardware_interface::JointModeHandle> joint_modes_;
	std::unique_ptr<realtime_tools::RealtimePublisher<frc_msgs::JointMode>> realtime_pub_;
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	double publish_rate_{100};

public:
	bool init(hardware_interface::JointModeInterface *hw,
			  ros::NodeHandle                        &root_nh,
			  ros::NodeHandle                        &controller_nh) override

	{
		ROS_INFO_STREAM_NAMED("joint_mode_controller", "init is running");

		// Get list of all JointMode joint names
		std::vector<std::string> joint_names = hw->getNames();

		//get publish rate from config file
		if (!controller_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_WARN("Could not read publish_rate in joint mode state controller");
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid pusblish_rate in joint mode state controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		//set up publisher
		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<frc_msgs::JointMode>>(root_nh, "joint_modes", 4);

		auto &m = realtime_pub_->msg_;

		for (const auto &n : joint_names)
		{
			m.name.push_back(n);
			m.mode.push_back(0);
			joint_modes_.push_back(hw->getHandle(n));
		}

		return true;
	}

    void starting(const ros::Time &time) override
    {
		interval_counter_->reset();
    }

    void update(const ros::Time &time, const ros::Duration &period) override
	{
		if (interval_counter_->update(period))
		{
			if (realtime_pub_->trylock())
			{
				auto &m = realtime_pub_->msg_;

				m.header.stamp = time;

				for (size_t i = 0; i < joint_modes_.size(); i++)
				{
					m.mode[i] = static_cast<unsigned char>(joint_modes_[i].getMode());
				}
				realtime_pub_->unlockAndPublish();
			}
			else
			{
				interval_counter_->force_publish();
			}
		}
	}

    void stopping(const ros::Time & ) override
    {}
}; // class

} // namespace

PLUGINLIB_EXPORT_CLASS(joint_mode_state_controller::JointModeStateController, controller_interface::ControllerBase)
