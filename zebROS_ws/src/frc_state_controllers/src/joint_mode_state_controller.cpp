#include "frc_state_controllers/joint_mode_state_controller.h"
#include <pluginlib/class_list_macros.h>

namespace joint_mode_state_controller
{
    bool JointModeStateController::init(hardware_interface::JointModeInterface *hw,
										ros::NodeHandle                        &root_nh,
										ros::NodeHandle                        &controller_nh)

	{
		ROS_INFO_STREAM_NAMED("joint_mode_controller", "init is running");

		// Get list of all JointMode joint names
		std::vector<std::string> joint_names = hw->getNames();

		//get publish rate from config file
		if (!controller_nh.getParam("publish_rate", publish_rate_))
			ROS_ERROR("Could not read publish_rate in match state controller");

		//set up publisher
		realtime_pub_.reset(new realtime_tools::RealtimePublisher<frc_msgs::JointMode>(root_nh, "joint_modes", 4));

		auto &m = realtime_pub_->msg_;

		for (const auto &n : joint_names)
		{
			m.name.push_back(n);
			m.mode.push_back(0);
			joint_modes_.push_back(hw->getHandle(n));
		}


		return true;
	}

    void JointModeStateController::starting(const ros::Time &time)
    {
		last_publish_time_ = time;
    }

    void JointModeStateController::update(const ros::Time &time, const ros::Duration & )
	{
		if ((publish_rate_ > 0.0) && (last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time))
		{
			if (realtime_pub_->trylock())
			{
				last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

				auto &m = realtime_pub_->msg_;

				m.header.stamp = time;

				for (size_t i = 0; i < joint_modes_.size(); i++)
				{
					m.mode[i] = static_cast<unsigned char>(joint_modes_[i].getMode());
				}
				realtime_pub_->unlockAndPublish();
			}
		}
	}

    void JointModeStateController::stopping(const ros::Time & )
    {}
}

PLUGINLIB_EXPORT_CLASS(joint_mode_state_controller::JointModeStateController, controller_interface::ControllerBase)
