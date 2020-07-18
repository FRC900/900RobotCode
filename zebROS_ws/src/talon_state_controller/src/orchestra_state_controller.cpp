#include <algorithm>
#include <cstddef>

#include <pluginlib/class_list_macros.h>
#include "talon_state_controller/orchestra_state_controller.h"

namespace orchestra_state_controller
{

bool OrchestraStateController::init(hardware_interface::OrchestraStateInterface *hw,
								ros::NodeHandle                         &root_nh,
								ros::NodeHandle                         &controller_nh)
{
	// get all joint names from the hardware interface
	const std::vector<std::string> &joint_names = hw->getNames();
	num_hw_joints_ = joint_names.size();
	for (size_t i = 0; i < num_hw_joints_; i++)
		ROS_DEBUG("Got joint %s", joint_names[i].c_str());

	// get publishing period
	if (!controller_nh.getParam("publish_rate", publish_rate_))
	{
		ROS_ERROR("Parameter 'publish_rate' not set");
		return false;
	}

	// realtime publisher
	realtime_pub_.reset(new
						realtime_tools::RealtimePublisher<talon_state_msgs::OrchestraState>(root_nh, "orchestra_states", 4));

	auto &m = realtime_pub_->msg_;
	talon_state_msgs::InstrumentList instrument_list;
	instrument_list.instruments = {};

	for (size_t i = 0; i < num_hw_joints_; i++)
	{
		m.instrument_list.push_back(instrument_list);
		m.chirp_file_path.push_back("");
		m.is_playing.push_back(false);
		m.is_paused.push_back(false);

		orchestra_state_.push_back(hw->getHandle(joint_names[i]));
	}

	return true;
}

void OrchestraStateController::starting(const ros::Time &time)
{
	// initialize time
	last_publish_time_ = time;
}

void OrchestraStateController::update(const ros::Time &time, const ros::Duration & /*period*/)
{
	talon_state_msgs::InstrumentList instrument_list_holder;

	// limit rate of publishing
	if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
	{
		// try to publish
		if (realtime_pub_->trylock())
		{
			// we're actually publishing, so increment time
			last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

			// populate joint state message:
			// - fill only joints that are present in the JointStateInterface, i.e. indices [0, num_hw_joints_)
			// - leave unchanged extra joints, which have static values, i.e. indices from num_hw_joints_ onwards
			auto &m = realtime_pub_->msg_;
			m.header.stamp = time;
			for (size_t i = 0; i < num_hw_joints_; i++)
			{
				auto &os = orchestra_state_[i];

				instrument_list_holder.instruments = os->getInstruments();
				m.instrument_list[i] = instrument_list_holder;
				m.chirp_file_path[i] = os->getChirpFilePath();
				m.is_playing[i] = os->getIsPlaying();
				m.is_paused[i] = os->getIsPaused();
			}
			realtime_pub_->unlockAndPublish();
		}
	}
}

void OrchestraStateController::stopping(const ros::Time & /*time*/)
{}

}

PLUGINLIB_EXPORT_CLASS(orchestra_state_controller::OrchestraStateController, controller_interface::ControllerBase)
