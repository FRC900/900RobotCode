#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <ctre_interfaces/orchestra_state_interface.h>
#include <talon_state_msgs/OrchestraState.h>
#include <talon_state_msgs/InstrumentList.h>
#include <periodic_interval_counter/periodic_interval_counter.h>

namespace orchestra_state_controller
{

/**
 * \brief Controller that publishes the state of all orchestras on a robot. (There should really only be one.)
 *
 * This controller publishes the state of all resources registered to a \c hardware_interface::OrchestraStateInterface to a
 * topic of type \c talon_state_msgs/OrchestraState. The following is a basic configuration of the controller.
 *
 * \code
 * orchestra_state_controller:
 *   type: talon_state_controller/OrchestraStateController
 *   publish_rate: 50
 * \endcode
 *
 */
class OrchestraStateController: public controller_interface::Controller<hardware_interface::OrchestraStateInterface>
{

private:
	std::vector<hardware_interface::OrchestraStateHandle> orchestra_state_;
	std::unique_ptr<realtime_tools::RealtimePublisher<talon_state_msgs::OrchestraState>> realtime_pub_;
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	double publish_rate_;
	size_t num_hw_joints_; ///< Number of joints present in the OrchestraStateInterface

	bool init(hardware_interface::OrchestraStateInterface *hw,
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
			ROS_ERROR("Parameter 'publish_rate' not set in orchestra state controller");
			return false;
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish_rate in orchestra state controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		// realtime publisher
		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<talon_state_msgs::OrchestraState>>(root_nh, "orchestra_states", 4);

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

	void starting(const ros::Time &time)
	{
		// initialize time
		interval_counter_->reset();
	}

	void update(const ros::Time &time, const ros::Duration &period)
	{
		talon_state_msgs::InstrumentList instrument_list_holder;

		// limit rate of publishing
		if (interval_counter_->update(period))
		{
			// try to publish
			if (realtime_pub_->trylock())
			{
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
			else
			{
				interval_counter_->force_publish();
			}
		}
	}

	void stopping(const ros::Time & /*time*/)
	{}

}; // class

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(orchestra_state_controller::OrchestraStateController, controller_interface::ControllerBase)