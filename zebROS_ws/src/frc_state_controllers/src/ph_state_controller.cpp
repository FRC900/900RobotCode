/*
 * Original joint_state_controller Author: Wim Meeussen
 */

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_interfaces/ph_state_interface.h>
#include <frc_msgs/PHState.h>
#include <periodic_interval_counter/periodic_interval_counter.h>

namespace ph_state_controller
{
/**
 * \brief Controller that publishes the state of all PHs in a robot.
 *
 * This controller publishes the state of all resources registered to a \c hardware_interface::PHStateInterface to a
 * topic of type \c ph_state_controller/PHState. The following is a basic configuration of the controller.
 *
 * \code
 * ph_state_controller:
 *   type: ph_state_controller/PHStateController
 *   publish_rate: 50
 * \endcode
 *
 */
class PHStateController: public controller_interface::Controller<hardware_interface::PHStateInterface>
{
private:
	std::vector<hardware_interface::PHStateHandle> ph_state_;
	std::unique_ptr<realtime_tools::RealtimePublisher<frc_msgs::PHState>> realtime_pub_;
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	double publish_rate_{20};
	size_t num_phs_{0};

public:
	bool init(hardware_interface::PHStateInterface *hw,
			  ros::NodeHandle                       &root_nh,
			  ros::NodeHandle                       &controller_nh)
	{
		ROS_INFO_NAMED("ph_state_controller", "PHStateController::init() called");
		// get all joint names from the hardware interface
		const std::vector<std::string> &ph_names = hw->getNames();
		num_phs_ = ph_names.size();
		if (num_phs_ < 1)
		{
			ROS_ERROR_STREAM("Cannot initialize zero PHs - need to add a ph joint def?");
			return false;
		}
		for (size_t i = 0; i < num_phs_; i++)
		{
			ROS_DEBUG("Got joint %s", ph_names[i].c_str());
		}

		// get publishing period
		if (!controller_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_WARN_STREAM("Parameter 'publish_rate' not set in PH controller, using default " << publish_rate_);
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish_rate in PH controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		// realtime publisher
		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<frc_msgs::PHState>>(root_nh, "ph_states", 4);

		// get joints and allocate message
		auto &m = realtime_pub_->msg_;
		for (size_t i = 0; i < num_phs_; i++)
		{
			m.name.push_back(ph_names[i]);
			m.id.push_back(-1);
			m.compressor_enabled.push_back(false);
			m.pressure_switch.push_back(false);
			m.compressor_current.push_back(0.0);
			m.analog_voltage0.push_back(0.0);
			m.analog_voltage1.push_back(0.0);
			m.pressure0.push_back(0.0);
			m.pressure1.push_back(0.0);
			m.compressor_min_analog_voltage.push_back(0.0);
			m.compressor_max_analog_voltage.push_back(0.0);
			m.compressor_force_disable.push_back(false);
			m.compressor_use_digital.push_back(false);

			ph_state_.push_back(hw->getHandle(ph_names[i]));
		}

		return true;
	}

	void starting(const ros::Time &time)
	{
		interval_counter_->reset();
	}

	void update(const ros::Time &time, const ros::Duration &period)
	{
		// limit rate of publishing
		if (interval_counter_->update(period))
		{
			// try to publish
			if (realtime_pub_->trylock())
			{
				auto &m = realtime_pub_->msg_;
				m.header.stamp = time;
				for (unsigned i = 0; i < num_phs_; i++)
				{
					auto &phs = ph_state_[i];
					m.id[i] = phs->getId();
					m.compressor_enabled[i] = phs->getCompressorEnabled();
					m.pressure_switch[i] = phs->getPressureSwitch();
					m.compressor_current[i] = phs->getCompressorCurrent();
					m.analog_voltage0[i] = phs->getAnalogVoltage(0);
					m.analog_voltage1[i] = phs->getAnalogVoltage(1);
					m.pressure0[i] = phs->getPressure(0);
					m.pressure1[i] = phs->getPressure(1);
					m.compressor_min_analog_voltage[i] = phs->getCompressorMinAnalogVoltage();
					m.compressor_max_analog_voltage[i] = phs->getCompressorMaxAnalogVoltage();
					m.compressor_force_disable[i] = phs->getCompressorForceDisable();
					m.compressor_use_digital[i] = phs->getCompressorUseDigital();
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

} //namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ph_state_controller::PHStateController, controller_interface::ControllerBase)
