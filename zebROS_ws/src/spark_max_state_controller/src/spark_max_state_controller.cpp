/*
 * Original joint_state_controller Author: Wim Meeussen
 */
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <spark_max_interface/spark_max_state_interface.h>
#include <spark_max_state_msgs/SparkMaxState.h>
#include <periodic_interval_counter/periodic_interval_counter.h>

namespace spark_max_state_controller
{
/**
 * \brief Controller that publishes the state of all spark_max&victor motor controller on a robot.
 *
 * This controller publishes the state of all resources registered to a \c hardware_interface::SparkMaxStateInterface to a
 * topic of type \c sensor_msgs/SparkMaxState. The following is a basic configuration of the controller.
 *
 * \code
 * spark_max_state_controller:
 *   type: spark_max_state_controller/JointStateController
 *   publish_rate: 50
 * \endcode
 *
 */
class SparkMaxStateController: public controller_interface::Controller<hardware_interface::SparkMaxStateInterface>
{
private:
	std::vector<hardware_interface::SparkMaxStateHandle> spark_max_state_;
	std::shared_ptr<realtime_tools::RealtimePublisher<spark_max_state_msgs::SparkMaxState> > realtime_pub_;
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	double publish_rate_;
	size_t num_hw_joints_; ///< Number of joints present in the SparkMaxStateInterface

public:
	bool init(hardware_interface::SparkMaxStateInterface *hw,
			  ros::NodeHandle                            &root_nh,
			  ros::NodeHandle                            &controller_nh)
	{
		// get all joint names from the hardware interface
		const std::vector<std::string> &joint_names = hw->getNames();
		num_hw_joints_ = joint_names.size();
		for (size_t i = 0; i < num_hw_joints_; i++)
			ROS_DEBUG("Got joint %s", joint_names[i].c_str());

		// get publishing period
		if (!controller_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_ERROR("Parameter 'publish_rate' not set in spark max state controller");
			return false;
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish_rate in spark max state controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		// realtime publisher
		realtime_pub_.reset(new
							realtime_tools::RealtimePublisher<spark_max_state_msgs::SparkMaxState>(root_nh, "spark_max_states", 2));

		auto &m = realtime_pub_->msg_;
		// get joints and allocate message

		for (size_t i = 0; i < num_hw_joints_; i++)
		{
			m.name.push_back(joint_names[i]);
			m.device_id.push_back(0);
			m.set_point.push_back(0);
			m.position.push_back(0);
			m.velocity.push_back(0);
			m.forward_limit_switch.push_back(false);
			m.reverse_limit_switch.push_back(false);
			m.forward_softlimit.push_back(false);
			m.reverse_softlimit.push_back(false);

			m.faults.push_back("");
			m.sticky_faults.push_back("");

			m.bus_voltage.push_back(0);
			m.applied_output.push_back(0);
			m.output_current.push_back(0);
			m.motor_temperature.push_back(0);

			spark_max_state_.push_back(hw->getHandle(joint_names[i]));
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
		// limit rate of publishing
		if (interval_counter_->update(period))
		{
			// try to publish
			if (realtime_pub_->trylock())
			{
				// populate joint state message:
				// - fill only joints that are present in the JointStateInterface, i.e. indices [0, num_hw_joints_)
				auto &m = realtime_pub_->msg_;
				m.header.stamp = time;
				for (size_t i = 0; i < num_hw_joints_; i++)
				{
					const auto &sms = spark_max_state_[i];
					m.device_id[i] = sms->getDeviceId();
					m.set_point[i] = sms->getSetPoint();
					m.position[i] = sms->getPosition();
					m.velocity[i] = sms->getVelocity();
					m.forward_limit_switch[i] = sms->getForwardLimitSwitch();
					m.reverse_limit_switch[i] = sms->getReverseLimitSwitch();
					m.forward_softlimit[i] = sms->getForwardSoftlimit();
					m.reverse_softlimit[i] = sms->getReverseSoftlimit();
					m.faults[i] = faultsToString(sms->getFaults());
					m.sticky_faults[i] = faultsToString(sms->getStickyFaults());
					m.bus_voltage[i] = sms->getBusVoltage();
					m.applied_output[i] = sms->getAppliedOutput();
					m.output_current[i] = sms->getOutputCurrent();
					m.motor_temperature[i] = sms->getMotorTemperature();
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

private:
	std::string faultsToString(unsigned faults) const
	{
		std::string str;
		if (faults)
		{
			unsigned mask = 1;
			if (faults & mask) str += "Brownout ";
			mask <<= 1;
			if (faults & mask) str += "Overcurrent ";
			mask <<= 1;
			if (faults & mask) str += "IWDTReset ";
			mask <<= 1;
			if (faults & mask) str += "MotorFault ";
			mask <<= 1;
			if (faults & mask) str += "SensorFault ";
			mask <<= 1;
			if (faults & mask) str += "Stall ";
			mask <<= 1;
			if (faults & mask) str += "EEPROMCRC ";
			mask <<= 1;
			if (faults & mask) str += "CANTX ";
			mask <<= 1;
			if (faults & mask) str += "CANRX ";
			mask <<= 1;
			if (faults & mask) str += "HasReset ";
			mask <<= 1;
			if (faults & mask) str += "DRVFault ";
			mask <<= 1;
			if (faults & mask) str += "OtherFault ";
			mask <<= 1;
			if (faults & mask) str += "SoftLimitFwd ";
			mask <<= 1;
			if (faults & mask) str += "SoftLimitRev ";
			mask <<= 1;
			if (faults & mask) str += "HardLimitFwd ";
			mask <<= 1;
			if (faults & mask) str += "HardLimitRev ";
		}
		return str;
	}
}; // class

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(spark_max_state_controller::SparkMaxStateController, controller_interface::ControllerBase)