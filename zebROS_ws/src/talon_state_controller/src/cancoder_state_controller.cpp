/*
 * Original joint_state_controller Author: Wim Meeussen
 */

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <ctre_interfaces/cancoder_state_interface.h>
#include <talon_state_msgs/CANCoderState.h>
#include <periodic_interval_counter/periodic_interval_counter.h>

namespace cancoder_state_controller
{
class CANCoderStateController: public controller_interface::Controller<hardware_interface::cancoder::CANCoderStateInterface>
{
private:
	std::vector<hardware_interface::cancoder::CANCoderStateHandle> cancoder_state_;
	std::unique_ptr<realtime_tools::RealtimePublisher<talon_state_msgs::CANCoderState>> realtime_pub_;
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	double publish_rate_;
	size_t num_hw_joints_; ///< Number of joints present in the CANCoderStateInterface

public:
	bool init(hardware_interface::cancoder::CANCoderStateInterface *hw,
			  ros::NodeHandle &root_nh,
			  ros::NodeHandle &controller_nh) override
	{
		// get all joint names from the hardware interface
		const std::vector<std::string> &joint_names = hw->getNames();
		num_hw_joints_ = joint_names.size();
		for (size_t i = 0; i < num_hw_joints_; i++)
			ROS_DEBUG("Got joint %s", joint_names[i].c_str());

		// get publishing period
		if (!controller_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_ERROR("Parameter 'publish_rate' not set in cancoder state controller");
			return false;
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish rate in cancoder state controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		// realtime publisher
		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<talon_state_msgs::CANCoderState>>(root_nh, "cancoder_states", 2);

		auto &m = realtime_pub_->msg_;

		// get joints and allocate message
		for (size_t i = 0; i < num_hw_joints_; i++)
		{
			m.name.push_back(joint_names[i]);
			cancoder_state_.push_back(hw->getHandle(joint_names[i]));

			m.device_number.push_back(cancoder_state_.back()->getDeviceNumber());

			m.sensor_direction.emplace_back();
			m.magnet_offset.push_back({0.0});
			m.absolute_sensor_discontinuity_point.push_back({});

			m.conversion_factor.push_back({1.0});

			m.version_major.push_back({});
			m.version_minor.push_back({});
			m.version_bugfix.push_back({});
			m.version_build.push_back({});
			m.velocity.push_back({});
			m.position.push_back({});
			m.absolute_position.push_back({});
			m.unfiltered_velocity.push_back({});
			m.position_since_boot.push_back({});
			m.supply_voltage.push_back({});
			m.magnet_health.emplace_back();

			m.fault_hardware.push_back({});
			m.fault_undervolage.push_back({});
			m.fault_boot_during_enable.push_back({});
			m.fault_unlicensed_feature_in_use.push_back({});
			m.fault_bad_magnet.push_back({});

			m.sticky_fault_hardware.push_back({});
			m.sticky_fault_undervolage.push_back({});
			m.sticky_fault_boot_during_enable.push_back({});
			m.sticky_fault_unlicensed_feature_in_use.push_back({});
			m.sticky_fault_bad_magnet.push_back({});
		}

		return true;
	}

	void starting(const ros::Time &time) override
	{
		interval_counter_->reset();
	}

	void update(const ros::Time &time, const ros::Duration & period) override
	{
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
					const auto &cs = cancoder_state_[i];

					switch (cs->getSensorDirection())
					{
					case hardware_interface::cancoder::SensorDirection::CounterClockwise_Positive:
						m.sensor_direction[i] = "CounterClockwise_Positive";
						break;
					case hardware_interface::cancoder::SensorDirection::Clockwise_Positive:
						m.sensor_direction[i] = "Clockwise_Positive";
						break;
					default:
						m.sensor_direction[i] = "Unknown";
						break;
					}
					m.magnet_offset[i] = cs->getMagnetOffset();
					m.absolute_sensor_discontinuity_point[i] = cs->getAbsoluteSensorDiscontinuityPoint();

					m.conversion_factor[i] = cs->getConversionFactor();

					m.version_major[i] = cs->getVersionMajor();
					m.version_minor[i] = cs->getVersionMinor();
					m.version_bugfix[i] = cs->getVersionBugfix();
					m.version_build[i] = cs->getVersionBuild();
					m.velocity[i] = cs->getVelocity();
					m.position[i] = cs->getPosition();
					m.absolute_position[i] = cs->getAbsolutePosition();
					m.unfiltered_velocity[i] = cs->getUnfilteredVelocity();
					m.position_since_boot[i] = cs->getPositionSinceBoot();
					m.supply_voltage[i] = cs->getSupplyVoltage();
					switch(cs->getMagnetHealth())
					{
						case hardware_interface::cancoder::MagnetHealth::Invalid:
							m.magnet_health[i] = "Invalid";
							break;
						case hardware_interface::cancoder::MagnetHealth::Green:
							m.magnet_health[i] = "Green";
							break;
						case hardware_interface::cancoder::MagnetHealth::Orange:
							m.magnet_health[i] = "Orange";
							break;
						case hardware_interface::cancoder::MagnetHealth::Red:
							m.magnet_health[i] = "Red";
							break;
						default:
							m.magnet_health[i] = "Unknown";
							break;
					}

					m.fault_hardware[i] = cs->getFaultHardware();
					m.fault_undervolage[i] = cs->getFaultUndervoltage();
					m.fault_boot_during_enable[i] = cs->getFaultBootDuringEnable();
					m.fault_unlicensed_feature_in_use[i] = cs->getFaultUnlicensedFeatureInUse();
					m.fault_bad_magnet[i] = cs->getFaultBadMagnet();

					m.sticky_fault_hardware[i] = cs->getStickyFaultHardware();
					m.sticky_fault_undervolage[i] = cs->getStickyFaultUndervoltage();
					m.sticky_fault_boot_during_enable[i] = cs->getStickyFaultBootDuringEnable();
					m.sticky_fault_unlicensed_feature_in_use[i] = cs->getStickyFaultUnlicensedFeatureInUse();
					m.sticky_fault_bad_magnet[i] = cs->getFaultBadMagnet();
					m.magnet_offset[i] = cs->getMagnetOffset();
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
	{
	}

}; // class

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cancoder_state_controller::CANCoderStateController, controller_interface::ControllerBase)
