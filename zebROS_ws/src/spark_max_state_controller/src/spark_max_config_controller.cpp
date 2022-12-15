/*
 * Original joint_state_controller Author: Wim Meeussen
 */

#include <algorithm>
#include <cstddef>

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <spark_max_interface/spark_max_state_interface.h>
#include <spark_max_state_msgs/SparkMaxConfig.h>
#include <periodic_interval_counter/periodic_interval_counter.h>

namespace spark_max_config_controller
{

/**
 * \brief Controller that publishes the config of all spark_maxs in a robot.
 *
 * This controller publishes the config of all resources registered to a \c hardware_interface::SparkMaxConfigInterface to a
 * topic of type \c sensor_msgs/SparkMaxConfig. The following is a basic configuration of the controller.
 *
 * \code
 * spark_max_config_controller:
 *   type: spark_max_config_controller/SparkMaxConfigController
 *   publish_rate: 50
 * \endcode
 *
 */
class SparkMaxConfigController: public controller_interface::Controller<hardware_interface::SparkMaxStateInterface>
{
private:
	std::vector<hardware_interface::SparkMaxStateHandle> spark_max_state_;
	std::shared_ptr<realtime_tools::RealtimePublisher<spark_max_state_msgs::SparkMaxConfig> > realtime_pub_;
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	double publish_rate_;
	size_t num_hw_joints_; ///< Number of joints present in the SparkMaxInterface

public : 
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
			ROS_ERROR("Parameter 'publish_rate' not set in spark max config controller");
			return false;
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish_rate in spark max config controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		// realtime publisher
		realtime_pub_.reset(new
							realtime_tools::RealtimePublisher<spark_max_state_msgs::SparkMaxConfig>(root_nh, "spark_max_configs", 1));

		auto &m = realtime_pub_->msg_;
		// get joints and allocate message

		for (size_t i = 0; i < num_hw_joints_; i++)
		{
			m.name.push_back(joint_names[i]);
			m.device_id.push_back(0);
			m.motor_type.push_back("");
			m.inverted.push_back(false);
			m.p0.push_back(0);
			m.i0.push_back(0);
			m.d0.push_back(0);
			m.f0.push_back(0);
			m.izone0.push_back(0);
			m.dfilter0.push_back(0);
			m.pidf_output_min0.push_back(0);
			m.pidf_output_max0.push_back(0);
			m.pidf_reference_output0.push_back(0);
			m.pidf_reference_ctrl0.push_back("");
			m.pidf_arb_feed_forward0.push_back(0);
			m.pidf_arb_feed_forward_units0.push_back("");
			m.p1.push_back(0);
			m.i1.push_back(0);
			m.d1.push_back(0);
			m.f1.push_back(0);
			m.izone1.push_back(0);
			m.dfilter1.push_back(0);
			m.pidf_output_min1.push_back(0);
			m.pidf_output_max1.push_back(0);
			m.pidf_reference_output1.push_back(0);
			m.pidf_reference_ctrl1.push_back("");
			m.pidf_arb_feed_forward1.push_back(0);
			m.pidf_arb_feed_forward_units1.push_back("");
			m.p2.push_back(0);
			m.i2.push_back(0);
			m.d2.push_back(0);
			m.f2.push_back(0);
			m.izone2.push_back(0);
			m.dfilter2.push_back(0);
			m.pidf_output_min2.push_back(0);
			m.pidf_output_max2.push_back(0);
			m.pidf_reference_output2.push_back(0);
			m.pidf_reference_ctrl2.push_back("");
			m.pidf_arb_feed_forward2.push_back(0);
			m.pidf_arb_feed_forward_units2.push_back("");
			m.p3.push_back(0);
			m.i3.push_back(0);
			m.d3.push_back(0);
			m.f3.push_back(0);
			m.izone3.push_back(0);
			m.dfilter3.push_back(0);
			m.pidf_output_min3.push_back(0);
			m.pidf_output_max3.push_back(0);
			m.pidf_reference_output3.push_back(0);
			m.pidf_reference_ctrl3.push_back("");
			m.pidf_arb_feed_forward3.push_back(0);
			m.pidf_arb_feed_forward_units3.push_back("");
			m.pidf_reference_slot.push_back(0);
			m.forward_limit_switch_polarity.push_back("");
			m.forward_limit_switch_enabled.push_back(false);
			m.reverse_limit_switch_polarity.push_back("");
			m.reverse_limit_switch_enabled.push_back(false);
			m.current_limit.push_back(0);
			m.current_limit_stall.push_back(0);
			m.current_limit_free.push_back(0);
			m.current_limit_rpm.push_back(0);
			m.secondary_current_limit.push_back(0);
			m.secondary_current_limit_cycles.push_back(0);
			m.idle_mode.push_back("");
			m.voltage_compensation_enable.push_back(false);
			m.voltage_compensation_nominal_voltage.push_back(0);
			m.open_loop_ramp_rate.push_back(0);
			m.closed_loop_ramp_rate.push_back(0);
			m.follower_type.push_back("");
			m.follower_id.push_back(0);
			m.follower_invert.push_back(false);
			m.forward_softlimit_enable.push_back(false);
			m.forward_softlimit.push_back(0);
			m.reverse_softlimit_enable.push_back(false);
			m.reverse_softlimit.push_back(0);
			m.encoder_ticks_per_rotation.push_back(0);
			m.encoder_type.push_back("");

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
				// populate config message:
				auto &m = realtime_pub_->msg_;
				m.header.stamp = time;
				for (size_t i = 0; i < num_hw_joints_; i++)
				{
					const auto &sms = spark_max_state_[i];
					m.device_id[i] = sms->getDeviceId();
					m.motor_type[i] = motorTypeToString(sms->getMotorType());
					m.inverted[i] = sms->getInverted();
					m.p0[i] = sms->getPGain(0);
					m.i0[i] = sms->getIGain(0);
					m.d0[i] = sms->getDGain(0);
					m.f0[i] = sms->getFGain(0);
					m.izone0[i] = sms->getIZone(0);
					m.dfilter0[i] = sms->getDFilter(0);
					m.pidf_output_min0[i] = sms->getPIDFOutputMin(0);
					m.pidf_output_max0[i] = sms->getPIDFOutputMax(0);
					m.pidf_reference_output0[i] = sms->getPIDFReferenceOutput(0);
					m.pidf_reference_ctrl0[i] = controlTypeToString(sms->getPIDFReferenceCtrl(0));
					m.pidf_arb_feed_forward0[i] = sms->getPIDFReferenceCtrl(0);
					m.pidf_arb_feed_forward_units0[i] = arbFFUnitsToString(sms->getPIDFArbFeedForwardUnits(0));
					m.p1[i] = sms->getPGain(1);
					m.i1[i] = sms->getIGain(1);
					m.d1[i] = sms->getDGain(1);
					m.f1[i] = sms->getFGain(1);
					m.izone1[i] = sms->getIZone(1);
					m.dfilter1[i] = sms->getDFilter(1);
					m.pidf_output_min1[i] = sms->getPIDFOutputMin(1);
					m.pidf_output_max1[i] = sms->getPIDFOutputMax(1);
					m.pidf_reference_output1[i] = sms->getPIDFReferenceOutput(1);
					m.pidf_reference_ctrl1[i] = controlTypeToString(sms->getPIDFReferenceCtrl(1));
					m.pidf_arb_feed_forward1[i] = sms->getPIDFReferenceCtrl(1);
					m.pidf_arb_feed_forward_units1[i] = arbFFUnitsToString(sms->getPIDFArbFeedForwardUnits(1));
					m.p2[i] = sms->getPGain(2);
					m.i2[i] = sms->getIGain(2);
					m.d2[i] = sms->getDGain(2);
					m.f2[i] = sms->getFGain(2);
					m.izone2[i] = sms->getIZone(2);
					m.dfilter2[i] = sms->getDFilter(2);
					m.pidf_output_min2[i] = sms->getPIDFOutputMin(2);
					m.pidf_output_max2[i] = sms->getPIDFOutputMax(2);
					m.pidf_reference_output2[i] = sms->getPIDFReferenceOutput(2);
					m.pidf_reference_ctrl2[i] = controlTypeToString(sms->getPIDFReferenceCtrl(2));
					m.pidf_arb_feed_forward2[i] = sms->getPIDFReferenceCtrl(2);
					m.pidf_arb_feed_forward_units2[i] = arbFFUnitsToString(sms->getPIDFArbFeedForwardUnits(2));
					m.p3[i] = sms->getPGain(3);
					m.i3[i] = sms->getIGain(3);
					m.d3[i] = sms->getDGain(3);
					m.f3[i] = sms->getFGain(3);
					m.izone3[i] = sms->getIZone(3);
					m.dfilter3[i] = sms->getDFilter(3);
					m.pidf_output_min3[i] = sms->getPIDFOutputMin(3);
					m.pidf_output_max3[i] = sms->getPIDFOutputMax(3);
					m.pidf_reference_output3[i] = sms->getPIDFReferenceOutput(3);
					m.pidf_reference_ctrl3[i] = controlTypeToString(sms->getPIDFReferenceCtrl(3));
					m.pidf_arb_feed_forward3[i] = sms->getPIDFReferenceCtrl(3);
					m.pidf_arb_feed_forward_units3[i] = arbFFUnitsToString(sms->getPIDFArbFeedForwardUnits(3));
					m.pidf_reference_slot[i] = sms->getPIDFReferenceSlot();

					m.forward_limit_switch_polarity[i] = limitSwitchPolarityToString(sms->getForwardLimitSwitchPolarity());
					m.forward_limit_switch_enabled[i] = sms->getForwardLimitSwitchEnabled();
					m.reverse_limit_switch_polarity[i] = limitSwitchPolarityToString(sms->getReverseLimitSwitchPolarity());
					m.reverse_limit_switch_enabled [i]= sms->getReverseLimitSwitchEnabled();
					m.current_limit[i] = sms->getCurrentLimit();
					m.current_limit_free[i] = sms->getCurrentLimitFree();
					m.current_limit_stall[i] = sms->getCurrentLimitStall();
					m.current_limit_rpm[i] = sms->getCurrentLimitRPM();
					m.secondary_current_limit[i] = sms->getSecondaryCurrentLimit();
					m.secondary_current_limit_cycles[i] = sms->getSecondaryCurrentLimitCycles();

					m.idle_mode[i] = idleModeToString(sms->getIdleMode());
					m.voltage_compensation_enable[i] = sms->getVoltageCompensationEnable();
					m.voltage_compensation_nominal_voltage[i] = sms->getVoltageCompensationNominalVoltage();
					m.open_loop_ramp_rate[i] = sms->getOpenLoopRampRate();
					m.closed_loop_ramp_rate[i] = sms->getClosedLoopRampRate();
					m.follower_type[i] = externalFollowerToString(sms->getFollowerType());
					m.follower_id[i] = sms->getFollowerID();
					m.follower_invert[i] = sms->getFollowerInvert();
					m.forward_softlimit_enable[i] = sms->getForwardSoftlimitEnable();
					m.forward_softlimit[i] = sms->getForwardSoftlimit();
					m.reverse_softlimit_enable[i] = sms->getReverseSoftlimitEnable();
					m.reverse_softlimit[i] = sms->getReverseSoftlimit();
					m.encoder_ticks_per_rotation[i] = sms->getEncoderTicksPerRotation();
					m.encoder_type[i] = sensorTypeToString(sms->getEncoderType());
				}
				realtime_pub_->unlockAndPublish();
			}
			else
			{
				interval_counter_.reset();
			}
		}
	}

	void stopping(const ros::Time & /*time*/)
	{}

private:
	std::string motorTypeToString(hardware_interface::MotorType motor_type) const
	{
		switch (motor_type)
		{
			case hardware_interface::kBrushed:
				return "Brushed";
			case hardware_interface::kBrushless:
				return "Brushless";
			default:
				return "Unknown";
		}
	}

	std::string controlTypeToString(hardware_interface::ControlType control_type) const
	{
		switch (control_type)
		{
			case hardware_interface::kDutyCycle:
				return "DutyCycle";
			case hardware_interface::kVelocity:
				return "Velocity";
			case hardware_interface::kVoltage:
				return "Voltage";
			case hardware_interface::kPosition:
				return "Position";
			case hardware_interface::kSmartMotion:
				return "SmartMotion";
			case hardware_interface::kCurrent:
				return "Current";
			case hardware_interface::kSmartVelocity:
				return "SmartVelocity";

			default:
				return "Unknown";
		}
	}

	std::string arbFFUnitsToString(hardware_interface::ArbFFUnits arb_ff_units) const
	{
		switch (arb_ff_units)
		{
			case hardware_interface::ArbFFUnits::kVoltage:
				return "Voltage";
			case hardware_interface::ArbFFUnits::kPercentOut:
				return "PercentOut";
			default:
				return "Unknown";
		}
	}

	std::string limitSwitchPolarityToString(hardware_interface::LimitSwitchPolarity limit_switch_polarity) const
	{
		switch (limit_switch_polarity)
		{
			case hardware_interface::kNormallyOpen:
				return "NormallyOpen";
			case hardware_interface::kNormallyClosed:
				return "NormallyClosed";
			default:
				return "Unknown";
		}
	}

	std::string idleModeToString(hardware_interface::IdleMode idle_mode) const
	{
		switch (idle_mode)
		{
			case hardware_interface::kCoast:
				return "Coast";
			case hardware_interface::kBrake:
				return "Brake";
			default:
				return "Unknown";
		}
	}

	std::string externalFollowerToString(hardware_interface::ExternalFollower external_follower) const
	{
		switch (external_follower)
		{
			case hardware_interface::kFollowerDisabled:
				return "FollowerDisabled";
			case hardware_interface::kFollowerSparkMax:
				return "FollowerSparkMax";
			case hardware_interface::kFollowerPhoenix:
				return "FollowerPhoenix";
			default:
				return "Unknown";
		}
	}

	std::string sensorTypeToString(hardware_interface::SensorType sensor_type) const
	{
		switch (sensor_type)
		{
			case hardware_interface::kNoSensor :
				return "NoSensor";
			case hardware_interface::kHallSensor :
				return "HallSensor";
			case hardware_interface::kQuadrature :
				return "Quadrature";
			default:
				return "Unknown";
		}
	}

}; // class

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(spark_max_config_controller::SparkMaxConfigController, controller_interface::ControllerBase)
