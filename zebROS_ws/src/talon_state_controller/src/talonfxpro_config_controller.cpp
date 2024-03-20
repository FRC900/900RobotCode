/*
 * Original joint_config_controller Author: Wim Meeussen
 */
#include <optional>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <ctre_interfaces/talonfxpro_state_interface.h>
#include <talon_state_msgs/TalonFXProConfig.h>
#include <periodic_interval_counter/periodic_interval_counter.h>

namespace talonfxpro_config_controller
{
/**
 * \brief Controller that publishes the state of all talon&victor motor controller on a robot.
 *
 * This controller publishes the state of all resources registered to a \c hardware_interface::TalonFXProStateInterface to a
 * topic of type \c talon_state_msgs/TalonFXProState. The following is a basic configuration of the controller.
 *
 * \code
 * talonfxpro_config_controller:
 *   type: talonfxpro_config_controller/TalonFXProConfigController
 *   publish_rate: 50
 * \endcode
 *
 */
class TalonFXProConfigController: public controller_interface::Controller<hardware_interface::talonfxpro::TalonFXProStateInterface>
{
private:
	std::vector<hardware_interface::talonfxpro::TalonFXProStateHandle> talonfxpro_state_;
	std::unique_ptr<realtime_tools::RealtimePublisher<talon_state_msgs::TalonFXProConfig> > realtime_pub_;
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	double publish_rate_;
	size_t num_hw_joints_; ///< Number of joints present in the TalonFXProStateInterface

public:
	bool init(hardware_interface::talonfxpro::TalonFXProStateInterface *hw,
			  ros::NodeHandle                                          &root_nh,
			  ros::NodeHandle                                          &controller_nh) override
	{
		// get all joint names from the hardware interface
		const std::vector<std::string> &joint_names = hw->getNames();
		num_hw_joints_ = joint_names.size();
		for (size_t i = 0; i < num_hw_joints_; i++)
			ROS_DEBUG("Got joint %s", joint_names[i].c_str());

		// get publishing period
		if (!controller_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_ERROR("Parameter 'publish_rate' not set in talon config controller");
			return false;
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish_rate in talon config controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		// realtime publisher
		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<talon_state_msgs::TalonFXProConfig>>(root_nh, "talonfxpro_configs", 4);

		auto &m = realtime_pub_->msg_;
		// get joints and allocate message
		for (size_t i = 0; i < num_hw_joints_; i++)
		{
			m.name.push_back(joint_names[i]);
			m.can_id.push_back(0);
			m.kp0.push_back(0);
			m.ki0.push_back(0);
			m.kd0.push_back(0);
			m.ks0.push_back(0);
			m.kv0.push_back(0);
			m.ka0.push_back(0);
			m.kg0.push_back(0);
			m.gravity_type0.emplace_back("Unknown");
			m.static_feedforward_sign0.emplace_back("Unknown");
			m.kp1.push_back(0);
			m.ki1.push_back(0);
			m.kd1.push_back(0);
			m.ks1.push_back(0);
			m.kv1.push_back(0);
			m.ka1.push_back(0);
			m.kg1.push_back(0);
			m.gravity_type1.emplace_back("Unknown");
			m.static_feedforward_sign1.emplace_back("Unknown");
			m.kp2.push_back(0);
			m.ki2.push_back(0);
			m.kd2.push_back(0);
			m.ks2.push_back(0);
			m.kv2.push_back(0);
			m.ka2.push_back(0);
			m.kg2.push_back(0);
			m.gravity_type2.emplace_back("Unknown");
			m.static_feedforward_sign2.emplace_back("Unknown");
			m.invert.emplace_back("");
			m.neutral_mode.emplace_back("");
			m.duty_cycle_neutral_deadband.push_back(0);
			m.peak_forward_duty_cycle.push_back(0);
			m.peak_reverse_duty_cycle.push_back(0);

			m.stator_current_limit.push_back(0);
			m.stator_current_limit_enable.push_back(false);

			m.supply_current_limit.push_back(0);
			m.supply_current_limit_enable.push_back(false);
			m.supply_current_threshold.push_back(0);
			m.supply_time_threshold.push_back(0);

			m.supply_voltage_time_constant.push_back(0);
			m.peak_forward_voltage.push_back(0);
			m.peak_reverse_voltage.push_back(0);

			m.peak_forward_torque_current.push_back(0);
			m.peak_reverse_torque_current.push_back(0);
			m.torque_neutral_deadband.push_back(0);

			m.feedback_rotor_offset.push_back(0);
			m.sensor_to_mechanism_ratio.push_back(0);
			m.rotor_to_sensor_ratio.push_back(0);
			m.feedback_sensor_source.emplace_back("");
			m.feedback_remote_sensor_id.push_back(0);

			m.differential_sensor_source.emplace_back("");
			m.differential_talonfx_sensor_id.push_back(0);
			m.differential_remote_sensor_id.push_back(0);

			m.peak_differential_duty_cycle.push_back(0);
			m.peak_differential_voltage.push_back(0);
			m.peak_differential_torque_current.push_back(0);

			m.duty_cycle_open_loop_ramp_period.push_back(0);
			m.voltage_open_loop_ramp_period.push_back(0);
			m.torque_open_loop_ramp_period.push_back(0);

			m.duty_cycle_closed_loop_ramp_period.push_back(0);
			m.voltage_closed_loop_ramp_period.push_back(0);
			m.torque_closed_loop_ramp_period.push_back(0);

			m.forward_limit_type.emplace_back("");
			m.forward_limit_autoset_position_enable.push_back(false);
			m.forward_limit_autoset_position_value.push_back(0);
			m.forward_limit_enable.push_back(false);
			m.forward_limit_source.emplace_back("");
			m.forward_limit_remote_sensor_id.push_back(0);

			m.reverse_limit_type.emplace_back("");
			m.reverse_limit_autoset_position_enable.push_back(false);
			m.reverse_limit_autoset_position_value.push_back(0);
			m.reverse_limit_enable.push_back(false);
			m.reverse_limit_source.emplace_back("");
			m.reverse_limit_remote_sensor_id.push_back(0);

			m.beep_on_boot.push_back(false);

			m.softlimit_forward_enable.push_back(false);
			m.softlimit_reverse_enable.push_back(false);
			m.softlimit_forward_threshold.push_back(0);
			m.softlimit_reverse_threshold.push_back(0);

			m.motion_magic_cruise_velocity.push_back(0);
			m.motion_magic_acceleration.push_back(0);
			m.motion_magic_jerk.push_back(0);
			m.motion_magic_expo_kV.push_back(0);
			m.motion_magic_expo_kA.push_back(0);

			m.continuous_wrap.push_back(false);
			m.enable_read_thread.push_back(false);

			talonfxpro_state_.push_back(hw->getHandle(joint_names[i]));
		}

		return true;
	}

	void starting(const ros::Time &time) override
	{
		interval_counter_->reset();
	}

	static std::string gravityTypeToString(const hardware_interface::talonfxpro::GravityType gravity_type)
	{
		if (gravity_type == hardware_interface::talonfxpro::GravityType::Elevator_Static)
		{
			return "elevator_static";
		}
		if (gravity_type == hardware_interface::talonfxpro::GravityType::Arm_Cosine)
		{
			return "arm_cosine";
		}
		return "Unknown";
	}

	static std::string staticFeedforwardSignToString(const hardware_interface::talonfxpro::StaticFeedforwardSign static_feedforward_sign)
	{
		if (static_feedforward_sign == hardware_interface::talonfxpro::StaticFeedforwardSign::UseVelocitySign)
		{
			return "UseVelocitySign";
		}
		if (static_feedforward_sign == hardware_interface::talonfxpro::StaticFeedforwardSign::UseClosedLoopSign)
		{
			return "UseClosedLoopSign";
		}
		return "Unknown";
	}

	static std::string differentialSensorSourceToString(const hardware_interface::talonfxpro::DifferentialSensorSource diffrential_sensor_source)
	{
		switch(diffrential_sensor_source)
		{
		case hardware_interface::talonfxpro::DifferentialSensorSource::Disabled:
			return "Disabled";
		case hardware_interface::talonfxpro::DifferentialSensorSource::RemoteTalonFX_Diff:
			return "RemoteTalonFX_Diff";
		case hardware_interface::talonfxpro::DifferentialSensorSource::RemotePigeon2_Yaw:
			return "RemotePigeon2_Yaw";
		case hardware_interface::talonfxpro::DifferentialSensorSource::RemotePigeon2_Pitch:
			return "RemotePigeon2_Pitch";
		case hardware_interface::talonfxpro::DifferentialSensorSource::RemotePigeon2_Roll:
			return "RemotePigeon2_Roll";
		case hardware_interface::talonfxpro::DifferentialSensorSource::RemoteCANcoder:
			return "RemoteCANcoder";
		}
		return "Unknown";
	}

	void update(const ros::Time &time, const ros::Duration &period) override
	{
		// limit rate of publishing
		if (interval_counter_->update(period))
		{
			// try to publish
			if (realtime_pub_->trylock())
			{
				// populate joint state message:
				auto &m = realtime_pub_->msg_;
				m.header.stamp = time;
				for (size_t i = 0; i < num_hw_joints_; i++)
				{
					auto const &ts = talonfxpro_state_[i];
					m.can_id[i] = ts->getCANID();
					m.kp0[i] = ts->getkP(0);
					m.ki0[i] = ts->getkI(0);
					m.kd0[i] = ts->getkD(0);
					m.ks0[i] = ts->getkS(0);
					m.kv0[i] = ts->getkV(0);
					m.ka0[i] = ts->getkA(0);
					m.kg0[i] = ts->getkG(0);
					m.gravity_type0[i] = gravityTypeToString(ts->getGravityType(0));
					m.static_feedforward_sign0[i] = staticFeedforwardSignToString(ts->getStaticFeedforwardSign(0));
					m.kp1[i] = ts->getkP(1);
					m.ki1[i] = ts->getkI(1);
					m.kd1[i] = ts->getkD(1);
					m.ks1[i] = ts->getkS(1);
					m.kv1[i] = ts->getkV(1);
					m.ka1[i] = ts->getkA(0);
					m.kg1[i] = ts->getkG(0);
					m.gravity_type1[i] = gravityTypeToString(ts->getGravityType(1));
					m.static_feedforward_sign1[i] = staticFeedforwardSignToString(ts->getStaticFeedforwardSign(1));
					m.kp2[i] = ts->getkP(2);
					m.ki2[i] = ts->getkI(2);
					m.kd2[i] = ts->getkD(2);
					m.ks2[i] = ts->getkS(2);
					m.kv2[i] = ts->getkV(2);
					m.ka2[i] = ts->getkA(0);
					m.kg2[i] = ts->getkG(0);
					m.gravity_type2[i] = gravityTypeToString(ts->getGravityType(2));
					m.static_feedforward_sign2[i] = staticFeedforwardSignToString(ts->getStaticFeedforwardSign(2));

					switch (ts->getInvert())
					{
					case hardware_interface::talonfxpro::Inverted::CounterClockwise_Positive:
						m.invert[i] = "CounterClockwise_Positive";
						break;
					case hardware_interface::talonfxpro::Inverted::Clockwise_Positive:
						m.invert[i] = "Clockwise_Positive";
						break;
					default:
						m.invert[i] = "Unknown";
						break;
					}
					switch (ts->getNeutralMode())
					{
					case hardware_interface::talonfxpro::NeutralMode::Brake:
						m.neutral_mode[i] = "Brake";
						break;
					case hardware_interface::talonfxpro::NeutralMode::Coast:
						m.neutral_mode[i] = "Coast";
						break;
					default:
						m.invert[i] = "Unknown";
						break;
					}
					m.duty_cycle_neutral_deadband[i] = ts->getDutyCycleNeutralDeadband();
					m.peak_forward_duty_cycle[i] = ts->getPeakForwardDutyCycle();
					m.peak_reverse_duty_cycle[i] = ts->getPeakReverseDutyCycle();

					m.stator_current_limit[i] = ts->getStatorCurrentLimit();
					m.stator_current_limit_enable[i] = ts->getStatorCurrentLimitEnable();

					m.supply_current_limit[i] = ts->getSupplyCurrentLimit();
					m.supply_current_limit_enable[i] = ts->getSupplyCurrentLimitEnable();
					m.supply_current_threshold[i] = ts->getSupplyCurrentThreshold();
					m.supply_time_threshold[i] = ts->getSupplyTimeThreshold();

					m.supply_voltage_time_constant[i] = ts->getSupplyVoltageTimeConstant();
					m.peak_forward_voltage[i] = ts->getPeakForwardVoltage();
					m.peak_reverse_voltage[i] = ts->getPeakReverseVoltage();

					m.peak_forward_torque_current[i] = ts->getPeakForwardTorqueCurrent();
					m.peak_reverse_torque_current[i] = ts->getPeakReverseTorqueCurrent();
					m.torque_neutral_deadband[i] = ts->getTorqueNeutralDeadband();

					m.feedback_rotor_offset[i] = ts->getFeedbackRotorOffset();
					m.sensor_to_mechanism_ratio[i] = ts->getSensorToMechanismRatio();
					m.rotor_to_sensor_ratio[i] = ts->getRotorToSensorRatio();
					switch (ts->getFeedbackSensorSource())
					{
					case hardware_interface::talonfxpro::FeedbackSensorSource::RotorSensor:
						m.feedback_sensor_source[i] = "RotorSensor";
						break;
					case hardware_interface::talonfxpro::FeedbackSensorSource::RemoteCANcoder:
						m.feedback_sensor_source[i] = "RemoteCANcoder";
						break;
					case hardware_interface::talonfxpro::FeedbackSensorSource::RemotePigeon2_Yaw:
						m.feedback_sensor_source[i] = "RemotePigeon2_Yaw";
						break;
					case hardware_interface::talonfxpro::FeedbackSensorSource::RemotePigeon2_Pitch:
						m.feedback_sensor_source[i] = "RemotePigeon2_Pitch";
						break;
					case hardware_interface::talonfxpro::FeedbackSensorSource::RemotePigeon2_Roll:
						m.feedback_sensor_source[i] = "RemotePigeon2_Roll";
						break;
					case hardware_interface::talonfxpro::FeedbackSensorSource::FusedCANcoder:
						m.feedback_sensor_source[i] = "FusedCANcoder";
						break;
					case hardware_interface::talonfxpro::FeedbackSensorSource::SyncCANcoder:
						m.feedback_sensor_source[i] = "SyncCANcoder";
						break;
					default:
						m.feedback_sensor_source[i] = "Unknown";
						break;
					}
					m.feedback_remote_sensor_id[i] = ts->getFeedbackRemoteSensorID();

					m.differential_sensor_source[i] = differentialSensorSourceToString(ts->getDifferentialSensorSource());
					m.differential_talonfx_sensor_id[i] = ts->getDifferentialTalonFXSensorID();
					m.differential_remote_sensor_id[i] = ts->getDifferentialRemoteSensorID();

					m.peak_differential_duty_cycle[i] = ts->getPeakDifferentialDutyCycle();
					m.peak_differential_voltage[i] = ts->getPeakDifferentialVoltage();
					m.peak_differential_torque_current[i] = ts->getPeakDifferentialTorqueCurrent();
					
					m.duty_cycle_open_loop_ramp_period[i] = ts->getDutyCycleOpenLoopRampPeriod();
					m.voltage_open_loop_ramp_period[i] = ts->getVoltageOpenLoopRampPeriod();
					m.torque_open_loop_ramp_period[i] = ts->getTorqueOpenLoopRampPeriod();

					m.duty_cycle_closed_loop_ramp_period[i] = ts->getDutyCycleOpenLoopRampPeriod();
					m.voltage_closed_loop_ramp_period[i] = ts->getVoltageClosedLoopRampPeriod();
					m.torque_closed_loop_ramp_period[i] = ts->getTorqueClosedLoopRampPeriod();

					auto limitTypeToString = [](const hardware_interface::talonfxpro::LimitType limit_type)
					{
						switch (limit_type)
						{
						case hardware_interface::talonfxpro::LimitType::NormallyOpen:
							return "NormallyOpen";
						case hardware_interface::talonfxpro::LimitType::NormallyClosed:
							return "NormallyClosed";
						default:
							return "Unknown";
						}
					};
					auto limitSourceToString = [](const hardware_interface::talonfxpro::LimitSource limit_source)
					{
						switch (limit_source)
						{
						case hardware_interface::talonfxpro::LimitSource::LimitSwitchPin:
							return "LimitSwitchPin";
						case hardware_interface::talonfxpro::LimitSource::RemoteTalonFX:
							return "RemoteTalonFX";
						case hardware_interface::talonfxpro::LimitSource::RemoteCANifier:
							return "RemoteCANifier";
						case hardware_interface::talonfxpro::LimitSource::RemoteCANcoder:
							return "RemoteCANcoder";
						case hardware_interface::talonfxpro::LimitSource::Disabled:
							return "Disabled";
						default:
							return "Unknown";
						}
					};
					m.forward_limit_type[i] = limitTypeToString(ts->getForwardLimitType());
					m.forward_limit_autoset_position_enable[i] = ts->getForwardLimitAutosetPositionEnable();
					m.forward_limit_autoset_position_value[i] = ts->getForwardLimitAutosetPositionValue();
					m.forward_limit_enable[i] = ts->getForwardLimitEnable();
					m.forward_limit_source[i] = limitSourceToString(ts->getForwardLimitSource());
					m.forward_limit_remote_sensor_id[i] = ts->getForwardLimitRemoteSensorID();

					m.reverse_limit_type[i] = limitTypeToString(ts->getReverseLimitType());
					m.reverse_limit_autoset_position_enable[i] = ts->getReverseLimitAutosetPositionEnable();
					m.reverse_limit_autoset_position_value[i] = ts->getReverseLimitAutosetPositionValue();
					m.reverse_limit_enable[i] = ts->getReverseLimitEnable();
					m.reverse_limit_source[i] = limitSourceToString(ts->getReverseLimitSource());
					m.reverse_limit_remote_sensor_id[i] = ts->getReverseLimitRemoteSensorID();

					m.beep_on_boot[i] = ts->getBeepOnBoot();

					m.softlimit_forward_enable[i] = ts->getForwardSoftLimitEnable();
					m.softlimit_reverse_enable[i] = ts->getReverseSoftLimitEnable();
					m.softlimit_forward_threshold[i] = ts->getForwardSoftLimitThreshold();
					m.softlimit_reverse_threshold[i] = ts->getReverseSoftLimitThreshold();

					m.motion_magic_cruise_velocity[i] = ts->getMotionMagicCruiseVelocity();
					m.motion_magic_acceleration[i] = ts->getMotionMagicAcceleration();
					m.motion_magic_jerk[i] = ts->getMotionMagicJerk();
					m.motion_magic_expo_kV[i] = ts->getMotionMagicExpoKV();
					m.motion_magic_expo_kA[i] = ts->getMotionMagicExpoKA();

					m.continuous_wrap[i] = ts->getContinuousWrap();
					m.enable_read_thread[i] = ts->getEnableReadThread();
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

} // namespate talonfxpro_config_controller

#if 0
namespace state_listener_controller
{
class TalonFXProConfigListenerController :
	public controller_interface::Controller<hardware_interface::talonfxpro::RemoteTalonFXProStateInterface>
{
private:
	ros::Subscriber sub_command_;
	std::vector<std::string> joint_names_;
	std::vector<hardware_interface::talonfxpro::TalonFXProWritableStateHandle> handles_;

	// Real-time buffer holds the last command value read from the
	// "command" topic.
	realtime_tools::RealtimeBuffer<std::vector<std::optional<hardware_interface::talonfxpro::TalonFXProHWState>>> command_buffer_;

	void commandCB(const talon_state_msgs::TalonFXProConfigConstPtr &msg)
	{
		// Each entry in data corresponds to an index in joint_names_
		// If the message has that name, copy the talon state into data
		// If the message doesn't have that name, set the entry to std::nullopt
		std::vector<std::optional<hardware_interface::talonfxpro::TalonFXProHWState>> data;
		for (size_t i = 0; i < joint_names_.size(); i++)
		{
			const auto it = std::find(msg->name.cbegin(), msg->name.cend(), joint_names_[i]);
			if (it != msg->name.cend())
			{
				const auto j = std::distance(msg->name.cbegin(), it);
				data.push_back(hardware_interface::TalonHWState(0)); // dummy CAN ID since it isn't used
				data[i]->setPosition(msg->position[j]);
				data[i]->setSpeed(msg->speed[j]);
				data[i]->setOutputCurrent(msg->output_voltage[j]);
				data[i]->setBusVoltage(msg->bus_voltage[j]);
				data[i]->setMotorOutputPercent(msg->motor_output_percent[j]);
				data[i]->setOutputVoltage(msg->output_voltage[j]);
				data[i]->setTemperature(msg->temperature[j]);
				data[i]->setClosedLoopError(msg->closed_loop_error[j]);
				data[i]->setIntegralAccumulator(msg->integral_accumulator[j]);
				data[i]->setErrorDerivative(msg->error_derivative[j]);
				data[i]->setClosedLoopTarget(msg->closed_loop_target[j]);
				data[i]->setActiveTrajectoryPosition(msg->active_trajectory_position[j]);
				data[i]->setActiveTrajectoryVelocity(msg->active_trajectory_velocity[j]);
				data[i]->setActiveTrajectoryHeading(msg->active_trajectory_heading[j]);
				data[i]->setMotionProfileTopLevelBufferCount(msg->motion_profile_top_level_buffer_count[j]);
				//data[i]->setFaults(msg->getFaults[j]);
				data[i]->setForwardLimitSwitch(msg->forward_limit_switch[j]);
				data[i]->setReverseLimitSwitch(msg->reverse_limit_switch[j]);
				data[i]->setForwardSoftlimitHit(msg->forward_softlimit[j]);
				data[i]->setReverseSoftlimitHit(msg->reverse_softlimit[j]);
				//data[i]->setStickyFaults(msg->getStickyFaults[j]);
			}
			else
			{
				data.push_back(std::nullopt);
			}
		}
		command_buffer_.writeFromNonRT(data);
	}

public:
	TalonFXProConfigListenerController() = default;
	~TalonFXProConfigListenerController()
	{
		sub_command_.shutdown();
	}

	bool init(hardware_interface::talonfxpro::RemoteTalonFXProStateInterface *hw, ros::NodeHandle &n)
	{
		// Read list of hw, make a list, grab handles for them, plus allocate storage space
		joint_names_ = hw->getNames();
		for (const auto &j : joint_names_)
		{
			ROS_INFO_STREAM("Joint State Listener Controller got joint " << j);
			handles_.push_back(hw->getHandle(j));
		}

		// get topic to subscribe to
		std::string topic;
		if (!n.getParam("topic", topic))
		{
			ROS_ERROR("Parameter 'topic' not set");
			return false;
		}

		sub_command_ = n.subscribe<talon_state_msgs::TalonFXProConfig>(topic, 1, &TalonFXProConfigListenerController::commandCB, this);
		return true;
	}

	void starting(const ros::Time & /*time*/)
	{
	}
	void stopping(const ros::Time & /*time*/)
	{
		//handles_.release();
	}

	void update(const ros::Time & /*time*/, const ros::Duration & /*period*/)
	{
		// Take the most recent set of values read from the joint_states
		// topic and write them to the local joints
		const auto vals = *command_buffer_.readFromRT();
		for (size_t i = 0; i < vals.size(); i++)
		{
			if (vals[i])
			{
				const auto &ts = (*vals[i]);
				handles_[i]->setPosition(ts.getPosition());
				handles_[i]->setSpeed(ts.getSpeed());
				handles_[i]->setOutputCurrent(ts.getOutputCurrent());
				handles_[i]->setBusVoltage(ts.getBusVoltage());
				handles_[i]->setMotorOutputPercent(ts.getMotorOutputPercent());
				handles_[i]->setOutputVoltage(ts.getOutputVoltage());
				handles_[i]->setTemperature(ts.getTemperature());
				handles_[i]->setClosedLoopError(ts.getClosedLoopError());
				handles_[i]->setIntegralAccumulator(ts.getIntegralAccumulator());
				handles_[i]->setErrorDerivative(ts.getErrorDerivative());
				handles_[i]->setClosedLoopTarget(ts.getClosedLoopTarget());
				handles_[i]->setActiveTrajectoryPosition(ts.getActiveTrajectoryPosition());
				handles_[i]->setActiveTrajectoryVelocity(ts.getActiveTrajectoryVelocity());
				handles_[i]->setActiveTrajectoryHeading(ts.getActiveTrajectoryHeading());
				handles_[i]->setMotionProfileTopLevelBufferCount(ts.getMotionProfileTopLevelBufferCount());
				handles_[i]->setFaults(ts.getFaults());
				handles_[i]->setForwardLimitSwitch(ts.getForwardLimitSwitch());
				handles_[i]->setReverseLimitSwitch(ts.getReverseLimitSwitch());
				handles_[i]->setForwardSoftlimitHit(ts.getForwardSoftlimitHit());
				handles_[i]->setReverseSoftlimitHit(ts.getReverseSoftlimitHit());
				handles_[i]->setStickyFaults(ts.getStickyFaults());
			}
		}
	}

}; // class

} // namespace state_listener_controller
#endif

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(talonfxpro_config_controller::TalonFXProConfigController, controller_interface::ControllerBase)
//PLUGINLIB_EXPORT_CLASS(state_listener_controller::TalonFXProConfigListenerController, controller_interface::ControllerBase)