/*
 * Original joint_state_controller Author: Wim Meeussen
 */
#include <optional>
#include <controller_interface/controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ctre_interfaces/talonfxpro_state_interface.h>
#include <talon_state_msgs/TalonFXProState.h>
#include <periodic_interval_counter/periodic_interval_counter.h>

namespace talonfxpro_state_controller
{
/**
 * \brief Controller that publishes the state of all talon&victor motor controller on a robot.
 *
 * This controller publishes the state of all resources registered to a \c hardware_interface::TalonFXProStateInterface to a
 * topic of type \c talon_state_msgs/TalonFXProState. The following is a basic configuration of the controller.
 *
 * \code
 * talonfxpro_state_controller:
 *   type: talonfxpro_state_controller/TalonFXProStateController
 *   publish_rate: 50
 * \endcode
 *
 */
class TalonFXProStateController: public controller_interface::Controller<hardware_interface::talonfxpro::TalonFXProStateInterface>
{
private:
	std::vector<hardware_interface::talonfxpro::TalonFXProStateHandle> talonfxpro_state_;
	std::unique_ptr<realtime_tools::RealtimePublisher<talon_state_msgs::TalonFXProState> > realtime_pub_;
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
			ROS_ERROR("Parameter 'publish_rate' not set in talon state controller");
			return false;
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish_rate in talon state controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		// realtime publisher
		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<talon_state_msgs::TalonFXProState>>(root_nh, "talonfxpro_states", 4);

		auto &m = realtime_pub_->msg_;
		// get joints and allocate message
		for (size_t i = 0; i < num_hw_joints_; i++)
		{
			m.name.push_back(joint_names[i]);
			m.control_mode.emplace_back("");
			m.control_output.push_back(0);
			m.control_position.push_back(0);
			m.control_velocity.push_back(0);
			m.control_acceleration.push_back(0);
			m.control_jerk.push_back(0);
			m.control_enable_foc.push_back(false);
			m.control_override_brake_dur_neutral.push_back(false);
			m.control_oppose_master_direction.push_back(false);
			m.control_max_abs_duty_cycle.push_back(0);
			m.control_deadband.push_back(0);
			m.control_feedforward.push_back(0);
			m.control_slot.push_back(0);
			m.control_limit_forward_motion.push_back(false);
			m.control_limit_reverse_motion.push_back(false);
            m.control_differential_position.push_back(0);
			m.control_differential_slot.push_back(0);
			m.version_major.push_back(0);
			m.version_minor.push_back(0);
			m.version_bugfix.push_back(0);
			m.version_build.push_back(0);
			m.motor_voltage.push_back(0);
			m.forward_limit.push_back(false);
			m.reverse_limit.push_back(false);
			m.applied_rotor_polarity.emplace_back("");

			m.duty_cycle.push_back(0);
			m.torque_current.push_back(0);
			m.stator_current.push_back(0);
			m.supply_current.push_back(0);
			m.supply_voltage.push_back(0);
			m.device_temp.push_back(0);
			m.processor_temp.push_back(0);

			m.rotor_velocity.push_back(0);
			m.rotor_position.push_back(0);
			m.velocity.push_back(0);
			m.position.push_back(0);
			m.acceleration.push_back(0);

			m.motion_magic_is_running.push_back(false);

			m.device_enable.push_back(false);

			m.differential_control_mode.emplace_back("");
			m.differential_average_velocity.push_back(0);
			m.differential_average_position.push_back(0);
			m.differential_difference_velocity.push_back(0);
			m.differential_difference_position.push_back(0);

			m.bridge_output_value.emplace_back("");

			m.fault_hardware.push_back(false);
			m.fault_proctemp.push_back(false);
			m.fault_devicetemp.push_back(false);
			m.fault_undervoltage.push_back(false);
			m.fault_bootduringenable.push_back(false);
			m.fault_bridgebrownout.push_back(false);
			m.fault_unlicensedfeatureinuse.push_back(false);
			m.fault_remotesensorreset.push_back(false);
			m.fault_missingdifferentialfx.push_back(false);
			m.fault_remotesensorposoverflow.push_back(false);
			m.fault_oversupplyv.push_back(false);
			m.fault_unstablesupplyv.push_back(false);
			m.fault_reversehardlimit.push_back(false);
			m.fault_forwardhardlimit.push_back(false);
			m.fault_reversesoftlimit.push_back(false);
			m.fault_forwardsoftlimit.push_back(false);
			m.fault_remotesensordatainvalid.push_back(false);
			m.fault_fusedsensoroutofsync.push_back(false);
			m.fault_statorcurrlimit.push_back(false);
			m.fault_supplycurrlimit.push_back(false);

			m.sticky_fault_hardware.push_back(false);
			m.sticky_fault_proctemp.push_back(false);
			m.sticky_fault_devicetemp.push_back(false);
			m.sticky_fault_undervoltage.push_back(false);
			m.sticky_fault_bootduringenable.push_back(false);
			m.sticky_fault_bridgebrownout.push_back(false);
			m.sticky_fault_unlicensedfeatureinuse.push_back(false);
			m.sticky_fault_remotesensorreset.push_back(false);
			m.sticky_fault_missingdifferentialfx.push_back(false);
			m.sticky_fault_remotesensorposoverflow.push_back(false);
			m.sticky_fault_oversupplyv.push_back(false);
			m.sticky_fault_unstablesupplyv.push_back(false);
			m.sticky_fault_reversehardlimit.push_back(false);
			m.sticky_fault_forwardhardlimit.push_back(false);
			m.sticky_fault_reversesoftlimit.push_back(false);
			m.sticky_fault_forwardsoftlimit.push_back(false);
			m.sticky_fault_remotesensordatainvalid.push_back(false);
			m.sticky_fault_fusedsensoroutofsync.push_back(false);
			m.sticky_fault_statorcurrlimit.push_back(false);
			m.sticky_fault_supplycurrlimit.push_back(false);

			m.closed_loop_proportional_output.push_back(0);
			m.closed_loop_integrated_output.push_back(0);
			m.closed_loop_feed_forward.push_back(0);
			m.closed_loop_derivative_output.push_back(0);
			m.closed_loop_output.push_back(0);
			m.closed_loop_reference.push_back(0);
			m.closed_loop_reference_slope.push_back(0);
			m.closed_loop_error.push_back(0);

            m.differential_output.push_back(0);
			m.differential_closed_loop_proportional_output.push_back(0);
			m.differential_closed_loop_integrated_output.push_back(0);
			m.differential_closed_loop_feed_forward.push_back(0);
			m.differential_closed_loop_derivative_output.push_back(0);
			m.differential_closed_loop_output.push_back(0);
			m.differential_closed_loop_reference.push_back(0);
			m.differential_closed_loop_reference_slope.push_back(0);
			m.differential_closed_loop_error.push_back(0);

			talonfxpro_state_.push_back(hw->getHandle(joint_names[i]));
		}

		return true;
	}

	void starting(const ros::Time &time) override
	{
		interval_counter_->reset();
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
				// - fill only joints that are present in the JointStateInterface, i.e. indices [0, num_hw_joints_)
				// - leave unchanged extra joints, which have static values, i.e. indices from num_hw_joints_ onwards
				auto &m = realtime_pub_->msg_;
				m.header.stamp = time;
				for (size_t i = 0; i < num_hw_joints_; i++)
				{
					auto const &ts = talonfxpro_state_[i];
					switch(ts->getControlMode())
					{
					case hardware_interface::talonfxpro::TalonMode::DutyCycleOut:
						m.control_mode[i] = "DutyCycleOut";
						break;
					case hardware_interface::talonfxpro::TalonMode::TorqueCurrentFOC:
						m.control_mode[i] = "TorqueCurrentFOC";
						break;
					case hardware_interface::talonfxpro::TalonMode::VoltageOut:
						m.control_mode[i] = "VoltageOut";
						break;
					case hardware_interface::talonfxpro::TalonMode::PositionDutyCycle:
						m.control_mode[i] = "PositionDutyCycle";
						break;
					case hardware_interface::talonfxpro::TalonMode::PositionVoltage:
						m.control_mode[i] = "PositionVoltage";
						break;
					case hardware_interface::talonfxpro::TalonMode::PositionTorqueCurrentFOC:
						m.control_mode[i] = "PositionTorqueCurrentFOC";
						break;
					case hardware_interface::talonfxpro::TalonMode::VelocityDutyCycle:
						m.control_mode[i] = "VelocityDutyCycle";
						break;
					case hardware_interface::talonfxpro::TalonMode::VelocityVoltage:
						m.control_mode[i] = "VelocityVoltage";
						break;
					case hardware_interface::talonfxpro::TalonMode::VelocityTorqueCurrentFOC:
						m.control_mode[i] = "VelocityTorqueCurrentFOC";
						break;
					case hardware_interface::talonfxpro::TalonMode::MotionMagicDutyCycle:
						m.control_mode[i] = "MotionMagicDutyCycle";
						break;
					case hardware_interface::talonfxpro::TalonMode::MotionMagicVoltage:
						m.control_mode[i] = "MotionMagicVoltage";
						break;
					case hardware_interface::talonfxpro::TalonMode::MotionMagicTorqueCurrentFOC:
						m.control_mode[i] = "MotionMagicTorqueCurrentFOC";
						break;
					case hardware_interface::talonfxpro::TalonMode::MotionMagicExpoDutyCycle:
						m.control_mode[i] = "MotionMagicExpoDutyCycle";
						break;
					case hardware_interface::talonfxpro::TalonMode::MotionMagicExpoVoltage:
						m.control_mode[i] = "MotionMagicExpoVoltage";
						break;
					case hardware_interface::talonfxpro::TalonMode::MotionMagicExpoTorqueCurrentFOC:
						m.control_mode[i] = "MotionMagicExpoTorqueCurrentFOC";
						break;
					case hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityDutyCycle:
						m.control_mode[i] = "MotionMagicVelocityDutyCycle";
						break;
					case hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityVoltage:
						m.control_mode[i] = "MotionMagicVelocityVoltage";
						break;
					case hardware_interface::talonfxpro::TalonMode::MotionMagicVelocityTorqueCurrentFOC:
						m.control_mode[i] = "MotionMagicVelocityTorqueCurrentFOC";
						break;
					case hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicDutyCycle:
						m.control_mode[i] = "DynamicMotionMagicDutyCycle";
						break;
					case hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicVoltage:
						m.control_mode[i] = "DynamicMotionMagicVoltage";
						break;
					case hardware_interface::talonfxpro::TalonMode::DynamicMotionMagicTorqueCurrentFOC:
						m.control_mode[i] = "DynamicMotionMagicTorqueCurrentFOC";
						break;
					case hardware_interface::talonfxpro::TalonMode::Follower:
						m.control_mode[i] = "Follower";
						break;
					case hardware_interface::talonfxpro::TalonMode::StrictFollower:
						m.control_mode[i] = "StrictFollower";
						break;
					case hardware_interface::talonfxpro::TalonMode::NeutralOut:
						m.control_mode[i] = "NeutralOut";
						break;
					case hardware_interface::talonfxpro::TalonMode::CoastOut:
						m.control_mode[i] = "CoastOut";
						break;
					case hardware_interface::talonfxpro::TalonMode::StaticBrake:
						m.control_mode[i] = "StaticBrake";
						break;
					case hardware_interface::talonfxpro::TalonMode::DifferentialDutyCycleOut:
						m.control_mode[i] = "DifferentialDutyCycleOut";
						break;
					case hardware_interface::talonfxpro::TalonMode::DifferentialVoltageOut:
						m.control_mode[i] = "DifferentialVoltageOut";
						break;
					case hardware_interface::talonfxpro::TalonMode::DifferentialPositionDutyCycle:
						m.control_mode[i] = "DifferentialPositionDutyCycle";
						break;
					case hardware_interface::talonfxpro::TalonMode::DifferentialPositionVoltage:
						m.control_mode[i] = "DifferentialPositionVoltage";
						break;
					case hardware_interface::talonfxpro::TalonMode::DifferentialVelocityDutyCycle:
						m.control_mode[i] = "DifferentialVelocityDutyCycle";
						break;
					case hardware_interface::talonfxpro::TalonMode::DifferentialVelocityVoltage:
						m.control_mode[i] = "DifferentialVelocityVoltage";
						break;
					case hardware_interface::talonfxpro::TalonMode::DifferentialMotionMagicDutyCycle:
						m.control_mode[i] = "DifferentialMotionMagicDutyCycle";
						break;
					case hardware_interface::talonfxpro::TalonMode::DifferentialMotionMagicVoltage:
						m.control_mode[i] = "DifferentialMotionMagicVoltage";
						break;
					case hardware_interface::talonfxpro::TalonMode::DifferentialFollower:
						m.control_mode[i] = "DifferentialFollower";
						break;
					case hardware_interface::talonfxpro::TalonMode::DifferentialStrictFollower:
						m.control_mode[i] = "DifferentialStrictFollower";
						break;
					case hardware_interface::talonfxpro::TalonMode::Disabled:
						m.control_mode[i] = "Disabled";
						break;
					default:
						m.control_mode[i] = "Unknown";
						break;
					}
					m.control_output[i] = ts->getControlOutput();
					m.control_position[i] = ts->getControlPosition();
					m.control_velocity[i] = ts->getControlVelocity();
					m.control_acceleration[i] = ts->getControlAcceleration();
					m.control_jerk[i] = ts->getControlJerk();
					m.control_enable_foc[i] = ts->getControlEnableFOC();
					m.control_override_brake_dur_neutral[i] = ts->getControlOverrideBrakeDurNeutral();
					m.control_oppose_master_direction[i] = ts->getControlOpposeMasterDirection();
					m.control_max_abs_duty_cycle[i] = ts->getControlMaxAbsDutyCycle();
					m.control_deadband[i] = ts->getControlDeadband();
					m.control_feedforward[i] = ts->getControlFeedforward();
					m.control_slot[i] = ts->getControlSlot();
					m.control_limit_forward_motion[i] = ts->getControlLimitForwardMotion();
					m.control_limit_reverse_motion[i] = ts->getControlLimitReverseMotion();
					m.control_differential_position[i] = ts->getControlDifferentialPosition();
					m.control_differential_slot[i] = ts->getControlDifferentialSlot();
					m.version_major[i] = ts->getVersionMajor();
					m.version_minor[i] = ts->getVersionMinor();
					m.version_bugfix[i] = ts->getVersionBugfix();
					m.version_build[i] = ts->getVersionBuild();
					m.motor_voltage[i] = ts->getMotorVoltage();
					m.forward_limit[i] = ts->getForwardLimit();
					m.reverse_limit[i] = ts->getReverseLimit();
					switch(ts->getAppliedRotorPolarity())
					{
						case hardware_interface::talonfxpro::Inverted::CounterClockwise_Positive:
						m.applied_rotor_polarity[i] = "CounterClockwise_Positive";
						break;
						case hardware_interface::talonfxpro::Inverted::Clockwise_Positive:
						m.applied_rotor_polarity[i] = "Clockwise_Positive";
						break;
						default:
						m.applied_rotor_polarity[i] = "Unknown";
						break;
					}

					m.duty_cycle[i] = ts->getDutyCycle();
					m.torque_current[i] = ts->getTorqueCurrent();
					m.stator_current[i] = ts->getStatorCurrent();
					m.supply_current[i] = ts->getSupplyCurrent();
					m.supply_voltage[i] = ts->getSupplyVoltage();
					m.device_temp[i] = ts->getDeviceTemp();
					m.processor_temp[i] = ts->getProcessorTemp();

					m.rotor_velocity[i] = ts->getRotorVelocity();
					m.rotor_position[i] = ts->getRotorPosition();
					m.velocity[i] = ts->getVelocity();
					m.position[i] = ts->getPosition();
					m.acceleration[i] = ts->getAcceleration();

					m.motion_magic_is_running[i] = ts->getMotionMagicIsRunning();

					switch(ts->getDifferentialControlMode())
					{
					case hardware_interface::talonfxpro::DifferentialControlMode::DisabledOutput:
						m.differential_control_mode[i] = "Disabled";
						break;
					case hardware_interface::talonfxpro::DifferentialControlMode::DutyCycleOut:
						m.differential_control_mode[i] = "DutyCycleOut";
						break;
					case hardware_interface::talonfxpro::DifferentialControlMode::TorqueCurrentFOC:
						m.differential_control_mode[i] = "TorqueCurrentFOC";
						break;
					case hardware_interface::talonfxpro::DifferentialControlMode::VoltageOut:
						m.differential_control_mode[i] = "VoltageOut";
						break;
					case hardware_interface::talonfxpro::DifferentialControlMode::PositionDutyCycle:
						m.differential_control_mode[i] = "PositionDutyCycle";
						break;
					case hardware_interface::talonfxpro::DifferentialControlMode::PositionVoltage:
						m.differential_control_mode[i] = "PositionVoltage";
						break;
					case hardware_interface::talonfxpro::DifferentialControlMode::PositionTorqueCurrentFOC:
						m.differential_control_mode[i] = "PositionTorqueCurrentFOC";
						break;
					case hardware_interface::talonfxpro::DifferentialControlMode::VelocityDutyCycle:
						m.differential_control_mode[i] = "VelocityDutyCycle";
						break;
					case hardware_interface::talonfxpro::DifferentialControlMode::VelocityVoltage:
						m.differential_control_mode[i] = "VelocityVoltage";
						break;
					case hardware_interface::talonfxpro::DifferentialControlMode::VelocityTorqueCurrentFOC:
						m.differential_control_mode[i] = "VelocityTorqueCurrentFOC";
						break;
					case hardware_interface::talonfxpro::DifferentialControlMode::MotionMagicDutyCycle:
						m.differential_control_mode[i] = "MotionMagicDutyCycle";
						break;
					case hardware_interface::talonfxpro::DifferentialControlMode::MotionMagicVoltage:
						m.differential_control_mode[i] = "MotionMagicVoltage";
						break;
					case hardware_interface::talonfxpro::DifferentialControlMode::MotionMagicTorqueCurrentFOC:
						m.differential_control_mode[i] = "MotionMagicTorqueCurrentFOC";
						break;
					case hardware_interface::talonfxpro::DifferentialControlMode::Follower:
						m.differential_control_mode[i] = "Follower";
						break;
					case hardware_interface::talonfxpro::DifferentialControlMode::NeutralOut:
						m.differential_control_mode[i] = "NeutralOut";
						break;
					case hardware_interface::talonfxpro::DifferentialControlMode::CoastOut:
						m.differential_control_mode[i] = "CoastOut";
						break;
					case hardware_interface::talonfxpro::DifferentialControlMode::StaticBrake:
						m.differential_control_mode[i] = "StaticBrake";
						break;
					default:
						m.differential_control_mode[i] = "Unknown";
						break;
					}
					m.differential_average_velocity[i] = ts->getDifferentialAverageVelocity();
					m.differential_average_position[i] = ts->getDifferentialAveragePosition();
					m.differential_difference_velocity[i] = ts->getDifferentialDifferenceVelocity();
					m.differential_difference_position[i] = ts->getDifferentialDifferencePosition();

					m.device_enable[i] = ts->getDeviceEnable();

					m.bridge_output_value[i] = ""; // todo
					switch(ts->getBridgeOutput())
					{
					case hardware_interface::talonfxpro::BridgeOutput::Coast:
						m.bridge_output_value[i] = "Coast";
						break;
					case hardware_interface::talonfxpro::BridgeOutput::Brake:
						m.bridge_output_value[i] = "Brake";
						break;
					case hardware_interface::talonfxpro::BridgeOutput::Trapez:
						m.bridge_output_value[i] = "Trapez";
						break;
					case hardware_interface::talonfxpro::BridgeOutput::FOCTorque:
						m.bridge_output_value[i] = "FOCTorque";
						break;
					case hardware_interface::talonfxpro::BridgeOutput::MusicTone:
						m.bridge_output_value[i] = "MusicTone";
						break;
					case hardware_interface::talonfxpro::BridgeOutput::FOCEasy:
						m.bridge_output_value[i] = "FOCEasy";
						break;
					case hardware_interface::talonfxpro::BridgeOutput::FaultBrake:
						m.bridge_output_value[i] = "FaultBrake";
						break;
					case hardware_interface::talonfxpro::BridgeOutput::FaultCoast:
						m.bridge_output_value[i] = "FaultCoast";
						break;
					default:
						m.bridge_output_value[i] = "Unknown";
						break;
					}

					m.fault_hardware[i] = ts->getFaultHardware();
					m.fault_proctemp[i] = ts->getFaultProcTemp();
					m.fault_devicetemp[i] = ts->getFaultDeviceTemp();
					m.fault_undervoltage[i] = ts->getFaultUndervoltage();
					m.fault_bootduringenable[i] = ts->getFaultBootDuringEnable();
					m.fault_bridgebrownout[i] = ts->getFaultBridgeBrownout();
					m.fault_unlicensedfeatureinuse[i] = ts->getFaultUnlicensedFeatureInUse();
					m.fault_remotesensorreset[i] = ts->getFaultRemoteSensorReset();
					m.fault_missingdifferentialfx[i] = ts->getFaultMissingDifferentialFX();
					m.fault_remotesensorposoverflow[i] = ts->getFaultRemoteSensorPosOverfow();
					m.fault_oversupplyv[i] = ts->getFaultOverSupplyV();
					m.fault_unstablesupplyv[i] = ts->getFaultUnstableSupplyV();
					m.fault_reversehardlimit[i] = ts->getFaultReverseHardLimit();
					m.fault_forwardhardlimit[i] = ts->getFaultForwardHardLimit();
					m.fault_reversesoftlimit[i] = ts->getFaultReverseSoftLimit();
					m.fault_forwardsoftlimit[i] = ts->getFaultForwardSoftLimit();
					m.fault_remotesensordatainvalid[i] = ts->getFaultRemoteSensorDataInvalid();
					m.fault_fusedsensoroutofsync[i] = ts->getFaultFusedSensorOutOfSync();
					m.fault_statorcurrlimit[i] = ts->getFaultStatorCurrLimit();
					m.fault_supplycurrlimit[i] = ts->getFaultSupplyCurrLimit();

					m.sticky_fault_hardware[i] = ts->getStickyFaultHardware();
					m.sticky_fault_proctemp[i] = ts->getStickyFaultProcTemp();
					m.sticky_fault_devicetemp[i] = ts->getStickyFaultDeviceTemp();
					m.sticky_fault_undervoltage[i] = ts->getStickyFaultUndervoltage();
					m.sticky_fault_bootduringenable[i] = ts->getStickyFaultBootDuringEnable();
					m.sticky_fault_bridgebrownout[i] = ts->getStickyFaultBridgeBrownout();
					m.sticky_fault_unlicensedfeatureinuse[i] = ts->getStickyFaultUnlicensedFeatureInUse();
					m.sticky_fault_remotesensorreset[i] = ts->getStickyFaultRemoteSensorReset();
					m.sticky_fault_missingdifferentialfx[i] = ts->getStickyFaultMissingDifferentialFX();
					m.sticky_fault_remotesensorposoverflow[i] = ts->getStickyFaultRemoteSensorPosOverfow();
					m.sticky_fault_oversupplyv[i] = ts->getStickyFaultOverSupplyV();
					m.sticky_fault_unstablesupplyv[i] = ts->getStickyFaultUnstableSupplyV();
					m.sticky_fault_reversehardlimit[i] = ts->getStickyFaultReverseHardLimit();
					m.sticky_fault_forwardhardlimit[i] = ts->getStickyFaultForwardHardLimit();
					m.sticky_fault_reversesoftlimit[i] = ts->getStickyFaultReverseSoftLimit();
					m.sticky_fault_forwardsoftlimit[i] = ts->getStickyFaultForwardSoftLimit();
					m.sticky_fault_remotesensordatainvalid[i] = ts->getStickyFaultRemoteSensorDataInvalid();
					m.sticky_fault_fusedsensoroutofsync[i] = ts->getStickyFaultFusedSensorOutOfSync();
					m.sticky_fault_statorcurrlimit[i] = ts->getStickyFaultStatorCurrLimit();
					m.sticky_fault_supplycurrlimit[i] = ts->getStickyFaultSupplyCurrLimit();

					m.closed_loop_proportional_output[i] = ts->getClosedLoopProportionalOutput();
					m.closed_loop_integrated_output[i] = ts->getClosedLoopIntegratedOutput();
					m.closed_loop_feed_forward[i] = ts->getClosedLoopFeedForward();
					m.closed_loop_derivative_output[i] = ts->getClosedLoopDerivativeOutput();
					m.closed_loop_output[i] = ts->getClosedLoopOutput();
					m.closed_loop_reference[i] = ts->getClosedLoopReference();
					m.closed_loop_reference_slope[i] = ts->getClosedLoopReferenceSlope();
					m.closed_loop_error[i] = ts->getClosedLoopError();

                    m.differential_output[i] = ts->getDifferentialOutput();
					m.differential_closed_loop_proportional_output[i] = ts->getDifferentialClosedLoopProportionalOutput();
					m.differential_closed_loop_integrated_output[i] = ts->getDifferentialClosedLoopIntegratedOutput();
					m.differential_closed_loop_feed_forward[i] = ts->getDifferentialClosedLoopFeedForward();
					m.differential_closed_loop_derivative_output[i] = ts->getDifferentialClosedLoopDerivativeOutput();
					m.differential_closed_loop_output[i] = ts->getDifferentialClosedLoopOutput();
					m.differential_closed_loop_reference[i] = ts->getDifferentialClosedLoopReference();
					m.differential_closed_loop_reference_slope[i] = ts->getDifferentialClosedLoopReferenceSlope();
					m.differential_closed_loop_error[i] = ts->getDifferentialClosedLoopError();
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

} // namespate talonfxpro_state_controller

#if 0
namespace state_listener_controller
{
class TalonFXProStateListenerController :
	public controller_interface::Controller<hardware_interface::talonfxpro::RemoteTalonFXProStateInterface>
{
private:
	ros::Subscriber sub_command_;
	std::vector<std::string> joint_names_;
	std::vector<hardware_interface::talonfxpro::TalonFXProWritableStateHandle> handles_;

	// Real-time buffer holds the last command value read from the
	// "command" topic.
	realtime_tools::RealtimeBuffer<std::vector<std::optional<hardware_interface::talonfxpro::TalonFXProHWState>>> command_buffer_;

	void commandCB(const talon_state_msgs::TalonFXProStateConstPtr &msg)
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
	TalonFXProStateListenerController() = default;
	~TalonFXProStateListenerController()
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

		sub_command_ = n.subscribe<talon_state_msgs::TalonFXProState>(topic, 1, &TalonFXProStateListenerController::commandCB, this);
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
PLUGINLIB_EXPORT_CLASS(talonfxpro_state_controller::TalonFXProStateController, controller_interface::ControllerBase)
//PLUGINLIB_EXPORT_CLASS(state_listener_controller::TalonFXProStateListenerController, controller_interface::ControllerBase)