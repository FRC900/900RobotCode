///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF Inc nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/*
 * Original joint_state_controller Author: Wim Meeussen
 */

#include <pluginlib/class_list_macros.h>
#include "talon_state_controller/talon_config_controller.h"

namespace talon_config_controller
{

bool TalonConfigController::init(hardware_interface::TalonStateInterface *hw,
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
						realtime_tools::RealtimePublisher<talon_state_msgs::TalonConfig>(root_nh, "talon_configs", 1));

	auto &m = realtime_pub_->msg_;

	// get joints and allocate message
	for (size_t i = 0; i < num_hw_joints_; i++)
	{
		m.name.push_back(joint_names[i]);

		m.feedback_sensor.push_back("");
		m.feedback_coefficient.push_back(0.0);
		m.remote_feedback_sensor.push_back("");
		m.remote_feedback_filter0.push_back("");
		m.remote_feedback_device_id0.push_back(0);
		m.remote_feedback_filter1.push_back("");
		m.remote_feedback_device_id1.push_back(0);
		m.sensor_term_sum0.push_back("");
		m.sensor_term_sum1.push_back("");
		m.sensor_term_diff0.push_back("");
		m.sensor_term_diff1.push_back("");
		m.encoder_ticks_per_rotation.push_back(0);

		m.pid_slot.push_back(0);
		m.pid_p0.push_back(0.0);
		m.pid_p1.push_back(0.0);
		m.pid_p2.push_back(0.0);
		m.pid_p3.push_back(0.0);

		m.pid_i0.push_back(0.0);
		m.pid_i1.push_back(0.0);
		m.pid_i2.push_back(0.0);
		m.pid_i3.push_back(0.0);

		m.pid_d0.push_back(0.0);
		m.pid_d1.push_back(0.0);
		m.pid_d2.push_back(0.0);
		m.pid_d3.push_back(0.0);

		m.pid_f0.push_back(0.0);
		m.pid_f1.push_back(0.0);
		m.pid_f2.push_back(0.0);
		m.pid_f3.push_back(0.0);

		m.pid_izone0.push_back(0);
		m.pid_izone1.push_back(0);
		m.pid_izone2.push_back(0);
		m.pid_izone3.push_back(0);
		m.pid_allowable_closed_loop_error0.push_back(0);
		m.pid_allowable_closed_loop_error1.push_back(0);
		m.pid_allowable_closed_loop_error2.push_back(0);
		m.pid_allowable_closed_loop_error3.push_back(0);
		m.pid_max_integral_accumulator0.push_back(0);
		m.pid_max_integral_accumulator1.push_back(0);
		m.pid_max_integral_accumulator2.push_back(0);
		m.pid_max_integral_accumulator3.push_back(0);
		m.pid_closed_loop_peak_output0.push_back(0);
		m.pid_closed_loop_peak_output1.push_back(0);
		m.pid_closed_loop_peak_output2.push_back(0);
		m.pid_closed_loop_peak_output3.push_back(0);
		m.pid_closed_loop_period0.push_back(0);
		m.pid_closed_loop_period1.push_back(0);
		m.pid_closed_loop_period2.push_back(0);
		m.pid_closed_loop_period3.push_back(0);
		m.aux_pid_polarity.push_back(false);
		m.can_id.push_back(0);
		m.invert.push_back(false);
		m.sensorPhase.push_back(false);
		m.neutral_mode.push_back("");

		m.closed_loop_ramp.push_back(0);
		m.open_loop_ramp.push_back(0);
		m.peak_output_forward.push_back(0);
		m.peak_output_reverse.push_back(0);
		m.nominal_output_forward.push_back(0);
		m.nominal_output_reverse.push_back(0);
		m.neutral_deadband.push_back(0);

		m.voltage_compensation_saturation.push_back(0);
		m.voltage_measurement_filter.push_back(0);
		m.voltage_compensation_enable.push_back(false);

		m.velocity_measurement_period.push_back(0);
		m.velocity_measurement_window.push_back(0);

		m.limit_switch_local_forward_source.push_back("");
		m.limit_switch_local_forward_normal.push_back("");
		m.limit_switch_local_reverse_source.push_back("");
		m.limit_switch_local_reverse_normal.push_back("");
		m.limit_switch_remote_forward_source.push_back("");
		m.limit_switch_remote_forward_normal.push_back("");
		m.limit_switch_remote_forward_id.push_back(0);
		m.limit_switch_remote_reverse_source.push_back("");
		m.limit_switch_remote_reverse_normal.push_back("");
		m.limit_switch_remote_reverse_id.push_back(0);
		m.softlimit_forward_threshold.push_back(0);
		m.softlimit_forward_enable.push_back(false);
		m.softlimit_reverse_threshold.push_back(0);
		m.softlimit_reverse_enable.push_back(false);
		m.softlimits_override_enable.push_back(false);

		m.current_limit_peak_amps.push_back(0);
		m.current_limit_peak_msec.push_back(0);
		m.current_limit_continuous_amps.push_back(0);
		m.current_limit_enable.push_back(false);

		m.supply_current_limit.push_back(0);
		m.supply_current_trigger_threshold_current.push_back(0);
		m.supply_current_trigger_threshold_time.push_back(0);
		m.supply_current_limit_enable.push_back(false);
		m.stator_current_limit.push_back(0);
		m.stator_current_trigger_threshold_current.push_back(0);
		m.stator_current_trigger_threshold_time.push_back(0);
		m.stator_current_limit_enable.push_back(false);

		m.motion_cruise_velocity.push_back(0);
		m.motion_acceleration.push_back(0);
		m.motion_s_curve_strength.push_back(0);
		m.status_1_general_period.push_back(0);
		m.status_2_feedback0_period.push_back(0);
		m.status_3_quadrature_period.push_back(0);
		m.status_4_aintempvbat_period.push_back(0);
		m.status_6_misc_period.push_back(0);
		m.status_7_commstatus_period.push_back(0);
		m.status_8_pulsewidth_period.push_back(0);
		m.status_9_motprofbuffer_period.push_back(0);
		m.status_10_motionmagic_period.push_back(0);
		m.status_11_uartgadgeteer_period.push_back(0);
		m.status_12_feedback1_period.push_back(0);
		m.status_13_base_pidf0_period.push_back(0);
		m.status_14_turn_pidf1_period.push_back(0);
		m.status_15_firmwareapistatus_period.push_back(0);
		m.control_3_general_period.push_back(0);
		m.control_4_advanced_period.push_back(0);
		m.control_5_feedbackoutputoverride_period.push_back(0);
		m.control_6_motprofaddtrajpoint_period.push_back(0);

		m.motion_profile_trajectory_period.push_back(0);
		m.conversion_factor.push_back(0.0);

		m.motor_commutation.push_back("");
		m.absolute_sensor_range.push_back("");
		m.sensor_initialization_strategy.push_back("");

		m.firmware_version.push_back("");
		m.water_game.push_back(true);

		talon_state_.push_back(hw->getHandle(joint_names[i]));
	}

	return true;
}

void TalonConfigController::starting(const ros::Time &time)
{
	// initialize time
	last_publish_time_ = time;
}

std::string TalonConfigController::limitSwitchSourceToString(const hardware_interface::LimitSwitchSource source) const
{
	switch (source)
	{
		case hardware_interface::LimitSwitchSource_Uninitialized:
			return "Uninitialized";
		case hardware_interface::LimitSwitchSource_FeedbackConnector:
			return "FeedbackConnector";
		case hardware_interface::LimitSwitchSource_RemoteTalonSRX:
			return "RemoteTalonSRX";
		case hardware_interface::LimitSwitchSource_RemoteCANifier:
			return "RemoteCANifier";
		case hardware_interface::LimitSwitchSource_Deactivated:
			return "Deactivated";
		default:
			return "Unknown";
	}
}

std::string TalonConfigController::remoteLimitSwitchSourceToString(const hardware_interface::RemoteLimitSwitchSource source) const
{
	switch (source)
	{
		case hardware_interface::RemoteLimitSwitchSource_Uninitialized:
			return "Uninitialized";
		case hardware_interface::RemoteLimitSwitchSource_RemoteTalonSRX:
			return "RemoteTalonSRX";
		case hardware_interface::RemoteLimitSwitchSource_RemoteCANifier:
			return "RemoteCANifier";
		case hardware_interface::RemoteLimitSwitchSource_Deactivated:
			return "Deactivated";
		default:
			return "Unknown";
	}
}

std::string TalonConfigController::limitSwitchNormalToString(const hardware_interface::LimitSwitchNormal normal) const
{
	switch (normal)
	{
		case hardware_interface::LimitSwitchNormal_Uninitialized:
			return "Uninitialized";
		case hardware_interface::LimitSwitchNormal_NormallyOpen:
			return "NormallyOpen";
		case hardware_interface::LimitSwitchNormal_NormallyClosed:
			return "NormallyClosed";
		case hardware_interface::LimitSwitchNormal_Disabled:
			return "Disabled";
		default:
			return "Unknown";
	}
}

std::string TalonConfigController::feedbackDeviceToString(const hardware_interface::FeedbackDevice feedback_device) const
{
	switch (feedback_device)
	{
		case hardware_interface::FeedbackDevice_Uninitialized:
			return "Uninitialized";
		case hardware_interface::FeedbackDevice_QuadEncoder:
			return "QuadEncoder/CTRE_MagEncoder_Relative";
		case hardware_interface::FeedbackDevice_IntegratedSensor:
			return "IntegratedSensor";
		case hardware_interface::FeedbackDevice_Analog:
			return "Analog";
		case hardware_interface::FeedbackDevice_Tachometer:
			return "Tachometer";
		case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
			return "PusleWidthEncodedPosition/CTRE_MagEncoder_Absolute";
		case hardware_interface::FeedbackDevice_SensorSum:
			return  "SensorSum";
		case hardware_interface::FeedbackDevice_SensorDifference:
			return "SensorDifference";
		case hardware_interface::FeedbackDevice_RemoteSensor0:
			return  "RemoteSensor0";
		case hardware_interface::FeedbackDevice_RemoteSensor1:
			return  "RemoteSensor1";
		case hardware_interface::FeedbackDevice_None:
			return "None";
		case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
			return "SoftwareEmulatedSensor";
		default:
			return "Unknown";
	}
}

std::string TalonConfigController::remoteSensorSourceToString(const hardware_interface::RemoteSensorSource remote_sensor_source) const
{
	switch (remote_sensor_source)
	{
		case hardware_interface::RemoteSensorSource_Off:
			return "Off";
		case hardware_interface::RemoteSensorSource_TalonSRX_SelectedSensor:
			return "TalonSRX_SelectedSensor";
		case hardware_interface::RemoteSensorSource_Pigeon_Yaw:
			return "Pigeon_Yaw";
		case hardware_interface::RemoteSensorSource_Pigeon_Pitch:
			return "Pigeon_Pitch";
		case hardware_interface::RemoteSensorSource_Pigeon_Roll:
			return "Pigeon_Roll";
		case hardware_interface::RemoteSensorSource_CANifier_Quadrature:
			return "CANifier_Quadrature";
		case hardware_interface::RemoteSensorSource_CANifier_PWMInput0:
			return "CANifier_PWMInput0";
		case hardware_interface::RemoteSensorSource_CANifier_PWMInput1:
			return "CANifier_PWMInput1";
		case hardware_interface::RemoteSensorSource_CANifier_PWMInput2:
			return "CANifier_PWMInput2";
		case hardware_interface::RemoteSensorSource_CANifier_PWMInput3:
			return "CANifier_PWMInput3";
		case hardware_interface::RemoteSensorSource_GadgeteerPigeon_Yaw:
			return "GadgeteerPigeon_Yaw";
		case hardware_interface::RemoteSensorSource_GadgeteerPigeon_Pitch:
			return "GadgeteerPigeon_Pitch";
		case hardware_interface::RemoteSensorSource_GadgeteerPigeon_Roll:
			return "GadgeteerPigeon_Roll";
		default:
			return "Unknown";
	}
}

void TalonConfigController::update(const ros::Time &time, const ros::Duration & /*period*/)
{
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
				auto &ts = talon_state_[i];
				m.can_id[i] = ts->getCANID();

				m.feedback_coefficient[i] = ts->getFeedbackCoefficient();
				m.feedback_sensor[i] = feedbackDeviceToString(ts->getEncoderFeedback());
				switch (ts->getRemoteEncoderFeedback())
				{
					case hardware_interface::RemoteFeedbackDevice_SensorSum:
						m.remote_feedback_sensor[i] = "SensorSum";
						break;
					case hardware_interface::RemoteFeedbackDevice_SensorDifference:
						m.remote_feedback_sensor[i] = "SensorDifference";
						break;
					case hardware_interface::RemoteFeedbackDevice_RemoteSensor0:
						m.remote_feedback_sensor[i] = "RemoteSensor0";
						break;
					case hardware_interface::RemoteFeedbackDevice_RemoteSensor1:
						m.remote_feedback_sensor[i] = "RemoteSensor1";
						break;
					case hardware_interface::RemoteFeedbackDevice_None:
						m.remote_feedback_sensor[i] = "None";
						break;
					case hardware_interface::RemoteFeedbackDevice_SoftwareEmulatedSensor:
						m.remote_feedback_sensor[i] = "SoftwareEmulatedSensor";
						break;
					default:
						m.remote_feedback_sensor[i] = "Unknown";
						break;
				}
				m.remote_feedback_device_id0[i] = ts->getRemoteFeedbackDeviceId(0);
				m.remote_feedback_filter0[i] = remoteSensorSourceToString(ts->getRemoteFeedbackFilter(0));
				m.remote_feedback_device_id1[i] = ts->getRemoteFeedbackDeviceId(1);
				m.remote_feedback_filter1[i] = remoteSensorSourceToString(ts->getRemoteFeedbackFilter(1));
				m.sensor_term_sum0[i] = feedbackDeviceToString(ts->getSensorTerm(hardware_interface::SensorTerm_Sum0));
				m.sensor_term_sum1[i] = feedbackDeviceToString(ts->getSensorTerm(hardware_interface::SensorTerm_Sum1));
				m.sensor_term_diff0[i] = feedbackDeviceToString(ts->getSensorTerm(hardware_interface::SensorTerm_Diff0));
				m.sensor_term_diff1[i] = feedbackDeviceToString(ts->getSensorTerm(hardware_interface::SensorTerm_Diff1));
				m.encoder_ticks_per_rotation[i] = ts->getEncoderTicksPerRotation();

				//publish the array of PIDF values
				m.pid_slot[i] = ts->getSlot();
				m.pid_p0[i] = ts->getPidfP(0);
				m.pid_i0[i] = ts->getPidfI(0);
				m.pid_d0[i] = ts->getPidfD(0);
				m.pid_f0[i] = ts->getPidfF(0);
				m.pid_izone0[i] = ts->getPidfIzone(0);
				m.pid_allowable_closed_loop_error0[i] = ts->getAllowableClosedLoopError(0);
				m.pid_max_integral_accumulator0[i] = ts->getMaxIntegralAccumulator(0);
				m.pid_closed_loop_peak_output0[i] = ts->getClosedLoopPeakOutput(0);
				m.pid_closed_loop_period0[i] = ts->getClosedLoopPeriod(0);

				m.pid_p1[i] = ts->getPidfP(1);
				m.pid_i1[i] = ts->getPidfI(1);
				m.pid_d1[i] = ts->getPidfD(1);
				m.pid_f1[i] = ts->getPidfF(1);
				m.pid_izone1[i] = ts->getPidfIzone(1);
				m.pid_allowable_closed_loop_error1[i] = ts->getAllowableClosedLoopError(1);
				m.pid_max_integral_accumulator1[i] = ts->getMaxIntegralAccumulator(1);
				m.pid_closed_loop_peak_output1[i] = ts->getClosedLoopPeakOutput(1);
				m.pid_closed_loop_period1[i] = ts->getClosedLoopPeriod(1);

				m.pid_p2[i] = ts->getPidfP(2);
				m.pid_i2[i] = ts->getPidfI(2);
				m.pid_d2[i] = ts->getPidfD(2);
				m.pid_f2[i] = ts->getPidfF(2);
				m.pid_izone2[i] = ts->getPidfIzone(2);
				m.pid_allowable_closed_loop_error2[i] = ts->getAllowableClosedLoopError(2);
				m.pid_max_integral_accumulator2[i] = ts->getMaxIntegralAccumulator(2);
				m.pid_closed_loop_peak_output2[i] = ts->getClosedLoopPeakOutput(2);
				m.pid_closed_loop_period2[i] = ts->getClosedLoopPeriod(2);

				m.pid_p3[i] = ts->getPidfP(3);
				m.pid_i3[i] = ts->getPidfI(3);
				m.pid_d3[i] = ts->getPidfD(3);
				m.pid_f3[i] = ts->getPidfF(3);
				m.pid_izone3[i] = ts->getPidfIzone(3);
				m.pid_allowable_closed_loop_error3[i] = ts->getAllowableClosedLoopError(3);
				m.pid_max_integral_accumulator3[i] = ts->getMaxIntegralAccumulator(3);
				m.pid_closed_loop_peak_output3[i] = ts->getClosedLoopPeakOutput(3);
				m.pid_closed_loop_period3[i] = ts->getClosedLoopPeriod(3);

				m.aux_pid_polarity[i] = ts->getAuxPidPolarity();

				m.invert[i] = ts->getInvert();
				m.sensorPhase[i] = ts->getSensorPhase();
				switch (ts->getNeutralMode())
				{
					case hardware_interface::NeutralMode_Uninitialized:
						m.neutral_mode[i] = "Uninitialized";
						break;
					case hardware_interface::NeutralMode_EEPROM_Setting:
						m.neutral_mode[i] = "EEPROM_Setting";
						break;
					case hardware_interface::NeutralMode_Coast:
						m.neutral_mode[i] = "Coast";
						break;
					case hardware_interface::NeutralMode_Brake:
						m.neutral_mode[i] = "Brake";
						break;
					case hardware_interface::NeutralMode_Last:
						m.neutral_mode[i] = "Last";
						break;
					default:
						m.neutral_mode[i] = "Unknown";
						break;
				}

				m.closed_loop_ramp[i] = ts->getClosedloopRamp();
				m.open_loop_ramp[i] = ts->getOpenloopRamp();
				m.peak_output_forward[i] = ts->getPeakOutputForward();
				m.peak_output_reverse[i] = ts->getPeakOutputReverse();
				m.nominal_output_forward[i] = ts->getNominalOutputForward();
				m.nominal_output_reverse[i] = ts->getNominalOutputReverse();
				m.neutral_deadband[i] = ts->getNeutralDeadband();

				m.voltage_compensation_saturation[i] = ts->getVoltageCompensationSaturation();
				m.voltage_measurement_filter[i] = ts->getVoltageMeasurementFilter();
				m.voltage_compensation_enable[i] = ts->getVoltageCompensationEnable();

				m.velocity_measurement_period[i] = ts->getVelocityMeasurementPeriod();
				m.velocity_measurement_window[i] = ts->getVelocityMeasurementWindow();
				hardware_interface::LimitSwitchSource ls_source;
				hardware_interface::LimitSwitchNormal ls_normal;
				ts->getForwardLimitSwitchSource(ls_source, ls_normal);

				m.limit_switch_local_forward_source[i] = limitSwitchSourceToString(ls_source);
				m.limit_switch_local_forward_normal[i] = limitSwitchNormalToString(ls_normal);

				ts->getReverseLimitSwitchSource(ls_source, ls_normal);
				m.limit_switch_local_reverse_source[i] = limitSwitchSourceToString(ls_source);
				m.limit_switch_local_reverse_normal[i] = limitSwitchNormalToString(ls_normal);

				hardware_interface::RemoteLimitSwitchSource remote_ls_source;
				unsigned int remote_ls_id;
				ts->getRemoteForwardLimitSwitchSource(remote_ls_source, ls_normal, remote_ls_id);

				m.limit_switch_remote_forward_source[i] = remoteLimitSwitchSourceToString(remote_ls_source);
				m.limit_switch_remote_forward_normal[i] = limitSwitchNormalToString(ls_normal);
				m.limit_switch_remote_forward_id[i] = remote_ls_id;

				ts->getRemoteReverseLimitSwitchSource(remote_ls_source, ls_normal, remote_ls_id);
				m.limit_switch_remote_reverse_source[i] = remoteLimitSwitchSourceToString(remote_ls_source);
				m.limit_switch_remote_reverse_normal[i] = limitSwitchNormalToString(ls_normal);
				m.limit_switch_remote_reverse_id[i] = remote_ls_id;

				m.softlimit_forward_threshold[i] = ts->getForwardSoftLimitThreshold();
				m.softlimit_forward_enable[i] = ts->getForwardSoftLimitEnable();
				m.softlimit_reverse_threshold[i] = ts->getReverseSoftLimitThreshold();
				m.softlimit_reverse_enable[i] = ts->getReverseSoftLimitEnable();
				m.softlimits_override_enable[i] = ts->getOverrideSoftLimitsEnable();

				m.current_limit_peak_amps[i] = ts->getPeakCurrentLimit();
				m.current_limit_peak_msec[i] = ts->getPeakCurrentDuration();
				m.current_limit_continuous_amps[i] = ts->getContinuousCurrentLimit();
				m.current_limit_enable[i] = ts->getCurrentLimitEnable();

				m.supply_current_limit[i] = ts->getSupplyCurrentLimit();
				m.supply_current_trigger_threshold_current[i] = ts->getSupplyCurrentTriggerThresholdCurrent();
				m.supply_current_trigger_threshold_time[i] = ts->getSupplyCurrentTriggerThresholdTime();
				m.supply_current_limit_enable[i] = ts->getSupplyCurrentLimitEnable();
				m.stator_current_limit[i] = ts->getStatorCurrentLimit();
				m.stator_current_trigger_threshold_current[i] = ts->getStatorCurrentTriggerThresholdCurrent();
				m.stator_current_trigger_threshold_time[i] = ts->getStatorCurrentTriggerThresholdTime();
				m.stator_current_limit_enable[i] = ts->getStatorCurrentLimitEnable();

				m.motion_cruise_velocity[i] = ts->getMotionCruiseVelocity();
				m.motion_acceleration[i] = ts->getMotionAcceleration();
				m.motion_s_curve_strength[i] = ts->getMotionSCurveStrength();

				m.status_1_general_period[i] = ts->getStatusFramePeriod(hardware_interface::Status_1_General);
				m.status_2_feedback0_period[i] = ts->getStatusFramePeriod(hardware_interface::Status_2_Feedback0);
				m.status_3_quadrature_period[i] = ts->getStatusFramePeriod(hardware_interface::Status_3_Quadrature);
				m.status_4_aintempvbat_period[i] = ts->getStatusFramePeriod(hardware_interface::Status_4_AinTempVbat);
				m.status_6_misc_period[i] = ts->getStatusFramePeriod(hardware_interface::Status_6_Misc);
				m.status_7_commstatus_period[i] = ts->getStatusFramePeriod(hardware_interface::Status_7_CommStatus);
				m.status_8_pulsewidth_period[i] = ts->getStatusFramePeriod(hardware_interface::Status_8_PulseWidth);
				m.status_9_motprofbuffer_period[i] = ts->getStatusFramePeriod(hardware_interface::Status_9_MotProfBuffer);
				m.status_10_motionmagic_period[i] = ts->getStatusFramePeriod(hardware_interface::Status_10_MotionMagic);
				m.status_11_uartgadgeteer_period[i] = ts->getStatusFramePeriod(hardware_interface::Status_11_UartGadgeteer);
				m.status_12_feedback1_period[i] = ts->getStatusFramePeriod(hardware_interface::Status_12_Feedback1);
				m.status_13_base_pidf0_period[i] = ts->getStatusFramePeriod(hardware_interface::Status_13_Base_PIDF0);
				m.status_14_turn_pidf1_period[i] = ts->getStatusFramePeriod(hardware_interface::Status_14_Turn_PIDF1);
				m.status_15_firmwareapistatus_period[i] = ts->getStatusFramePeriod(hardware_interface::Status_15_FirmwareApiStatus);

				m.control_3_general_period[i] = ts->getControlFramePeriod(hardware_interface::Control_3_General);
				m.control_4_advanced_period[i] = ts->getControlFramePeriod(hardware_interface::Control_4_Advanced);
				m.control_5_feedbackoutputoverride_period[i] = ts->getControlFramePeriod(hardware_interface::Control_5_FeedbackOutputOverride);
				m.control_6_motprofaddtrajpoint_period[i] = ts->getControlFramePeriod(hardware_interface::Control_6_MotProfAddTrajPoint);
				m.motion_profile_trajectory_period[i] = ts->getMotionProfileTrajectoryPeriod();

				m.conversion_factor[i] = ts->getConversionFactor();


				const auto motor_commutation = ts->getMotorCommutation();
				switch (motor_commutation)
				{
					case hardware_interface::MotorCommutation::Trapezoidal:
						m.motor_commutation[i] = "Trapezoidal";
						break;
					default:
						m.motor_commutation[i] = "Unknown";
						break;
				}

				const auto absolute_sensor_range = ts->getAbsoluteSensorRange();
				switch (absolute_sensor_range)
				{
					case hardware_interface::Unsigned_0_to_360:
						m.absolute_sensor_range[i] = "Unsigned_0_to_360";
						break;
					case hardware_interface::Signed_PlusMinus180:
						m.absolute_sensor_range[i] = "Signed_PlusMinus180";
						break;
					default:
						m.absolute_sensor_range[i] = "Unknown";
						break;
				}

				const auto sensor_initialization_strategy = ts->getSensorInitializationStrategy();
				switch (sensor_initialization_strategy)
				{
					case hardware_interface::BootToZero:
						m.sensor_initialization_strategy[i] = "BootToZero";
						break;
					case hardware_interface::BootToAbsolutePosition:
						m.sensor_initialization_strategy[i] = "BootToAbsolutePosition";
						break;
					default:
						m.sensor_initialization_strategy[i] = "Unknown";
						break;
				}

				int fw_ver = ts->getFirmwareVersion();

				if (fw_ver >= 0)
				{
					char str[256];
					sprintf(str, "2.2%d.%2.2d", (fw_ver >> 8) & 0xFF, fw_ver & 0xFF);
					m.firmware_version[i] = str;
				}
			}
			realtime_pub_->unlockAndPublish();
		}
	}
}

void TalonConfigController::stopping(const ros::Time & /*time*/)
{}

}

PLUGINLIB_EXPORT_CLASS(talon_config_controller::TalonConfigController, controller_interface::ControllerBase)
