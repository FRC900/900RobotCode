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

#include <algorithm>
#include <cstddef>

#include <pluginlib/class_list_macros.h>
#include "talon_state_controller/talon_state_controller.h"

namespace talon_state_controller
{

bool TalonStateController::init(hardware_interface::TalonStateInterface *hw,
								ros::NodeHandle                         &root_nh,
								ros::NodeHandle                         &controller_nh)
{
	// get all joint names from the hardware interface
	const std::vector<std::string> &joint_names = hw->getNames();
	num_hw_joints_ = joint_names.size();
	for (unsigned i = 0; i < num_hw_joints_; i++)
		ROS_DEBUG("Got joint %s", joint_names[i].c_str());

	// get publishing period
	if (!controller_nh.getParam("publish_rate", publish_rate_))
	{
		ROS_ERROR("Parameter 'publish_rate' not set");
		return false;
	}

	// realtime publisher
	realtime_pub_.reset(new
						realtime_tools::RealtimePublisher<talon_state_controller::TalonState>(root_nh, "talon_states",
								4));

	auto &m = realtime_pub_->msg_;
	// get joints and allocate message
	talon_state_controller::CustomProfileStatus custom_profile_status_holder;

	for (unsigned i = 0; i < num_hw_joints_; i++)
	{
		m.name.push_back(joint_names[i]);
		m.talon_mode.push_back("");
		m.demand1_type.push_back("");
		m.demand1_value.push_back(0);
		m.position.push_back(0.0);
		m.speed.push_back(0.0);
		m.output_voltage.push_back(0.0);
		m.output_current.push_back(0.0);
		m.bus_voltage.push_back(0.0);
		m.motor_output_percent.push_back(0.0);
		m.temperature.push_back(0.0);

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
		m.set_point.push_back(0.0);
		m.can_id.push_back(0);
		m.closed_loop_error.push_back(0);
		m.integral_accumulator.push_back(0);
		m.error_derivative.push_back(0);
		m.closed_loop_target.push_back(0);
		m.p_term.push_back(0);
		m.i_term.push_back(0);
		m.d_term.push_back(0);
		m.f_term.push_back(0);
		m.active_trajectory_position.push_back(0);
		m.active_trajectory_velocity.push_back(0);
		m.active_trajectory_arbitrary_feed_forward.push_back(0);
		m.active_trajectory_heading.push_back(0);
		m.forward_limit_switch.push_back(false);
		m.reverse_limit_switch.push_back(false);
		m.forward_softlimit.push_back(false);
		m.reverse_softlimit.push_back(false);
		m.invert.push_back(false);
		m.sensorPhase.push_back(false);
		m.neutral_mode.push_back("");
		m.neutral_output.push_back(false);

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

		m.motion_cruise_velocity.push_back(0);
		m.motion_acceleration.push_back(0);
		m.motion_s_curve_strength.push_back(0);
		m.motion_profile_top_level_buffer_count.push_back(0);
		m.motion_profile_top_level_buffer_full.push_back(false);
		m.motion_profile_status_top_buffer_rem.push_back(0);
		m.motion_profile_status_top_buffer_cnt.push_back(0);
		m.motion_profile_status_btm_buffer_cnt.push_back(0);
		m.motion_profile_status_has_underrun.push_back(false);
		m.motion_profile_status_is_underrun.push_back(false);
		m.motion_profile_status_active_point_valid.push_back(false);
		m.motion_profile_status_is_last.push_back(false);
		m.motion_profile_status_profile_slot_select0.push_back(0);
		m.motion_profile_status_profile_slot_select1.push_back(0);
		m.motion_profile_status_output_enable.push_back("");
		m.motion_profile_time_dur_ms.push_back(0);
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
		m.faults.push_back("");
		m.sticky_faults.push_back("");
		m.conversion_factor.push_back(0.0);
		m.custom_profile_status.push_back(custom_profile_status_holder);

		m.water_game.push_back(true);

		talon_state_.push_back(hw->getHandle(joint_names[i]));
	}
	addExtraJoints(controller_nh, m);

	return true;
}

void TalonStateController::starting(const ros::Time &time)
{
	// initialize time
	last_publish_time_ = time;
}

std::string TalonStateController::limitSwitchSourceToString(const hardware_interface::LimitSwitchSource source) const
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

std::string TalonStateController::remoteLimitSwitchSourceToString(const hardware_interface::RemoteLimitSwitchSource source) const
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
std::string TalonStateController::limitSwitchNormalToString(const hardware_interface::LimitSwitchNormal normal) const
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

std::string TalonStateController::feedbackDeviceToString(const hardware_interface::FeedbackDevice feedback_device) const
{
	switch (feedback_device)
	{
		case hardware_interface::FeedbackDevice_Uninitialized:
			return "Uninitialized";
		case hardware_interface::FeedbackDevice_QuadEncoder:
			return "QuadEncoder";
		case hardware_interface::FeedbackDevice_Analog:
			return "Analog";
		case hardware_interface::FeedbackDevice_Tachometer:
			return "Tachometer";
		case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
			return "PusleWidthEncodedPosition";
		case hardware_interface::FeedbackDevice_SensorSum:
			return  "SensorSum";
		case hardware_interface::FeedbackDevice_SensorDifference:
			return "SensorDifference";
		case hardware_interface::FeedbackDevice_RemoteSensor0:
			return  "RemoteSensor0";
		case hardware_interface::FeedbackDevice_RemoteSensor1:
			return  "RemoteSensor0";
		case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
			return "SoftwareEmulatedSensor";
		default:
			return "Unknown";
	}
}

std::string TalonStateController::remoteSensorSourceToString(const hardware_interface::RemoteSensorSource remote_sensor_source) const
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

void TalonStateController::update(const ros::Time &time, const ros::Duration & /*period*/)
{
	talon_state_controller::CustomProfileStatus custom_profile_status_holder;

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
			for (unsigned i = 0; i < num_hw_joints_; i++)
			{
				auto &ts = talon_state_[i];
				m.set_point[i] = ts->getSetpoint();
				m.position[i] = ts->getPosition();
				m.speed[i] = ts->getSpeed();
				m.output_voltage[i] = ts->getOutputVoltage();
				m.can_id[i] = ts->getCANID();
				m.output_current[i] = ts->getOutputCurrent();
				m.bus_voltage[i] = ts->getBusVoltage();
				m.motor_output_percent[i] = ts->getMotorOutputPercent();
				m.temperature[i] = ts->getTemperature();

				m.feedback_sensor[i] = feedbackDeviceToString(ts->getEncoderFeedback());
				m.feedback_coefficient[i] = ts->getFeedbackCoefficient();
				switch (ts->getRemoteEncoderFeedback())
				{
					case hardware_interface::RemoteFeedbackDevice_FactoryDefaultOff:
						m.remote_feedback_sensor[i] = "FactoryDefaultOff";
						break;
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
				m.encoder_ticks_per_rotation[i] = ts->getEncoderTicksPerRotation();
				m.sensor_term_sum0[i] = feedbackDeviceToString(ts->getSensorTerm(hardware_interface::SensorTerm_Sum0));
				m.sensor_term_sum1[i] = feedbackDeviceToString(ts->getSensorTerm(hardware_interface::SensorTerm_Sum1));
				m.sensor_term_diff0[i] = feedbackDeviceToString(ts->getSensorTerm(hardware_interface::SensorTerm_Diff0));
				m.sensor_term_diff1[i] = feedbackDeviceToString(ts->getSensorTerm(hardware_interface::SensorTerm_Diff1));

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

				m.closed_loop_error[i] = ts->getClosedLoopError();
				m.integral_accumulator[i] = ts->getIntegralAccumulator();
				m.error_derivative[i] = ts->getErrorDerivative();
				m.closed_loop_error[i] = ts->getClosedLoopError();
				m.closed_loop_target[i] = ts->getClosedLoopTarget();
				m.p_term[i] = ts->getPTerm();
				m.i_term[i] = ts->getITerm();
				m.d_term[i] = ts->getDTerm();
				m.f_term[i] = ts->getFTerm();
				m.active_trajectory_position[i] = ts->getActiveTrajectoryPosition();
				m.active_trajectory_velocity[i] = ts->getActiveTrajectoryVelocity();
				m.active_trajectory_arbitrary_feed_forward[i] = ts->getActiveTrajectoryArbitraryFeedForward();
				m.active_trajectory_heading[i] = ts->getActiveTrajectoryHeading();
				m.forward_limit_switch[i] = ts->getForwardLimitSwitch();
				m.reverse_limit_switch[i] = ts->getReverseLimitSwitch();
				m.forward_softlimit[i] = ts->getForwardSoftlimitHit();
				m.reverse_softlimit[i] = ts->getReverseSoftlimitHit();
				m.invert[i] = ts->getInvert();
				m.sensorPhase[i] = ts->getSensorPhase();
				hardware_interface::TalonMode talonMode = ts->getTalonMode();
				switch (talonMode)
				{
					case hardware_interface::TalonMode_First:
						m.talon_mode[i] = "First";
						break;
					case hardware_interface::TalonMode_PercentOutput:
						m.talon_mode[i] = "Percent Output";
						break;
					case hardware_interface::TalonMode_Position:
						m.talon_mode[i] = "Closed Loop Position";
						break;
					case hardware_interface::TalonMode_Velocity:
						m.talon_mode[i] = "Closed Loop Velocity";
						break;
					case hardware_interface::TalonMode_Current:
						m.talon_mode[i] = "Closed Loop Current";
						break;
					case hardware_interface::TalonMode_Follower:
						m.talon_mode[i] = "Follower";
						break;
					case hardware_interface::TalonMode_MotionProfile:
						m.talon_mode[i] = "Motion Profile";
						break;
					case hardware_interface::TalonMode_MotionMagic:
						m.talon_mode[i] = "Motion Magic";
						break;
					case hardware_interface::TalonMode_MotionProfileArc:
						m.talon_mode[i] = "Motion Profile Arc";
						break;
					case hardware_interface::TalonMode_Disabled:
						m.talon_mode[i] = "Disabled";
						break;
					case hardware_interface::TalonMode_Last:
						m.talon_mode[i] = "Last";
						break;
					default:
						m.talon_mode[i] = "Unknown";
						break;
				}
				hardware_interface::DemandType demand1Type = ts->getDemand1Type();
				switch (demand1Type)
				{
					case hardware_interface::DemandType_Neutral:
						m.demand1_type[i] = "Neutral";
						break;
					case hardware_interface::DemandType_AuxPID:
						m.demand1_type[i] = "AuxPID";
						break;
					case hardware_interface::DemandType_ArbitraryFeedForward:
						m.demand1_type[i] = "ArbitraryFeedForward";
						break;
					default:
						m.demand1_type[i] = "Unknown";
						break;
				}
				m.demand1_value[i] = ts->getDemand1Value();
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
				m.neutral_output[i] = ts->getNeutralOutput();
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

				m.motion_cruise_velocity[i] = ts->getMotionCruiseVelocity();
				m.motion_acceleration[i] = ts->getMotionAcceleration();
				m.motion_s_curve_strength[i] = ts->getMotionSCurveStrength();
				m.motion_profile_top_level_buffer_count[i] = ts->getMotionProfileTopLevelBufferCount();
				m.motion_profile_top_level_buffer_full[i] = ts->getMotionProfileTopLevelBufferFull();
				hardware_interface::MotionProfileStatus mp_status(ts->getMotionProfileStatus());
				m.motion_profile_status_top_buffer_rem[i] = mp_status.topBufferRem;
				m.motion_profile_status_top_buffer_cnt[i] = mp_status.topBufferCnt;
				m.motion_profile_status_btm_buffer_cnt[i] = mp_status.btmBufferCnt;
				m.motion_profile_status_has_underrun[i] = mp_status.hasUnderrun;
				m.motion_profile_status_is_underrun[i] = mp_status.isUnderrun;
				m.motion_profile_status_active_point_valid[i] = mp_status.activePointValid;
				m.motion_profile_status_is_last[i] = mp_status.isLast;
				m.motion_profile_status_profile_slot_select0[i] = mp_status.profileSlotSelect0;
				m.motion_profile_status_profile_slot_select1[i] = mp_status.profileSlotSelect1;
				switch (mp_status.outputEnable)
				{
					case hardware_interface::Disable:
						m.motion_profile_status_output_enable[i] = "Disable";
						break;
					case hardware_interface::Enable:
						m.motion_profile_status_output_enable[i] = "Enable";
						break;
					case hardware_interface::Hold:
						m.motion_profile_status_output_enable[i] = "Hold";
						break;
					default:
						m.motion_profile_status_output_enable[i] = "Unknown";
						break;
				}
				m.motion_profile_time_dur_ms[i] = mp_status.timeDurMs;

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
				{
					unsigned faults = ts->getFaults();
					unsigned int mask = 1;
					std::string str;
					if (faults)
					{
						if (faults & mask) str += "UnderVoltage "; mask <<= 1;
						if (faults & mask) str += "ForwardLimitSwitch "; mask <<= 1;
						if (faults & mask) str += "ReverseLimitSwitch "; mask <<= 1;
						if (faults & mask) str += "ForwardSoftLimit "; mask <<= 1;
						if (faults & mask) str += "ReverseSoftLimit "; mask <<= 1;
						if (faults & mask) str += "HardwareFailure "; mask <<= 1;
						if (faults & mask) str += "ResetDuringEn "; mask <<= 1;
						if (faults & mask) str += "SensorOverflow "; mask <<= 1;
						if (faults & mask) str += "SensorOutOfPhase "; mask <<= 1;
						if (faults & mask) str += "HardwareESDReset "; mask <<= 1;
						if (faults & mask) str += "RemoteLossOfSignal ";
					}
					m.faults[i] = str;
				}

				{
					unsigned faults = ts->getStickyFaults();
					unsigned int mask = 1;
					std::string str;
					if (faults)
					{
						if (faults & mask) str += "UnderVoltage "; mask <<= 1;
						if (faults & mask) str += "ForwardLimitSwitch "; mask <<= 1;
						if (faults & mask) str += "ReverseLimitSwitch "; mask <<= 1;
						if (faults & mask) str += "ForwardSoftLimit "; mask <<= 1;
						if (faults & mask) str += "ReverseSoftLimit "; mask <<= 1;
						if (faults & mask) str += "ResetDuringEn "; mask <<= 1;
						if (faults & mask) str += "SensorOverflow "; mask <<= 1;
						if (faults & mask) str += "SensorOutOfPhase "; mask <<= 1;
						if (faults & mask) str += "HardwareESDReset "; mask <<= 1;
						if (faults & mask) str += "RemoteLossOfSignal ";
					}
					m.sticky_faults[i] = str;
				}

				hardware_interface::CustomProfileStatus temp_status = ts->getCustomProfileStatus();
				custom_profile_status_holder.running = temp_status.running;
				custom_profile_status_holder.slotRunning = temp_status.slotRunning;
				custom_profile_status_holder.remainingPoints  = temp_status.remainingPoints;
				custom_profile_status_holder.remainingTime = temp_status.remainingTime;
				custom_profile_status_holder.outOfPoints = temp_status.outOfPoints;

				m.custom_profile_status[i] = custom_profile_status_holder;

				m.conversion_factor[i] = ts->getConversionFactor();
			}
			realtime_pub_->unlockAndPublish();
		}
	}
}

void TalonStateController::stopping(const ros::Time & /*time*/)
{}

void TalonStateController::addExtraJoints(const ros::NodeHandle &nh,
		talon_state_controller::TalonState &msg)
{

	// Preconditions
	XmlRpc::XmlRpcValue list;
	if (!nh.getParam("extra_joints", list))
	{
		ROS_DEBUG("No extra joints specification found.");
		return;
	}

	if (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
	{
		ROS_ERROR("Extra joints specification is not an array. Ignoring.");
		return;
	}

	for (int i = 0; i < list.size(); ++i)
	{
		if (list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
		{
			ROS_ERROR_STREAM("Extra joint specification is not a struct, but rather '" << list[i].getType() <<
							 "'. Ignoring.");
			continue;
		}

		if (!list[i].hasMember("name"))
		{
			ROS_ERROR_STREAM("Extra joint does not specify name. Ignoring.");
			continue;
		}

		const std::string name = list[i]["name"];
		if (std::find(msg.name.begin(), msg.name.end(), name) != msg.name.end())
		{
			ROS_WARN_STREAM("Joint state interface already contains specified extra joint '" << name << "'.");
			continue;
		}

		const bool has_pos = list[i].hasMember("position");
		const bool has_vel = list[i].hasMember("velocity");
		const bool has_eff = list[i].hasMember("effort");

		const XmlRpc::XmlRpcValue::Type typeDouble = XmlRpc::XmlRpcValue::TypeDouble;
		if (has_pos && list[i]["position"].getType() != typeDouble)
		{
			ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default position. Ignoring.");
			continue;
		}
		if (has_vel && list[i]["velocity"].getType() != typeDouble)
		{
			ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default velocity. Ignoring.");
			continue;
		}
		if (has_eff && list[i]["effort"].getType() != typeDouble)
		{
			ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default effort. Ignoring.");
			continue;
		}

		// State of extra joint
		const double pos = has_pos ? static_cast<double>(list[i]["position"]) : 0.0;
		const double vel = has_vel ? static_cast<double>(list[i]["velocity"]) : 0.0;
		const double eff = has_eff ? static_cast<double>(list[i]["effort"])   : 0.0;

		// Add extra joints to message
		msg.position.push_back(pos);
		msg.speed.push_back(vel);
		msg.output_voltage.push_back(eff);
		msg.can_id.push_back(0);
	}
}

}

PLUGINLIB_EXPORT_CLASS( talon_state_controller::TalonStateController, controller_interface::ControllerBase)
