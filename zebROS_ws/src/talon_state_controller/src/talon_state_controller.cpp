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
						realtime_tools::RealtimePublisher<talon_state_msgs::TalonState>(root_nh, "talon_states", 4));

	auto &m = realtime_pub_->msg_;
	// get joints and allocate message
	talon_state_msgs::CustomProfileStatus custom_profile_status_holder;

	for (size_t i = 0; i < num_hw_joints_; i++)
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

		m.set_point.push_back(0.0);
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
		m.neutral_output.push_back(false);

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

		m.faults.push_back("");
		m.sticky_faults.push_back("");
		m.custom_profile_status.push_back(custom_profile_status_holder);

		talon_state_.push_back(hw->getHandle(joint_names[i]));
	}

	return true;
}

void TalonStateController::starting(const ros::Time &time)
{
	// initialize time
	last_publish_time_ = time;
}

void TalonStateController::update(const ros::Time &time, const ros::Duration & /*period*/)
{
	talon_state_msgs::CustomProfileStatus custom_profile_status_holder;

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
				m.set_point[i] = ts->getSetpoint();
				m.position[i] = ts->getPosition();
				m.speed[i] = ts->getSpeed();
				m.output_voltage[i] = ts->getOutputVoltage();
				m.output_current[i] = ts->getOutputCurrent();
				m.bus_voltage[i] = ts->getBusVoltage();
				m.motor_output_percent[i] = ts->getMotorOutputPercent();
				m.temperature[i] = ts->getTemperature();

				m.closed_loop_error[i] = ts->getClosedLoopError();
				m.integral_accumulator[i] = ts->getIntegralAccumulator();
				m.error_derivative[i] = ts->getErrorDerivative();
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
					case hardware_interface::TalonMode_Music:
						m.talon_mode[i] = "Music";
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
				m.neutral_output[i] = ts->getNeutralOutput();
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

				{
					unsigned faults = ts->getFaults();
					unsigned int mask = 1;
					std::string str;
					if (faults)
					{
						if (faults & mask) str += "UnderVoltage ";
						mask <<= 1;
						if (faults & mask) str += "ForwardLimitSwitch ";
						mask <<= 1;
						if (faults & mask) str += "ReverseLimitSwitch ";
						mask <<= 1;
						if (faults & mask) str += "ForwardSoftLimit ";
						mask <<= 1;
						if (faults & mask) str += "ReverseSoftLimit ";
						mask <<= 1;
						if (faults & mask) str += "HardwareFailure ";
						mask <<= 1;
						if (faults & mask) str += "ResetDuringEn ";
						mask <<= 1;
						if (faults & mask) str += "SensorOverflow ";
						mask <<= 1;
						if (faults & mask) str += "SensorOutOfPhase ";
						mask <<= 1;
						if (faults & mask) str += "HardwareESDReset ";
						mask <<= 1;
						if (faults & mask) str += "RemoteLossOfSignal ";
						mask <<= 1;
						if (faults & mask) str += "APIError ";
						mask <<= 1;
						if (faults & mask) str += "SupplyOverV ";
						mask <<= 1;
						if (faults & mask) str += "SupplyUnstable ";
					}
					m.faults[i] = str;
				}

				{
					unsigned faults = ts->getStickyFaults();
					unsigned int mask = 1;
					std::string str;
					if (faults)
					{
						if (faults & mask) str += "UnderVoltage ";
						mask <<= 1;
						if (faults & mask) str += "ForwardLimitSwitch ";
						mask <<= 1;
						if (faults & mask) str += "ReverseLimitSwitch ";
						mask <<= 1;
						if (faults & mask) str += "ForwardSoftLimit ";
						mask <<= 1;
						if (faults & mask) str += "ReverseSoftLimit ";
						mask <<= 1;
						if (faults & mask) str += "ResetDuringEn ";
						mask <<= 1;
						if (faults & mask) str += "SensorOverflow ";
						mask <<= 1;
						if (faults & mask) str += "SensorOutOfPhase ";
						mask <<= 1;
						if (faults & mask) str += "HardwareESDReset ";
						mask <<= 1;
						if (faults & mask) str += "RemoteLossOfSignal ";
						mask <<= 1;
						if (faults & mask) str += "APIError ";
						mask <<= 1;
						if (faults & mask) str += "SupplyOverV ";
						mask <<= 1;
						if (faults & mask) str += "SupplyUnstable ";
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
			}
			realtime_pub_->unlockAndPublish();
		}
	}
}

void TalonStateController::stopping(const ros::Time & /*time*/)
{}

} // namespate talon_state_controller

namespace state_listener_controller
{
TalonStateListenerController::TalonStateListenerController() {}
TalonStateListenerController::~TalonStateListenerController()
{
	sub_command_.shutdown();
}

bool TalonStateListenerController::init(hardware_interface::RemoteTalonStateInterface *hw, ros::NodeHandle &n)
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

	sub_command_ = n.subscribe<talon_state_msgs::TalonState>(topic, 1, &TalonStateListenerController::commandCB, this);
	return true;
}

void TalonStateListenerController::starting(const ros::Time & /*time*/)
{
}
void TalonStateListenerController::stopping(const ros::Time & /*time*/)
{
	//handles_.release();
}

void TalonStateListenerController::update(const ros::Time & /*time*/, const ros::Duration & /*period*/)
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

void TalonStateListenerController::commandCB(const talon_state_msgs::TalonStateConstPtr &msg)
{
	// Each entry in data corresponds to an index in joint_names_
	// If the message has that name, copy the talon state into data
	// If the message doesn't have that name, set the entry to std::nullopt
	std::vector<std::optional<hardware_interface::TalonHWState>> data;
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
} // namespace state_listener_controller

PLUGINLIB_EXPORT_CLASS(talon_state_controller::TalonStateController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(state_listener_controller::TalonStateListenerController, controller_interface::ControllerBase)
