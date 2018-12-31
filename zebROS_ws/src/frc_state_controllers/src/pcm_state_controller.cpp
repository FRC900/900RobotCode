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
#include "frc_state_controllers/pcm_state_controller.h"

namespace pcm_state_controller 
{

bool PCMStateController::init(hardware_interface::PCMStateInterface *hw,
							  ros::NodeHandle                       &root_nh,
							  ros::NodeHandle                       &controller_nh)
{
	ROS_INFO_NAMED("pcm_state_controller", "PCMStateController::init() called");
	// get all joint names from the hardware interface
	const std::vector<std::string> &pcm_names = hw->getNames();
	num_pcms_ = pcm_names.size();
	if (num_pcms_ < 1)
	{
		ROS_ERROR_STREAM("Cannot initialize zero PCMs - need to add a compressor joint def?");
		return false;
	}
	for (size_t i = 0; i < num_pcms_; i++)
		ROS_DEBUG("Got joint %s", pcm_names[i].c_str());

	// get publishing period
	if (!controller_nh.getParam("publish_rate", publish_rate_))
	{
		ROS_ERROR("Parameter 'publish_rate' not set");
		return false;
	}

	// realtime publisher
	realtime_pub_.reset(new
						realtime_tools::RealtimePublisher<frc_msgs::PCMState>(root_nh, "pcm_states", 4));

	// get joints and allocate message
	auto &m = realtime_pub_->msg_;
	for (size_t i = 0; i < num_pcms_; i++)
	{
		m.name.push_back(pcm_names[i]);
		m.pcm_id.push_back(-1);
		m.enabled.push_back(false);
		m.pressure_switch.push_back(false);
		m.compressor_current.push_back(0.0);
		m.closed_loop_control.push_back(false);
		m.current_too_high.push_back(false);
		m.current_too_high_sticky.push_back(false);
		m.shorted.push_back(false);
		m.shorted_sticky.push_back(false);
		m.not_connected.push_back(false);
		m.not_connected_sticky.push_back(false);
		m.voltage_fault.push_back(false);
		m.voltage_sticky_fault.push_back(false);
		m.solenoid_blacklist.push_back(0);

		pcm_state_.push_back(hw->getHandle(pcm_names[i]));
	}

	return true;
}

void PCMStateController::starting(const ros::Time &time)
{
	// initialize time
	last_publish_time_ = time;
}

void PCMStateController::update(const ros::Time &time, const ros::Duration & /*period*/)
{
	// limit rate of publishing
	if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
	{
		// try to publish
		if (realtime_pub_->trylock())
		{
			// we're actually publishing, so increment time
			last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

			auto &m = realtime_pub_->msg_;
			m.header.stamp = time;
			for (unsigned i = 0; i < num_pcms_; i++)
			{
				auto &pcms = pcm_state_[i];
				m.pcm_id[i] = pcms->getPCMId();
				m.enabled[i] = pcms->getEnabled();
				m.pressure_switch[i] = pcms->getPressureSwitch();
				m.compressor_current[i] = pcms->getCompressorCurrent();
				m.closed_loop_control[i] = pcms->getClosedLoopControl();
				m.current_too_high[i] = pcms->getCurrentTooHigh();
				m.current_too_high_sticky[i] = pcms->getCurrentTooHighSticky();
				m.shorted[i] = pcms->getShorted();
				m.shorted_sticky[i] = pcms->getShortedSticky();
				m.not_connected[i] = pcms->getNotConnected();
				m.not_connected_sticky[i] = pcms->getNotConnectedSticky();
				m.voltage_fault[i] = pcms->getVoltageFault();
				m.voltage_sticky_fault[i] = pcms->getVoltageStickyFault();
				m.solenoid_blacklist[i] = pcms->getSolenoidBlacklist();
			}
			realtime_pub_->unlockAndPublish();
		}
	}
}

void PCMStateController::stopping(const ros::Time & /*time*/)
{}

}

PLUGINLIB_EXPORT_CLASS(pcm_state_controller::PCMStateController, controller_interface::ControllerBase)
