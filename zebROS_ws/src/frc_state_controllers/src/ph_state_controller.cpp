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
#include "frc_state_controllers/ph_state_controller.h"

namespace ph_state_controller
{

bool PHStateController::init(hardware_interface::PHStateInterface *hw,
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
		ROS_DEBUG("Got joint %s", ph_names[i].c_str());

	// get publishing period
	if (!controller_nh.getParam("publish_rate", publish_rate_))
	{
		ROS_ERROR("Parameter 'publish_rate' not set");
		return false;
	}

	// realtime publisher
	realtime_pub_.reset(new
						realtime_tools::RealtimePublisher<frc_msgs::PHState>(root_nh, "ph_states", 4));

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

void PHStateController::starting(const ros::Time &time)
{
	// initialize time
	last_publish_time_ = time;
}

void PHStateController::update(const ros::Time &time, const ros::Duration & /*period*/)
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
	}
}

void PHStateController::stopping(const ros::Time & /*time*/)
{}

}

PLUGINLIB_EXPORT_CLASS(ph_state_controller::PHStateController, controller_interface::ControllerBase)
