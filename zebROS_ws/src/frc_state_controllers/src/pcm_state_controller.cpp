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

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <frc_interfaces/pcm_state_interface.h>
#include <frc_msgs/PCMState.h>
#include <pluginlib/class_list_macros.h>
#include "periodic_interval_counter/periodic_interval_counter.h"

namespace pcm_state_controller
{
/**
 * \brief Controller that publishes the state of all PCMs in a robot.
 *
 * This controller publishes the state of all resources registered to a \c hardware_interface::PCMStateInterface to a
 * topic of type \c pcm_state_controller/PCMState. The following is a basic configuration of the controller.
 *
 * \code
 * pcm_state_controller:
 *   type: pcm_state_controller/PCMStateController
 *   publish_rate: 50
 * \endcode
 *
 */
class PCMStateController: public controller_interface::Controller<hardware_interface::PCMStateInterface>
{
private:
	std::vector<hardware_interface::PCMStateHandle> pcm_state_;
	std::unique_ptr<realtime_tools::RealtimePublisher<frc_msgs::PCMState> > realtime_pub_;
	std::unique_ptr<PeriodicIntervalCounter> interval_counter_;
	double publish_rate_{20};
	size_t num_pcms_{0};

public:
	bool init(hardware_interface::PCMStateInterface *hw,
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
		{
			ROS_DEBUG("Got joint %s", pcm_names[i].c_str());
		}

		// get publishing period
		if (!controller_nh.getParam("publish_rate", publish_rate_))
		{
			ROS_WARN_STREAM("Parameter 'publish_rate' not set, using default " << publish_rate_);
		}
		else if (publish_rate_ <= 0.0)
		{
			ROS_ERROR_STREAM("Invalid publish_rate in pcm controller (" << publish_rate_ << ")");
			return false;
		}
		interval_counter_ = std::make_unique<PeriodicIntervalCounter>(publish_rate_);

		// realtime publisher
		realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<frc_msgs::PCMState>>(root_nh, "pcm_states", 2);

		// get joints and allocate message
		auto &m = realtime_pub_->msg_;
		for (size_t i = 0; i < num_pcms_; i++)
		{
			m.name.push_back(pcm_names[i]);
			m.id.push_back(-1);
			m.compressor_enabled.push_back(false);
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
			m.solenoid_disabled_list.push_back(0);

			pcm_state_.push_back(hw->getHandle(pcm_names[i]));
		}

		return true;
	}

	void starting(const ros::Time &time)
	{
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
				auto &m = realtime_pub_->msg_;
				m.header.stamp = time;
				for (unsigned i = 0; i < num_pcms_; i++)
				{
					auto &pcms = pcm_state_[i];
					m.id[i] = pcms->getId();
					m.compressor_enabled[i] = pcms->getCompressorEnabled();
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
					m.voltage_sticky_fault[i] = pcms->getVoltageSticky();
					m.solenoid_disabled_list[i] = pcms->getSolenoidDisabledList();
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
}; // class

} // namespace

PLUGINLIB_EXPORT_CLASS(pcm_state_controller::PCMStateController, controller_interface::ControllerBase)
