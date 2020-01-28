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

//#include <algorithm>
//#include <cstddef>

#include <pluginlib/class_list_macros.h>
#include "as726x_controllers/as726x_state_controller.h"

namespace as726x_state_controller
{

bool AS726xStateController::init(hardware_interface::as726x::AS726xStateInterface *hw,
								 ros::NodeHandle                                  &root_nh,
								 ros::NodeHandle                                  &controller_nh)
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
						realtime_tools::RealtimePublisher<as726x_msgs::AS726xState>(root_nh, "as726x_states", 2));

	auto &m = realtime_pub_->msg_;
	// get joints and allocate message

	for (size_t i = 0; i < num_hw_joints_; i++)
	{
		m.name.push_back(joint_names[i]);
		m.port.push_back("");
		m.address.push_back(0);
		m.ind_led_current_limit.push_back("");
		m.ind_led_enable.push_back(false);
		m.drv_led_current_limit.push_back("");
		m.drv_led_enable.push_back(false);
		m.conversion_type.push_back("");
		m.gain.push_back("");
		m.integration_time.push_back(0);
		m.temperature.push_back(0);
		m.raw_channel_data.push_back(as726x_msgs::AS726xRawChannelData());
		m.calibrated_channel_data.push_back(as726x_msgs::AS726xCalibratedChannelData());
		for (size_t j = 0; j < 6; j++)
		{
			m.raw_channel_data[i].raw_channel_data.push_back(0);
			m.calibrated_channel_data[i].calibrated_channel_data.push_back(0);
		}

		as726x_state_.push_back(hw->getHandle(joint_names[i]));
	}

	return true;
}

void AS726xStateController::starting(const ros::Time &time)
{
	// initialize time
	last_publish_time_ = time;
}

void AS726xStateController::update(const ros::Time &time, const ros::Duration & /*period*/)
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
				auto &as = as726x_state_[i];
				m.port[i] = as->getPort();
				m.address[i] = as->getAddress();
				const auto ind_led_current_limit = as->getIndLedCurrentLimit();
				switch (ind_led_current_limit)
				{
					case hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_1MA:
						m.ind_led_current_limit[i] = "1mA";
						break;
					case hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_2MA:
						m.ind_led_current_limit[i] = "2mA";
						break;
					case hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_4MA:
						m.ind_led_current_limit[i] = "4mA";
						break;
					case hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_8MA:
						m.ind_led_current_limit[i] = "8mA";
						break;
					default:
						m.ind_led_current_limit[i] = "Unknown";
						break;
				}
				m.ind_led_enable[i] = as->getIndLedEnable();
				const auto drv_led_current_limit = as->getDrvLedCurrentLimit();
				switch (drv_led_current_limit)
				{
					case hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_12MA5:
						m.drv_led_current_limit[i] = "12mA5";
						break;
					case hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_25MA:
						m.drv_led_current_limit[i] = "25mA";
						break;
					case hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_50MA:
						m.drv_led_current_limit[i] = "50mA";
						break;
					case hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_100MA:
						m.drv_led_current_limit[i] = "100mA";
						break;
					default:
						m.drv_led_current_limit[i] = "Unknown";
						break;
				}
				m.drv_led_enable[i] = as->getDrvLedEnable();
				const auto conversion_type = as->getConversionType();
				switch (conversion_type)
				{
					case hardware_interface::as726x::ConversionTypes::MODE_0:
						m.conversion_type[i] = "MODE_0";
						break;
					case hardware_interface::as726x::ConversionTypes::MODE_1:
						m.conversion_type[i] = "MODE_1";
						break;
					case hardware_interface::as726x::ConversionTypes::MODE_2:
						m.conversion_type[i] = "MODE_2";
						break;
					case hardware_interface::as726x::ConversionTypes::ONE_SHOT:
						m.conversion_type[i] = "ONE_SHOT";
						break;
					default:
						m.conversion_type[i] = "Unknown";
						break;
				}
				const auto gain = as->getGain();
				switch (gain)
				{
					case hardware_interface::as726x::ChannelGain::GAIN_1X:
						m.gain[i] = "GAIN_1X";
						break;
					case hardware_interface::as726x::ChannelGain::GAIN_3X7:
						m.gain[i] = "GAIN_3X7";
						break;
					case hardware_interface::as726x::ChannelGain::GAIN_16X:
						m.gain[i] = "GAIN_16X";
						break;
					case hardware_interface::as726x::ChannelGain::GAIN_64X:
						m.gain[i] = "GAIN_64X";
						break;
					default:
						m.gain[i] = "Unknown";
						break;
				}
				m.integration_time[i] = static_cast<int>(as->getIntegrationTime());
				m.temperature[i] = as->getTemperature();
				const auto raw_channel_data = as->getRawChannelData();
				const auto calibrated_channel_data = as->getCalibratedChannelData();
				for (size_t j = 0; j < 6; j++)
				{
					m.raw_channel_data[i].raw_channel_data[j] = raw_channel_data[j];
					m.calibrated_channel_data[i].calibrated_channel_data[j] = calibrated_channel_data[j];
				}
			}
			realtime_pub_->unlockAndPublish();
		}
	}
}

void AS726xStateController::stopping(const ros::Time & /*time*/)
{}

}

PLUGINLIB_EXPORT_CLASS(as726x_state_controller::AS726xStateController, controller_interface::ControllerBase)
