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
//   * Neither the name of hiDOF, Inc. nor the names of its
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

#pragma once

#include <atomic>

#include <controller_interface/controller.h>

#include "ddynamic_reconfigure/ddynamic_reconfigure.h"
#include "as726x_interface/as726x_interface.h"

namespace as726x_controller
{

/**
 * \brief Controller for changing configuration of AS726x color sensor
 *
 * This controller reads params and monitors dynamic reconfigure settings for an AS726x color
 * sensor.  It then forwards those settings to the hardware
 *
 * \code
 * as726x_controller:
 *   type: as726x_controller/AS726xController
 *   joint_name: <as726 hardware name>
 * \endcode
 *
 */
class AS726xController: public controller_interface::Controller<hardware_interface::as726x::AS726xCommandInterface>
{
	public:
		AS726xController(void) {}

		bool init(hardware_interface::as726x::AS726xCommandInterface *hw,
						  ros::NodeHandle                                    &root_nh,
						  ros::NodeHandle                                    &controller_nh) override;
		void starting(const ros::Time &time) override;
		void update(const ros::Time &time, const ros::Duration & /*period*/) override;
		void stopping(const ros::Time & /*time*/) override;

	private:
		hardware_interface::as726x::AS726xCommandHandle              as726x_command_;
		std::atomic<hardware_interface::as726x::IndLedCurrentLimits> ind_led_current_limit_;
		std::atomic<bool>                                            ind_led_enable_;
		std::atomic<hardware_interface::as726x::DrvLedCurrentLimits> drv_led_current_limit_;
		std::atomic<bool>                                            drv_led_enable_;
		std::atomic<hardware_interface::as726x::ConversionTypes>     conversion_type_;
		std::atomic<hardware_interface::as726x::ChannelGain>         channel_gain_;
		std::atomic<uint8_t>                                         integration_time_;
		ddynamic_reconfigure::DDynamicReconfigure                    ddr_;

		const std::map<std::string, int> ind_led_current_limit_enum_map_ =
		{
			{"IND_LIMIT_1MA", static_cast<int>(hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_1MA)},
			{"IND_LIMIT_2MA", static_cast<int>(hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_2MA)},
			{"IND_LIMIT_4MA", static_cast<int>(hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_4MA)},
			{"IND_LIMIT_8MA", static_cast<int>(hardware_interface::as726x::IndLedCurrentLimits::IND_LIMIT_8MA)}
		};
		const std::map<std::string, int> drv_led_current_limit_enum_map_ =
		{
			{"DRV_LIMIT_12MA5", hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_12MA5},
			{"DRV_LIMIT_25MA", hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_25MA},
			{"DRV_LIMIT_50MA", hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_50MA},
			{"DRV_LIMIT_100MA", hardware_interface::as726x::DrvLedCurrentLimits::DRV_LIMIT_100MA}
		};
		const std::map<std::string, int> conversion_types_enum_map_ =
		{
			{"MODE_0", hardware_interface::as726x::ConversionTypes::MODE_0},
			{"MODE_1", hardware_interface::as726x::ConversionTypes::MODE_1},
			{"MODE_2", hardware_interface::as726x::ConversionTypes::MODE_2},
			{"ONE_SHOT", hardware_interface::as726x::ConversionTypes::ONE_SHOT}
		};

		const std::map<std::string, int> channel_gain_enum_map_ =
		{
			{"GAIN_1X", hardware_interface::as726x::ChannelGain::GAIN_1X},
			{"GAIN_3X7", hardware_interface::as726x::ChannelGain::GAIN_3X7},
			{"GAIN_16X", hardware_interface::as726x::ChannelGain::GAIN_16X},
			{"GAIN_64X", hardware_interface::as726x::ChannelGain::GAIN_64X}
		};


		bool convertIndLedCurrentLimit(const std::string &param_str,
				std::atomic<hardware_interface::as726x::IndLedCurrentLimits> &ind_led_current_limit) const;
		bool convertDrvLedCurrentLimit(const std::string &param_str,
				std::atomic<hardware_interface::as726x::DrvLedCurrentLimits> &drv_led_current_limit) const;
		bool convertChannelGain(const std::string &param_str,
				std::atomic<hardware_interface::as726x::ChannelGain> &channel_gain) const;
		bool convertConversionType(const std::string &param_str,
				std::atomic<hardware_interface::as726x::ConversionTypes> &conversion_type) const;

		template <typename T>
		bool convertStringToEnum(const std::string &param_str,
				const std::string &param_name,
				const std::map<std::string, int> &mymap,
				std::atomic<T>&out) const
		{
			const auto it = mymap.find(param_str);
			if (it != mymap.cend())
			{
				out = static_cast<T>(it->second);
				return true;
			}
			ROS_ERROR_STREAM("Could not convert param_name "
					<< param_name
					<< " param_str="
					<< param_str
					<< " didn't match list of valid types");
			return false;
		}
		void indLedCurrentLimitCB(int ind_led_current_limit);
		void indLedEnableCB(bool ind_led_enable);
		void drvLedCurrentLimitCB(int drv_led_current_limit);
		void drvLedEnableCB(bool drv_led_enable);
		void conversionTypeCB(int conversion_type);
		void channelGainCB(int channel_gain);
		void integrationTimeCB(uint8_t integration_time);

		int  getIndLedCurrentLimit(void) const;
		bool getIndLedEnable(void) const;
		int  getDrvLedCurrentLimit(void) const;
		bool getDrvLedEnable(void) const;
		int  getConversionType(void) const;
		int  getChannelGain(void) const;
		int  getIntegrationTime(void) const;
};
} // namespace
