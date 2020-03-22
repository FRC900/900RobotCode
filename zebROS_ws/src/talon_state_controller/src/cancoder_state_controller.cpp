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

#include "talon_state_controller/cancoder_state_controller.h"

namespace cancoder_state_controller
{
bool CANCoderStateController::init(hardware_interface::cancoder::CANCoderStateInterface *hw,
								   ros::NodeHandle                                      &root_nh,
								   ros::NodeHandle                                      &controller_nh)
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
	realtime_pub_ = std::make_shared<realtime_tools::RealtimePublisher<talon_state_msgs::CANCoderState>>(root_nh, "cancoder_states", 2);

	auto &m = realtime_pub_->msg_;

	// get joints and allocate message
	for (size_t i = 0; i < num_hw_joints_; i++)
	{
		m.name.push_back(joint_names[i]);
		cancoder_state_.push_back(hw->getHandle(joint_names[i]));

		m.device_number.push_back(cancoder_state_.back()->getDeviceNumber());
		m.position.push_back(0);
		m.velocity.push_back(0);
		m.absolute_position.push_back(0);
		m.velocity_meas_period.push_back("");
		m.velocity_meas_window.push_back(0);
		m.absolute_sensor_range.push_back("");
		m.magnet_offset.push_back(0);
		m.initialization_strategy.push_back("");
		m.feedback_coefficient.push_back(0);
		m.unit_string.push_back("");
		m.time_base.push_back("");
		m.bus_voltage.push_back(0);
		m.magnet_field_strength.push_back("");
		m.direction.push_back(false);
		m.last_timestamp.push_back(0);
		m.sensor_data_status_frame_period.push_back(0);
		m.vbat_and_faults_status_frame_period.push_back(0);
		m.firmware_version.push_back("");
		m.faults.push_back("");
		m.sticky_faults.push_back("");
		m.conversion_factor.push_back(0);
	}

	return true;
}

void CANCoderStateController::starting(const ros::Time &time)
{
	// initialize time
	last_publish_time_ = time;
}

void CANCoderStateController::update(const ros::Time &time, const ros::Duration & /*period*/)
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
				auto &cs = cancoder_state_[i];

				m.position[i] = cs->getPosition();
				m.velocity[i] = cs->getVelocity();
				m.absolute_position[i] = cs->getAbsolutePosition();
				switch (cs->getVelocityMeasPeriod())
				{
					case hardware_interface::cancoder::SensorVelocityMeasPeriod::Sensor_Period_1Ms:
						m.velocity_meas_period[i] = "1Ms";
						break;
					case hardware_interface::cancoder::SensorVelocityMeasPeriod::Sensor_Period_2Ms:
						m.velocity_meas_period[i] = "2Ms";
						break;
					case hardware_interface::cancoder::SensorVelocityMeasPeriod::Sensor_Period_5Ms:
						m.velocity_meas_period[i] = "5Ms";
						break;
					case hardware_interface::cancoder::SensorVelocityMeasPeriod::Sensor_Period_10Ms:
						m.velocity_meas_period[i] = "10Ms";
						break;
					case hardware_interface::cancoder::SensorVelocityMeasPeriod::Sensor_Period_20Ms:
						m.velocity_meas_period[i] = "20Ms";
						break;
					case hardware_interface::cancoder::SensorVelocityMeasPeriod::Sensor_Period_25Ms:
						m.velocity_meas_period[i] = "25Ms";
						break;
					case hardware_interface::cancoder::SensorVelocityMeasPeriod::Sensor_Period_50Ms:
						m.velocity_meas_period[i] = "50Ms";
						break;
					case hardware_interface::cancoder::SensorVelocityMeasPeriod::Sensor_Period_100Ms:
						m.velocity_meas_period[i] = "100Ms";
						break;
					default:
						m.velocity_meas_period[i] = "Unknown";
						break;
				}
				m.velocity_meas_window[i] = cs->getVelocityMeasWindow();
				switch (cs->getAbsoluteSensorRange())
				{
					case hardware_interface::cancoder::AbsoluteSensorRange::Unsigned_0_to_360:
						m.absolute_sensor_range[i] = "Unsigned_0_to_360";
						break;
					case hardware_interface::cancoder::AbsoluteSensorRange::Signed_PlusMinus180:
						m.absolute_sensor_range[i] = "Signed_PlusMinus180";
						break;
					default:
						m.absolute_sensor_range[i] = "Unknown";
						break;
				}
				m.magnet_offset[i] = cs->getMagnetOffset();
				switch(cs->getInitializationStrategy())
				{
					case hardware_interface::cancoder::SensorInitializationStrategy::BootToAbsolutePosition:
						m.initialization_strategy[i] = "BootToAbsolutePosition";
						break;
					case hardware_interface::cancoder::SensorInitializationStrategy::BootToZero:
						m.initialization_strategy[i] = "BootToZero";
						break;
					default:
						m.initialization_strategy[i] = "Unknown";
						break;
				}
				m.feedback_coefficient[i] = cs->getFeedbackCoefficient();
				m.unit_string[i] = cs->getUnitString();
				switch (cs->getTimeBase())
				{
					case hardware_interface::cancoder::SensorTimeBase::Per100Ms_Legacy:
						m.time_base[i] = "Per100Ms_Legacy";
						break;
					case hardware_interface::cancoder::SensorTimeBase::PerSecond:
						m.time_base[i] = "PerSecond";
						break;
					case hardware_interface::cancoder::SensorTimeBase::PerMinute:
						m.time_base[i] = "PerMinute";
						break;
					default:
						m.time_base[i] = "Unknown";
						break;
				}
				m.bus_voltage[i] = cs->getBusVoltage();
				switch (cs->getMagnetFieldStrength())
				{
					case hardware_interface::cancoder::MagnetFieldStrength::BadRange_RedLED:
						m.magnet_field_strength[i] = "BadRange_RedLED";
						break;
					case hardware_interface::cancoder::MagnetFieldStrength::Adequate_OrangeLED:
						m.magnet_field_strength[i] = "Adequate_OrangeLED";
						break;
					case hardware_interface::cancoder::MagnetFieldStrength::Good_GreenLED:
						m.magnet_field_strength[i] = "Good_GreenLED";
						break;
					case hardware_interface::cancoder::MagnetFieldStrength::Invalid_Unknown:
						m.magnet_field_strength[i] = "Invalid_Unknown";
						break;
					default:
						m.magnet_field_strength[i] = "Unknown";
						break;
				}
				m.direction[i] = cs->getDirection();
				m.last_timestamp[i] = cs->getLastTimestamp();
				m.sensor_data_status_frame_period[i] = cs->getSensorDataStatusFramePeriod();
				m.vbat_and_faults_status_frame_period[i] = cs->getVbatAndFaultsStatusFramePeriod();
				const int fw_ver = cs->getFirmwareVersion();

				if (fw_ver >= 0)
				{
					char str[256];
					sprintf(str, "2.2%d.%2.2d", (fw_ver >> 8) & 0xFF, fw_ver & 0xFF);
					m.firmware_version[i] = str;
				}
				const auto faults = cs->getFaults();
				m.faults[i] = "";
				unsigned mask = 0;
				if (faults & mask) m.faults[i] += "HardwareFault, ";
				mask <<= 1;
				if (faults & mask) m.faults[i] += "APIError ";
				mask <<= 1;
				if (faults & mask) m.faults[i] += "UnderVoltage ";
				mask <<= 1;
				if (faults & mask) m.faults[i] += "ResetDuringEn ";
				mask <<= 4; // 1 + 3 unused faults
				if (faults & mask) m.faults[i] += "MagnetTooWeak ";

				const auto sticky_faults = cs->getStickyFaults();
				m.sticky_faults[i] = "";
				mask = 0;
				if (sticky_faults & mask) m.sticky_faults[i] += "HardwareFault, ";
				mask <<= 1;
				if (sticky_faults & mask) m.sticky_faults[i] += "APIError ";
				mask <<= 1;
				if (sticky_faults & mask) m.sticky_faults[i] += "UnderVoltage ";
				mask <<= 1;
				if (sticky_faults & mask) m.sticky_faults[i] += "ResetDuringEn ";
				mask <<= 4; // 1 + 3 unused sticky_faults
				if (sticky_faults & mask) m.sticky_faults[i] += "MagnetTooWeak ";
			}
			realtime_pub_->unlockAndPublish();
		}
	}
}

void CANCoderStateController::stopping(const ros::Time & /*time*/)
{}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cancoder_state_controller::CANCoderStateController, controller_interface::ControllerBase)
