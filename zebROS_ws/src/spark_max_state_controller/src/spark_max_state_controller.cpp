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
#include "spark_max_state_controller/spark_max_state_controller.h"

namespace spark_max_state_controller
{

bool SparkMaxStateController::init(hardware_interface::SparkMaxStateInterface *hw,
								ros::NodeHandle                               &root_nh,
								ros::NodeHandle                               &controller_nh)
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
						realtime_tools::RealtimePublisher<spark_max_state_msgs::SparkMaxState>(root_nh, "spark_max_states", 2));

	auto &m = realtime_pub_->msg_;
	// get joints and allocate message

	for (size_t i = 0; i < num_hw_joints_; i++)
	{
		m.name.push_back(joint_names[i]);
		m.device_id.push_back(0);
		m.set_point.push_back(0);
		m.position.push_back(0);
		m.velocity.push_back(0);
		m.forward_limit_switch.push_back(false);
		m.reverse_limit_switch.push_back(false);
		m.forward_softlimit.push_back(false);
		m.reverse_softlimit.push_back(false);

		m.faults.push_back("");
		m.sticky_faults.push_back("");

		m.bus_voltage.push_back(0);
		m.applied_output.push_back(0);
		m.output_current.push_back(0);
		m.motor_temperature.push_back(0);

		spark_max_state_.push_back(hw->getHandle(joint_names[i]));
	}

	return true;
}

void SparkMaxStateController::starting(const ros::Time &time)
{
	// initialize time
	last_publish_time_ = time;
}

std::string SparkMaxStateController::faultsToString(unsigned faults) const
{
	std::string str;
	if (faults)
	{
		unsigned mask = 1;
		if (faults & mask) str += "Brownout ";
		mask <<= 1;
		if (faults & mask) str += "Overcurrent ";
		mask <<= 1;
		if (faults & mask) str += "IWDTReset ";
		mask <<= 1;
		if (faults & mask) str += "MotorFault ";
		mask <<= 1;
		if (faults & mask) str += "SensorFault ";
		mask <<= 1;
		if (faults & mask) str += "Stall ";
		mask <<= 1;
		if (faults & mask) str += "EEPROMCRC ";
		mask <<= 1;
		if (faults & mask) str += "CANTX ";
		mask <<= 1;
		if (faults & mask) str += "CANRX ";
		mask <<= 1;
		if (faults & mask) str += "HasReset ";
		mask <<= 1;
		if (faults & mask) str += "DRVFault ";
		mask <<= 1;
		if (faults & mask) str += "OtherFault ";
		mask <<= 1;
		if (faults & mask) str += "SoftLimitFwd ";
		mask <<= 1;
		if (faults & mask) str += "SoftLimitRev ";
		mask <<= 1;
		if (faults & mask) str += "HardLimitFwd ";
		mask <<= 1;
		if (faults & mask) str += "HardLimitRev ";
	}
	return str;
}

void SparkMaxStateController::update(const ros::Time &time, const ros::Duration & /*period*/)
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
			auto &m = realtime_pub_->msg_;
			m.header.stamp = time;
			for (size_t i = 0; i < num_hw_joints_; i++)
			{
				const auto &sms = spark_max_state_[i];
				m.device_id[i] = sms->getDeviceId();
				m.set_point[i] = sms->getSetPoint();
				m.position[i] = sms->getPosition();
				m.velocity[i] = sms->getVelocity();
				m.forward_limit_switch[i] = sms->getForwardLimitSwitch();
				m.reverse_limit_switch[i] = sms->getReverseLimitSwitch();
				m.forward_softlimit[i] = sms->getForwardSoftlimit();
				m.reverse_softlimit[i] = sms->getReverseSoftlimit();
				m.faults[i] = faultsToString(sms->getFaults());
				m.sticky_faults[i] = faultsToString(sms->getStickyFaults());
				m.bus_voltage[i] = sms->getBusVoltage();
				m.applied_output[i] = sms->getAppliedOutput();
				m.output_current[i] = sms->getOutputCurrent();
				m.motor_temperature[i] = sms->getMotorTemperature();
			}
			realtime_pub_->unlockAndPublish();
		}
	}
}

void SparkMaxStateController::stopping(const ros::Time & /*time*/)
{}

}

PLUGINLIB_EXPORT_CLASS(spark_max_state_controller::SparkMaxStateController, controller_interface::ControllerBase)
