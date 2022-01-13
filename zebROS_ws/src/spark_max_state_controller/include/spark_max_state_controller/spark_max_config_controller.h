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

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <spark_max_interface/spark_max_state_interface.h>
#include <spark_max_state_msgs/SparkMaxConfig.h>

namespace spark_max_config_controller
{

/**
 * \brief Controller that publishes the config of all spark_maxs in a robot.
 *
 * This controller publishes the config of all resources registered to a \c hardware_interface::SparkMaxConfigInterface to a
 * topic of type \c sensor_msgs/SparkMaxConfig. The following is a basic configuration of the controller.
 *
 * \code
 * spark_max_config_controller:
 *   type: spark_max_config_controller/SparkMaxConfigController
 *   publish_rate: 50
 * \endcode
 *
 */
class SparkMaxConfigController: public controller_interface::Controller<hardware_interface::SparkMaxStateInterface>
{
	public:
		SparkMaxConfigController() : publish_rate_(0.0) {}

		virtual bool init(hardware_interface::SparkMaxStateInterface *hw,
						  ros::NodeHandle                            &root_nh,
						  ros::NodeHandle                            &controller_nh) override;
		virtual void starting(const ros::Time &time) override;
		virtual void update(const ros::Time &time, const ros::Duration & /*period*/) override;
		virtual void stopping(const ros::Time & /*time*/) override;

	private:
		std::vector<hardware_interface::SparkMaxStateHandle> spark_max_state_;
		std::shared_ptr<realtime_tools::RealtimePublisher<spark_max_state_msgs::SparkMaxConfig> > realtime_pub_;
		ros::Time last_publish_time_;
		double publish_rate_;
		size_t num_hw_joints_; ///< Number of joints present in the SparkMaxInterface

		std::string motorTypeToString(hardware_interface::MotorType motor_type) const;
		std::string controlTypeToString(hardware_interface::ControlType control_type) const;
		std::string arbFFUnitsToString(hardware_interface::ArbFFUnits arb_ff_units) const;
		std::string limitSwitchPolarityToString(hardware_interface::LimitSwitchPolarity limit_switch_polarity) const;
		std::string idleModeToString(hardware_interface::IdleMode idle_mode) const;
		std::string externalFollowerToString(hardware_interface::ExternalFollower external_follower) const;
		std::string sensorTypeToString(hardware_interface::SensorType sensor_type) const;
};

}
