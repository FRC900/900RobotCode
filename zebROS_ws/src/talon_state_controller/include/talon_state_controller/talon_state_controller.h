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
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <talon_interface/talon_state_interface.h>
#include <talon_state_msgs/TalonState.h>

namespace talon_state_controller
{

/**
 * \brief Controller that publishes the state of all talon&victor motor controller on a robot.
 *
 * This controller publishes the state of all resources registered to a \c hardware_interface::TalonStateInterface to a
 * topic of type \c sensor_msgs/TalonState. The following is a basic configuration of the controller.
 *
 * \code
 * talon_state_controller:
 *   type: talon_state_controller/JointStateController
 *   publish_rate: 50
 * \endcode
 *
 */
class TalonStateController: public controller_interface::Controller<hardware_interface::TalonStateInterface>
{
	public:
		TalonStateController() : publish_rate_(0.0) {}

		bool init(hardware_interface::TalonStateInterface *hw,
						  ros::NodeHandle                         &root_nh,
						  ros::NodeHandle                         &controller_nh) override;
		void starting(const ros::Time &time) override;
		void update(const ros::Time &time, const ros::Duration & /*period*/) override;
		void stopping(const ros::Time & /*time*/) override;

	private:
		std::vector<hardware_interface::TalonStateHandle> talon_state_;
		std::shared_ptr<realtime_tools::RealtimePublisher<talon_state_msgs::TalonState> > realtime_pub_;
		ros::Time last_publish_time_;
		double publish_rate_;
		size_t num_hw_joints_; ///< Number of joints present in the TalonStateInterface
};
} // namespace


namespace state_listener_controller
{
	// since not all joint names are guaranteed to be found in the
// joint state message, keep track of which ones have using
// this class. Only write values during update if valid end up
// being true.
template <class T>
class ValueValid
{
	public:
		ValueValid() : valid_(false) { }
		ValueValid(const T &value) : value_(value), valid_(false) {}
		T      value_;
		bool   valid_;
};

class TalonStateListenerController :
	public controller_interface::Controller<hardware_interface::RemoteTalonStateInterface>
{
	public:
		TalonStateListenerController();
		~TalonStateListenerController();

		bool init(hardware_interface::RemoteTalonStateInterface *hw, ros::NodeHandle &n) override;
		void starting(const ros::Time & /*time*/) override;
		void stopping(const ros::Time & /*time*/) override;
		void update(const ros::Time & /*time*/, const ros::Duration & /*period*/) override;

	private:
		ros::Subscriber sub_command_;
		std::vector<std::string> joint_names_;
		std::vector<hardware_interface::TalonWritableStateHandle> handles_;

		// Real-time buffer holds the last command value read from the
		// "command" topic.
		realtime_tools::RealtimeBuffer<std::vector<ValueValid<hardware_interface::TalonHWState>>> command_buffer_;

		void commandCB(const talon_state_msgs::TalonStateConstPtr &msg);
};

} // namespace state_listener_controller
