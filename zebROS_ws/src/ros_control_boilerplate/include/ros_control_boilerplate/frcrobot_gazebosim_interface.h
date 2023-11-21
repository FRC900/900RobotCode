/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Original Author: Dave Coleman
*/

#ifndef INC_FRCROBOT_GAZEBOSIM_INTERFACE
#define INC_FRCROBOT_GAZEBOSIM_INTERFACE

#include "gazebo_frcrobot_control/robot_hw_sim.h"
#include "ros_control_boilerplate/frcrobot_sim_interface.h"

namespace frcrobot_control
{
/// \brief Hardware interface for a robot
class FRCRobotGazeboSimInterface : public gazebo_ros_control::RobotHWSim
{
public:
	/**
	 * \brief Constructor
	 * \param nh - Node handle for topics.
	 */
	FRCRobotGazeboSimInterface() = default;
	FRCRobotGazeboSimInterface(const FRCRobotGazeboSimInterface &) = delete;
	FRCRobotGazeboSimInterface(const FRCRobotGazeboSimInterface &&) noexcept = delete;
	~FRCRobotGazeboSimInterface() override = default;

	FRCRobotGazeboSimInterface &operator=(const FRCRobotGazeboSimInterface &) = delete;
	FRCRobotGazeboSimInterface &operator=(const FRCRobotGazeboSimInterface &&) noexcept = delete;

	/// \brief Initialize the simulated robot hardware
	///
	/// Initialize the simulated robot hardware.
	///
	/// \param robot_namespace  Robot namespace.
	/// \param model_nh  Model node handle.
	/// \param parent_model  Parent model.
	/// \param urdf_model  URDF model.
	/// \param transmissions  Transmissions.
	///
	/// \return  \c true if the simulated robot hardware is initialized successfully, \c false if not.
	bool initSim(
		const std::string &robot_namespace,
		ros::NodeHandle model_nh,
		gazebo::physics::ModelPtr parent_model,
		const urdf::Model *const urdf_model,
		const std::vector<transmission_interface::TransmissionInfo> &transmissions) override;

	/// \brief Read state data from the simulated robot hardware
	///
	/// Read state data, such as joint positions and velocities, from the simulated robot hardware.
	///
	/// \param time  Simulation time.
	/// \param period  Time since the last simulation step.
	void readSim(ros::Time time, ros::Duration period) override;

	/// \brief Write commands to the simulated robot hardware
	///
	/// Write commands, such as joint position and velocity commands, to the simulated robot hardware.
	///
	/// \param time  Simulation time.
	/// \param period  Time since the last simulation step.
	void writeSim(ros::Time time, ros::Duration period) override;

	/// \brief Set the emergency stop state
	///
	/// Set the simulated robot's emergency stop state. The default implementation of this function does nothing.
	///
	/// \param active  \c true if the emergency stop is active, \c false if not.
	void eStopActive(const bool active) override {}

private:
	ros_control_boilerplate::FRCRobotSimInterface frcrobot_sim_interface_;
	std::string physics_type_;
	// e_stop_active_ is true if the emergency stop is active.
	bool e_stop_active_;
	bool last_e_stop_active_;
}; // class

} // namespace

#endif