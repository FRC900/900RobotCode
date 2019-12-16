/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
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
 *   * Neither the name of the PAL Robotics nor the names of its
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

/*
 * Original Author: Enrique Fernández
 */

#pragma once

#include <array>
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <controller_interface/controller.h>
#include <talon_controllers/talon_controller_interface.h>
#include <pluginlib/class_list_macros.h>

#include <talon_swerve_drive_controller/MotionProfile.h>
#include <talon_swerve_drive_controller/speed_limiter.h>
#include "talon_swerve_drive_controller/SetXY.h"
#include <talon_swerve_drive_controller/Swerve.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/tfMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <talon_state_controller/TalonState.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <Eigen/Dense>

namespace talon_swerve_drive_controller
{

/**
 * This class makes some assumptions on the model of the robot:
 *  - the rotation axes of wheels are collinear
 *  - the wheels are identical in radius
 * Additional assumptions to not duplicate information readily available in the URDF:
 *  - the wheels have the same parent frame
 *  - a wheel collision geometry is a cylinder or sphere in the urdf
 *  - a wheel joint frame center's vertical projection on the floor must lie within the contact patch
*/
class TalonSwerveDriveController
	: public controller_interface::Controller<hardware_interface::TalonCommandInterface>
{
	public:
		TalonSwerveDriveController();

		/**
		 * \brief Initialize controller
		 * \param hw            Velocity joint interface for the wheels
		 * \param root_nh       Node handle at root namespace
		 * \param controller_nh Node handle inside the controller namespace
		 */
		bool init(hardware_interface::TalonCommandInterface *hw,
				  ros::NodeHandle &root_nh,
				  ros::NodeHandle &controller_nh);

		/**
		 * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
		 * \param time   Current time
		 * \param period Time since the last called to update
		 */
		void update(const ros::Time &time, const ros::Duration &period);

		/**
		 * \brief Starts controller
		 * \param time Current time
		 */
		void starting(const ros::Time &time);

		/**
		 * \brief Stops controller
		 * \param time Current time
		 */
		void stopping(const ros::Time & /*time*/);

	private:
		int num_profile_slots_;

		Eigen::MatrixX2d new_wheel_pos_;
		std::array<double, WHEELCOUNT> last_wheel_rot_;	//

		Eigen::Vector2d neg_wheel_centroid_;
		bool comp_odom_;

		std::string name_;

		std::shared_ptr<swerve> swerveC_;

		/// Hardware handles:
		//TODO: IMPORTANT, make generalized, and check
		std::vector<talon_controllers::TalonControllerInterface> speed_joints_;
		std::vector<talon_controllers::TalonControllerInterface> steering_joints_;
		/// Velocity command related:

		struct Commands
		{
			Eigen::Vector2d lin;
			double ang;
			ros::Time stamp;

			Commands() : lin(
			{
				0.0, 0.0
			}), ang(0.0), stamp(0.0) {}
		};

		void compOdometry(const ros::Time &time, const double inv_delta_t, const std::array<double, WHEELCOUNT> &steer_angles);

		struct cmd_points
		{
			std::vector<std::vector<double>> drive_pos;
			std::vector<std::vector<double>> drive_f;
			std::vector<std::vector<double>> steer_pos;
			std::vector<std::vector<double>> steer_f;
			std::vector<std::vector<bool>> hold;
			double dt;
			int slot;

			cmd_points() : dt(0.0), slot(0) {}
			void Print(void) const
			{
				ROS_INFO_STREAM("dt : " << (int)dt);
				ROS_INFO_STREAM("slot : " << (int)slot);
				for (size_t i = 0; i < drive_pos.size(); i++)
				{
					ROS_INFO_STREAM("cmd_point[" << i << "]");
					for (size_t j = 0; j < drive_pos[i].size(); j++)
						ROS_INFO_STREAM("  drive_pos[" << j << "] = " << drive_pos[i][j]);
					for (size_t j = 0; j < steer_pos[i].size(); j++)
						ROS_INFO_STREAM("  steer_pos[" << j << "] = " << steer_pos[i][j]);
					for (size_t j = 0; j < drive_f[i].size(); j++)
						ROS_INFO_STREAM("  drive_f[" << j << "] = " << drive_f[i][j]);
					for (size_t j = 0; j < steer_f[i].size(); j++)
						ROS_INFO_STREAM("  steer_f[" << j << "] = " <<steer_f[i][j]);
				}
			}
		};
		struct full_profile_cmd
		{
			std::vector<cmd_points> profiles;
			bool wipe_all;
			bool buffer;
			bool run;
			bool brake;
			int run_slot;
			bool change_queue;
			std::vector<int> new_queue;
			bool newly_set;
			int id_counter;
			full_profile_cmd() : wipe_all(false), buffer(false), run(false), brake(false),  run_slot(0), change_queue(false), newly_set(false), id_counter(0) {}
			void Print(void) const
			{
				for (const auto & p: profiles)
					p.Print();
				ROS_INFO_STREAM("wipe_all : " << (int)wipe_all);
				ROS_INFO_STREAM("buffer : " << (int)buffer);
				ROS_INFO_STREAM("run : " << (int)run);
				ROS_INFO_STREAM("brake : " << (int)brake);
				ROS_INFO_STREAM("run_slot : " << (int)run_slot);
				ROS_INFO_STREAM("change_queue : " << (int)change_queue);
				ROS_INFO_STREAM("newly_set : " << (int)newly_set);
				ROS_INFO_STREAM("id_counter : " << (int)id_counter);
				for (size_t i = 0; i < new_queue.size(); i++)
					ROS_INFO_STREAM("new_queue[" << i << "] = " << new_queue[i]);
			}
		};

		std::deque<full_profile_cmd> full_profile_buffer_;

		// True if running cmd_vel, false if running profile
		std::atomic<bool> cmd_vel_mode_;
		std::atomic<bool> dont_set_angle_mode_;

		std::mutex profile_mutex_;

		//realtime_tools::RealtimeBuffer<bool> wipe_all_; //TODO, add this functionality
		realtime_tools::RealtimeBuffer<Commands> command_;

		ros::Subscriber sub_command_;
		ros::Subscriber talon_states_sub_;

		ros::ServiceServer motion_profile_serv_;
		ros::ServiceServer change_center_of_rotation_serv_;
		realtime_tools::RealtimeBuffer<Eigen::Vector2d> center_of_rotation_;
		ros::ServiceServer brake_serv_;
		ros::ServiceServer dont_set_angle_mode_serv_;

		std::array<std::array<hardware_interface::CustomProfilePoint, 2>, WHEELCOUNT> holder_points_;
		std::array<std::array<std::vector<hardware_interface::CustomProfilePoint>, 2>, WHEELCOUNT> full_profile_;

		realtime_tools::RealtimeBuffer<bool> run_;
		realtime_tools::RealtimeBuffer<int> slot_;

		/// Publish executed commands
		std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped> > cmd_vel_pub_;

		/// Wheel radius (assuming it's the same for the left and right wheels):
		double wheel_radius_;

		swerveVar::driveModel model_;

		swerveVar::ratios driveRatios_;

		swerveVar::encoderUnits units_;

		// Feed forward terms
		double f_s_;
		double f_a_;
		double f_v_;
		double f_s_v_;
		double f_s_s_;

		/// Timeout to consider cmd_vel commands old:
		double cmd_vel_timeout_;

		/// Whether to allow multiple publishers on cmd_vel topic or not:
		bool allow_multiple_cmd_vel_publishers_;

		/// Frame to use for the robot base:
		std::string base_frame_id_;

		/// Frame to use for odometry and odom tf:
		std::string odom_frame_id_;

		/// Whether to publish odometry to tf or not:
		//bool enable_odom_tf_;

		/// Number of wheel joints:
		size_t wheel_joints_size_;

		/// Speed limiters:
		//Commands last1_cmd_;
		//Commands last0_cmd_;

		/// Publish limited velocity:
		bool publish_cmd_;

		/**
		 * \brief Brakes the wheels, i.e. sets the velocity to 0
		 * RG: also sets to parking config
		 */
		void brake();

		std::array<double, WHEELCOUNT> steer_angles_;
		std::mutex steer_angles_mutex_;

		/**
		 * \brief Velocity command callback
		 * \param command Velocity command message (twist)
		 */
		void cmdVelCallback(const geometry_msgs::Twist &command);
		bool motionProfileService(talon_swerve_drive_controller::MotionProfile::Request &req, talon_swerve_drive_controller::MotionProfile::Response &res);
		bool changeCenterOfRotationService(talon_swerve_drive_controller::SetXY::Request &req, talon_swerve_drive_controller::SetXY::Response &res);
		bool brakeService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool dontSetAngleModeService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

		/**
		 * \brief Get the wheel names from a wheel param
		 * \param [in]  controller_nh Controller node handler
		 * \param [in]  wheel_param   Param name
		 * \param [out] wheel_names   Vector with the whel names
		 * \return true if the wheel_param is available and the wheel_names are
		 *        retrieved successfully from the param server; false otherwise
		 */
		bool getWheelNames(ros::NodeHandle &controller_nh,
						   const std::string &wheel_param,
						   std::vector<std::string> &wheel_names);

		/**
		 * \brief Sets odometry parameters from the URDF, i.e. the wheel radius and separation
		 * \param root_nh Root node handle
		 * \param left_wheel_name Name of the left wheel joint
		 * \param right_wheel_name Name of the right wheel joint
		 */
		/*

		bool setOdomParamsFromUrdf(ros::NodeHandle& root_nh,
		                            const std::string& steering_name,
		                            const std::string& speed_name,
		                            bool lookup_wheel_radius);

		 */
		/**
		* \brief Sets the odometry publishing fields
		* \param root_nh Root node handle
		* \param controller_nh Node handle inside the controller namespace
		*/
		/*
		void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
		*/
		static const double DEF_ODOM_PUB_FREQ;
		static const bool DEF_PUB_ODOM_TO_BASE;
		static const std::string DEF_ODOM_FRAME;
		static const std::string DEF_BASE_FRAME;
		static const double DEF_INIT_X;
		static const double DEF_INIT_Y;
		static const double DEF_INIT_YAW;
		static const double DEF_SD;

		std::array<Eigen::Vector2d, WHEELCOUNT> wheel_coords_;

		bool pub_odom_to_base_;       // Publish the odometry to base frame transform
		ros::Duration odom_pub_period_;    // Odometry publishing period
		Eigen::Affine2d init_odom_to_base_;  // Initial odometry to base frame transform
		Eigen::Affine2d odom_to_base_;       // Odometry to base frame transform
		Eigen::Affine2d odom_rigid_transf_;
		Eigen::Matrix2Xd wheel_pos_;

		ros::Publisher profile_queue_num;

		realtime_tools::RealtimePublisher<nav_msgs::Odometry> odom_pub_;
		tf2_ros::TransformBroadcaster odom_tf_;
		realtime_tools::RealtimePublisher<tf::tfMessage> odom_tf_pub_;
		ros::Time last_odom_pub_time_;
		ros::Time last_odom_tf_pub_time_;
};

PLUGINLIB_EXPORT_CLASS(talon_swerve_drive_controller::TalonSwerveDriveController, controller_interface::ControllerBase);

} // namespace diff_drive_controller
