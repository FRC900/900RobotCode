/*
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
 * Original Author: Bence Magyar
 */

#include <cmath>

#include <ros/ros.h>

#include <angles/angles.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#include <talon_controllers/talon_controller_interface.h>

#include "talon_swerve_drive_controller/SetXY.h"
#include <talon_swerve_drive_controller/Swerve.h>

namespace talon_swerve_drive_controller
{

template <size_t WHEELCOUNT>
class TalonSwerveDriveController
	: public controller_interface::Controller<hardware_interface::TalonCommandInterface>
{
	public:

TalonSwerveDriveController()
{
}

bool init(hardware_interface::TalonCommandInterface *hw,
		ros::NodeHandle &/*root_nh*/,
		ros::NodeHandle &controller_nh)
{
	const std::string complete_ns = controller_nh.getNamespace();
	const size_t id = complete_ns.find_last_of('/');
	name_ = complete_ns.substr(id + 1);

	// Get joint names from the parameter server
	std::array<std::string, WHEELCOUNT> speed_names;
	std::array<std::string, WHEELCOUNT> steering_names;
	if (!getWheelNames(controller_nh, "speed", speed_names) or
		!getWheelNames(controller_nh, "steering", steering_names))
	{
		return false;
	}

	if (speed_names.size() != steering_names.size())
	{
		ROS_ERROR_STREAM_NAMED(name_,
							   "#speed (" << speed_names.size() << ") != " <<
							   "#steering (" << steering_names.size() << ").");
		return false;
	}
	if (speed_names.size() != WHEELCOUNT)
	{
		ROS_ERROR_STREAM_NAMED(name_,
							   "#speed (" << speed_names.size() << ") != " <<
							   "WHEELCOUNT (" << WHEELCOUNT << ").");
		return false;
	}

	// Publish limited velocity:
	controller_nh.param("publish_cmd", publish_cmd_, publish_cmd_);

	// TODO : see if model_, driveRatios, units can be local instead of member vars
	// If either parameter is not available, we need to look up the value in the URDF
	//bool lookup_wheel_coordinates = !controller_nh.getParam("wheel_coordinates", wheel_coordinates_);
	if (!controller_nh.getParam("wheel_radius", wheel_radius_))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read wheel_radius");
		return false;
	}
	if (!controller_nh.getParam("cmd_vel_timout",cmd_vel_timeout_))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read cmd_vel_timout");
		return false;
	}
	if (!controller_nh.getParam("max_speed", model_.maxSpeed))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read max_speed");
		return false;
	}
	if (!controller_nh.getParam("mass", model_.mass))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read mass");
		return false;
	}
	if (!controller_nh.getParam("motor_free_speed", model_.motorFreeSpeed))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read motor_free_speed");
		return false;
	}
	if (!controller_nh.getParam("motor_stall_torque", model_.motorStallTorque))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read motor_stall_torque");
		return false;
	}
	// TODO : why not just use the number of wheels read from yaml?
	if (!controller_nh.getParam("motor_quantity", model_.motorQuantity))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read motor_quantity");
		return false;
	}
	if (!controller_nh.getParam("ratio_encoder_to_rotations", driveRatios_.encodertoRotations))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read ratio_encoder_to_rotations");
		return false;
	}
	if (!controller_nh.getParam("ratio_motor_to_rotations", driveRatios_.motortoRotations))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read ratio_motor_to_rotations");
		return false;
	}
	if (!controller_nh.getParam("ratio_motor_to_steering", driveRatios_.motortoSteering)) // TODO : not used
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read ratio_motor_to_steering");
		return false;
	}
	if (!controller_nh.getParam("encoder_drive_get_V_units", units_.rotationGetV))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read encoder_drive_get_V_units");
		return false;
	}
	if (!controller_nh.getParam("encoder_drive_get_P_units", units_.rotationGetP))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read encoder_drive_get_P_units");
		return false;
	}
	if (!controller_nh.getParam("encoder_drive_set_V_units", units_.rotationSetV))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read encoder_drive_set_V_units");
		return false;
	}
	if (!controller_nh.getParam("encoder_drive_set_P_units", units_.rotationSetP))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read encoder_drive_set_P_units");
		return false;
	}
	if (!controller_nh.getParam("encoder_steering_get_units", units_.steeringGet))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read encoder_steering_get_units");
		return false;
	}
	if (!controller_nh.getParam("encoder_steering_set_units", units_.steeringSet))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read encoder_steering_set_units");
		return false;
	}
	if (!controller_nh.getParam("f_s", f_s_))
	{
		ROS_ERROR("Could not read f_s in talon swerve drive controller");
		return false;
	}
	if (!controller_nh.getParam("f_a", f_a_))
	{
		ROS_ERROR("Could not read f_a in talon swerve drive controller");
		return false;
	}
	if (!controller_nh.getParam("f_v", f_v_))
	{
		ROS_ERROR("Could not read f_v in talon swerve drive controller");
		return false;
	}
	if (!controller_nh.getParam("f_s_v", f_s_v_))
	{
		ROS_ERROR("Could not read f_s_v in talon swerve drive controller");
		return false;
	}
	if (!controller_nh.getParam("f_s_s", f_s_s_))
	{
		ROS_ERROR("Could not read f_s_s in talon swerve drive controller");
		return false;
	}

	XmlRpc::XmlRpcValue wheel_coords;

	if(!controller_nh.getParam("wheel_coords", wheel_coords))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read wheel_coords");
		return false;
	}
	if(wheel_coords.getType() != XmlRpc::XmlRpcValue::TypeArray )
	{
	    ROS_ERROR("talon_swerve_drive_controller : param 'wheel_coords' is not a list");
		return false;
	}
	if (wheel_coords.size() != WHEELCOUNT)
	{
	    ROS_ERROR_STREAM("talon_swerve_drive_controller : param 'wheel_coords' is not correct length (expecting WHEELCOUNT = " << WHEELCOUNT << ")");
		return false;
	}
	for(int i=0; i < wheel_coords.size(); ++i)
	{
		if(wheel_coords[i].getType() != XmlRpc::XmlRpcValue::TypeArray )
		{
			ROS_ERROR("talon_swerve_drive_controller : param wheel_coords[%d] is not a list", i);
			return false;
		}
		if(wheel_coords[i].size() != 2)
		{
			ROS_ERROR("talon_swerve_drive_controller: param wheel_coords[%d] is not a pair", i);
			return false;
		}
		if(	wheel_coords[i][0].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
			wheel_coords[i][1].getType() != XmlRpc::XmlRpcValue::TypeDouble)
		{
			ROS_ERROR("talon_swerve_drive_controller : param wheel_coords[%d] is not a pair of doubles", i);
			return false;
		}
		wheel_coords_[i][0] = wheel_coords[i][0];
		wheel_coords_[i][1] = wheel_coords[i][1];
	}

	ROS_INFO_STREAM("Coords: " << wheel_coords_[0] << "   " << wheel_coords_[1] << "   " << wheel_coords_[2] << "   " << wheel_coords_[3]);
	std::array<double, WHEELCOUNT> offsets;
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		ros::NodeHandle nh(controller_nh, steering_names[i]);
		if (!nh.getParam("offset", offsets[i]))
		{
			ROS_ERROR_STREAM("Can not read offset for " << steering_names[i]);
			return false;
		}
	}
	for (const auto o : offsets)
	{
		ROS_INFO_STREAM("\t SWERVE: offset = " << o);
	}

	center_of_rotation_.writeFromNonRT(Eigen::Vector2d{0,0});
	/*
	if (!setOdomParamsFromUrdf(root_nh,
	                          speed_names[0],
	                          steering_names[0],
	                          //lookup_wheel_coordinates,
	                          lookup_wheel_radius))
	{
	  return false;
	}

	// Regardless of how we got the separation and radius, use them
	*/
	// to set the odometry parameters
	//setOdomPubFields(root_nh, controller_nh);

	// Get the joint object to use in the realtime loop

	// TODO : all of these need to be read from params
	/*
	model.maxSpeed = 3.3528;
	model.mass = 70;
	model.motorFreeSpeed = 5330;
	model.motorStallTorque = 2.41;
	model.motorQuantity = 4;
	*/
	model_.wheelRadius = wheel_radius_;

	/*
	swerveVar::ratios driveRatios({20, 7, 7});
	swerveVar::encoderUnits units({1,1,1,1,1,1});
	*/

	swerveC_ = std::make_unique<swerve<WHEELCOUNT>>(wheel_coords_, offsets, driveRatios_, units_, model_);
	for (size_t i = 0; i < WHEELCOUNT; ++i)
	{
		ROS_INFO_STREAM_NAMED(name_,
							  "Adding speed motors with joint name: " << speed_names[i]
							  << " and steering motors with joint name: " << steering_names[i]);

		ros::NodeHandle l_nh(controller_nh, speed_names[i]);
		speed_joints_[i].initWithNode(hw, nullptr, l_nh);
		ros::NodeHandle r_nh(controller_nh, steering_names[i]);
		steering_joints_[i].initWithNode(hw, nullptr, r_nh);
	}

	sub_command_ = controller_nh.subscribe("cmd_vel", 1, &TalonSwerveDriveController::cmdVelCallback, this);
	if (publish_cmd_)
	{
	  cmd_vel_pub_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>>(controller_nh, "cmd_vel_out", 2);
	}
	brake_serv_ = controller_nh.advertiseService("brake", &TalonSwerveDriveController::brakeService, this);
	reset_odom_serv_ = controller_nh.advertiseService("reset_odom", &TalonSwerveDriveController::resetOdomService, this);
	dont_set_angle_mode_serv_ = controller_nh.advertiseService("dont_set_angle", &TalonSwerveDriveController::dontSetAngleModeService, this);
	percent_out_drive_mode_serv_ = controller_nh.advertiseService("percent_out_drive_mode", &TalonSwerveDriveController::percentOutDriveModeService, this);
	change_center_of_rotation_serv_ = controller_nh.advertiseService("change_center_of_rotation", &TalonSwerveDriveController::changeCenterOfRotationService, this);

	double odom_pub_freq;
	controller_nh.param("odometry_publishing_frequency", odom_pub_freq, DEF_ODOM_PUB_FREQ);

	comp_odom_ = odom_pub_freq > 0;
	//ROS_WARN("COMPUTING ODOM");
	if (comp_odom_)
	{
		odom_pub_period_ = ros::Duration(1 / odom_pub_freq);
		controller_nh.param("publish_odometry_to_base_transform", pub_odom_to_base_,
							pub_odom_to_base_);

		double init_x, init_y, init_yaw;
		controller_nh.param("initial_x", init_x, DEF_INIT_X);
		controller_nh.param("initial_y", init_y, DEF_INIT_Y);
		controller_nh.param("initial_yaw", init_yaw, DEF_INIT_YAW);
		double x_sd, y_sd, yaw_sd;
		controller_nh.param("x_sd", x_sd, DEF_SD);
		controller_nh.param("y_sd", y_sd, DEF_SD);
		controller_nh.param("yaw_sd", yaw_sd, DEF_SD);
		double x_speed_sd, y_speed_sd, yaw_speed_sd;
		controller_nh.param("x_speed_sd", x_speed_sd, DEF_SD);
		controller_nh.param("y_speed_sd", y_speed_sd, DEF_SD);
		controller_nh.param("yaw_speed_sd", yaw_speed_sd, DEF_SD);

		init_odom_to_base_.setIdentity();
		init_odom_to_base_.rotate(init_yaw);
		init_odom_to_base_.translation() = Eigen::Vector2d(init_x, init_y);
		odom_to_base_ = init_odom_to_base_;
		odom_rigid_transf_.setIdentity();

		wheel_pos_.resize(2, WHEELCOUNT);
		for (size_t i = 0; i < WHEELCOUNT; i++)
		{
			wheel_pos_.col(i) = wheel_coords_[i];
		}

		const auto centroid = wheel_pos_.rowwise().mean();
		wheel_pos_.colwise() -= centroid;
		neg_wheel_centroid_ = -centroid;

		std::string odom_frame, base_frame;
		controller_nh.param("odometry_frame", odom_frame, DEF_ODOM_FRAME);
		controller_nh.param("base_frame", base_frame, DEF_BASE_FRAME);

		odom_pub_.msg_.header.frame_id = odom_frame;
		odom_pub_.msg_.child_frame_id = base_frame;

		odom_pub_.msg_.pose.pose.position.z = 0;

		odom_pub_.msg_.pose.covariance.assign(0);
		odom_pub_.msg_.pose.covariance[0] = x_sd * x_sd;
		odom_pub_.msg_.pose.covariance[7] = y_sd * y_sd;
		odom_pub_.msg_.pose.covariance[35] = yaw_sd * yaw_sd;

		odom_pub_.msg_.twist.twist.linear.z = 0;
		odom_pub_.msg_.twist.twist.angular.x = 0;
		odom_pub_.msg_.twist.twist.angular.y = 0;

		odom_pub_.msg_.twist.covariance.assign(0);
		odom_pub_.msg_.twist.covariance[0] = x_speed_sd * x_speed_sd;
		odom_pub_.msg_.twist.covariance[7] = y_speed_sd * y_speed_sd;
		odom_pub_.msg_.twist.covariance[35] = yaw_speed_sd * yaw_speed_sd;
		odom_pub_.init(controller_nh, "odom", 1);

		if (pub_odom_to_base_)
		{
			odom_tf_pub_.msg_.transforms.resize(1);
			geometry_msgs::TransformStamped &odom_tf_trans =
				odom_tf_pub_.msg_.transforms[0];
			odom_tf_trans.header.frame_id = odom_pub_.msg_.header.frame_id;
			odom_tf_trans.child_frame_id = odom_pub_.msg_.child_frame_id;
			odom_tf_trans.transform.translation.z = 0;
			odom_tf_pub_.init(controller_nh, "/tf", 1);
		}

		for (size_t row = 0; row < WHEELCOUNT; row++)
		{
			last_wheel_rot_[row] = speed_joints_[row].getPosition();
		}
	}
	controller_nh.param("use_cos_scaling", use_cos_scaling_, use_cos_scaling_);

	controller_nh.param("parking_config_time_delay", parking_config_time_delay_, parking_config_time_delay_);
	controller_nh.param("drive_speed_time_delay", drive_speed_time_delay_, parking_config_time_delay_);
	controller_nh.param("use_cos_scaling", use_cos_scaling_, use_cos_scaling_);

	return true;
}

void starting(const ros::Time &time)
{
	std::array<double, WHEELCOUNT> steer_angles;
	for (size_t k = 0; k < WHEELCOUNT; k++)
	{
		steer_angles[k] = steering_joints_[k].getPosition();
	}
	brake(steer_angles);
	// Assume we braked infinitely long ago - this will keep the
	// drive base in parking config until a non-zero command comes in
	time_before_brake_ = 0;

	// Register starting time used to keep fixed rate
	if (comp_odom_)
	{
		last_odom_pub_time_ = time;
		last_odom_tf_pub_time_ = time;
	}
	//odometry_.init(time);
}

void stopping(const ros::Time & time)
{
	brake_last_ = time.toSec();
	time_before_brake_ = 0;
	std::array<double, WHEELCOUNT> steer_angles;
	for (size_t k = 0; k < WHEELCOUNT; k++)
	{
		steer_angles[k] = steering_joints_[k].getPosition();
	}
	brake(steer_angles);
}

void update(const ros::Time &time, const ros::Duration &period)
{
	// Grab current steering angle
	std::array<double, WHEELCOUNT> steer_angles;
	for (size_t k = 0; k < WHEELCOUNT; k++)
	{
		steer_angles[k] = steering_joints_[k].getPosition();
	}

	if (comp_odom_) compOdometry(time, 1.0 / period.toSec(), steer_angles);

	Commands curr_cmd = *(command_.readFromRT());
	const double dt = (time - curr_cmd.stamp).toSec();
	const bool dont_set_angle_mode = dont_set_angle_mode_.load(std::memory_order_relaxed);
	const bool percent_out_drive_mode = percent_out_drive_mode_.load(std::memory_order_relaxed);

	//ROS_INFO_STREAM("ang_vel_tar: " << curr_cmd.ang << " lin_vel_tar: " << curr_cmd.lin);

	// Brake if cmd_vel has timeout:
	if (dt > cmd_vel_timeout_)
	{
		curr_cmd.lin = {0.0, 0.0};
		curr_cmd.ang = 0.0;
	}

	for (size_t i = 0; i < WHEELCOUNT; ++i)
	{
		if (!dont_set_angle_mode_)
		{
			steering_joints_[i].setPIDFSlot(0);
			steering_joints_[i].setMode(hardware_interface::TalonMode::TalonMode_MotionMagic);
			steering_joints_[i].setDemand1Value(0);
		}
		speed_joints_[i].setPIDFSlot(0);
	}

	// Special case for when the drive base is stopped
	if (fabs(curr_cmd.lin[0]) <= 1e-6 && fabs(curr_cmd.lin[1]) <= 1e-6 && fabs(curr_cmd.ang) <= 1e-6)
	{
		for (size_t i = 0; i < WHEELCOUNT; ++i)
		{
			speed_joints_[i].setCommand(0);
			speed_joints_[i].setMode(hardware_interface::TalonMode::TalonMode_PercentOutput);
		}
		if ((time.toSec() - time_before_brake_) > parking_config_time_delay_)
		{
			brake(steer_angles);
		}
		else
		{
			for (size_t i = 0; i < WHEELCOUNT; ++i)
			{
				if (!dont_set_angle_mode)
					steering_joints_[i].setCommand(speeds_angles_[i][1]);
				speed_joints_[i].setNeutralMode(hardware_interface::NeutralMode::NeutralMode_Coast);
			}
		}
		if (publish_cmd_ && cmd_vel_pub_->trylock())
		{
			cmd_vel_pub_->msg_.header.stamp = time;
			cmd_vel_pub_->msg_.twist.linear.x = 0;
			cmd_vel_pub_->msg_.twist.linear.y = 0;
			cmd_vel_pub_->msg_.twist.linear.z = 0;
			cmd_vel_pub_->msg_.twist.angular.x = 0;
			cmd_vel_pub_->msg_.twist.angular.y = 0;
			cmd_vel_pub_->msg_.twist.angular.z = 0;
			cmd_vel_pub_->unlockAndPublish();
		}
		return;
	}

	// We are not stopped, so reset the time which we were
	// last not-stopped here. This is used later to give
	// a delay between stopping and moving into parking config
	time_before_brake_ = time.toSec();

	// Compute wheels velocities:
	//Parse curr_cmd to get velocity vector and rotation (z axis)
	speeds_angles_ = swerveC_->motorOutputs(curr_cmd.lin, curr_cmd.ang, M_PI / 2.0, steer_angles, true, *(center_of_rotation_.readFromRT()), use_cos_scaling_);

	// Set wheel steering angles, as long as dont_set_angle_mode is false
	for (size_t i = 0; !dont_set_angle_mode && (i < WHEELCOUNT); ++i)
	{
		steering_joints_[i].setCommand(speeds_angles_[i][1]);
	}

	// Wait a bit after coming out of parking
	// config before setting wheel velocity
	if ((time.toSec() - brake_last_) > drive_speed_time_delay_)
	{
		for (size_t i = 0; i < WHEELCOUNT; ++i)
		{
			if (!percent_out_drive_mode)
			{
				speed_joints_[i].setMode(hardware_interface::TalonMode::TalonMode_Velocity);

				// Add static feed forward in direction of current velocity
				if(fabs(speeds_angles_[i][0]) > 1e-5)
				{
					speed_joints_[i].setDemand1Type(hardware_interface::DemandType::DemandType_ArbitraryFeedForward);
					speed_joints_[i].setDemand1Value(copysign(f_s_, speeds_angles_[i][0]));
				}
				else
				{
					speed_joints_[i].setDemand1Type(hardware_interface::DemandType::DemandType_Neutral);
					speed_joints_[i].setDemand1Value(0);
				}

				speed_joints_[i].setCommand(speeds_angles_[i][0]);
			}
			else
			{
				// Debugging mode - used for robot characterization by sweeping
				// from 0-100% output and measuring response
				speed_joints_[i].setMode(hardware_interface::TalonMode::TalonMode_PercentOutput);
				speed_joints_[i].setCommand(hypot(curr_cmd.lin[0], curr_cmd.lin[1]));
				speed_joints_[i].setDemand1Type(hardware_interface::DemandType::DemandType_Neutral);
				speed_joints_[i].setDemand1Value(0);
			}
		}
	}
	else
	{
		for (size_t i = 0; i < WHEELCOUNT; ++i)
		{
			speed_joints_[i].setCommand(0);
			speed_joints_[i].setMode(hardware_interface::TalonMode::TalonMode_PercentOutput);
			speed_joints_[i].setDemand1Type(hardware_interface::DemandType::DemandType_Neutral);
			speed_joints_[i].setDemand1Value(0);
		}
	}
	for (size_t i = 0; i < WHEELCOUNT; ++i)
	{
		speed_joints_[i].setNeutralMode(hardware_interface::NeutralMode::NeutralMode_Coast);
	}

	if (publish_cmd_ && cmd_vel_pub_->trylock())
	{
		cmd_vel_pub_->msg_.header.stamp = time;
		cmd_vel_pub_->msg_.twist.linear.x = curr_cmd.lin[0];
		cmd_vel_pub_->msg_.twist.linear.y = curr_cmd.lin[1];
		cmd_vel_pub_->msg_.twist.linear.z = 0;
		cmd_vel_pub_->msg_.twist.angular.x = 0;
		cmd_vel_pub_->msg_.twist.angular.y = 0;
		cmd_vel_pub_->msg_.twist.angular.z = curr_cmd.ang;
		cmd_vel_pub_->unlockAndPublish();
	}
}

private:

double angle_midpoint(double start_angle, double end_angle) const
{
	double deltaAngle = end_angle - start_angle;
	while (deltaAngle < -M_PI)
		deltaAngle += 2.0 * M_PI;
	while (deltaAngle > M_PI)
		deltaAngle -= 2.0 * M_PI;
	// TODO - this angle is just going to have cos and sin called on it,
	// might not need to be normalized?
	return angles::normalize_angle(start_angle + deltaAngle / 2.0);
}

void compOdometry(const ros::Time &time, const double inv_delta_t, const std::array<double, WHEELCOUNT> &steer_angles)
{
	if (reset_odom_.exchange(false))
	{
		odom_to_base_.setIdentity();
	}
	// Compute the rigid transform from last timestep's robot orientation
	// to the current robot orientation

	// Use the encoder-reported wheel angle plus the difference between
	// the previous and current wheel rotation to calculate motion for each
	// wheel since the last odom value update.
	// Add the movement of each wheel to the config-defined wheel coords
	// From this, figure out a rigid body rotation and translation
	// which best matches the difference in position of each of the wheels
	Eigen::MatrixX2d new_wheel_pos;
	new_wheel_pos.resize(WHEELCOUNT, 2);
	for (size_t k = 0; k < WHEELCOUNT; k++)
	{
		// Read wheel rotation from encoder.  Subtract previous position
		// to get the motion since the last odom update. Convert from rotation
		// to linear distance using wheel radius and drive ratios
		const double new_wheel_rot = speed_joints_[k].getPosition();
		const double delta_rot     = new_wheel_rot - last_wheel_rot_[k];
		const double dist          = delta_rot * wheel_radius_ * driveRatios_.encodertoRotations;

		// Get the offset-corrected steering angle
		const double steer_angle = swerveC_->getWheelAngle(k, steer_angles[k]);
		// And then average it with the last reading - moving the full
		// wheel rot distance in a direction of the average of the start
		// and end steer directions for this timestep is an approximation
		// of the actual direction. It ignores changes in speed of the
		// steer motor, but it is the best we can do given the info we have.
		const double average_steer_angle = angle_midpoint(last_wheel_angle_[k], steer_angle);

		// delta_pos of the wheel in x,y - decompose vector into x&y components
		const Eigen::Vector2d delta_pos{dist * cos(average_steer_angle), dist * sin(average_steer_angle)};

		// new_wheel_pos is constructed to hold x,y position given the assumption
		// that the robot started with each wheel at the wheel_coords_ position
		// Since we're just measuring a change in position, the initial position
		// is arbitrary just as long as it is a valid wheel position shape.
		// Note this is actually built as a transpose of the shape of wheel_coords
		// to simplyify the math later
		new_wheel_pos(k, 0) = wheel_coords_[k][0] + delta_pos[0];
		new_wheel_pos(k, 1) = wheel_coords_[k][1] + delta_pos[1];
#if 0
		ROS_INFO_STREAM("last_wheel_angle_[k]=" << last_wheel_angle_[k] << " steer_angle=" << steer_angle << " average_steer_angle=" << average_steer_angle)
		ROS_INFO_STREAM("dist=" << dist << " delta_pos=" << delta_pos);
		ROS_INFO_STREAM("x from " <<  wheel_coords_[k][0] << " to " << new_wheel_pos(k, 0));
		ROS_INFO_STREAM("y from " <<  wheel_coords_[k][1] << " to " << new_wheel_pos(k, 1));
#endif

		// Save the previous wheel rotation / steer angle to use next time odom is run
		last_wheel_rot_[k] = new_wheel_rot;
		last_wheel_angle_[k] = steer_angle;
	}

	// http://nghiaho.com/?page_id=671, but that page talks about 3d transform where
	//    are just looking for a 2d one (assumine the robot stays on the ground, not
	//    that we can measure if it doesn't...)
	// Also, https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
	// Find the translation+rotation that produces the least square error
	// between (prev pos + transform) and current measured position
	const Eigen::RowVector2d new_wheel_centroid = new_wheel_pos.colwise().mean();
	new_wheel_pos.rowwise() -= new_wheel_centroid;

	const Eigen::Matrix2d h = wheel_pos_ * new_wheel_pos;
	const Eigen::JacobiSVD<Eigen::Matrix2d> svd(h, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix2d rot = svd.matrixV() * svd.matrixU().transpose();
	if (rot.determinant() < 0)
		rot.col(1) *= -1;

	odom_rigid_transf_.matrix().block(0, 0, 2, 2) = rot;
	odom_rigid_transf_.translation() = rot * neg_wheel_centroid_ + new_wheel_centroid.transpose();
	// add motion for this timestep to the overall odom_to_base transform
	odom_to_base_ = odom_to_base_ * odom_rigid_transf_;

	const double odom_x = odom_to_base_.translation().x();
	const double odom_y = odom_to_base_.translation().y();
	const double odom_yaw = atan2(odom_to_base_(1, 0), odom_to_base_(0, 0));

	//ROS_INFO_STREAM("odom_x: " << odom_x << " odom_y: " << odom_y << " odom_yaw: " << odom_yaw);
	// Publish the odometry.
	//TODO CHECK THIS PUB

	geometry_msgs::Quaternion orientation;
	bool orientation_comped = false;

	// tf
	if (pub_odom_to_base_ && time - last_odom_tf_pub_time_ >= odom_pub_period_ &&
			odom_tf_pub_.trylock())
	{
		orientation = tf::createQuaternionMsgFromYaw(odom_yaw);
		orientation_comped = true;

		geometry_msgs::TransformStamped &odom_tf_trans =
			odom_tf_pub_.msg_.transforms[0];
		odom_tf_trans.header.stamp = time;
		odom_tf_trans.transform.translation.x = odom_y; // TODO terrible hacky
		odom_tf_trans.transform.translation.y = -odom_y; // TODO terrible hacky
		odom_tf_trans.transform.rotation = orientation;
		ROS_INFO_STREAM(odom_x);
		odom_tf_pub_.unlockAndPublish();
		last_odom_tf_pub_time_ += odom_pub_period_;
	}

	// odom
	if (time - last_odom_pub_time_ >= odom_pub_period_ && odom_pub_.trylock())
	{
		if (!orientation_comped)
			orientation = tf::createQuaternionMsgFromYaw(odom_yaw);

		odom_pub_.msg_.header.stamp = time;
		odom_pub_.msg_.pose.pose.position.x = odom_y; //TODO terrible hacky
		odom_pub_.msg_.pose.pose.position.y = - odom_x; //TODO terrible hacky
		odom_pub_.msg_.pose.pose.orientation = orientation;

		odom_pub_.msg_.twist.twist.linear.x =
			odom_rigid_transf_.translation().x() * inv_delta_t;
		odom_pub_.msg_.twist.twist.linear.y =
			odom_rigid_transf_.translation().y() * inv_delta_t;
		odom_pub_.msg_.twist.twist.angular.z =
			atan2(odom_rigid_transf_(1, 0), odom_rigid_transf_(0, 0)) * inv_delta_t;

		odom_pub_.unlockAndPublish();

		last_odom_pub_time_ += odom_pub_period_;
	}
}



void brake(const std::array<double, WHEELCOUNT> &steer_angles)
{
	//Use parking config
	const bool dont_set_angle_mode = dont_set_angle_mode_.load(std::memory_order_relaxed);
	const std::array<double, WHEELCOUNT> park_angles = swerveC_->parkingAngles(steer_angles);
	for (size_t i = 0; i < WHEELCOUNT; ++i)
	{
		speed_joints_[i].setCommand(0.0);
		speed_joints_[i].setDemand1Type(hardware_interface::DemandType::DemandType_Neutral);
		speed_joints_[i].setDemand1Value(0);
		speed_joints_[i].setNeutralMode(hardware_interface::NeutralMode::NeutralMode_Brake);
		if (!dont_set_angle_mode)
			steering_joints_[i].setCommand(park_angles[i]);
	}
	// Reset the timer which delays drive wheel velocity a bit after
	// the robot's been stopped
	brake_last_ = ros::Time::now().toSec();
}

void cmdVelCallback(const geometry_msgs::Twist &command)
{
	if (isRunning())
	{
		const auto now = ros::Time::now();
		// check that we don't have multiple publishers on the command topic
		//ROS_WARN("Time Difference: %f", ros::Time::now().toSec() - command->header.stamp.toSec());
		if (!allow_multiple_cmd_vel_publishers_ && sub_command_.getNumPublishers() > 1)
		{
			ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, name_, "Detected " << sub_command_.getNumPublishers()
											<< " publishers. Only 1 publisher is allowed. Going to brake.");
			Commands brake_struct;
			brake_struct.lin[0] = 0;
			brake_struct.lin[1] = 0;
			brake_struct.ang = 0;
			brake_struct.stamp = now;
			command_.writeFromNonRT(brake_struct);
			return;
		}

		//These below are some simple bounds checks on the cmd vel input so we don't make dumb mistakes. (like try to get the swerve drive to fly away)
		if (command.linear.z != 0)
		{
			ROS_ERROR_THROTTLE(1.0, "Rotors not up to speed!");
		}
		if ((command.angular.x != 0) || (command.angular.y != 0))
		{
			ROS_ERROR_THROTTLE(1.0, "Reaction wheels need alignment. Please reverse polarity on neutron flux capacitor");
		}
		if (hypot(command.linear.x, command.linear.y) > 300000000)
		{
			ROS_ERROR_THROTTLE(1.0, "PHYSICS VIOLATION DETECTED. DISABLE TELEPORTATION UNIT!");
		}

		//TODO change to twist msg

		Commands command_struct;
		command_struct.ang = command.angular.z;
		command_struct.lin[0] = command.linear.x;
		command_struct.lin[1] = command.linear.y;
		command_struct.stamp = now;
		command_.writeFromNonRT (command_struct);

#if 0
		//TODO fix debug
		ROS_DEBUG_STREAM_NAMED(name_,
							   "Added values to command. "
							   << "Ang: "   << command_struct_.ang << ", "
							   << "Lin X: "   << command_struct_.lin[0] << ", "
							   << "Lin Y: "   << command_struct_.lin[1] << ", "
							   << "Stamp: " << command_struct_.stamp);
#endif
	}
	else
	{
		ROS_ERROR_STREAM_NAMED(name_, __PRETTY_FUNCTION__ << " : Can't accept new commands. Controller is not running.");
	}
}

bool changeCenterOfRotationService(talon_swerve_drive_controller::SetXY::Request& req, talon_swerve_drive_controller::SetXY::Response &/*res*/)
{
	if(isRunning())
	{
		Eigen::Vector2d vector;
		vector[0] = req.x;
		vector[1] = req.y;
		center_of_rotation_.writeFromNonRT(vector);
		return true;
	}
	else
	{
		ROS_ERROR_STREAM_NAMED(name_, __PRETTY_FUNCTION__ << " : Can't accept new commands. Controller is not running.");
		return false;
	}
}

bool resetOdomService(std_srvs::Empty::Request &/*req*/, std_srvs::Empty::Response &/*res*/)
{
	if (isRunning())
	{
		reset_odom_.exchange(true);
		return true;
	}
	else
	{
		ROS_ERROR_NAMED(name_, "resetOdom service cann't accept new commands. Controller is not running.");
		return false;
	}

}

bool brakeService(std_srvs::Empty::Request &/*req*/, std_srvs::Empty::Response &/*res*/)
{
	if (isRunning())
	{
		Commands brake_struct;
		brake_struct.lin[0] = 0;
		brake_struct.lin[1] = 0;
		brake_struct.ang = 0;
		brake_struct.stamp = ros::Time::now();
		ROS_WARN("brakeService called in controller");
		command_.writeFromNonRT(brake_struct);

		return true;
	}
	else
	{
		ROS_ERROR_NAMED(name_, "brakeService can't accept new commands. Controller is not running.");
		return false;
	}
}

bool dontSetAngleModeService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	if (isRunning())
	{
		ROS_WARN_STREAM("dont_set_angle_mode_ set to = " << static_cast<int>(req.data));
		dont_set_angle_mode_.store(req.data, std::memory_order_relaxed);

		res.success = true;
		res.message = "SUCCESS!";

		return true;
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't set dont_set_angle_mode_. Controller is not running.");
		return false;
	}
}

bool percentOutDriveModeService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	if (isRunning())
	{
		ROS_WARN_STREAM("percent_out_drive_mode_mode_ set to = " << static_cast<int>(req.data));
		percent_out_drive_mode_.store(req.data, std::memory_order_relaxed);

		res.success = true;
		res.message = "SUCCESS!";

		return true;
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't set percent_out_drive_mode_. Controller is not running.");
		return false;
	}
}

bool getWheelNames(ros::NodeHandle &controller_nh,
		const std::string &wheel_param,
		std::array<std::string, WHEELCOUNT> &wheel_names)
{
	XmlRpc::XmlRpcValue wheel_list;
	if (!controller_nh.getParam(wheel_param, wheel_list))
	{
		ROS_ERROR_STREAM_NAMED(name_,
							   "Couldn't retrieve wheel param '" << wheel_param << "'.");
		return false;
	}

	if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
	{
		if (wheel_list.size() == 0)
		{
			ROS_ERROR_STREAM_NAMED(name_,
								   "Wheel param '" << wheel_param << "' is an empty list");
			return false;
		}
		if (wheel_list.size() != WHEELCOUNT)
		{
			ROS_ERROR_STREAM_NAMED(name_,
								   "Wheel param size (" << wheel_list.size() <<
								   " != WHEELCOUNT (" << WHEELCOUNT <<")." );
			return false;
		}

		for (int i = 0; i < wheel_list.size(); ++i)
		{
			if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
			{
				ROS_ERROR_STREAM_NAMED(name_,
									   "Wheel param '" << wheel_param << "' #" << i <<
									   " isn't a string.");
				return false;
			}
		}

		for (int i = 0; i < wheel_list.size(); ++i)
		{
			wheel_names[i] = static_cast<std::string>(wheel_list[i]);
		}
	}
	else
	{
		ROS_ERROR_STREAM_NAMED(name_,
							   "Wheel param '" << wheel_param <<
							   "' is neither a list of strings nor a string.");
		return false;
	}

	return true;
}

bool comp_odom_;
Eigen::Matrix2Xd wheel_pos_;
Eigen::Vector2d neg_wheel_centroid_;
std::array<double, WHEELCOUNT> last_wheel_rot_;	    // used for odom calcs
std::array<double, WHEELCOUNT> last_wheel_angle_;	//

std::string name_;

std::unique_ptr<swerve<WHEELCOUNT>> swerveC_;

/// Hardware handles:
std::array<talon_controllers::TalonControllerInterface, WHEELCOUNT> speed_joints_;
std::array<talon_controllers::TalonControllerInterface, WHEELCOUNT> steering_joints_;

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

realtime_tools::RealtimeBuffer<Commands> command_;

std::atomic<bool> dont_set_angle_mode_; // used to lock wheels into place
std::atomic<bool> percent_out_drive_mode_; // run drive wheels in open-loop mode
double brake_last_{ros::Time::now().toSec()};
double time_before_brake_{0};
double parking_config_time_delay_{0.1};
double drive_speed_time_delay_{0.1};
std::array<Eigen::Vector2d, WHEELCOUNT> speeds_angles_;

ros::Subscriber sub_command_;

ros::ServiceServer change_center_of_rotation_serv_;
realtime_tools::RealtimeBuffer<Eigen::Vector2d> center_of_rotation_;

ros::ServiceServer brake_serv_;
ros::ServiceServer dont_set_angle_mode_serv_;
ros::ServiceServer percent_out_drive_mode_serv_;

ros::ServiceServer reset_odom_serv_;
std::atomic<bool> reset_odom_{false};

/// Publish executed commands
std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped> > cmd_vel_pub_;

/// Wheel radius (assuming it's the same for the left and right wheels):
double wheel_radius_{};

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
double cmd_vel_timeout_{0.5};

/// Whether to allow multiple publishers on cmd_vel topic or not:
bool allow_multiple_cmd_vel_publishers_{true};

/// Whether to publish odometry to tf or not:
//bool enable_odom_tf_;

/// Number of wheel joints:
size_t wheel_joints_size_{0};

/// Publish limited velocity:
bool publish_cmd_{true};

static constexpr double DEF_ODOM_PUB_FREQ{100.};
static const std::string DEF_ODOM_FRAME;
static const std::string DEF_BASE_FRAME;
static constexpr double DEF_INIT_X{0};
static constexpr double DEF_INIT_Y{0};
static constexpr double DEF_INIT_YAW{0};
static constexpr double DEF_SD{0.01};

std::array<Eigen::Vector2d, WHEELCOUNT> wheel_coords_;

bool pub_odom_to_base_{false};       // Publish the odometry to base frame transform
ros::Duration odom_pub_period_;      // Odometry publishing period
Eigen::Affine2d init_odom_to_base_;  // Initial odometry to base frame transform
Eigen::Affine2d odom_to_base_;       // Odometry to base frame transform
Eigen::Affine2d odom_rigid_transf_;

realtime_tools::RealtimePublisher<nav_msgs::Odometry> odom_pub_;
tf2_ros::TransformBroadcaster odom_tf_;
realtime_tools::RealtimePublisher<tf::tfMessage> odom_tf_pub_;
ros::Time last_odom_pub_time_;
ros::Time last_odom_tf_pub_time_;

// Attempt to limit speed of wheels which are pointing further from
// their target angle. Should help to reduce the robot being pulled
// off course coming out of parking config or when maki
bool use_cos_scaling_{false};

}; // class definition

template <size_t WHEELCOUNT> const std::string TalonSwerveDriveController<WHEELCOUNT>::DEF_ODOM_FRAME = "odom";
template <size_t WHEELCOUNT> const std::string TalonSwerveDriveController<WHEELCOUNT>::DEF_BASE_FRAME = "base_link";

} // namespace talon_swerve_drive_controller

#include <pluginlib/class_list_macros.h>
template class talon_swerve_drive_controller::TalonSwerveDriveController<4>;
PLUGINLIB_EXPORT_CLASS(talon_swerve_drive_controller::TalonSwerveDriveController<4>, controller_interface::ControllerBase)
