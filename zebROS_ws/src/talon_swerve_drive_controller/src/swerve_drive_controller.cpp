/*
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this listweof conditions and the following disclaimer.
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
#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <periodic_interval_counter/periodic_interval_counter.h>
#include <sensor_msgs/Imu.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#include <talon_controllers/talon_controller_interface.h>
#include <talon_controllers/talonfxpro_controller_interface.h>
#include <ctre_interfaces/latency_compensation_state_interface.h>

#include "talon_swerve_drive_controller_msgs/SetXY.h"
#include <talon_swerve_drive_controller/Swerve.h>
#include "talon_swerve_drive_controller/get_wheel_names.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace talon_swerve_drive_controller
{

template <size_t WHEELCOUNT>
class TalonSwerveDriveController
	: public controller_interface::MultiInterfaceController<hardware_interface::talonfxpro::TalonFXProCommandInterface, hardware_interface::latency_compensation::CTRELatencyCompensationStateInterface>
{
public:

bool init(hardware_interface::RobotHW *hw,
		ros::NodeHandle &/*root_nh*/,
		ros::NodeHandle &controller_nh) override
{
	const std::string complete_ns = controller_nh.getNamespace();
	const size_t id = complete_ns.find_last_of('/');
	name_ = complete_ns.substr(id + 1);

	// Get joint names from the parameter server
	if (!getWheelNames(controller_nh, name_, "speed", speed_names_) ||
		!getWheelNames(controller_nh, name_, "steering", steering_names_))
	{
		return false;
	}

	if (speed_names_.size() != steering_names_.size())
	{
		ROS_ERROR_STREAM_NAMED(name_,
							   "#speed (" << speed_names_.size() << ") != " <<
							   "#steering (" << steering_names_.size() << ").");
		return false;
	}
	if (speed_names_.size() != WHEELCOUNT)
	{
		ROS_ERROR_STREAM_NAMED(name_,
							   "#speed/steering (" << speed_names_.size() << ") != " <<
							   "WHEELCOUNT (" << WHEELCOUNT << ").");
		return false;
	}

	// TODO : see if model_, driveRatios, units can be local instead of member vars
	// If either parameter is not available, we need to look up the value in the URDF
	//bool lookup_wheel_coordinates = !controller_nh.getParam("wheel_coordinates", wheel_coordinates_);
	if (!controller_nh.getParam("wheel_radius", model_.wheelRadius))
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

	if (!controller_nh.getParam("ratio_encoder_to_rotations", driveRatios_.encodertoRotations))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read ratio_encoder_to_rotations");
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
	if (!controller_nh.getParam("stopping_ff", stopping_ff_))
	{
		ROS_ERROR("Could not read stoping_ff in talon swerve drive controller");
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
		ros::NodeHandle nh(controller_nh, steering_names_[i]);
		if (!nh.getParam("offset", offsets[i]))
		{
			ROS_ERROR_STREAM("Can not read offset for " << steering_names_[i]);
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
	                          speed_names_[0],
	                          steering_names_[0],
	                          //lookup_wheel_coordinates,
	                          lookup_wheel_radius))
	{
	  return false;
	}

	*/
	// Regardless of how we got the separation and radius, use them
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
	model_.wheelRadius = wheel_radius_;
	*/

	/*
	swerveVar::ratios driveRatios({20, 7, 7});
	swerveVar::encoderUnits units({1,1,1,1,1,1});
	*/

	swerveC_ = std::make_unique<swerve<WHEELCOUNT>>(wheel_coords_, offsets, driveRatios_, units_, model_);
	for (size_t i = 0; i < WHEELCOUNT; ++i)
	{
		ROS_INFO_STREAM_NAMED(name_,
							  "Adding speed motors with joint name: " << speed_names_[i]
							  << " and steering motors with joint name: " << steering_names_[i]);

		ros::NodeHandle l_nh(controller_nh, speed_names_[i]);
		if (!speed_joints_[i].initWithNode(hw->get<hardware_interface::talonfxpro::TalonFXProCommandInterface>(), nullptr, l_nh))
		{
			return false;
		}
		ros::NodeHandle r_nh(controller_nh, steering_names_[i]);
		if (!steering_joints_[i].initWithNode(hw->get<hardware_interface::talonfxpro::TalonFXProCommandInterface>(), nullptr, r_nh))
		{
			return false;
		}
	}

	sub_command_ = controller_nh.subscribe("cmd_vel", 1, &TalonSwerveDriveController::cmdVelCallback, this);
	// Publish limited velocity:
	bool publish_cmd;
	controller_nh.param("publish_cmd", publish_cmd, true);
	if (publish_cmd)
	{
	  cmd_vel_pub_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>>(controller_nh, "cmd_vel_out", 2);
	}
	brake_serv_ = controller_nh.advertiseService("brake", &TalonSwerveDriveController::brakeService, this);
	reset_odom_serv_ = controller_nh.advertiseService("reset_odom", &TalonSwerveDriveController::resetOdomService, this);
	park_serv_ = controller_nh.advertiseService("toggle_park", &TalonSwerveDriveController::parkService, this);
	dont_set_angle_mode_serv_ = controller_nh.advertiseService("dont_set_angle", &TalonSwerveDriveController::dontSetAngleModeService, this);
	percent_out_drive_mode_serv_ = controller_nh.advertiseService("percent_out_drive_mode", &TalonSwerveDriveController::percentOutDriveModeService, this);
	change_center_of_rotation_serv_ = controller_nh.advertiseService("change_center_of_rotation", &TalonSwerveDriveController::changeCenterOfRotationService, this);
	set_neutral_mode_serv_ = controller_nh.advertiseService("set_neutral_mode", &TalonSwerveDriveController::setNeturalModeService, this);

	controller_nh.param("odometry_publishing_frequency", odom_pub_freq_, DEF_ODOM_PUB_FREQ);

	std::string latency_compensation_group;
	if (!controller_nh.getParam("latency_compensation_group", latency_compensation_group))
	{
		ROS_ERROR("Parameter 'latency_compensation_group' not set in swerve drive controller");
		return false;
	}

	size_t latency_compensation_group_index;

	auto latency_compensation_hw = hw->get<hardware_interface::latency_compensation::CTRELatencyCompensationStateInterface>();

	// get all joint names from the hardware interface
	const std::vector<std::string> &joint_names = latency_compensation_hw->getNames();
	latency_compensation_group_index = std::find(joint_names.begin(), joint_names.end(), latency_compensation_group) - joint_names.begin();
	if (latency_compensation_group_index >= joint_names.size()) {
		ROS_ERROR_STREAM("Latency compensation group " << latency_compensation_group << " not found in swerve drive controller");
		return false;
	}

	latency_compensation_state_ = latency_compensation_hw->getHandle(joint_names[latency_compensation_group_index]);

	comp_odom_ = odom_pub_freq_ > 0;
	//ROS_WARN("COMPUTING ODOM");
	if (comp_odom_)
	{
		controller_nh.param("publish_odometry_to_base_transform", pub_odom_to_base_,
							pub_odom_to_base_);

		double init_x;
		double init_y;
		double init_yaw;
		controller_nh.param("initial_x", init_x, DEF_INIT_X);
		controller_nh.param("initial_y", init_y, DEF_INIT_Y);
		controller_nh.param("initial_yaw", init_yaw, DEF_INIT_YAW);
		double x_sd;
		double y_sd;
		double yaw_sd;
		controller_nh.param("x_sd", x_sd, DEF_SD);
		controller_nh.param("y_sd", y_sd, DEF_SD);
		controller_nh.param("yaw_sd", yaw_sd, DEF_SD);
		double x_speed_sd;
		double y_speed_sd;
		double yaw_speed_sd;
		controller_nh.param("x_speed_sd", x_speed_sd, DEF_SD);
		controller_nh.param("y_speed_sd", y_speed_sd, DEF_SD);
		controller_nh.param("yaw_speed_sd", yaw_speed_sd, DEF_SD);

		init_odom_to_base_.setIdentity();
		init_odom_to_base_.rotate(init_yaw);
		init_odom_to_base_.translation() = Eigen::Vector2d(init_x, init_y);
		odom_to_base_ = init_odom_to_base_;
		initial_yaw_ = latency_compensation_state_->getLatencyCompensatedValue("pigeon2", ros::Time::now());
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
			last_wheel_rot_[row] = latency_compensation_state_->getLatencyCompensatedValue(speed_names_[row], ros::Time::now());
		}
	}
	controller_nh.param("use_cos_scaling", use_cos_scaling_, use_cos_scaling_);

	controller_nh.param("parking_config_time_delay", parking_config_time_delay_, parking_config_time_delay_);
	controller_nh.param("drive_speed_time_delay", drive_speed_time_delay_, parking_config_time_delay_);
	controller_nh.param("use_cos_scaling", use_cos_scaling_, use_cos_scaling_);

	return true;
}

void starting(const ros::Time &time) override
{
	std::array<double, WHEELCOUNT> steer_angles;
	for (size_t k = 0; k < WHEELCOUNT; k++)
	{
		steer_angles[k] = latency_compensation_state_->getLatencyCompensatedValue(steering_names_[k], time);
		last_wheel_angle_[k] = swerveC_->getWheelAngle(k, steer_angles[k]);
		last_wheel_sign_[k] = 1.0;
		speeds_angles_[k][0] = 0;
		speeds_angles_[k][1] = steer_angles[k];
	}
	brake(steer_angles, time, true);
	// Assume we braked infinitely long ago - this will keep the
	// drive base in parking config until a non-zero command comes in
	time_before_brake_ = 0;

	// Register starting time used to keep fixed rate
	if (comp_odom_)
	{
		odom_pub_interval_counter_ = std::make_unique<PeriodicIntervalCounter>(odom_pub_freq_);
		odom_tf_pub_interval_counter_ = std::make_unique<PeriodicIntervalCounter>(odom_pub_freq_);
	}
	//odometry_.init(time);
}

void stopping(const ros::Time &time) override
{
	time_before_brake_ = 0;
	std::array<double, WHEELCOUNT> steer_angles;
	for (size_t k = 0; k < WHEELCOUNT; k++)
	{
		steer_angles[k] = latency_compensation_state_->getLatencyCompensatedValue(steering_names_[k], time);
	}
	brake(steer_angles, time, true);
}

void update(const ros::Time &time, const ros::Duration &period) override
{
	// Grab current steering angle
	std::array<double, WHEELCOUNT> steer_angles;
	for (size_t k = 0; k < WHEELCOUNT; k++)
	{
		steer_angles[k] = latency_compensation_state_->getLatencyCompensatedValue(steering_names_[k], time);
	}

	if (comp_odom_) compOdometry(time, period, steer_angles);

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
			steering_joints_[i].setControlSlot(0);
			steering_joints_[i].setControlMode(hardware_interface::talonfxpro::TalonMode::MotionMagicExpoVoltage);
			// TODO - don't think this is ever set to any other value, might be OK to move it
			// to init or starting or something
			steering_joints_[i].setControlFeedforward(0);
		}
		speed_joints_[i].setControlSlot(0);
	}

	// Special case for when the drive base is stopped
	if (fabs(curr_cmd.lin[0]) <= 1e-6 && fabs(curr_cmd.lin[1]) <= 1e-6 && fabs(curr_cmd.ang) <= 1e-6)
	{
		if ((time.toSec() - time_before_brake_) > parking_config_time_delay_)
		{
			// If the time since we last moved is long enough, go into parking config
			brake(steer_angles, time);
		}
		else
		{
			// Otherwise, leave the wheels pointed in the same direction they were
			// previously and either coast or brake to a stop depending on how
			// the drive base is currently configured.
			for (size_t i = 0; i < WHEELCOUNT; ++i)
			{
				speed_joints_[i].setControlMode(hardware_interface::talonfxpro::TalonMode::DutyCycleOut);
				speed_joints_[i].setControlFeedforward(0);

				if (neutral_mode_ == hardware_interface::talonfxpro::NeutralMode::Coast)
				{
					// coast mode is now just ff term like before.  The small force commanded
					// from the motor will slow the robot gradually vs. brake mode's immediate stop
					speed_joints_[i].setControlOutput(last_wheel_sign_[i] * stopping_ff_);
				}
				else
				{
					// if we are in brake mode it is time to stop immediately.
					// Setting speed to 0 will trigger brake mode
					speed_joints_[i].setControlOutput(0);
				}
				speed_joints_[i].setNeutralMode(neutral_mode_);

				if (!dont_set_angle_mode)
				{
					steering_joints_[i].setControlOutput(0);
					steering_joints_[i].setControlPosition(speeds_angles_[i][1]);
					steering_joints_[i].setControlVelocity(0);
					steering_joints_[i].setControlAcceleration(0);
				}
			}
		}
		if (cmd_vel_pub_ && cmd_vel_pub_->trylock())
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
		steering_joints_[i].setControlOutput(0);
		steering_joints_[i].setControlPosition(speeds_angles_[i][1]);
		steering_joints_[i].setControlVelocity(0);
		steering_joints_[i].setControlAcceleration(0);
	}

	// Wait a bit after coming out of parking
	// config before setting wheel velocity
	if ((time.toSec() - brake_last_) > drive_speed_time_delay_)
	{
		for (size_t i = 0; i < WHEELCOUNT; ++i)
		{
			if (!percent_out_drive_mode)
			{
				speed_joints_[i].setControlMode(hardware_interface::talonfxpro::TalonMode::VelocityVoltage);
				speed_joints_[i].setControlVelocity(speeds_angles_[i][0]);

				// Add static feed forward in direction of current velocity
				speed_joints_[i].setControlFeedforward(copysign(f_s_, speeds_angles_[i][0]));
				last_wheel_sign_[i] = copysign(1., speeds_angles_[i][0]);
			}
			else
			{
				// ROS_INFO_STREAM("Percent out drive mode stopping wheels!!!!!!!!!!!!!!!!======================================");
				// Debugging mode - used for robot characterization by sweeping
				// from 0-100% output and measuring response
				speed_joints_[i].setControlMode(hardware_interface::talonfxpro::TalonMode::DutyCycleOut);
				speed_joints_[i].setControlOutput(hypot(curr_cmd.lin[0], curr_cmd.lin[1]));
				speed_joints_[i].setControlFeedforward(0);
			}
		}
	}
	else
	{
		// For a small time after coming out of parking config keep the drive motors
		// stopped.
		// TODO - experiment with coast mode, and perhaps position PID at the current
		// position?
		for (size_t i = 0; i < WHEELCOUNT; ++i)
		{
			speed_joints_[i].setControlOutput(0);
			speed_joints_[i].setControlMode(hardware_interface::talonfxpro::TalonMode::DutyCycleOut);
			speed_joints_[i].setControlFeedforward(0);
		}
	}
	for (size_t i = 0; i < WHEELCOUNT; ++i)
	{
		speed_joints_[i].setNeutralMode(neutral_mode_);
	}

	if (cmd_vel_pub_ && cmd_vel_pub_->trylock())
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
	//ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " start_angle = " << start_angle << " end_angle = " << end_angle);
	const double deltaAngle = end_angle - start_angle;
	// TODO - this angle is just going to have cos and sin called on it,
	// might not need to be normalized?
	return angles::normalize_angle(start_angle + deltaAngle / 2.0);
}

void compOdometry(const ros::Time &time, const ros::Duration &period, const std::array<double, WHEELCOUNT> &steer_angles)
{
	if (reset_odom_.exchange(false))
	{
		initial_yaw_ = latency_compensation_state_->getLatencyCompensatedValue("pigeon2", time);
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
		const double new_wheel_rot = latency_compensation_state_->getLatencyCompensatedValue(speed_names_[k], time);
		const double delta_rot     = new_wheel_rot - last_wheel_rot_[k];
		const double dist          = delta_rot * model_.wheelRadius * driveRatios_.encodertoRotations;

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
		ROS_INFO_STREAM("last_wheel_angle_[k]=" << last_wheel_angle_[k] << " steer_angle=" << steer_angle << " average_steer_angle=" << average_steer_angle);
		ROS_INFO_STREAM("dist=" << dist << " delta_pos=" << delta_pos[0] << " " << delta_pos[1]);
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
	Eigen::Matrix2d old_rot = svd.matrixV() * svd.matrixU().transpose();
	if (old_rot.determinant() < 0)
		old_rot.col(1) *= -1;

	// Find difference between last IMU yaw and current IMU yaw
	double yaw = latency_compensation_state_->getLatencyCompensatedValue("pigeon2", time);
	if (initial_yaw_ == 0) {
		initial_yaw_ = yaw;
	}
	const double delta_yaw = yaw - initial_yaw_;

	// Set rotation matrix to the rotation measured using the IMU
	Eigen::Matrix2d rot;
	rot << cos(delta_yaw), -sin(delta_yaw),
		   sin(delta_yaw), cos(delta_yaw);

	// Set last IMU yaw to current IMU yaw
	initial_yaw_ = yaw;

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
	if (pub_odom_to_base_ &&
		odom_tf_pub_interval_counter_ &&
		odom_tf_pub_interval_counter_->update(period))
	{
		if (odom_tf_pub_.trylock())
		{
			orientation = tf::createQuaternionMsgFromYaw(odom_yaw);
			orientation_comped = true;

			geometry_msgs::TransformStamped &odom_tf_trans =
				odom_tf_pub_.msg_.transforms[0];
			odom_tf_trans.header.stamp = time;
			odom_tf_trans.transform.translation.x = odom_y;	 // TODO terrible hacky
			odom_tf_trans.transform.translation.y = -odom_x; // TODO terrible hacky
			odom_tf_trans.transform.rotation = orientation;
			// ROS_INFO_STREAM(odom_x);
			odom_tf_pub_.unlockAndPublish();
		}
		else
		{
			odom_tf_pub_interval_counter_->force_publish();
		}
	}

	// odom
	if (odom_pub_interval_counter_ &&
		odom_pub_interval_counter_->update(period))
	{
		if (odom_pub_.trylock())
		{
			if (!orientation_comped)
			{
				orientation = tf::createQuaternionMsgFromYaw(odom_yaw);
			}

			odom_pub_.msg_.header.stamp = time;
			odom_pub_.msg_.pose.pose.position.x = odom_y;  // TODO terrible hacky
			odom_pub_.msg_.pose.pose.position.y = -odom_x; // TODO terrible hacky
			odom_pub_.msg_.pose.pose.orientation = orientation;

			const double inv_delta_t = 1.0 / period.toSec();
			odom_pub_.msg_.twist.twist.linear.x =
				odom_rigid_transf_.translation().y() * inv_delta_t;
			odom_pub_.msg_.twist.twist.linear.y =
				-odom_rigid_transf_.translation().x() * inv_delta_t;
			odom_pub_.msg_.twist.twist.angular.z =
				atan2(odom_rigid_transf_(1, 0), odom_rigid_transf_(0, 0)) * inv_delta_t;

			odom_pub_.unlockAndPublish();
		}
		else
		{
			odom_pub_interval_counter_->force_publish();
		}
	}
}

void brake(const std::array<double, WHEELCOUNT> &steer_angles, const ros::Time &time, bool force_parking_mode = false)
{
	//Use parking config
	const bool dont_set_angle_mode = dont_set_angle_mode_.load(std::memory_order_relaxed);
	const std::array<double, WHEELCOUNT> park_angles = swerveC_->parkingAngles(steer_angles);
	for (size_t i = 0; i < WHEELCOUNT; ++i)
	{
		speed_joints_[i].setControlMode(hardware_interface::talonfxpro::TalonMode::DutyCycleOut);
		speed_joints_[i].setControlOutput(0.0);
		speed_joints_[i].setControlFeedforward(0);
		speed_joints_[i].setNeutralMode(hardware_interface::talonfxpro::NeutralMode::Brake);
		if (!dont_set_angle_mode && (park_when_stopped_ || force_parking_mode))
		{
			steering_joints_[i].setControlOutput(0);
			steering_joints_[i].setControlPosition(park_angles[i]);
			steering_joints_[i].setControlVelocity(0);
			steering_joints_[i].setControlAcceleration(0);
		}
	}
	// Reset the timer which delays drive wheel velocity a bit after
	// the robot's been stopped
	brake_last_ = time.toSec();
}

void cmdVelCallback(const geometry_msgs::Twist &command)
{
	if (this->isRunning())
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

bool changeCenterOfRotationService(talon_swerve_drive_controller_msgs::SetXY::Request& req, talon_swerve_drive_controller_msgs::SetXY::Response &/*res*/)
{
	if(this->isRunning())
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

bool setNeturalModeService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response&/*res*/)
{
	if(this->isRunning())
	{
		if (req.data) {
			neutral_mode_ = hardware_interface::talonfxpro::NeutralMode::Coast;
		}
		else {
			neutral_mode_ = hardware_interface::talonfxpro::NeutralMode::Brake;
		}
	}
	else
	{
		ROS_ERROR_STREAM_NAMED(name_, __PRETTY_FUNCTION__ << " : Can't accept new commands. Controller is not running.");
		return false;
	}
	return true;
}

bool resetOdomService(std_srvs::Empty::Request &/*req*/, std_srvs::Empty::Response &/*res*/)
{
	if (this->isRunning())
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
	if (this->isRunning())
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

bool parkService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	if (this->isRunning())
	{
		ROS_WARN_STREAM("swerve_drive_controller : park when stopped = " << std::to_string(req.data));
		park_when_stopped_.store(req.data, std::memory_order_relaxed);
		res.success = true;
		res.message = "zebracones!"; // 2023-specific message
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
	if (this->isRunning())
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
	if (this->isRunning())
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


std::string name_; // Controller name, for debugging

std::unique_ptr<swerve<WHEELCOUNT>> swerveC_; // Swerve math calculations object

/// Hardware handles:
std::array<talonfxpro_controllers::TalonFXProControllerInterface, WHEELCOUNT> speed_joints_;
std::array<talonfxpro_controllers::TalonFXProControllerInterface, WHEELCOUNT> steering_joints_;

/// Velocity command related:
struct Commands
{
	Eigen::Vector2d lin{0.,0.};
	double ang{0.};
	ros::Time stamp;

	Commands() = default;
};

realtime_tools::RealtimeBuffer<Commands> command_;

double brake_last_{ros::Time::now().toSec()};
double time_before_brake_{0};
double parking_config_time_delay_{0.1};
double drive_speed_time_delay_{0.1};
std::array<Eigen::Vector2d, WHEELCOUNT> speeds_angles_;

double initial_yaw_ = 0;

// Attempt to limit speed of wheels which are pointing further from
// their target angle. Should help to reduce the robot being pulled
// off course coming out of parking config or when making quick direction changes
bool use_cos_scaling_{false};

ros::Subscriber sub_command_; // subscriber for drive base commands

/// Optionally publish executed commands on an output topic
std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>> cmd_vel_pub_;

// Used to change coords of center of robot rotation. By default this is set to
// 0,0 (the center of the drive base), but it can be set to anywhere to allow
// pure rotation commands to rotate the robot around any point in the ground plane.
ros::ServiceServer change_center_of_rotation_serv_;
realtime_tools::RealtimeBuffer<Eigen::Vector2d> center_of_rotation_;

// Used to force the robot to stop
ros::ServiceServer brake_serv_;

// If set, angle motors will not be given a setpoint. Used for debugging
ros::ServiceServer dont_set_angle_mode_serv_;
std::atomic<bool> dont_set_angle_mode_{false};	  // used to lock wheels into place

// If set, the robot will lock the angle motors into tank mode and drive
// the speed motors using the cmd_vel linear x as a % out value rather than
// a speed in m/s.  Used for determining static feed forward values during bring-up
ros::ServiceServer percent_out_drive_mode_serv_;
std::atomic<bool> percent_out_drive_mode_{false}; // run drive wheels in open-loop mode

// Switch between brake and coast mode for speed wheels when stopping the robot
ros::ServiceServer set_neutral_mode_serv_;
std::atomic<hardware_interface::talonfxpro::NeutralMode> neutral_mode_ = hardware_interface::talonfxpro::NeutralMode::Brake;

// Used to toggle parking mode when stopped
ros::ServiceServer park_serv_;
std::atomic<bool> park_when_stopped_{false};      // toggle parking mode when stopped

// Various constants used for drive calcs
swerveVar::driveModel model_;
swerveVar::ratios driveRatios_;
swerveVar::encoderUnits units_;

// Feed forward terms
double f_s_{0};
double stopping_ff_{0};
double f_a_{0};
double f_v_{0};

std::array<double, WHEELCOUNT> last_wheel_sign_;

/// Timeout to consider cmd_vel commands old:
// If no command more recent than this has been seen, stop the robot
double cmd_vel_timeout_{0.5};

/// Whether to allow multiple publishers on cmd_vel topic or not:
bool allow_multiple_cmd_vel_publishers_{true};

static constexpr double DEF_ODOM_PUB_FREQ{100.};
inline static const std::string DEF_ODOM_FRAME{"odom"};
inline static const std::string DEF_BASE_FRAME{"base_link"};
static constexpr double DEF_INIT_X{0};
static constexpr double DEF_INIT_Y{0};
static constexpr double DEF_INIT_YAW{0};
static constexpr double DEF_SD{0.05};

std::array<Eigen::Vector2d, WHEELCOUNT> wheel_coords_;

bool pub_odom_to_base_{false};       // Publish the odometry to base frame transform
double odom_pub_freq_{-1};			 // odom publishing frequency
Eigen::Affine2d init_odom_to_base_;  // Initial odometry to base frame transform
Eigen::Affine2d odom_to_base_;       // Odometry to base frame transform
Eigen::Affine2d odom_rigid_transf_;

bool comp_odom_;
Eigen::Matrix2Xd wheel_pos_;
Eigen::Vector2d neg_wheel_centroid_;
std::array<double, WHEELCOUNT> last_wheel_rot_;	    // used for odom calcs
std::array<double, WHEELCOUNT> last_wheel_angle_;	//

realtime_tools::RealtimePublisher<nav_msgs::Odometry> odom_pub_;
realtime_tools::RealtimePublisher<tf::tfMessage> odom_tf_pub_;
std::unique_ptr<PeriodicIntervalCounter> odom_tf_pub_interval_counter_;
std::unique_ptr<PeriodicIntervalCounter> odom_pub_interval_counter_;

ros::ServiceServer reset_odom_serv_;
std::atomic<bool> reset_odom_{false};

hardware_interface::latency_compensation::CTRELatencyCompensationStateHandle latency_compensation_state_;

std::array<std::string, WHEELCOUNT> speed_names_;
std::array<std::string, WHEELCOUNT> steering_names_;

}; // class definition

} // namespace talon_swerve_drive_controller

#include <pluginlib/class_list_macros.h>
template class talon_swerve_drive_controller::TalonSwerveDriveController<4>;
PLUGINLIB_EXPORT_CLASS(talon_swerve_drive_controller::TalonSwerveDriveController<4>, controller_interface::ControllerBase)
