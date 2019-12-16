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

#include <boost/assign.hpp>

#include <Eigen/Dense>

#include <tf/transform_datatypes.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf/urdfdom_compatibility.h>

#include <talon_swerve_drive_controller/swerve_drive_controller.h>

//TODO: include swerve stuff from C-Control
using Eigen::Vector2d;
using std::array;
using Eigen::Affine2d;
using Eigen::Matrix2d;
using Eigen::Vector2d;

using ros::Time;
using geometry_msgs::TwistConstPtr;
using ros::Duration;

const double talon_swerve_drive_controller::TalonSwerveDriveController::DEF_ODOM_PUB_FREQ = 100.;
const bool talon_swerve_drive_controller::TalonSwerveDriveController::DEF_PUB_ODOM_TO_BASE = false;
const std::string talon_swerve_drive_controller::TalonSwerveDriveController::DEF_ODOM_FRAME = "odom";
const std::string talon_swerve_drive_controller::TalonSwerveDriveController::DEF_BASE_FRAME = "base_link";
const double talon_swerve_drive_controller::TalonSwerveDriveController::DEF_INIT_X = 0.;
const double talon_swerve_drive_controller::TalonSwerveDriveController::DEF_INIT_Y = 0.;
const double talon_swerve_drive_controller::TalonSwerveDriveController::DEF_INIT_YAW = 0.;
const double talon_swerve_drive_controller::TalonSwerveDriveController::DEF_SD = 0.01;

#if 0
static double euclideanOfVectors(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
  return std::sqrt(std::pow(vec1.x-vec2.x,2) +
                   std::pow(vec1.y-vec2.y,2) +
                   std::pow(vec1.z-vec2.z,2));
}
/*
* \brief Check that a link exists and has a geometry collision.
* \param link The link
* \return true if the link has a collision element with geometry
*/
static bool hasCollisionGeometry(const urdf::LinkConstSharedPtr &link)
{
	if (!link)
	{
		ROS_ERROR("Link == NULL.");
		return false;
	}

	if (!link->collision)
	{
		ROS_ERROR_STREAM("Link " << link->name << " does not have collision description. Add collision description for link to urdf.");
		return false;
	}

	if (!link->collision->geometry)
	{
		ROS_ERROR_STREAM("Link " << link->name << " does not have collision geometry description. Add collision geometry description for link to urdf.");
		return false;
	}
	return true;
}
/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
static bool isCylinder(const urdf::LinkConstSharedPtr &link)
{
	if (!hasCollisionGeometry(link))
	{
		return false;
	}

	if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
	{
		ROS_DEBUG_STREAM("Link " << link->name << " does not have cylinder geometry");
		return false;
	}

	return true;
}

/*
 * \brief Check if the link is modeled as a sphere
 * \param link Link
 * \return true if the link is modeled as a Sphere; false otherwise
 *
 * \param link Link
 * \return true if the link is modeled as a Sphere; false otherwise
 */
static bool isSphere(const urdf::LinkConstSharedPtr &link)
{
	if (!hasCollisionGeometry(link))
	{
		return false;
	}

	if (link->collision->geometry->type != urdf::Geometry::SPHERE)
	{
		ROS_DEBUG_STREAM("Link " << link->name << " does not have sphere geometry");
		return false;
	}

	return true;
}
#endif

/*
 * \brief Get the wheel radius
 * \param [in]  wheel_link   Wheel link
 * \param [out] wheel_radius Wheel radius [m]
 * \return true if the wheel radius was found; false otherwise
 */
#if 0
static bool getWheelRadius(const urdf::LinkConstSharedPtr &wheel_link, double &wheel_radius)
{
	if (isCylinder(wheel_link))
	{
		wheel_radius = (static_cast<urdf::Cylinder *>(wheel_link->collision->geometry.get()))->radius;
		return true;
	}
	else if (isSphere(wheel_link))
	{
		wheel_radius = (static_cast<urdf::Sphere *>(wheel_link->collision->geometry.get()))->radius;
		return true;
	}

	ROS_ERROR_STREAM("Wheel link " << wheel_link->name << " is NOT modeled as a cylinder or sphere!");
	return false;
}
#endif

namespace talon_swerve_drive_controller
{

TalonSwerveDriveController::TalonSwerveDriveController() :
	wheel_radius_(0.0),
	cmd_vel_timeout_(0.5), //Change to 5.0 for auto path planning testing
	allow_multiple_cmd_vel_publishers_(true),
	base_frame_id_("base_link"),
	odom_frame_id_("odom"),
	wheel_joints_size_(0)
{
}

bool TalonSwerveDriveController::init(hardware_interface::TalonCommandInterface *hw,
									  ros::NodeHandle &/*root_nh*/,
									  ros::NodeHandle &controller_nh)
{
	const std::string complete_ns = controller_nh.getNamespace();
	std::size_t id = complete_ns.find_last_of("/");
	name_ = complete_ns.substr(id + 1);

	// Get joint names from the parameter server
	std::vector<std::string> speed_names, steering_names;
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
	else
	{
		wheel_joints_size_ = speed_names.size();

		speed_joints_.resize(wheel_joints_size_);
		steering_joints_.resize(wheel_joints_size_);
	}

	/*ros::NodeHandle n; //Is this bad?

	ros::NodeHandle n_params_behaviors(n, "auto_params");

	if (!n_params_behaviors.getParam("num_profile_slots", num_profile_slots_))
	        ROS_ERROR("Didn't read param num_profile_slots in talon_swerve");
	*/
	num_profile_slots_ = 20;

	// Publish limited velocity:
	controller_nh.param("publish_cmd", publish_cmd_, true);

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
	if (!controller_nh.getParam("wheel_coords1x", wheel_coords_[0][0]))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read wheel_coords1x");
		return false;
	}
	if (!controller_nh.getParam("wheel_coords2x", wheel_coords_[1][0]))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read wheel_coords2x");
		return false;
	}
	if (!controller_nh.getParam("wheel_coords3x", wheel_coords_[2][0]))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read wheel_coords3x");
		return false;
	}
	if (!controller_nh.getParam("wheel_coords4x", wheel_coords_[3][0]))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read wheel_coords4x");
		return false;
	}
	if (!controller_nh.getParam("wheel_coords1y", wheel_coords_[0][1]))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read wheel_coords1y");
		return false;
	}
	if (!controller_nh.getParam("wheel_coords2y", wheel_coords_[1][1]))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read wheel_coords2y");
		return false;
	}
	if (!controller_nh.getParam("wheel_coords3y", wheel_coords_[2][1]))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read wheel_coords3y");
		return false;
	}
	if (!controller_nh.getParam("wheel_coords4y", wheel_coords_[3][1]))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read wheel_coords4y");
		return false;
	}

	ROS_INFO_STREAM("Coords: " << wheel_coords_[0] << "   " << wheel_coords_[1] << "   " << wheel_coords_[2] << "   " << wheel_coords_[3]);
	std::vector<double> offsets;
	for (auto it = steering_names.cbegin(); it != steering_names.cend(); ++it)
	{
		ros::NodeHandle nh(controller_nh, *it);
		double dbl_val = 0;
		if (!nh.getParam("offset", dbl_val))
		{
			ROS_ERROR_STREAM("Can not read offset for " << *it);
			return false;
		}
		offsets.push_back(dbl_val);
	}

	profile_queue_num = controller_nh.advertise<std_msgs::UInt16>("profile_queue_num", 1);

	cmd_vel_mode_.store(true, std::memory_order_relaxed);
	dont_set_angle_mode_.store(false, std::memory_order_relaxed);
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

	if (publish_cmd_)
	{
	  cmd_vel_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>(controller_nh, "cmd_vel_out", 5));
	}
	// Get the joint object to use in the realtime loop

	// TODO : all of these need to be read from params
	/*
	model.maxSpeed = 3.3528;
	model.mass = 70;
	model.motorFreeSpeed = 5330;
	model.motorStallTorque = 2.41;
	model.motorQuantity = 4;
	*/
	model_.wheelRadius =  wheel_radius_;

	/*
	swerveVar::ratios driveRatios({20, 7, 7});
	swerveVar::encoderUnits units({1,1,1,1,1,1});
	*/

	swerveC_ = std::make_shared<swerve>(wheel_coords_, offsets, driveRatios_, units_, model_);
	for (size_t i = 0; i < wheel_joints_size_; ++i)
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
	brake_serv_ = controller_nh.advertiseService("brake", &TalonSwerveDriveController::brakeService, this);
	dont_set_angle_mode_serv_ = controller_nh.advertiseService("dont_set_angle", &TalonSwerveDriveController::dontSetAngleModeService, this);
	motion_profile_serv_ = controller_nh.advertiseService("run_profile", &TalonSwerveDriveController::motionProfileService, this);
	change_center_of_rotation_serv_ = controller_nh.advertiseService("change_center_of_rotation", &TalonSwerveDriveController::changeCenterOfRotationService, this);

	double odom_pub_freq;
	controller_nh.param("odometry_publishing_frequency", odom_pub_freq, DEF_ODOM_PUB_FREQ);

	comp_odom_ = odom_pub_freq > 0;
	//ROS_WARN("COMPUTING ODOM");
	if (comp_odom_)
	{
		odom_pub_period_ = Duration(1 / odom_pub_freq);
		controller_nh.param("publish_odometry_to_base_transform", pub_odom_to_base_,
							DEF_PUB_ODOM_TO_BASE);

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
		init_odom_to_base_.translation() = Vector2d(init_x, init_y);
		odom_to_base_ = init_odom_to_base_;
		odom_rigid_transf_.setIdentity();

		wheel_pos_.resize(2, WHEELCOUNT);
		for (size_t i = 0; i < WHEELCOUNT; i++)
		{
			wheel_pos_.col(i) = wheel_coords_[i];
		}

		const Vector2d centroid = wheel_pos_.rowwise().mean();
		wheel_pos_.colwise() -= centroid;
		neg_wheel_centroid_ = -centroid;

		new_wheel_pos_.resize(WHEELCOUNT, 2);

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

	return true;
}

void TalonSwerveDriveController::compOdometry(const Time &time, const double inv_delta_t, const std::array<double, WHEELCOUNT> &steer_angles)
{
	// Compute the rigid transform from wheel_pos_ to new_wheel_pos_.

	// Use the encoder-reported wheel angle plus the difference between
	// the previous and current wheel rotation to calculate motion for each
	// wheel since the last odom value update.
	// Add the movement of each wheel to the config-defined wheel coords
	// From this, figure out a rigid body rotation and translation
	// which best matches the difference in position of each of the wheels
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

		// delta_pos of the wheel in x,y - decompose vector into x&y components
		const Eigen::Vector2d delta_pos = {dist * cos(steer_angle), dist * sin(steer_angle)};

		// new_wheel_pos is constructed to hold x,y position given the assumption
		// that the robot started with each wheel at the wheel_coords_ position
		// Since we're just measuring a change in position, the initial position
		// is arbitrary just as long as it is a valid wheel position shape.
		// Note this is actually built as a transpose of the shape of wheel_coords
		// to simplyify the math later
		new_wheel_pos_(k, 0) = wheel_coords_[k][0] + delta_pos[0];
		new_wheel_pos_(k, 1) = wheel_coords_[k][1] + delta_pos[1];
#if 0
		ROS_INFO_STREAM("steer_angle = " << steer_angle << " dist = " << dist << " delta_pos = " << delta_pos);
		ROS_INFO_STREAM("x from " <<  wheel_coords_[k][0] << " to " << new_wheel_pos_(k, 0));
		ROS_INFO_STREAM("y from " <<  wheel_coords_[k][1] << " to " << new_wheel_pos_(k, 1));
#endif

		// Save the previous wheel rotation to use next time odom is run
		last_wheel_rot_[k] = new_wheel_rot;
	}

	// http://nghiaho.com/?page_id=671, but that page talks about 3d transform where
	//    are just looking for a 2d one (assumine the robot stays on the ground, not
	//    that we can measure if it doesn't...)
	// Also, https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
	// Find the translation+rotation that produces the least square error
	// between (prev pos + transform) and current measured position
	const Eigen::RowVector2d new_wheel_centroid = new_wheel_pos_.colwise().mean();
	new_wheel_pos_.rowwise() -= new_wheel_centroid;

	const Matrix2d h = wheel_pos_ * new_wheel_pos_;
	const Eigen::JacobiSVD<Matrix2d> svd(h, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Matrix2d rot = svd.matrixV() * svd.matrixU().transpose();
	if (rot.determinant() < 0)
		rot.col(1) *= -1;

	odom_rigid_transf_.matrix().block(0, 0, 2, 2) = rot;
	odom_rigid_transf_.translation() = rot * neg_wheel_centroid_ + new_wheel_centroid.transpose();
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
		odom_tf_trans.transform.translation.x = odom_x;
		odom_tf_trans.transform.translation.y = odom_y;
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
		odom_pub_.msg_.pose.pose.position.x = odom_x;
		odom_pub_.msg_.pose.pose.position.y = odom_y;
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

void TalonSwerveDriveController::update(const ros::Time &time, const ros::Duration &period)
{
	const double delta_t = period.toSec();
	const double inv_delta_t = 1.0 / delta_t;

	// Grab current steering angle, store it
	// for other code to use
	std::array<double, WHEELCOUNT> steer_angles;
	for (size_t k = 0; k < WHEELCOUNT; k++)
	{
		steer_angles[k] = steering_joints_[k].getPosition();
	}
	{
		std::lock_guard<std::mutex> lock(steer_angles_mutex_);
		steer_angles_ = steer_angles;
	}

	if (comp_odom_) compOdometry(time, inv_delta_t, steer_angles);

	// MOVE ROBOT
	// Retreive current velocity command and time step:

	//ROS_INFO_STREAM("mode: " << cmd_vel_mode_);

	static bool set_profile_run = false;

	size_t full_profile_buffer_size = 0;
	full_profile_cmd cur_prof_cmd;

	// Protect access to full_profile_buffer_
	// with a mutex to enforce thread safety
	{
		std::unique_lock<std::mutex> l(profile_mutex_, std::try_to_lock);
		if (l.owns_lock())
		{
			full_profile_buffer_size = full_profile_buffer_.size();
			if (full_profile_buffer_size != 0)
			{
				cur_prof_cmd = full_profile_buffer_.front();
				full_profile_buffer_.pop_front();
			}
		}
	}

	if (full_profile_buffer_size != 0)
	{
		if (cur_prof_cmd.brake)
		{
			ROS_WARN("profile_reset");
			//required for reset
			for (size_t k = 0; k < WHEELCOUNT; k++)
			{
				steering_joints_[k].setCustomProfileRun(false);
				speed_joints_[k].setCustomProfileRun(false);
				set_profile_run = false;
			}
			Commands brake_struct;
			brake_struct.lin[0] = 0;
			brake_struct.lin[1] = 0;
			brake_struct.ang = 0;
			brake_struct.stamp = ros::Time::now();
			ROS_WARN("called in controller");
			command_.writeFromNonRT(brake_struct);
			cmd_vel_mode_.store(true, std::memory_order_relaxed);
		}

		if (cur_prof_cmd.wipe_all)
		{
			ROS_WARN("profile_wipe");
			for (int i = 0; i < num_profile_slots_; i++)
			{
				for (size_t k = 0; k < WHEELCOUNT; k++)
				{
					full_profile_[k][0].clear();
					full_profile_[k][1].clear();
					speed_joints_[k].overwriteCustomProfilePoints(full_profile_[k][0], i);
					steering_joints_[k].overwriteCustomProfilePoints(full_profile_[k][1], i);
				}
			}
		}

		if (cur_prof_cmd.buffer)
		{
			ROS_WARN("buffer in controller - pre loop");
			for (size_t p = 0; p < cur_prof_cmd.profiles.size(); p++)
			{
				ROS_WARN("buffer in controller");
				const int point_count2 = cur_prof_cmd.profiles[p].drive_pos.size();
				ROS_INFO_STREAM("points: " << point_count2);
				for (size_t i = 0; i < WHEELCOUNT; i++)
				{
					holder_points_[i][0].mode = cur_prof_cmd.profiles[p].hold[0][i] ? hardware_interface::TalonMode_PercentOutput : hardware_interface::TalonMode_Position;
					holder_points_[i][1].mode = cur_prof_cmd.profiles[p].hold[0][i] ? hardware_interface::TalonMode_MotionMagic : hardware_interface::TalonMode_Position;

					holder_points_[i][0].pidSlot = 1;
					holder_points_[i][1].pidSlot = cur_prof_cmd.profiles[p].hold[0][i] ? 0 : 1; //0 and 1 are the same right now

					holder_points_[i][0].setpoint = cur_prof_cmd.profiles[p].hold[0][i] ? 0 : cur_prof_cmd.profiles[p].drive_pos[0][i];
					holder_points_[i][1].setpoint = cur_prof_cmd.profiles[p].steer_pos[0][i];

					holder_points_[i][0].fTerm = cur_prof_cmd.profiles[p].hold[0][i] ? 0 : cur_prof_cmd.profiles[p].drive_f[0][i];
					holder_points_[i][1].fTerm = cur_prof_cmd.profiles[p].hold[0][i] ? 0 : cur_prof_cmd.profiles[p].steer_f[0][i];

					holder_points_[i][0].duration = cur_prof_cmd.profiles[p].dt;
					holder_points_[i][1].duration = cur_prof_cmd.profiles[p].dt;

					holder_points_[i][0].zeroPos = true;
					holder_points_[i][1].zeroPos = false;

					full_profile_[i][0].clear();
					full_profile_[i][1].clear();

					full_profile_[i][0].push_back(holder_points_[i][0]); //Rather than buffering like this we should write directly to full profile at some point
					full_profile_[i][1].push_back(holder_points_[i][1]); //Rather than buffering like this we should write directly to full profile at some point

					holder_points_[i][0].zeroPos = false;
				}

				const int point_count = cur_prof_cmd.profiles[p].drive_pos.size();
				ROS_INFO_STREAM("points: " << point_count);
				for (int i = 1; i < point_count; i++)
				{
					for (size_t k = 0; k < WHEELCOUNT; k++)
					{
						holder_points_[k][0].mode = cur_prof_cmd.profiles[p].hold[i][k] ? hardware_interface::TalonMode_PercentOutput : hardware_interface::TalonMode_Position;
						holder_points_[k][1].mode = cur_prof_cmd.profiles[p].hold[i][k] ? hardware_interface::TalonMode_MotionMagic : hardware_interface::TalonMode_Position;

						holder_points_[k][0].setpoint = cur_prof_cmd.profiles[p].hold[i][k] ? 0 : cur_prof_cmd.profiles[p].drive_pos[i][k];
						holder_points_[k][1].setpoint = cur_prof_cmd.profiles[p].steer_pos[i][k];

						holder_points_[k][0].fTerm = cur_prof_cmd.profiles[p].hold[i][k] ? 0 : cur_prof_cmd.profiles[p].drive_f[i][k];
						holder_points_[k][1].fTerm = cur_prof_cmd.profiles[p].hold[i][k] ? 0 : cur_prof_cmd.profiles[p].steer_f[i][k];
						//ROS_INFO_STREAM("f: " << holder_points_[k][0].fTerm);

						holder_points_[k][1].pidSlot = cur_prof_cmd.profiles[p].hold[i][k] ? 0 : 1;

						full_profile_[k][0].push_back(holder_points_[k][0]); //Rather than buffering like this we should write directly to full profile at some point
						full_profile_[k][1].push_back(holder_points_[k][1]); //Rather than buffering like this we should write directly to full profile at some point
					}
				}
				ROS_WARN("done1");
				for (size_t k = 0; k < WHEELCOUNT; k++)
				{
					speed_joints_[k].overwriteCustomProfilePoints(full_profile_[k][0], cur_prof_cmd.profiles[p].slot);
					steering_joints_[k].overwriteCustomProfilePoints(full_profile_[k][1], cur_prof_cmd.profiles[p].slot);
				}

				ROS_WARN("done");
			}
		}

		if (cur_prof_cmd.run)
		{
			ROS_WARN("running from  controller");
			cmd_vel_mode_.store(false, std::memory_order_relaxed);
			for (size_t k = 0; k < WHEELCOUNT; k++)
			{
				steering_joints_[k].setCustomProfileSlot(cur_prof_cmd.run_slot);
				speed_joints_[k].setCustomProfileSlot(cur_prof_cmd.run_slot);
			}
		}

		if (cur_prof_cmd.change_queue)
		{
			for (size_t k = 0; k < WHEELCOUNT; k++)
			{
				steering_joints_[k].setCustomProfileNextSlot(cur_prof_cmd.new_queue);
				speed_joints_[k].setCustomProfileNextSlot(cur_prof_cmd.new_queue);
			}
		}

	}
	static double mode_last_time = ros::Time::now().toSec();
	if (cmd_vel_mode_.load(std::memory_order_relaxed))
	{
		Commands curr_cmd = *(command_.readFromRT());
		const double dt = (time - curr_cmd.stamp).toSec();
		const bool dont_set_angle_mode = dont_set_angle_mode_.load(std::memory_order_relaxed);

		//ROS_INFO_STREAM("ang_vel_tar: " << curr_cmd.ang << " lin_vel_tar: " << curr_cmd.lin);

		// Brake if cmd_vel has timeout:
		if (dt > cmd_vel_timeout_)
		{
			curr_cmd.lin = {0.0, 0.0};
			curr_cmd.ang = 0.0;
		}

		static std::array<Vector2d, WHEELCOUNT> speeds_angles;

		for (size_t i = 0; i < wheel_joints_size_; ++i)
		{
			steering_joints_[i].setCustomProfileRun(false);
			speed_joints_[i].setCustomProfileRun(false);
			set_profile_run = false;

			if (!dont_set_angle_mode_)
			{
				steering_joints_[i].setPIDFSlot(0);
				steering_joints_[i].setMode(hardware_interface::TalonMode::TalonMode_MotionMagic);
				steering_joints_[i].setDemand1Value(0);
			}

			speed_joints_[i].setPIDFSlot(0);
			speed_joints_[i].setClosedloopRamp(0);
			speed_joints_[i].setDemand1Value(0);

		}
		static double brake_last = ros::Time::now().toSec();
		static double time_before_brake = 0;
		if (fabs(curr_cmd.lin[0]) <= 1e-6 && fabs(curr_cmd.lin[1]) <= 1e-6 && fabs(curr_cmd.ang) <= 1e-6)
		{
			brake_last = ros::Time::now().toSec();

			for (size_t i = 0; i < wheel_joints_size_; ++i)
			{
				speed_joints_[i].setCommand(0);
				speed_joints_[i].setMode(hardware_interface::TalonMode::TalonMode_PercentOutput);
			}
			if (ros::Time::now().toSec() - time_before_brake > .5)
			{
				brake();
			}
			else
			{
				for (size_t i = 0; !dont_set_angle_mode && (i < wheel_joints_size_); ++i)
				{
					steering_joints_[i].setCommand(speeds_angles[i][1]);
				}
			}
			if (publish_cmd_ && cmd_vel_pub_->trylock())
			{
				cmd_vel_pub_->msg_.header.stamp = ros::Time::now();
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

		time_before_brake = ros::Time::now().toSec();

		// Limit velocities and accelerations:
		//const double cmd_dt(period.toSec());

		// Compute wheels velocities:
		//Parse curr_cmd to get velocity vector and rotation (z axis)
		//TODO: check unit conversions/coordinate frames

		speeds_angles = swerveC_->motorOutputs(curr_cmd.lin, curr_cmd.ang, M_PI / 2.0, steer_angles, true, *(center_of_rotation_.readFromRT()));

		// Set wheel steering angles, as long as dont_set_angle_mode is false
		for (size_t i = 0; !dont_set_angle_mode && (i < wheel_joints_size_); ++i)
		{
			steering_joints_[i].setCommand(speeds_angles[i][1]);
		}

		if (ros::Time::now().toSec() - .1 > brake_last || ros::Time::now().toSec() - .1 > mode_last_time)
		{
			for (size_t i = 0; i < wheel_joints_size_; ++i)
			{
				speed_joints_[i].setMode(hardware_interface::TalonMode::TalonMode_Velocity);
				speed_joints_[i].setCommand(speeds_angles[i][0]);
			}
		}
		else
		{
			for (size_t i = 0; i < wheel_joints_size_; ++i)
			{
				speed_joints_[i].setCommand(0);
				speed_joints_[i].setMode(hardware_interface::TalonMode::TalonMode_PercentOutput);
			}
		}
		if (publish_cmd_ && cmd_vel_pub_->trylock())
		{
			cmd_vel_pub_->msg_.header.stamp = ros::Time::now();
			cmd_vel_pub_->msg_.twist.linear.x = curr_cmd.lin[0];
			cmd_vel_pub_->msg_.twist.linear.y = curr_cmd.lin[1];
			cmd_vel_pub_->msg_.twist.linear.z = 0;
			cmd_vel_pub_->msg_.twist.angular.x = 0;
			cmd_vel_pub_->msg_.twist.angular.y = 0;
			cmd_vel_pub_->msg_.twist.angular.z = curr_cmd.ang;
			cmd_vel_pub_->unlockAndPublish();
		}
	}
	else
	{
		ROS_INFO_STREAM_THROTTLE(.5, "out of points = " << steering_joints_[0].getCustomProfileStatus().outOfPoints);
		mode_last_time =::Time::now().toSec();
		for (size_t i = 0; !set_profile_run && (i < wheel_joints_size_); ++i)
		{
			steering_joints_[i].setCustomProfileRun(true);
			speed_joints_[i].setCustomProfileRun(true);
		}
		// Make controller only set CustomProfileRun once, so that it can
		// be cleared out by the hwi if needed when e.g. disabling the robot
		set_profile_run = true;

		// Assume all joints are running the same length profile and thus have
		// the same outOfPoints status
		if(steering_joints_[0].getCustomProfileStatus().outOfPoints)
		{
			ROS_WARN("profile_reset");
			//required for reset
			for (size_t k = 0; k < WHEELCOUNT; k++)
			{
				steering_joints_[k].setCustomProfileRun(false);
				speed_joints_[k].setCustomProfileRun(false);
			}
			set_profile_run = false;
			Commands brake_struct;
			brake_struct.lin[0] = 0;
			brake_struct.lin[1] = 0;
			brake_struct.ang = 0;
			brake_struct.stamp = ros::Time::now();
			ROS_WARN("called in controller");
			command_.writeFromNonRT(brake_struct);
			cmd_vel_mode_.store(true, std::memory_order_relaxed);
		}
	}

	static uint16_t slot_ret = 0;
	static int slot_ret_diff_last_sum;

	for (size_t i = 0; i < wheel_joints_size_; ++i)
	{
		if (slot_ret != steering_joints_[i].getCustomProfileSlot()) slot_ret_diff_last_sum += 1;

		slot_ret = steering_joints_[i].getCustomProfileSlot();

		//ROS_ERROR_STREAM(slot_local);
	}

	std_msgs::UInt16 pub_queue_hold;
	pub_queue_hold.data = slot_ret;

	profile_queue_num.publish(pub_queue_hold);
	if (slot_ret_diff_last_sum > 20)
	{
		ROS_ERROR("potential profile slot issue with swerve");
	}
}

void TalonSwerveDriveController::starting(const ros::Time &time)
{
	brake();

	// Register starting time used to keep fixed rate
	if (comp_odom_)
	{
		last_odom_pub_time_ = time;
		last_odom_tf_pub_time_ = time;
	}
	//odometry_.init(time);
}

void TalonSwerveDriveController::stopping(const ros::Time & /*time*/)
{
	brake();
}

void TalonSwerveDriveController::brake()
{
	//Use parking config
	const bool dont_set_angle_mode = dont_set_angle_mode_.load(std::memory_order_relaxed);
	std::array<double, WHEELCOUNT> steer_angles;
	{
		std::lock_guard<std::mutex> lock(steer_angles_mutex_);
		steer_angles = steer_angles_;
	}
	const std::array<double, WHEELCOUNT> park_angles = swerveC_->parkingAngles(steer_angles);
	for (size_t i = 0; i < wheel_joints_size_; ++i)
	{
		speed_joints_[i].setCommand(0.0);
		if (!dont_set_angle_mode)
			steering_joints_[i].setCommand(park_angles[i]);
	}
}

void TalonSwerveDriveController::cmdVelCallback(const geometry_msgs::Twist &command)
{
	if (isRunning())
	{
		// check that we don't have multiple publishers on the command topic
		//ROS_WARN("Time Difference: %f", ros::Time::now().toSec() - command->header.stamp.toSec());
		if (!allow_multiple_cmd_vel_publishers_ && sub_command_.getNumPublishers() > 1)
		{
			ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, name_, "Detected " << sub_command_.getNumPublishers()
											<< " publishers. Only 1 publisher is allowed. Going to brake.");
			brake();
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
		command_struct.stamp = ros::Time::now();
		command_.writeFromNonRT (command_struct);

		cmd_vel_mode_.store(true, std::memory_order_relaxed);

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
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
	}
}

bool TalonSwerveDriveController::motionProfileService(talon_swerve_drive_controller::MotionProfile::Request &req, talon_swerve_drive_controller::MotionProfile::Response &/*res*/)
{
	ROS_INFO_STREAM("running motionProfileService");
	if (isRunning())
	{
		//ros::message_operations::Printer< ::talon_swerve_drive_controller::MotionProfileRequest_<std::allocator<void>> >::stream(std::cout, "", req);
		//ROS_INFO("------------------------------------");
		// Generate MotionProfilePoints (testing)
		// fill in full_profile_cmd
		// lock
		// push to full_profile_buf
		// unlock
		std::array<double, WHEELCOUNT> steer_angles;
		{
			std::lock_guard<std::mutex> lock(steer_angles_mutex_);
			steer_angles = steer_angles_;
		}

		std::array<double, WHEELCOUNT> prev_vels;
		for (size_t i = 0; i < WHEELCOUNT; i++)
		{
			ROS_INFO_STREAM("Steer angle : " << i <<  "=" << steer_angles[i]);
			prev_vels[i] = 0;
		}
		const auto &points = req.joint_trajectory.points;
		const size_t point_count = req.joint_trajectory.points.size();
		if (point_count < 2)
		{
			ROS_ERROR("Need more than 1 point for a motion profile");
			return false;
		}
		const double defined_dt = (points[1].time_from_start - points[0].time_from_start).toSec();
		full_profile_cmd full_profile_struct;
		if (req.buffer)
		{
			full_profile_struct.profiles.resize(1);
			auto &fps = full_profile_struct.profiles[0];

			fps.drive_pos.resize(point_count - 1);
			fps.drive_f.resize(point_count - 1);
			fps.steer_pos.resize(point_count - 1);
			fps.steer_f.resize(point_count - 1);
			fps.hold.resize(point_count - 1);
			fps.dt = req.dt;
			fps.slot = req.slot;

			bool holding = false;
			std::array<Eigen::Vector2d, WHEELCOUNT> angles_positions;
			std::array<Eigen::Vector2d, WHEELCOUNT> angles_velocities;
			std::array<double,WHEELCOUNT> prev_drive_pos;
			for (size_t k = 0; k < WHEELCOUNT; k++)
			{
				prev_drive_pos[k] = 0;
			}
			for (size_t i = 0; i < point_count - 1; i++)
			{
				//ROS_INFO_STREAM("pos_0:" << points[i].positions[0] << " pos_1:" << points[i].positions[1] <<" pos_2:" <<  points[i].positions[2] << " counts: " << point_count << " i: "<< i << " wheels: " << WHEELCOUNT);
				//ROS_INFO_STREAM("pos_0:" << points[i+1].positions[0] << " pos_1:" << points[i+1].positions[1] <<" pos_2:" <<  points[i+1].positions[2] << " counts: " << point_count << " i: "<< i << " wheels: " << WHEELCOUNT);

				// When transitioning from not-hold to hold, calculate
				// the necessary angles once. Reuse them until hold is released
				// This will keep the wheels pointed in the correct direction
				// the entire duration of the requested hold duration
				if (!(holding && req.hold[i]))
				{
					angles_positions =
						swerveC_->motorOutputs({points[i + 1].positions[0] - points[i].positions[0],
								points[i + 1].positions[1] - points[i].positions[1]},
								points[i + 1].positions[2] - points[i].positions[2],
								points[i + 1].positions[2],
								steer_angles,
								false);


					angles_velocities =
						swerveC_->motorOutputs({points[i + 1].velocities[0], points[i + 1].velocities[1]},
								points[i + 1].velocities[2],
								points[i + 1].positions[2],
								steer_angles,
								false);

					for (size_t k = 0; (i > 0) && (k < WHEELCOUNT); k++)
					{
						prev_drive_pos[k] = fps.drive_pos[i - 1][k];
					}
				}
				// Don't drive while hold is active
				for (size_t k = 0; req.hold[i] && (k < WHEELCOUNT); k++)
				{
					angles_positions[k][0] = 0;
				}

				holding = req.hold[i];

				for (size_t k = 0; k < WHEELCOUNT; k++)
				{
					fps.hold[i].push_back(req.hold[i]);
					//ROS_INFO_STREAM("a_p : " << angles_positions[k][0] << " " << angles_positions[k][1] << "hold: " << (bool)req.hold[i]);
					fps.drive_pos[i].push_back(angles_positions[k][0] + prev_drive_pos[k]);
					fps.steer_pos[i].push_back(angles_positions[k][1]);

					if ((i > point_count - 2) || req.hold[i])
					{
						fps.drive_f[i].push_back(0);
						fps.steer_f[i].push_back(0);
					}
					else
					{
						int sign_v = (angles_velocities[k][0] < 0) ? -1 : ((angles_velocities[k][0] > 0) ? 1 : 0);
						fps.drive_f[i].push_back(angles_velocities[k][0] * f_v_ + sign_v * f_s_ + f_a_ /* / ( -fabs(angles_velocities[k][0]) / (model.maxSpeed * 1.2) + 1.05 ) */ * (angles_velocities[k][0] - prev_vels[k]) / defined_dt);
						prev_vels[k] = angles_velocities[k][0];

						const double prev_steer_pos = (i > 0) ? fps.steer_pos[i - 1][k] : steer_angles[k];
						const double steer_v = (angles_positions[k][1] - prev_steer_pos) / defined_dt;
						const int sign_steer_v = (steer_v < 0) ? -1 : ((steer_v > 0) ? 1 : 0);
						fps.steer_f[i].push_back(steer_v * f_s_v_ + sign_steer_v * f_s_s_);
					}

					steer_angles[k] = angles_positions[k][1]; // update for next iteration of the loop
				}
			}
		}

		full_profile_struct.buffer         = req.buffer;
		full_profile_struct.wipe_all		= req.wipe_all;
		full_profile_struct.run			= req.run;
		full_profile_struct.brake			= req.brake;
		full_profile_struct.run_slot		= req.run_slot;
		full_profile_struct.change_queue	= req.change_queue;
		full_profile_struct.newly_set		= true;
		for (size_t i = 0; i < req.new_queue.size(); i++)
		{
			full_profile_struct.new_queue.push_back(req.new_queue[i]);
		}
		ROS_WARN("serv points called");

		full_profile_struct.Print();

		// TODO : use shared_ptr to prevent copies when queuing
		// Guard access to full_profile_buffer with a lock
		std::lock_guard<std::mutex> l(profile_mutex_);
		full_profile_buffer_.push_back(full_profile_struct);

		return true;
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
		return false;
	}
}

bool TalonSwerveDriveController::changeCenterOfRotationService(talon_swerve_drive_controller::SetXY::Request& req, talon_swerve_drive_controller::SetXY::Response &/*res*/)
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
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
		return false;
	}
}


bool TalonSwerveDriveController::brakeService(std_srvs::Empty::Request &/*req*/, std_srvs::Empty::Response &/*res*/)
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
		cmd_vel_mode_.store(true, std::memory_order_relaxed);

		return true;
	}
	else
	{
		ROS_ERROR_NAMED(name_, "brakeService can't accept new commands. Controller is not running.");
		return false;
	}
}

bool TalonSwerveDriveController::dontSetAngleModeService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	if (isRunning())
	{
		ROS_WARN_STREAM("dont_set_angle_mode set to = " << static_cast<int>(req.data));
		dont_set_angle_mode_.store(req.data, std::memory_order_relaxed);

		res.success = true;
		res.message = "SUCCESS!";

		return true;
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't set dont_set_angle_mode. Controller is not running.");
		return false;
	}
}
/*
void TalonSwerveDriveController::cmdCallback(const talon_swerve_drive_controller::CompleteCmd &command)
{
	if (isRunning())
	{
		// check that we don't have multiple publishers on the command topic
		if (sub_command_.getNumPublishers() > 1)
		{
			ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, name_, "Detected " << sub_command_.getNumPublishers()
											<< " publishers. Only 1 publisher is allowed. Going to brake.");
			brake();
			return;
		}
		cmd_vel_mode_.store(command.cmd_vel_or_points, std::memory_order_relaxed);
		if(command.cmd_vel_or_points)
		{
			command_struct_.ang = command.twist_.angular.z;
			command_struct_.lin[0] = command.twist_.linear.x;
			command_struct_.lin[1] = command.twist_.linear.y;
			command_struct_.stamp = ros::Time::now();
			command_.writeFromNonRT (command_struct_);
		}
		else
		{
			points_struct_.lin_points_pos.clear();
			points_struct_.lin_points_vel.clear();
			points_struct_.ang_pos.clear();
			points_struct_.ang_vel.clear();
			double duration = command.joint_trajectory.points[1].time_from_start.toSec()
			- command.joint_trajectory.points[0].time_from_start.toSec();

			if(duration < .0025)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_0ms;
			}
			else if(duration < .0075)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_5ms;
			}
			else if(duration < .015)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_10ms;
			}
			else if(duration < .025)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_20ms;
			}
			else if(duration < .035)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_30ms;
			}
			else if(duration < .045)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_40ms;
			}
			else if(duration < .075)
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_50ms;
			}
			else
			{
			     points_struct_.dt = hardware_interface::TrajectoryDuration::TrajectoryDuration_100ms;
			}
			for(size_t i = 0; i < command.joint_trajectory.points.size(); i++)
			{
				points_struct_.lin_points_pos.push_back({command.joint_trajectory.points[i].positions[0], command.joint_trajectory.points[i].positions[1]});
				points_struct_.lin_points_vel.push_back({command.joint_trajectory.points[i].velocities[0], command.joint_trajectory.points[i].velocities[1]});
				points_struct_.ang_pos.push_back(command.joint_trajectory.points[i].positions[2]);
				points_struct_.ang_vel.push_back(command.joint_trajectory.points[i].velocities[2]);
			}
			command_points_.writeFromNonRT(points_struct_);
		}
	}
	else
	{
		ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
	}
}

*/

bool TalonSwerveDriveController::getWheelNames(ros::NodeHandle &controller_nh,
		const std::string &wheel_param,
		std::vector<std::string> &wheel_names)
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

		wheel_names.resize(wheel_list.size());
		for (int i = 0; i < wheel_list.size(); ++i)
		{
			wheel_names[i] = static_cast<std::string>(wheel_list[i]);
		}
	}
	else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
	{
		wheel_names.push_back(wheel_list);
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
}
/*
  bool TalonSwerveDriveController::setOdomParamsFromUrdf(ros::NodeHandle& root_nh,
                             const std::string& steering_name,
                             const std::string& speed_name,
                             bool lookup_wheel_radius)
  //{
    if (!(lookup_wheel_radius))
    {
      // Short-circuit in case we don't need to look up anything, so we don't have to parse the URDF
      return true;
    }

    // Parse robot description
    const std::string model_param_name = "robot_description";
    bool res = root_nh.hasParam(model_param_name);
    std::string robot_model_str="";
    if (!res || !root_nh.getParam(model_param_name,robot_model_str))
    {
      ROS_ERROR_NAMED(name_, "Robot descripion couldn't be retrieved from param server.");
      return false;
    }

    urdf::ModelInterfaceSharedPtr model(urdf::parseURDF(robot_model_str));

    //TODO: replace with swerve equivalent
    //urdf::JointConstSharedPtr left_wheel_joint(model->getJoint(left_wheel_name));
    //urdf::JointConstSharedPtr right_wheel_joint(model->getJoint(right_wheel_name));

    if(lookup_wheel_radius)
    {
      // Get wheel radius
      if (!getWheelRadius(model->getLink(left_wheel_joint->child_link_name), wheel_radius_))
      {
        ROS_ERROR_STREAM_NAMED(name_, "Couldn't retrieve " << left_wheel_name << " wheel radius");
        return false;
      }
    XmlRpc::XmlRpcValue twist_cov_list;
    controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
      ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = odom_frame_id_;
    odom_pub_->msg_.child_frame_id = base_frame_id_;
    odom_pub_->msg_.pose.pose.position.z = 0;
    odom_pub_->msg_.pose.covariance = boost::assign::list_of
        (static_cast<double>(pose_cov_list[0])) (0)  (0)  (0)  (0)  (0)
        (0)  (static_cast<double>(pose_cov_list[1])) (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(pose_cov_list[3])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[4])) (0)
        (0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));
    odom_pub_->msg_.twist.twist.linear.z  = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;
    odom_pub_->msg_.twist.covariance = boost::assign::list_of
        (static_cast<double>(twist_cov_list[0])) (0)  (0)  (0)  (0)  (0)
        (0)  (static_cast<double>(twist_cov_list[1])) (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(twist_cov_list[3])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[4])) (0)
        (0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));
    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;
  }
*/
//}

