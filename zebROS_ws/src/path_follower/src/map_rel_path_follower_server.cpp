#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <geometry_msgs/Quaternion.h>
#include <path_follower_msgs/PathAction.h>
#include <path_follower_msgs/PathGoal.h>
#include <axis_state/axis_state.h>
#include <path_follower/path_follower.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <algorithm>
#include "tf2_ros/transform_listener.h"
#include "pid_velocity_msg/PIDVelocity.h"
#include <angles/angles.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Odometry.h>

#define DEBUG

class PathAction
{
	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<path_follower_msgs::PathAction> as_;
		std::string action_name_;
		geometry_msgs::TransformStamped map_to_baselink_;
		//ros::Subscriber pose_sub_;
		//geometry_msgs::PoseStamped pose_;
		ros::Subscriber yaw_sub_;
		geometry_msgs::Quaternion orientation_;

		ros::Publisher orientation_command_pub_;

		std::map<std::string, AlignActionAxisStatePositionVelocity> axis_states_;
		ros::Publisher combine_cmd_vel_pub_;

		ros::Publisher robot_relative_yaw_pub_;
		ros::Publisher zero_cmd_vel_pub_;

		ros::Subscriber odom_sub_;
		nav_msgs::Odometry latest_odom_;

		PathFollower path_follower_;
		double final_pos_tol_;
		double final_vel_tol_;
		double final_rot_tol_;
		double final_angvel_tol_;
		double server_timeout_;

		bool debug_;
		double ros_rate_;

		tf2_ros::Buffer tf_buffer_;
		tf2_ros::TransformListener tf_listener_;

		std::string map_frame_;

		ros::ServiceServer pause_path_server_;

		bool paused_ = false;
		bool last_paused_ = false;

	public:
		PathAction(const std::string &name, const ros::NodeHandle &nh,
				   double final_pos_tol,
				   double final_vel_tol,
				   double final_rot_tol,
				   double final_angvel_tol,
				   double server_timeout,
				   double ros_rate,
				   double time_offset,
				   bool debug,
				   const std::string &map_frame)
			: nh_(nh)
			, as_(nh_, name, boost::bind(&PathAction::executeCB, this, _1), false)
			, action_name_(name)
			, yaw_sub_(nh_.subscribe("/imu/zeroed_imu", 1, &PathAction::yawCallback, this, ros::TransportHints().tcpNoDelay()))
			, orientation_command_pub_(nh_.advertise<pid_velocity_msg::PIDVelocity>("/teleop/velocity_orientation_command", 1))
			, combine_cmd_vel_pub_(nh_.advertise<std_msgs::Bool>("path_follower_pid/pid_enable", 1, true))
			, robot_relative_yaw_pub_(nh_.advertise<std_msgs::Float64>("robot_relative_yaw", 1, true))
			, odom_sub_(nh_.subscribe("/frcrobot_jetson/swerve_drive_controller/odom", 1, &PathAction::odomCallback, this))
			, path_follower_(time_offset)
			, final_pos_tol_(final_pos_tol)
			, final_vel_tol_(final_vel_tol)
			, final_rot_tol_(final_rot_tol)
			, final_angvel_tol_(final_angvel_tol)
			, server_timeout_(server_timeout)
			, debug_(debug)
			, ros_rate_(ros_rate)
			, tf_listener_(tf_buffer_)
			, map_frame_(map_frame)
			, pause_path_server_(nh_.advertiseService("pause_path", &PathAction::pausePath, this))

		{
			std_msgs::Bool bool_msg;
			bool_msg.data = false;
			combine_cmd_vel_pub_.publish(bool_msg);

			as_.start();
		}

		bool pausePath(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
		{
			paused_ = req.data;
			return true;
		}

		void odomCallback(const nav_msgs::Odometry &odom_msg) {
			latest_odom_ = odom_msg;
		}

		void yawCallback(const sensor_msgs::Imu &yaw_msg)
		{
			if (debug_)
			{
				ROS_INFO_STREAM("yawCallback : msg = " << yaw_msg.orientation);
			}
			orientation_ = yaw_msg.orientation;
		}

        // x & y axis are each controlled by a PID node which
		// tries to close the error between the current odom / pose
		// position and the desired location for each time step
		// along the path.  This is a helper to create a map
		// of state names to stucts which hold the 
		// topics for each PID node
		bool addAxis(const AlignActionAxisConfig &axis_config)
		{
			axis_states_.emplace(std::make_pair(axis_config.name_,
												AlignActionAxisStatePositionVelocity(nh_,
														axis_config.enable_pub_topic_,
														axis_config.command_pub_topic_,
														axis_config.state_pub_topic_)));
			return true;
		}

		void executeCB(const path_follower_msgs::PathGoalConstPtr &goal)
		{
			if (goal->position_path.poses.size() < 1) {
				ROS_ERROR_STREAM("path_follower: no poses were given, please provide path :(");
				return;
			}
			path_follower_msgs::PathFeedback initial_feedback;
			initial_feedback.percent_complete = 0;
			initial_feedback.percent_next_waypoint = 0;
			initial_feedback.current_waypoint = 0;
			as_.publishFeedback(initial_feedback);
			
			bool preempted = false;
			bool timed_out = false;
			bool succeeded = false;

			const size_t num_waypoints = goal->position_path.poses.size();

			std::vector<int> waypointsIdx = goal->waypointsIdx;
			// Spin once to get the most up to date odom and yaw info
			ros::spinOnce();

			try{
				// This gives us the transform from a point in base_link to map.
				// This is correct, although counterintuitive and we don't know why
				// Because it feels like it should be the opposite, but breaks when we do what is "correct".
				map_to_baselink_ = tf_buffer_.lookupTransform(map_frame_, "base_link", ros::Time(0));
			}
			catch (tf2::TransformException &ex) {
				ROS_ERROR_STREAM("path_follower: no map to base link transform found! (!!)" << ex.what());
			}
			// ROS_INFO_STREAM("Diff between tf and now " << ros::Time::now() - map_to_baselink_.header.stamp); 

			const double final_pos_tol = (goal->final_pos_tol > 0) ? goal->final_pos_tol : final_pos_tol_;
			const double final_vel_tol = (goal->final_vel_tol > 0) ? goal->final_vel_tol : final_vel_tol_;
			const double final_rot_tol = (goal->final_rot_tol > 0) ? goal->final_rot_tol : final_rot_tol_;
			const double final_angvel_tol = (goal->final_angvel_tol > 0) ? goal->final_angvel_tol : final_angvel_tol_;
			// ROS_INFO_STREAM("Path following with final_pos_tol = " << final_pos_tol << " final_rot_tol = " << final_rot_tol);

			// Since paths are robot-centric, the initial odom value is 0,0,0 for the path.
			// Set this up as a transfrom to apply to each point in the path. This has the
			// effect of changing robot centric coordinates into odom-centric coordinates
			// Since we're using odom-centric values to drive against, this simplifies a
			// lot of the code later.

			//ros::message_operations::Printer< ::geometry_msgs::TransformStamped_<std::allocator<void>> >::stream(std::cout, "", odom_to_base_link_tf);

			const double initial_field_relative_yaw = path_follower_.getYaw(orientation_);
			ROS_INFO_STREAM("==== initial_field_relative_yaw = " << initial_field_relative_yaw);

			const double initial_pose_yaw = path_follower_.getYaw(orientation_);
			ROS_INFO_STREAM("==== initial_pose_yaw = " << initial_pose_yaw);

			// Transform the final point from robot to odom coordinates. Used each iteration to
			// see if we've reached the end point, so do it once here rather than each time through
			// the loop
			geometry_msgs::Pose final_pose_transformed = goal->position_path.poses.back().pose;
			// tf2::doTransform(final_pose_transformed, final_pose_transformed, map_to_base_link_tf);

			const auto starting_tf = map_to_baselink_;

			//debug
			ROS_INFO_STREAM(goal->position_path.poses[num_waypoints - 1].pose.position.x << ", " << goal->position_path.poses[num_waypoints - 1].pose.position.y << ", " << path_follower_.getYaw(goal->position_path.poses[num_waypoints - 1].pose.orientation));

			ROS_INFO_STREAM("Current odom values X = " << map_to_baselink_.transform.translation.x << " Y = " << map_to_baselink_.transform.translation.y << " Rot " << path_follower_.getYaw(orientation_)); 
			for (size_t i = 0; i < num_waypoints - 1; i++) {
				ROS_INFO_STREAM("Untransformed waypoint: X = " << goal->position_path.poses[i].pose.position.x << " Y = " << goal->position_path.poses[i].pose.position.y << " rotation = " << path_follower_.getYaw(goal->position_path.poses[i].pose.orientation));
				// geometry_msgs::Pose temp_pose = goal->position_path.poses[i].pose;
				// geometry_msgs::Pose new_pose;
				// tf2::doTransform(temp_pose, new_pose, map_to_base_link_tf);
				// tf2::Quaternion q;
				// tf2::fromMsg(map_to_base_link_tf.transform.rotation, q);
				// double r, p, y;
				// tf2::Matrix3x3(q).getRPY(r, p, y);
				// ROS_INFO_STREAM("Transforming by the transform x = " << map_to_base_link_tf.transform.translation.x << ", y = "  << map_to_base_link_tf.transform.translation.y << ", z = "  << map_to_base_link_tf.transform.translation.z << ", r = " << r << ", p = " << p << ", y = " << y);
				// ROS_INFO_STREAM("Transformed waypoint: X = " << new_pose.position.x << " Y = " << new_pose.position.y << " rotation = " << path_follower_.getYaw(new_pose.orientation));
			}
			ROS_INFO_STREAM("========End path follower logs ==========");

			ros::Rate r(ros_rate_);

			size_t current_index = 0;

			std_msgs::Bool enable_msg;
			pid_velocity_msg::PIDVelocity command_msg; 
			auto x_axis_it = axis_states_.find("x");
			auto &x_axis = x_axis_it->second;
			auto y_axis_it = axis_states_.find("y");
			auto &y_axis = y_axis_it->second;

			// Align to first waypoint (within tolerance)
			x_axis.setEnable(true);
			x_axis.setCommand(goal->position_path.poses[0].pose.position.x, 0); 
			ROS_WARN_STREAM("Setting inital command to pos:" << goal->position_path.poses[0].pose.position.x << " Velocity: " << goal->velocity_path.poses[0].pose.position.x);
			y_axis.setEnable(true);
			y_axis.setCommand(goal->position_path.poses[0].pose.position.y, 0); 
			ROS_WARN_STREAM("Setting inital command to pos:" << goal->position_path.poses[0].pose.position.x << " Velocity: " << goal->velocity_path.poses[0].pose.position.x);

			while (ros::ok() && !preempted && !timed_out && !succeeded)
			
			{
				if (goal->dont_go_to_start) {
					ROS_INFO_STREAM("path_follower: not going to starting waypoint");
					break;
				}
				ros::spinOnce();
				try{
					// This gives us the transform from a point in base_link to map.
					// This is correct, although counterintuitive and we don't know why
					// Because it feels like it should be the opposite, but breaks when we do what is "correct".
					map_to_baselink_ = tf_buffer_.lookupTransform(map_frame_, "base_link", ros::Time(0));
				}
				catch (tf2::TransformException &ex) {
					ROS_ERROR_STREAM("path_follower: no map to base link transform found! (!!)" << ex.what());
					continue;
				}

				initial_feedback.x_error = std::fabs(map_to_baselink_.transform.translation.x - goal->position_path.poses[0].pose.position.x);
				initial_feedback.y_error = std::fabs(map_to_baselink_.transform.translation.y - goal->position_path.poses[0].pose.position.y);
				initial_feedback.angle_error = std::fabs(path_follower_.getYaw(map_to_baselink_.transform.rotation) - path_follower_.getYaw(goal->position_path.poses[0].pose.orientation));
				as_.publishFeedback(initial_feedback);

				enable_msg.data = true;
				combine_cmd_vel_pub_.publish(enable_msg);
				x_axis.setEnable(true);
				x_axis.setCommand(goal->position_path.poses[0].pose.position.x, 0); 
				y_axis.setEnable(true);
				y_axis.setCommand(goal->position_path.poses[0].pose.position.y, 0);
				x_axis.setState(map_to_baselink_.transform.translation.x);
				y_axis.setState(map_to_baselink_.transform.translation.y);
				const double orientation_state = path_follower_.getYaw(orientation_);

				std_msgs::Float64 yaw_msg;
				yaw_msg.data = orientation_state;
				robot_relative_yaw_pub_.publish(yaw_msg);

				command_msg.position = path_follower_.getYaw(goal->position_path.poses[0].pose.orientation);
				command_msg.velocity = path_follower_.getYaw(goal->velocity_path.poses[0].pose.orientation);
				if (std::isfinite(command_msg.position)) 
				{
					orientation_command_pub_.publish(command_msg);
					#ifdef DEBUG
					ROS_INFO_STREAM("Orientation: " << command_msg.position << " at angular velocity " << command_msg.velocity);
					#endif
				}

				// NOTE: this is enforcing zero velocity at the end of a path
				// If we want to end with a nonzero velocity, need to transform vel to be odom-relative to compare with robot velocity
				if ((fabs(goal->position_path.poses[0].pose.position.x - map_to_baselink_.transform.translation.x) < final_pos_tol) &&
					(fabs(goal->position_path.poses[0].pose.position.y - map_to_baselink_.transform.translation.y) < final_pos_tol) &&
					(fabs(latest_odom_.twist.twist.linear.x) < final_vel_tol) &&
					(fabs(latest_odom_.twist.twist.linear.y) < final_vel_tol) &&
					(fabs(latest_odom_.twist.twist.angular.z) < final_angvel_tol) &&
					(angles::shortest_angular_distance(command_msg.position, orientation_state) < final_rot_tol))
				{
					ROS_INFO_STREAM("path_follower: successfully aligned to initial waypoint");
					break;
				}

				if (as_.isPreemptRequested() || !ros::ok())
				{
					ROS_ERROR_STREAM(action_name_ << ": preempted");
					preempted = true;
					x_axis.setEnable(false);
					y_axis.setEnable(false);
				}
				r.sleep();
			}

			double final_distance = 0;
			// send path to initialize path follower
			if (!path_follower_.loadPath(goal->position_path, goal->velocity_path, final_distance))
			{
				ROS_ERROR_STREAM("Failed to load path");
				preempted = true;
			}

			//in loop, send PID enable commands to rotation, x, y
			double distance_travelled = 0;

			auto start_time = ros::Time::now().toSec();

			auto last_pause_time = start_time;
			// If we pause, we need to offset the start time by the amount we paused for.

			while (ros::ok() && !preempted && !timed_out && !succeeded)
			{
				if (paused_ && !last_paused_) {
					ROS_INFO_STREAM("Path follower paused");
					last_pause_time = ros::Time::now().toSec();
					// Shut off publishing both combined x+y+rotation messages
					// along with the combined cmd_vel output generated from them
					enable_msg.data = false;
					combine_cmd_vel_pub_.publish(enable_msg);

					x_axis.setEnable(false);
					y_axis.setEnable(false);

					ROS_WARN_STREAM("Path follower paused, publishing zero cmd_vel");
					geometry_msgs::Twist zero_msg = geometry_msgs::Twist();
					zero_msg.linear.x = 0.0;
					zero_msg.linear.y = 0.0;
					zero_msg.angular.z = 0.0;
					zero_cmd_vel_pub_.publish(zero_msg);
				}

				if (last_paused_ && !paused_) {
					ROS_INFO_STREAM("Path follower unpaused");
					start_time += ros::Time::now().toSec() - last_pause_time;
					path_follower_.addTimeOffset(ros::Duration(ros::Time::now().toSec() - last_pause_time));
				}

				last_paused_ = paused_;

				if (paused_) {
					ROS_INFO_STREAM_THROTTLE(0.5, "Path follower paused");
					continue;
				}

				auto start_time_ = std::chrono::high_resolution_clock::now();
				path_follower_msgs::PathFeedback feedback; // FIXME: Add velocity (probably?)
				// Spin once to get the most up to date odom and yaw info
				ros::spinOnce();

				try{
					// This gives us the transform from a point in base_link to map.
					// This is correct, although counterintuitive and we don't know why
					// Because it feels like it should be the opposite, but breaks when we do what is "correct".
					map_to_baselink_ = tf_buffer_.lookupTransform(map_frame_, "base_link", ros::Time(0));
				}
				catch (tf2::TransformException &ex) {
					ROS_ERROR_STREAM("path_follower: no map to base link transform found! (!!)" << ex.what());
					continue;
				}
				// must be less than 0.01 100hz 
				// ROS_INFO_STREAM("Diff between tf and now " << ros::Time::now() - map_to_baselink_.header.stamp); 
#ifdef DEBUG
				// This gets the point closest to current time plus lookahead distance
				// on the path. We use this to generate a target for the x,y,orientation
				ROS_INFO_STREAM("----------------------------------------------");
				ROS_INFO_STREAM("current_position = " << map_to_baselink_.transform.translation.x
					<< " " << map_to_baselink_.transform.translation.y
					<< " " << path_follower_.getYaw(orientation_));	// PID controllers.
#endif
				std::optional<PositionVelocity> next_waypoint_opt = path_follower_.run(distance_travelled, current_index); 
				PositionVelocity next_waypoint = *next_waypoint_opt;

				int current_waypoint = waypointsIdx[current_index];
				feedback.current_waypoint = current_waypoint;

				// gets total distance of the path, then finds how far we have traveled so far
				feedback.percent_complete = (distance_travelled != 0) ? (distance_travelled / final_distance) : 0.0;

				// The path's have even spacing so if you find the total amount of path waypoints and then divide by which on you are on
				// The result is pretty close to how far you are along to the next waypoint
				// Can be off by at most by the total number of poses in the generated path / 100
				const auto low = std::lower_bound(waypointsIdx.begin(), waypointsIdx.end(), current_waypoint);
				const auto high = std::upper_bound(waypointsIdx.begin(), waypointsIdx.end(), current_waypoint);
				const auto lowidx = low - waypointsIdx.begin();
				const auto highidx = high - waypointsIdx.begin();
				const auto waypoint_size = highidx - lowidx;
				feedback.percent_next_waypoint = double(current_index - lowidx) / waypoint_size;

				feedback.x_error = std::fabs(map_to_baselink_.transform.translation.x - next_waypoint.position.position.x);
				feedback.y_error = std::fabs(map_to_baselink_.transform.translation.y - next_waypoint.position.position.y);
				feedback.angle_error = std::fabs(path_follower_.getYaw(map_to_baselink_.transform.rotation) - path_follower_.getYaw(next_waypoint.position.orientation));

				as_.publishFeedback(feedback);
#ifdef DEBUG
				ROS_INFO_STREAM("Before transform: next_waypoint = (" << next_waypoint.position.position.x << ", " << next_waypoint.position.position.y << ", " << path_follower_.getYaw(next_waypoint.position.orientation) << ")");
#endif
				// tf2::doTransform(next_waypoint.position, next_waypoint.position, map_to_base_link_tf);
#ifdef DEBUG
				ROS_INFO_STREAM("After transform: next_waypoint = (" << next_waypoint.position.position.x << ", " << next_waypoint.position.position.y << ", " << path_follower_.getYaw(next_waypoint.position.orientation) << ")");
#endif
				enable_msg.data = true;
				combine_cmd_vel_pub_.publish(enable_msg);

				// For each axis of motion, publish the corresponding next
				// waypoint coordinate to each of the PID controllers
				// And also make sure they continue to be enabled
				x_axis.setEnable(true);
				x_axis.setCommand(next_waypoint.position.position.x, next_waypoint.velocity.linear.x); 

				y_axis.setEnable(true);
				y_axis.setCommand(next_waypoint.position.position.y, next_waypoint.velocity.linear.y); 

				command_msg.position = path_follower_.getYaw(next_waypoint.position.orientation) - initial_pose_yaw + initial_field_relative_yaw;
				command_msg.velocity = next_waypoint.velocity.angular.z;
				if (std::isfinite(command_msg.position)) 
				{
					orientation_command_pub_.publish(command_msg);
					#ifdef DEBUG
					ROS_INFO_STREAM("Orientation: " << command_msg.position << " at angular velocity " << command_msg.velocity);
					#endif
				}

				if (as_.isPreemptRequested() || !ros::ok())
				{
					ROS_ERROR_STREAM(action_name_ << ": preempted");
					preempted = true;
				}
				else if (ros::Time::now().toSec() - start_time > server_timeout_)
				{
					ROS_ERROR_STREAM(action_name_ << ": timed out");
					timed_out = true;
				}

				const double orientation_state = path_follower_.getYaw(orientation_);
				//ROS_INFO_STREAM("orientation_state = " << orientation_state);
				
				auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - start_time_).count();
				// ROS_INFO_STREAM("Path follower calculations took " << duration);

				// NOTE: this is enforcing zero velocity at the end of a path
				// If we want to end with a nonzero velocity, need to transform vel to be odom-relative to compare with robot velocity
				// Right now, there's a bug where pf will think it finished immediately when start == end
				if ((fabs(final_pose_transformed.position.x - map_to_baselink_.transform.translation.x) < final_pos_tol) &&
					(fabs(final_pose_transformed.position.y - map_to_baselink_.transform.translation.y) < final_pos_tol) &&
					(angles::shortest_angular_distance(path_follower_.getYaw(final_pose_transformed.orientation), orientation_state) < final_rot_tol) &&
					(fabs(latest_odom_.twist.twist.linear.x) < final_vel_tol) &&
					(fabs(latest_odom_.twist.twist.linear.y) < final_vel_tol) &&
					(fabs(latest_odom_.twist.twist.angular.z) < final_angvel_tol) &&
					(current_index >= (goal->position_path.poses.size() - 2)))
				{
					ROS_INFO_STREAM(action_name_ << ": succeeded");
					ROS_INFO_STREAM("    endpoint_x = " << final_pose_transformed.position.x << ", odom_x = " << map_to_baselink_.transform.translation.x);
					ROS_INFO_STREAM("    endpoint_y = " << final_pose_transformed.position.y << ", odom_y = " << map_to_baselink_.transform.translation.y);
					ROS_INFO_STREAM("    endpoint_rot = " << path_follower_.getYaw(final_pose_transformed.orientation) << ", odom_rot = " << orientation_state);
					ROS_INFO_STREAM("    distance_travelled = " << distance_travelled);
					succeeded = true;
				}
				else if (!preempted && !timed_out)
				{
					#ifdef DEBUG
					ROS_INFO_STREAM("X ERROR: " << fabs(final_pose_transformed.position.x - map_to_baselink_.transform.translation.x) << " tolerance " << final_pos_tol);
					ROS_INFO_STREAM("Y ERROR: " << fabs(final_pose_transformed.position.y - map_to_baselink_.transform.translation.y) << " tolerance " << final_pos_tol);
					ROS_INFO_STREAM("ROT ERROR: " << angles::shortest_angular_distance(path_follower_.getYaw(final_pose_transformed.orientation), orientation_state) << " tolerance " << final_rot_tol);
					ROS_INFO_STREAM("CURRENT INDEX: " << current_index << " goal poses -2: " << goal->position_path.poses.size() - 2);
					#endif
					ros::spinOnce();
					// Pass along the current x, y robot states
					// to the PID controllers for each axis
					// Orient is handled from combined orient
					// node in teleop code
					x_axis.setState(map_to_baselink_.transform.translation.x);
					y_axis.setState(map_to_baselink_.transform.translation.y);

					// x, y PID inputs are odom-relative
					// cmd_vel is base link relative
					// if command = 5.0, current = 4.0 for x
					// cmd_vel is positive (let's say 1)
					// e.g. initial odom orientation is 0, current robot orientation is 90
					// these are both relative to the same frame. so it is -odom orientation.
					// sending positive here because it is inverted in publish_pid_cmd_vel
					// should be y of -1, x of 0
					// cmd_vel_msg.linear.x = x_command * cos(rotate_angle) - y_command * sin(rotate_angle);
					// 0 = 1.0 * cos(theta) - 0.0 * sin(theta)
					// cos(theta) = 0
					// theta = pi/2 or 3pi/2
					// cmd_vel_msg.linear.y = x_command * sin(rotate_angle) + y_command * cos(rotate_angle);
					// -1 = 1.0 * sin(theta) + 0.0 * cos(theta)
					// angle = 3pi/2 aka -90 degrees
					std_msgs::Float64 yaw_msg;
					yaw_msg.data = orientation_state;
					robot_relative_yaw_pub_.publish(yaw_msg);

					r.sleep();
				}
			}
			#ifdef DEBUG
			ROS_INFO_STREAM("    final delta odom_ = " << map_to_baselink_.transform.translation.x - starting_tf.transform.translation.x
					<< ", " << map_to_baselink_.transform.translation.y - starting_tf.transform.translation.y);
			// ROS_INFO_STREAM("    final delta pose_ = " << pose_.pose.position.x - starting_pose.pose.position.x
			// 		<< ", " << pose_.pose.position.y - starting_pose.pose.position.y);
			#endif

			// Shut off publishing both combined x+y+rotation messages
			// along with the combined cmd_vel output generated from them
			enable_msg.data = false;
			combine_cmd_vel_pub_.publish(enable_msg);

			x_axis.setEnable(false);
			y_axis.setEnable(false);

			ROS_WARN_STREAM("Path follower finished, publishing zero cmd_vel");
			geometry_msgs::Twist zero_msg = geometry_msgs::Twist();
			zero_msg.linear.x = 0.0;
			zero_msg.linear.y = 0.0;
			zero_msg.angular.z = 0.0;
			zero_cmd_vel_pub_.publish(zero_msg);
			
			//log result and set actionlib server state appropriately
			path_follower_msgs::PathResult result;

			if (preempted)
			{
				ROS_WARN("%s: Finished - Preempted", action_name_.c_str());
				result.timed_out = false;
				result.success = false;
				as_.setPreempted(result);
			}
			else if (timed_out)
			{
				ROS_WARN("%s: Finished - Timed Out", action_name_.c_str());
				result.timed_out = true;
				result.success = false;
				as_.setSucceeded(result); //timed out is encoded as succeeded b/c actionlib doesn't have a timed out state
			}
			else   //implies succeeded
			{
				ROS_INFO("%s: Finished - Succeeded", action_name_.c_str());
				result.timed_out = false;
				result.success = true;
				as_.setSucceeded(result);
			}

			ROS_INFO_STREAM("Elapsed time driving = " << ros::Time::now().toSec() - start_time);
		}

		// Assorted get/set methods used by dynamic reoconfigure callback code
		void setFinalPosTol(double final_pos_tol)
		{
			final_pos_tol_ = final_pos_tol;
		}

		double getFinalPosTol(void) const
		{
			return final_pos_tol_;
		}

		void setFinalRotTol(double final_rot_tol)
		{
			final_rot_tol_ = final_rot_tol;
		}

		double getFinalRotTol(void) const
		{
			return final_rot_tol_;
		}

		void setServerTimeout(double server_timeout)
		{
			server_timeout_ = server_timeout;
		}

		double getServerTimeout(void) const
		{
			return server_timeout_;
		}

		void setRosRate(double ros_rate)
		{
			ros_rate_ = ros_rate;
		}

		double getRosRate(void) const
		{
			return ros_rate_;
		}

		void setDebug(bool debug)
		{
			debug_ = debug;
		}

		double getDebug(void) const
		{
			return debug_;
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_follower_server");
	ros::NodeHandle nh;

	double final_pos_tol = 0.01;
	double final_vel_tol = 0.1;
	double final_rot_tol = 0.01;
	double final_angvel_tol = 0.1;
	double server_timeout = 15.0;
	double ros_rate = 20;
	double time_offset = 0;
	bool use_pose_for_odom = false;
	bool debug = false;
	std::string map_frame;

	nh.getParam("/path_follower/path_follower/final_pos_tol", final_pos_tol);
	nh.getParam("/path_follower/path_follower/final_vel_tol", final_vel_tol);
	nh.getParam("/path_follower/path_follower/final_rot_tol", final_rot_tol);
	nh.getParam("/path_follower/path_follower/final_angvel_tol", final_angvel_tol);
	nh.getParam("/path_follower/path_follower/server_timeout", server_timeout);
	nh.getParam("/path_follower/path_follower/ros_rate", ros_rate);
	nh.getParam("/path_follower/path_follower/time_offset", time_offset);
	nh.getParam("/path_follower/path_follower/use_pose_for_odom", use_pose_for_odom);
	nh.getParam("/path_follower/path_follower/debug", debug);
	nh.getParam("/path_follower/path_follower/map_frame", map_frame);

	PathAction path_action_server("path_follower_server", nh,
								  final_pos_tol,
								  final_vel_tol,
								  final_rot_tol,
								  final_angvel_tol,
								  server_timeout,
								  ros_rate,
								  time_offset,
								  debug,
								  map_frame);

	// Set up structs to access PID nodes tracking x and y position
	AlignActionAxisConfig x_axis("x", "x_position_pid/pid_enable", "x_position_pid/x_cmd_pub", "x_position_pid/x_state_pub", "x_position_pid/pid_debug", "x_timeout_param", "x_error_threshold_param");
	AlignActionAxisConfig y_axis("y", "y_position_pid/pid_enable", "y_position_pid/y_cmd_pub", "y_position_pid/y_state_pub", "y_position_pid/pid_debug", "y_timeout_param", "y_error_threshold_param");
	
	ros::Publisher zero_cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("path_follower/swerve_drive_controller/cmd_vel", 1);

	if (!path_action_server.addAxis(x_axis))
	{
		ROS_ERROR_STREAM("Error adding x_axis to path_action_server.");
		return -1;
	}
	if (!path_action_server.addAxis(y_axis))
	{
		ROS_ERROR_STREAM("Error adding y_axis to path_action_server.");
		return -1;
	}

	bool dynamic_reconfigure = true;
	nh.getParam("/path_follower/path_follower/dynamic_reconfigure", dynamic_reconfigure);
	if (dynamic_reconfigure)
	{
		ddynamic_reconfigure::DDynamicReconfigure ddr;
		ddr.registerVariable<double>
			("final_pos_tol",
			boost::bind(&PathAction::getFinalPosTol, &path_action_server),
			boost::bind(&PathAction::setFinalPosTol, &path_action_server, _1),
			"linear tolerance for hitting final waypoint", 0, 1);
		ddr.registerVariable<double>
			("final_rot_tol",
			boost::bind(&PathAction::getFinalRotTol, &path_action_server),
			boost::bind(&PathAction::setFinalRotTol, &path_action_server, _1),
			"rotational tolerance for hitting final waypoint", 0, 1);
		ddr.registerVariable<double>
			("server_timeout",
			boost::bind(&PathAction::getServerTimeout, &path_action_server),
			boost::bind(&PathAction::setServerTimeout, &path_action_server, _1),
			"how long to wait before timing out", 0, 150);
		ddr.registerVariable<int>
			("ros_rate",
			boost::bind(&PathAction::getRosRate, &path_action_server),
			boost::bind(&PathAction::setRosRate, &path_action_server, _1),
			"frequency which the path follower loop updates pose -> PID -> motor output callbacks", 0, 200);
		ddr.registerVariable<bool>
			("debug",
			boost::bind(&PathAction::getDebug, &path_action_server),
			boost::bind(&PathAction::setDebug, &path_action_server, _1),
			"enable debug messaged");
		ddr.publishServicesTopics();
	}

	ros::spin();

	return 0;
}
