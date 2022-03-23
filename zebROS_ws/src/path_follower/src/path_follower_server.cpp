#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <path_follower_msgs/PathAction.h>
#include <path_follower_msgs/PathGoal.h>
#include <path_follower/axis_state.h>
#include <path_follower/path_follower.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

class PathAction
{
	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<path_follower_msgs::PathAction> as_;
		std::string action_name_;

		ros::Subscriber odom_sub_;
		nav_msgs::Odometry odom_;
		ros::Subscriber pose_sub_;
		geometry_msgs::PoseStamped pose_;
		ros::Subscriber yaw_sub_;
		geometry_msgs::Quaternion orientation_;

		std::map<std::string, AlignActionAxisState> axis_states_;
		ros::Publisher combine_cmd_vel_pub_;

		PathFollower path_follower_;
		double final_pos_tol_;
		double final_rot_tol_;
		double server_timeout_;

		bool debug_;
		int ros_rate_;

		// If true, use odom for x and y position and a separate yaw topic
		// (from an IMU, perhaps) for the orientation.
		// If false, use odom for x, y and orientation.
		bool use_odom_orientation_;

		// If true, use the subscribed pose topic for odom rather than the odom subscriber
		bool use_pose_for_odom_;

	public:
		PathAction(const std::string &name, const ros::NodeHandle &nh,
				   double final_pos_tol,
				   double final_rot_tol,
				   double server_timeout,
				   int ros_rate,
				   const std::string &odom_topic,
				   const std::string &pose_topic,
				   bool use_odom_orientation,
				   bool use_pose_for_odom,
				   double time_offset)
			: nh_(nh)
			, as_(nh_, name, boost::bind(&PathAction::executeCB, this, _1), false)
			, action_name_(name)
			, path_follower_(time_offset)
			, final_pos_tol_(final_pos_tol)
			, final_rot_tol_(final_rot_tol)
			, server_timeout_(server_timeout)
			, debug_(true) // TODO - config item?
			, ros_rate_(ros_rate)
			, use_odom_orientation_(use_odom_orientation)
			, use_pose_for_odom_(use_pose_for_odom)
		{

			// TODO - not sure which namespace base_trajectory should go in
			odom_sub_ = nh_.subscribe(odom_topic, 1, &PathAction::odomCallback, this);
			if (!use_odom_orientation_)
			{
				yaw_sub_ = nh_.subscribe("/imu/zeroed_imu", 1, &PathAction::yawCallback, this);
			}
			pose_sub_ = nh_.subscribe(pose_topic, 1, &PathAction::poseCallback, this);

			combine_cmd_vel_pub_ = nh_.advertise<std_msgs::Bool>("path_follower_pid/pid_enable", 1, true);
			std_msgs::Bool bool_msg;
			bool_msg.data = false;
			combine_cmd_vel_pub_.publish(bool_msg);

			as_.start();
		}

		void odomCallback(const nav_msgs::Odometry &odom_msg)
		{
			//ROS_INFO_STREAM("odomCallback : msg = " << odom_msg);
			if (!use_pose_for_odom_)
				odom_ = odom_msg;
			//odom_.pose.pose.position.y *= -1;
		}

		void poseCallback(const geometry_msgs::PoseStamped &pose_msg)
		{
			//ROS_INFO_STREAM("poseCallback : msg = " << pose_msg);
			pose_ = pose_msg;
#if 0
			pose_.pose.position.x *= -1; // TODO - the camera is mounted facing backwards
			pose_.pose.position.y *= -1; // make this better - pass in the source frame and do a tf
#endif
			if (use_pose_for_odom_)
			{
				odom_.header = pose_msg.header;
				odom_.pose.pose = pose_msg.pose;
			}
		}

		void yawCallback(const sensor_msgs::Imu &yaw_msg)
		{
			orientation_ = yaw_msg.orientation;
		}

		bool addAxis(const AlignActionAxisConfig &axis_config)
		{
			// TODO - give defaults so these aren't random values if getParam fails
			double timeout = 10;
			if (!nh_.getParam(axis_config.timeout_param_, timeout))
			{
				ROS_ERROR_STREAM("Could not read param "
								 << axis_config.timeout_param_
								 << " in align_server");
				//return false;
			}
			double error_threshold = 1; //TODO this is not being used
			if (!nh_.getParam(axis_config.error_threshold_param_, error_threshold))
			{
				ROS_ERROR_STREAM("Could not read param "
								 << axis_config.error_threshold_param_
								 << " in align_server");
				//return false;
			}

			axis_states_.emplace(std::make_pair(axis_config.name_,
												AlignActionAxisState(axis_config.name_,
														nh_,
														axis_config.enable_pub_topic_,
														axis_config.command_pub_topic_,
														axis_config.state_pub_topic_,
														axis_config.error_sub_topic_,
														boost::bind(&PathAction::error_term_cb, this, _1, axis_config.name_),
														timeout,
														error_threshold)));
			return true;
		}

		// Callback for error term from PID node.  Compares error
		// reported from PID vs. threshhold for the given axis and
		// sets both the saved error as well as the aligned flag for that axis
		void error_term_cb(const std_msgs::Float64MultiArrayConstPtr &msg, const std::string &name)
		{
			auto axis_it = axis_states_.find(name);
			if (axis_it == axis_states_.end())
			{
				ROS_ERROR_STREAM("Could not find align axis " << name << " in error_term_cb");
				return;
			}
			auto &axis = axis_it->second;
			//Check if error less than threshold
			axis.aligned_ = fabs(msg->data[0]) < axis.error_threshold_;
			axis.error_ = msg->data[0];
			if (debug_)
				ROS_WARN_STREAM_THROTTLE(1, name << " error: " << axis.error_ << " aligned: " << axis.aligned_);
		}

		void executeCB(const path_follower_msgs::PathGoalConstPtr &goal)
		{
			bool preempted = false;
			bool timed_out = false;
			bool succeeded = false;

			const size_t num_waypoints = goal->path.poses.size();

			// Spin once to get the most up to date odom and yaw info
			ros::spinOnce();

			// Since paths are robot-centric, the initial odom value is 0,0,0 for the path.
			// Set this up as a transfrom to apply to each point in the path. This has the
			// effect of changing robot centric coordinates into odom-centric coordinates
			// Since we're using odom-centric values to drive against, this simplifies a
			// lot of the code later.
			geometry_msgs::TransformStamped odom_to_base_link_tf;
			odom_to_base_link_tf.transform.translation.x = odom_.pose.pose.position.x;
			odom_to_base_link_tf.transform.translation.y = odom_.pose.pose.position.y;
			odom_to_base_link_tf.transform.translation.z = 0;
			if (use_odom_orientation_)
			{
				odom_to_base_link_tf.transform.rotation = odom_.pose.pose.orientation;
			}
			else
			{
				odom_to_base_link_tf.transform.rotation = orientation_;
			}
			//ros::message_operations::Printer< ::geometry_msgs::TransformStamped_<std::allocator<void>> >::stream(std::cout, "", odom_to_base_link_tf);

			// Transform the final point from robot to odom coordinates. Used each iteration to
			// see if we've reached the end point, so do it once here rather than each time through
			// the loop
			geometry_msgs::Pose final_pose_transformed = goal->path.poses.back().pose;
			tf2::doTransform(final_pose_transformed, final_pose_transformed, odom_to_base_link_tf);

			const auto starting_odom = odom_;
			const auto starting_pose = pose_;

			//debug
			ROS_INFO_STREAM(goal->path.poses[num_waypoints - 1].pose.position.x << ", " << goal->path.poses[num_waypoints - 1].pose.position.y << ", " << path_follower_.getYaw(goal->path.poses[num_waypoints - 1].pose.orientation));

			ros::Rate r(ros_rate_);

			// send path to initialize path follower
			if (!path_follower_.loadPath(goal->path))
			{
				ROS_ERROR_STREAM("Failed to load path");
				preempted = true;
			}

			//in loop, send PID enable commands to rotation, x, y
			double distance_travelled = 0;
			const auto start_time = ros::Time::now().toSec();
			while (ros::ok() && !preempted && !timed_out && !succeeded)
			{
				// Spin once to get the most up to date odom and yaw info
				ros::spinOnce();

				// If using a separate topic for orientation, merge the x+y from odom
				// with the orientiation from that separate topic here
				if (!use_odom_orientation_)
				{
					odom_.pose.pose.orientation = orientation_;
				}
				// This gets the point closest to current time plus lookahead distance
				// on the path. We use this to generate a target for the x,y,orientation
				ROS_INFO_STREAM("----------------------------------------------");
				ROS_INFO_STREAM("current_position = " << odom_.pose.pose.position.x
					<< " " << odom_.pose.pose.position.y
					<< " " << path_follower_.getYaw(odom_.pose.pose.orientation));	// PID controllers.
				geometry_msgs::Pose next_waypoint = path_follower_.run(distance_travelled);

				ROS_INFO_STREAM("Before transform: next_waypoint = (" << next_waypoint.position.x << ", " << next_waypoint.position.y << ", " << path_follower_.getYaw(next_waypoint.orientation) << ")");
				tf2::doTransform(next_waypoint, next_waypoint, odom_to_base_link_tf);
				ROS_INFO_STREAM("After transform: next_waypoint = (" << next_waypoint.position.x << ", " << next_waypoint.position.y << ", " << path_follower_.getYaw(next_waypoint.orientation) << ")");

				std_msgs::Bool enable_msg;
				enable_msg.data = true;
				std_msgs::Float64 command_msg;

				combine_cmd_vel_pub_.publish(enable_msg);

				// For each axis of motion, publish the corresponding next
				// waypoint coordinate to each of the PID controllers
				// And also make sure they continue to be enabled
				// TODO - see if we can enable once at the start of the callback instead?
				auto x_axis_it = axis_states_.find("x");
				auto &x_axis = x_axis_it->second;
				x_axis.enable_pub_.publish(enable_msg);
				command_msg.data = next_waypoint.position.x;
				x_axis.command_pub_.publish(command_msg);

				auto y_axis_it = axis_states_.find("y");
				auto &y_axis = y_axis_it->second;
				y_axis.enable_pub_.publish(enable_msg);
				command_msg.data = next_waypoint.position.y;
				y_axis.command_pub_.publish(command_msg);

				auto z_axis_it = axis_states_.find("z");
				auto &z_axis = z_axis_it->second;
				z_axis.enable_pub_.publish(enable_msg);
				command_msg.data = path_follower_.getYaw(next_waypoint.orientation);
				z_axis.command_pub_.publish(command_msg);

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

				const double orientation_state = path_follower_.getYaw(odom_.pose.pose.orientation);
				//ROS_INFO_STREAM("orientation_state = " << orientation_state);

				if ((fabs(final_pose_transformed.position.x - odom_.pose.pose.position.x) < final_pos_tol_) &&
					(fabs(final_pose_transformed.position.y - odom_.pose.pose.position.y) < final_pos_tol_) &&
					(fabs(path_follower_.getYaw(final_pose_transformed.orientation) - orientation_state) < final_rot_tol_))
				{
					ROS_INFO_STREAM(action_name_ << ": succeeded");
					ROS_INFO_STREAM("    endpoint_x = " << final_pose_transformed.position.x << ", odom_x = " << odom_.pose.pose.position.x);
					ROS_INFO_STREAM("    endpoint_y = " << final_pose_transformed.position.y << ", odom_y = " << odom_.pose.pose.position.y);
					ROS_INFO_STREAM("    endpoint_rot = " << path_follower_.getYaw(final_pose_transformed.orientation) << ", odom_rot = " << orientation_state);
					ROS_INFO_STREAM("    distance_travelled = " << distance_travelled);
					succeeded = true;
				}
				else if (!preempted && !timed_out)
				{
					// Pass along the current x, y and orient robot states
					// to the PID controllers for each axis
					std_msgs::Float64 state_msg;
					state_msg.data = odom_.pose.pose.position.x;
					x_axis.state_pub_.publish(state_msg);

					state_msg.data = odom_.pose.pose.position.y;
					y_axis.state_pub_.publish(state_msg);

					state_msg.data = orientation_state;
					z_axis.state_pub_.publish(state_msg);

					r.sleep();
				}
			}
			ROS_INFO_STREAM("    delta odom_ = " << odom_.pose.pose.position.x - starting_odom.pose.pose.position.x
					<< ", " << odom_.pose.pose.position.y - starting_odom.pose.pose.position.y);
			ROS_INFO_STREAM("    delta pose_ = " << pose_.pose.position.x - starting_pose.pose.position.x
					<< ", " << pose_.pose.position.y - starting_pose.pose.position.y);

			std_msgs::Bool enable_msg;
			enable_msg.data = false;

			combine_cmd_vel_pub_.publish(enable_msg);

			auto x_axis_it = axis_states_.find("x");
			auto &x_axis = x_axis_it->second;
			x_axis.enable_pub_.publish(enable_msg);

			auto y_axis_it = axis_states_.find("y");
			auto &y_axis = y_axis_it->second;
			y_axis.enable_pub_.publish(enable_msg);

			auto z_axis_it = axis_states_.find("z");
			auto &z_axis = z_axis_it->second;
			z_axis.enable_pub_.publish(enable_msg);

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
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_follower_server");
	ros::NodeHandle nh;

	double final_pos_tol = 0.01;
	double final_rot_tol = 0.01;
	double server_timeout = 15.0;
	int ros_rate = 20;
	double time_offset = 0;
	bool use_odom_orientation = false;
	bool use_pose_for_odom = false;

	std::string odom_topic = "/frcrobot_jetson/swerve_drive_controller/odom";
	std::string pose_topic = "/zed_objdetect/pose";
	nh.getParam("/path_follower/path_follower/final_pos_tol", final_pos_tol);
	nh.getParam("/path_follower/path_follower/final_rot_tol", final_rot_tol);
	nh.getParam("/path_follower/path_follower/server_timeout", server_timeout);
	nh.getParam("/path_follower/path_follower/ros_rate", ros_rate);
	nh.getParam("/path_follower/path_follower/odom_topic", odom_topic);
	nh.getParam("/path_follower/path_follower/pose_topic", pose_topic);
	nh.getParam("/path_follower/path_follower/use_odom_orientation", use_odom_orientation);
	nh.getParam("/path_follower/path_follower/time_offset", time_offset);
	nh.getParam("/path_follower/path_follower/use_pose_for_odom", use_pose_for_odom);

	PathAction path_action_server("path_follower_server", nh,
								  final_pos_tol,
								  final_rot_tol,
								  server_timeout,
								  ros_rate,
								  odom_topic,
								  pose_topic,
								  use_odom_orientation,
								  use_pose_for_odom,
								  time_offset);

	AlignActionAxisConfig x_axis("x", "x_position_pid/pid_enable", "x_position_pid/x_cmd_pub", "x_position_pid/x_state_pub", "x_position_pid/pid_debug", "x_timeout_param", "x_error_threshold_param");
	AlignActionAxisConfig y_axis("y", "y_position_pid/pid_enable", "y_position_pid/y_cmd_pub", "y_position_pid/y_state_pub", "y_position_pid/pid_debug", "y_timeout_param", "y_error_threshold_param");
	AlignActionAxisConfig z_axis("z", "orient_pid/pid_enable", "orient_pid/orient_cmd_pub", "orient_pid/orient_state", "orient_pid/pid_debug", "z_timeout_param", "z_error_threshold_param");

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
	if (!path_action_server.addAxis(z_axis))
	{
		ROS_ERROR_STREAM("Error adding z_axis to path_action_server.");
		return -1;
	}

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
		 "how far ahead the path follower looks to pick the next point to drive to", 0, 100);
    ddr.publishServicesTopics();

	ros::spin();

	return 0;
}
