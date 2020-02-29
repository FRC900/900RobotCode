#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <base_trajectory/GenerateSpline.h>
#include <path_follower/PathAction.h>
#include <path_follower/PathGoal.h>
#include <path_follower/axis_state.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <path_follower/path_follower.h>
#include <geometry_msgs/Quaternion.h>

class PathAction
{
	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<path_follower::PathAction> as_;
		std::string action_name_;

		ros::ServiceClient spline_gen_cli_;

		ros::Subscriber odom_sub_;
		nav_msgs::Odometry odom_;
		ros::Subscriber yaw_sub_;
		geometry_msgs::Quaternion orientation_;

		std::map<std::string, AlignActionAxisState> axis_states_;
		ros::Publisher combine_cmd_vel_pub_;

		std::shared_ptr<PathFollower> path_follower_;
		double lookahead_distance_;
		double final_pos_tol_;
		double server_timeout_;
		double start_point_radius_;
		std::string odom_topic_;

		bool debug_;
		double start_time_;
		int ros_rate_;

	public:
		PathAction(const std::string &name, ros::NodeHandle nh,
				   double lookahead_distance,
				   double final_pos_tol,
				   double server_timeout,
				   int ros_rate,
				   double start_point_radius,
				   const std::string &odom_topic)
			: nh_(nh)
			, as_(nh_, name, boost::bind(&PathAction::executeCB, this, _1), false)
			, action_name_(name)
			, debug_(false) // TODO - config item?
		{
			as_.start();

			odom_topic_ = odom_topic;
			lookahead_distance_ = lookahead_distance;
			final_pos_tol_ = final_pos_tol;
			server_timeout_ = server_timeout;
			ros_rate_ = ros_rate;
			start_point_radius_ = start_point_radius;

			std::map<std::string, std::string> service_connection_header;
			service_connection_header["tcp_nodelay"] = "1";

			// TODO - not sure which namespace base_trajectory should go in
			spline_gen_cli_ = nh_.serviceClient<base_trajectory::GenerateSpline>("/path_follower/base_trajectory/spline_gen", false, service_connection_header);

			odom_sub_ = nh_.subscribe(odom_topic_, 1, &PathAction::odomCallback, this);
			// TODO : maybe grab this from the odom topic as well?
			yaw_sub_ = nh_.subscribe("/imu/imu_zeroed", 1, &PathAction::yawCallback, this);

			combine_cmd_vel_pub_ = nh_.advertise<std_msgs::Bool>("path_follower_pid/pid_enable", 1000);

			path_follower_ = std::make_shared<PathFollower>(lookahead_distance_, start_point_radius_);
		}

		void odomCallback(const nav_msgs::Odometry &odom_msg)
		{
			odom_ = odom_msg;
			//odom_.pose.pose.position.y *= -1;
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

		void executeCB(const path_follower::PathGoalConstPtr &goal)
		{
			bool preempted = false;
			bool timed_out = false;
			bool succeeded = false;

			// Generate the waypoints of the spline
			base_trajectory::GenerateSpline spline_gen_srv;
			const size_t point_num = goal->points.size();
			spline_gen_srv.request.points.resize(point_num);
			for (size_t i = 0; i < point_num; i++)
			{
				spline_gen_srv.request.points[i].positions.resize(3);
				spline_gen_srv.request.points[i].positions[0] = goal->points[i].x;
				spline_gen_srv.request.points[i].positions[1] = goal->points[i].y;
				spline_gen_srv.request.points[i].positions[2] = goal->points[i].z;
			}
			if (!spline_gen_cli_.call(spline_gen_srv))
			{
				ROS_ERROR_STREAM("Can't call spline gen service in path_follower_server");

			}
			size_t num_waypoints = spline_gen_srv.response.path.poses.size();

			//debug
			for (size_t i = 0; i < spline_gen_srv.response.end_points.size(); i++)
			{
				ROS_INFO_STREAM("end point at " << i << " is " << spline_gen_srv.response.end_points[i]);
			}

			//replacement for actual transforms, for now
			for (size_t i = 0; i < num_waypoints; i++) //TODO hacky fix
			{
				spline_gen_srv.response.path.poses[i].pose.position.x += odom_.pose.pose.position.x;
				spline_gen_srv.response.path.poses[i].pose.position.y += odom_.pose.pose.position.y;
			}

			//debug
			ROS_INFO_STREAM(spline_gen_srv.response.path.poses[num_waypoints - 1].pose.position.x << ", " << spline_gen_srv.response.path.poses[num_waypoints - 1].pose.position.y << ", " << PathFollower::getYaw(spline_gen_srv.response.path.poses[num_waypoints - 1].pose.orientation));

			ros::Rate r(ros_rate_);

			// send path to pure pursuit
			if (!path_follower_->loadPath(spline_gen_srv.response.path))
			{
				ROS_ERROR_STREAM("Failed to load path");
				preempted = true;
			}

			//in loop, send PID enable commands to rotation, x, y
			double distance_travelled = 0;
			start_time_ = ros::Time::now().toSec();
			while (ros::ok() && !preempted && !timed_out && !succeeded)
			{

				// odom_.pose.pose.orientation = orientation_;
				geometry_msgs::Pose next_waypoint = path_follower_->run(odom_, distance_travelled);

				// TODO - think about what the target point and axis are
				// We want to end up driving to a point on the path some
				// distance ahead of where we currently are
				// Since the segments are each straight lines, should be fairly
				// simple to advance some distance to find the target waypoint
				// Need to worry about coordinate frames, since the robot will
				// potentially be rotated such that it's x&y don't correspond
				// to the path x&y coordinate axes
				std_msgs::Bool enable_msg;
				enable_msg.data = true;
				std_msgs::Float64 command_msg;
				std_msgs::Float64 state_msg;

				combine_cmd_vel_pub_.publish(enable_msg);

				auto x_axis_it = axis_states_.find("x");
				auto &x_axis = x_axis_it->second;
				x_axis.enable_pub_.publish(enable_msg);
				command_msg.data = next_waypoint.position.x;
				x_axis.command_pub_.publish(command_msg);
				state_msg.data = odom_.pose.pose.position.x;
				x_axis.state_pub_.publish(state_msg);

				auto y_axis_it = axis_states_.find("y");
				auto &y_axis = y_axis_it->second;
				y_axis.enable_pub_.publish(enable_msg);
				command_msg.data = next_waypoint.position.y;
				y_axis.command_pub_.publish(command_msg);
				state_msg.data = odom_.pose.pose.position.y;
				y_axis.state_pub_.publish(state_msg);

				auto z_axis_it = axis_states_.find("z");
				auto &z_axis = z_axis_it->second;
				z_axis.enable_pub_.publish(enable_msg);

				command_msg.data = PathFollower::getYaw(next_waypoint.orientation);
				z_axis.command_pub_.publish(command_msg);
				// state_msg.data = PathFollower::getYaw(orientation_);
				state_msg.data = PathFollower::getYaw(odom_.pose.pose.orientation);
				z_axis.state_pub_.publish(state_msg);

				if (as_.isPreemptRequested() || !ros::ok())
				{
					ROS_ERROR_STREAM(action_name_ << ": preempted");
					preempted = true;
				}
				else if ((fabs(spline_gen_srv.response.path.poses[num_waypoints - 1].pose.position.x - odom_.pose.pose.position.x) < final_pos_tol_) &&
						 (fabs(spline_gen_srv.response.path.poses[num_waypoints - 1].pose.position.y - odom_.pose.pose.position.y) < final_pos_tol_))
				{
					ROS_INFO_STREAM(action_name_ << ": succeeded");
					ROS_INFO_STREAM("endpoint_x = " << spline_gen_srv.response.path.poses[num_waypoints - 1].pose.position.x << ", odom_x = " << odom_.pose.pose.position.x);
					succeeded = true;
				}
				else if (ros::Time::now().toSec() - start_time_ > server_timeout_)
				{
					ROS_ERROR_STREAM(action_name_ << ": timed out");
					timed_out = true;
				}

				ros::spinOnce();
				r.sleep();
			}

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
			path_follower::PathResult result;

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
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_follower_server");
	ros::NodeHandle nh;

	double lookahead_distance = 0.1;
	double final_pos_tol = 0.01;
	double server_timeout = 5.0;
	int ros_rate = 20;
	double start_point_radius = 0.05;
	std::string odom_topic = "/frcrobot_jetson/swerve_drive_controller/odom";
	nh.getParam("/path_follower/path_follower/lookahead_distance", lookahead_distance);
	nh.getParam("/path_follower/path_follower/final_pos_tol", final_pos_tol);
	nh.getParam("/path_follower/path_follower/server_timeout", server_timeout);
	nh.getParam("/path_follower/path_follower/ros_rate", ros_rate);
	nh.getParam("/path_follower/path_follower/start_point_radius", start_point_radius);
	nh.getParam("/path_follower/path_follower/odom_topic", odom_topic);

	PathAction path_action_server("path_follower_server", nh,
								  lookahead_distance,
								  final_pos_tol,
								  server_timeout,
								  ros_rate,
								  start_point_radius,
								  odom_topic);

	AlignActionAxisConfig x_axis("x", "x_position_pid/pid_enable", "x_position_pid/x_cmd_pub", "x_position_pid/x_state_pub", "x_position_pid/pid_debug", "x_timeout_param", "x_error_threshold_param");
	AlignActionAxisConfig y_axis("y", "y_position_pid/pid_enable", "y_position_pid/y_cmd_pub", "y_position_pid/y_state_pub", "y_position_pid/pid_debug", "y_timeout_param", "y_error_threshold_param");
	AlignActionAxisConfig z_axis("z", "orient_pid/pid_enable", "orient_pid/orient_cmd_pub", "orient_pid/orient_state", "orient_pid/pid_debug", "z_timeout_param", "z_error_threshold_param");

	if (!path_action_server.addAxis(x_axis))
	{
		ROS_ERROR_STREAM("Error adding x_axis to path_action_server.");
		return false;
	}
	if (!path_action_server.addAxis(y_axis))
	{
		ROS_ERROR_STREAM("Error adding y_axis to path_action_server.");
		return false;
	}
	if (!path_action_server.addAxis(z_axis))
	{
		ROS_ERROR_STREAM("Error adding z_axis to path_action_server.");
		return false;
	}

	ros::spin();

	return 0;
}
