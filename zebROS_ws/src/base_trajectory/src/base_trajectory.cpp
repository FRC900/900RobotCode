#include <time.h>
#include <joint_trajectory_controller/init_joint_trajectory.h>
#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <trajectory_msgs/JointTrajectory.h>

typedef joint_trajectory_controller::JointTrajectorySegment<trajectory_interface::QuinticSplineSegment<double>> Segment;

typedef std::vector<Segment> TrajectoryPerJoint;
typedef boost::shared_ptr<TrajectoryPerJoint> TrajectoryPerJointPtr;

typedef std::vector<TrajectoryPerJoint> Trajectory;
typedef boost::shared_ptr<Trajectory> TrajectoryPtr;
typedef trajectory_msgs::JointTrajectory::ConstPtr JointTrajectoryConstPtr;

ros::Duration period;
ros::Publisher pub;

void callback(const JointTrajectoryConstPtr& msg)
{
	ros::Time start = ros::Time::now();
	// Hold current position if trajectory is empty
	if (msg->points.empty())
	{
		ROS_DEBUG("Empty trajectory command, stopping.");
		return;
	}

	// Hard code 3 dimensions for paths to
	// follow - x&y translation and z rotation
	std::vector<std::string> joint_names = {"x_linear_joint", "y_linear_joint", "z_rotation_joint"};
	const size_t n_joints = joint_names.size();
	std::vector<bool> angle_wraparound;

	// Assume the path starts at time 0
	ros::Time next_update_time = ros::Time(0) + period;
	ros::Time next_update_uptime = next_update_time;

	// Set this to false to prevent the code
	// from thinking we're driving rotation joints
	// rather than running linear mition
	for (size_t i = 0; i < joint_names.size(); i++)
		angle_wraparound.push_back(false);

	// Allocate memory to hold an
	// initial trajectory
	Trajectory hold_trajectory;
	typename Segment::State current_joint_state = typename Segment::State(1);
	for (unsigned int i = 0; i < n_joints; ++i)
	{
		current_joint_state.position[0] = 0;
		current_joint_state.velocity[0] = 0;
		Segment hold_segment(0.0, current_joint_state, 0.0, current_joint_state);

		TrajectoryPerJoint joint_segment;
		joint_segment.resize(1, hold_segment);
		hold_trajectory.push_back(joint_segment);
	}

	// This generates a starting trajectory
	// with the robot sitting still at location 0,0,0.
	// It is needed as an initial condition for the
	// robot to connect it to the first waypoint
	// TODO : make the starting position and 
	// velocity a variable passed in to the 
	// path generation request.
	{
		typename Segment::State hold_start_state = typename Segment::State(1);
		typename Segment::State hold_end_state = typename Segment::State(1);

		double stop_trajectory_duration = 0.5;
		const typename Segment::Time start_time  = 0;
		const typename Segment::Time end_time    = stop_trajectory_duration;
		const typename Segment::Time end_time_2x = 2.0 * stop_trajectory_duration;

		// Create segment that goes from current (pos,vel) to (pos,-vel)
		for (size_t i = 0; i < n_joints; ++i)
		{
			hold_start_state.position[0]     = 0.0;
			hold_start_state.velocity[0]     = 0.0;
			hold_start_state.acceleration[0] = 0.0;

			hold_end_state.position[0]       = 0.0;
			hold_end_state.velocity[0]       = -0.0;
			hold_end_state.acceleration[0]   = 0.0;

			hold_trajectory[i].front().init(start_time, hold_start_state, end_time_2x, hold_end_state);

			// Sample segment at its midpoint, that should have zero velocity
			hold_trajectory[i].front().sample(end_time, hold_end_state);

			// Now create segment that goes from current state to one with zero end velocity
			hold_trajectory[i].front().init(start_time, hold_start_state, end_time, hold_end_state);
		}
	}

	ros::Time init = ros::Time::now();
	std::cout << "Init time = " << (init - start).toSec() << std::endl;

	// Set basic options for the trajectory
	// generation controller
	joint_trajectory_controller::InitJointTrajectoryOptions<Trajectory> options;
	options.other_time_base           = &next_update_uptime;
	options.current_trajectory        = &hold_trajectory;
	options.joint_names               = &joint_names;
	options.angle_wraparound          = &angle_wraparound;
	options.rt_goal_handle            = NULL;
	options.default_tolerances        = NULL;
	options.allow_partial_joints_goal = true;

	// Actually generate the new trajectory
	// This will create spline coefficents for
	// each of the x,y,z paths
	// TODO : take input from a service rather
	// than a topic
	Trajectory trajectory;
	try
	{
		trajectory = joint_trajectory_controller::initJointTrajectory<Trajectory>(*msg, next_update_time, options);
		if (trajectory.empty())
		{
			ROS_WARN("Not publishing empty trajectory");
			return;
		}
	}
	catch(const std::invalid_argument& ex)
	{
		ROS_ERROR_STREAM(ex.what());
		return;
	}
	catch(...)
	{
		ROS_ERROR("Unexpected exception caught when initializing trajectory from ROS message data.");
		return;
	}

	ros::Time gen = ros::Time::now();
	std::cout << "Gen time = " << (gen - init).toSec() << std::endl;

	// Generate and publish output message
	// TODO : make this the results of the 
	// service
	trajectory_msgs::JointTrajectory out_msg;
	out_msg.header.stamp = ros::Time::now();

	// Add names of the joints to the message
	for (auto it = joint_names.cbegin(); it != joint_names.cend(); ++it)
		out_msg.joint_names.push_back(*it);

	// Figure out how many points are going to be 
	// calculated along the  path. 
	// TODO :should be a simple divide, worry about
	// boundary conditions
	size_t point_count = 0;
	const ros::Duration end_time = msg->points.back().time_from_start;
	for (ros::Duration now(0); now <= end_time; now += period)
		point_count += 1;

	// Resize the message to hold that many points
	out_msg.points.resize(point_count);

	// Loop through and generate 1 point per timestep
	// each point has a pos/vel/acc data for each of the
	// three x,y,z paths
	// TODO : this takes the majority of the runtime
	//        it is also a prime target for multi-threading
	//        since each point is generated independently of
	//        the last.  
	point_count = 0;
	for (ros::Duration now(0); now <= end_time; now += period, point_count += 1)
	{
		out_msg.points[point_count].time_from_start = now;
		for (size_t i = 0; i < joint_names.size(); ++i)
		{
			typename Segment::State desired_state;
			typename TrajectoryPerJoint::const_iterator segment_it = sample(trajectory[i], now.toSec(), desired_state);
			if (trajectory[i].end() == segment_it)
			{
				// Non-realtime safe, but should never happen under normal operation
				ROS_ERROR("Unexpected error: No trajectory defined at current time. Please contact the package maintainer.");
				return;
			}
			out_msg.points[point_count].positions.push_back(desired_state.position[0]);
			out_msg.points[point_count].velocities.push_back(desired_state.velocity[0]);
			out_msg.points[point_count].accelerations.push_back(desired_state.acceleration[0]);
		}
	}
	pub.publish(out_msg);

	ros::Time pub_time = ros::Time::now();
	std::cout << "Pub time = " << (pub_time - gen).toSec() << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "base_trajectory");
	ros::NodeHandle nh;
	double loop_hz;

	nh.param<double>("loop_hz", loop_hz, 50.);

	period = ros::Duration(1./loop_hz);

	ros::Subscriber trajectory_command_sub_ = nh.subscribe("command", 1, callback);

	pub = nh.advertise<trajectory_msgs::JointTrajectory>("/frcrobot/swerve_drive_controller/trajectory_points", 10);

	ros::spin();
}
