#include "spline_util/spline_util.h"
template <class T>
bool initSpline(Trajectory<T> &trajectory,
				std::vector<std::string> jointNames,
				const std::vector<trajectory_msgs::JointTrajectoryPoint> &points)
{
	trajectory.resize(jointNames.size());

	for (size_t joint_id = 0; joint_id < jointNames.size(); joint_id++)
	{
		std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator it = points.cbegin();
		if (!joint_trajectory_controller::isValid(*it, it->positions.size()))
			throw(std::invalid_argument("Size mismatch in trajectory point position, velocity or acceleration data."));

		TrajectoryPerJoint<T> result_traj_per_joint; // Currently empty

		// Initialize offsets due to wrapping joints to zero
		std::vector<Scalar<T>> position_offset(1, 0.0);

#if 0
		//Initialize segment tolerance per joint
		joint_trajectory_controller::SegmentTolerancesPerJoint<Scalar> tolerances_per_joint;
		tolerances_per_joint.state_tolerance = tolerances.state_tolerance[joint_id];
		tolerances_per_joint.goal_state_tolerance = tolerances.goal_state_tolerance[joint_id];
		tolerances_per_joint.goal_time_tolerance = tolerances.goal_time_tolerance;
#endif

		while (std::distance(it, points.end()) >= 2)
		{
			std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator next_it = it; ++next_it;

			trajectory_msgs::JointTrajectoryPoint it_point_per_joint, next_it_point_per_joint;

			if (!joint_trajectory_controller::isValid(*it, it->positions.size()))
				throw(std::invalid_argument("Size mismatch in trajectory point position, velocity or acceleration data."));
			if (!it->positions.empty())     {it_point_per_joint.positions.resize(1, it->positions[joint_id]);}
			if (!it->velocities.empty())    {it_point_per_joint.velocities.resize(1, it->velocities[joint_id]);}
			if (!it->accelerations.empty()) {it_point_per_joint.accelerations.resize(1, it->accelerations[joint_id]);}
			it_point_per_joint.time_from_start = it->time_from_start;

			if (!joint_trajectory_controller::isValid(*next_it, next_it->positions.size()))
				throw(std::invalid_argument("Size mismatch in trajectory point position, velocity or acceleration data."));
			if (!next_it->positions.empty()) {next_it_point_per_joint.positions.resize(1, next_it->positions[joint_id]);}
			if (!next_it->velocities.empty()) {next_it_point_per_joint.velocities.resize(1, next_it->velocities[joint_id]);}
			if (!next_it->accelerations.empty()) {next_it_point_per_joint.accelerations.resize(1, next_it->accelerations[joint_id]);}
			next_it_point_per_joint.time_from_start = next_it->time_from_start;

			Segment<T> segment(ros::Time(0), it_point_per_joint, next_it_point_per_joint, position_offset);
#if 0
			segment.setTolerances(tolerances_per_joint);
#endif
			result_traj_per_joint.push_back(segment);
			++it;
		}

		if (result_traj_per_joint.size() > 0)
			trajectory[joint_id] = result_traj_per_joint;
	}

	// If the trajectory for all joints is empty, empty the trajectory vector
	auto trajIter = std::find_if (trajectory.begin(), trajectory.end(), joint_trajectory_controller::isNotEmpty<Trajectory<T>>);
	if (trajIter == trajectory.cend())
	{
		ROS_INFO_STREAM("Clearing traj");
		trajectory.clear();
	}

	return true;
}

template bool initSpline(Trajectory<double> &trajectory,
				std::vector<std::string> jointNames,
				const std::vector<trajectory_msgs::JointTrajectoryPoint> &points);


