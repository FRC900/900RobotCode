#include "spline_util/spline_util.h"

template <class T, size_t Order, size_t Joints>
bool initSpline(Trajectory<T, Order> &trajectory, const std::vector<InitSplinePoints<T, Joints>> &points)
{
#if 0
	// Code to generate test code for test_sample_trajectory
	ROS_INFO_STREAM(" ---- initTrajectory : points ---- ");
	ROS_INFO_STREAM("std::vector<InitSplinePoints<T, " << Joints << ">> points;");
	ROS_INFO_STREAM("InitSplinePoints<T, " << Joints << "> isp;");
	for (size_t i = 0; i < points.size(); i++)
	{
		for (size_t j = 0; j < Joints; j++)
		{
			ROS_INFO_STREAM("isp.state[" << j << "].position = " << points[i].state[j].position << ";");
			ROS_INFO_STREAM("isp.state[" << j << "].velocity = " << points[i].state[j].velocity << ";");
			ROS_INFO_STREAM("isp.state[" << j << "].acceleration = " << points[i].state[j].acceleration << ";");
		}
		ROS_INFO_STREAM("isp.time_from_start = " << points[i].time_from_start << ";");
		ROS_INFO_STREAM("points.push_back(isp);");
	}

	ROS_INFO_STREAM(" ---- initTrajectory : points ---- ");
#endif

	trajectory.resize(Joints);

	for (size_t joint_id = 0; joint_id < Joints; joint_id++)
	{
		trajectory[joint_id].clear();
		//TrajectoryPerJoint<T, Order> result_traj_per_joint; // Currently empty
		auto it = points.cbegin();
		while (std::distance(it, points.cend()) >= 2)
		{
			const auto next_it = it + 1;
			// Segment actually creates the spline segment between points it and next_it
			trajectory[joint_id].emplace_back(Segment<T, Order>(it->time_from_start, it->state[joint_id],
																next_it->time_from_start, next_it->state[joint_id]));
			++it;
		}
		if (trajectory[joint_id].size() == 0)
		{
			ROS_INFO_STREAM("Empty trajectory for joint " << joint_id);
			trajectory.clear();
			return false;
		}
	}

	return true;
}

template bool initSpline(Trajectory<double, ARCLENGTH_TRAJECTORY_ORDER> &trajectory, const std::vector<InitSplinePoints<double, 1>> &points);
template bool initSpline(Trajectory<double, 5> &trajectory, const std::vector<InitSplinePoints<double, 3>> &points);

template bool initSpline(Trajectory<float, ARCLENGTH_TRAJECTORY_ORDER> &trajectory, const std::vector<InitSplinePoints<float, 1>> &points);
template bool initSpline(Trajectory<float, 5> &trajectory, const std::vector<InitSplinePoints<float, 3>> &points);
