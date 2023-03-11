#ifndef INC_SPLINE_UTIL_H__
#define INC_SPLINE_UTIL_H__
#include <ros/console.h>
#include <spline_util/simple_spline_segment.h>
#include <array>
#include <vector>

/** Coefficients represent a quintic polynomial like so:
  *
  * <tt> coefs_[0] + coefs_[1]*x + coefs_[2]*x^2 + coefs_[3]*x^3 + coefs_[4]*x^4 + coefs_[5]*x^5 </tt>
  */

// A segment needs a variable type for the data - in our case, floats or doubles
// The order s 1 for linear, 3 for cubic and 5 for quintic splines
using trajectory_interface::SegmentState;
template <class T, size_t Order>
using Segment = trajectory_interface::SimpleSplineSegment<T, Order>;

// Each TrajectoryPerJoint is a vector of segments, each a spline which makes up
// the total path for that dimension (x, y, orientation)
template <class T, size_t Order>
using TrajectoryPerJoint = std::vector<Segment<T, Order>>;

// A vector of those is created to hold x, y and orientation in one struct.
template <class T, size_t Order>
using Trajectory = std::vector<TrajectoryPerJoint<T, Order>>;

constexpr size_t ARCLENGTH_TRAJECTORY_ORDER = 3;
template <class T>
using ArcLengthTrajectory = Trajectory<T, ARCLENGTH_TRAJECTORY_ORDER>;

template <class T>
using XYTTrajectory = Trajectory<T, 5>;

// Create a set of spline segments which smoothly pass through
// each of the points defined in points.
//
// Each entry in points[i] holds an array of position, velocity and acceleration values.
//   points[i].position[j] is the i-th position for the j-th curve
//     For example, position[0] might be x position, position[1] might be y
//     points[0].position[0] is the 1st x waypoint while points[2].position[1] is the 2nd y waypoint
//  points[i] also has a time_from_start entry, indicating the time axis value for each for the positions
//
//  joint names is the same length as each point[i].position array, and holds the name corresponding
//     to that entries' curve
//
//  Returns a trajectory.  This is an array of spline segments, one per jointNames length
//  Each trajectory[i] can be sampled to find the corresponding value of the spline
//  at a given time.

// static typename Segment<T>::State xState;
// typename TrajectoryPerJoint<T>::const_iterator it = sample(trajectory[0], <time in seconds>, state);
// if (midXIt == trajectory[0].cend())
// {
//	ROS_ERROR_STREAM("base_trajectory : could not sample state at time " << <whatever time>);
//	return false;
// }
// auto traj_0_pos = state.position[0];
//
template <class T, size_t Joints>
struct InitSplinePoints
{
	T time_from_start;
	std::array<trajectory_interface::SegmentState<T>, Joints> state;
};
template <class T, size_t Order, size_t Joints>
bool initSpline(Trajectory<T, Order> &trajectory,
				const std::vector<InitSplinePoints<T, Joints>> &points);

#endif