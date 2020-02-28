#pragma once
// Let's break C++!
// Need to access a private variable from quintic_spline_segment
// by using a derived class. What could possibly go wrong?
#define private protected
#include <trajectory_interface/quintic_spline_segment.h>
#undef private
#include <joint_trajectory_controller/init_joint_trajectory.h>
#include <joint_trajectory_controller/joint_trajectory_segment.h>

// Some template / polymorphism magic to add a getCoefs()
// method to the spline type used by the rest of the code
namespace trajectory_interface
{
template<class ScalarType>
class MyQuinticSplineSegment: public QuinticSplineSegment<ScalarType>
{
	public:
		std::vector<typename QuinticSplineSegment<ScalarType>::SplineCoefficients> getCoefs(void) const
		{
			return this->coefs_;
		}
};
}
/** Coefficients represent a quintic polynomial like so:
  *
  * <tt> coefs_[0] + coefs_[1]*x + coefs_[2]*x^2 + coefs_[3]*x^3 + coefs_[4]*x^4 + coefs_[5]*x^5 </tt>
  */
typedef std::vector<std::array<double, 6>> SplineCoefs;

// Define typedefs - we're generating Quinitc (5-th order polynomial) splines
// which have doubles as their datatype
template <class T>
using Segment = joint_trajectory_controller::JointTrajectorySegment<trajectory_interface::MyQuinticSplineSegment<T>>;

// Each TrajectoryPerJoint is a vector of segments, each a spline which makes up
// the total path for that dimension (x, y, orientation)
template <class T>
using TrajectoryPerJoint = std::vector<Segment<T>>;

template <class T>
using Scalar = typename Segment<T>::Scalar;

// A vector of those is created to hold x, y and orientation in one struct.
template <class T>
using Trajectory = std::vector<TrajectoryPerJoint<T>>;

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
// double traj_0_pos = state.position[0];
//
template <class T>
bool initSpline(Trajectory<T> &trajectory,
				std::vector<std::string> jointNames,
				const std::vector<trajectory_msgs::JointTrajectoryPoint> &points);

