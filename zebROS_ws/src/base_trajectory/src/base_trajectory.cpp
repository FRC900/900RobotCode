#include <time.h>
// Let's break C++!
// Need to access a private variable from quintic_spline_segment
// by using a derived class. What could possibly go wrong?
#define private protected
#include <trajectory_interface/quintic_spline_segment.h>
#undef private
#include <joint_trajectory_controller/init_joint_trajectory.h>
#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <boost/assign.hpp>
#include <base_trajectory/GenerateSpline.h>

#include <angles/angles.h>

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
typedef joint_trajectory_controller::JointTrajectorySegment<trajectory_interface::MyQuinticSplineSegment<double>> Segment;

// Each TrajectoryPerJoint is a vector of segments, each a spline which makes up
// the total path for that dimension (x, y, orientation)
typedef std::vector<Segment> TrajectoryPerJoint;

// A vector of those is created to hold x, y and orientation in one struct.
typedef std::vector<TrajectoryPerJoint> Trajectory;

typedef trajectory_msgs::JointTrajectory::ConstPtr JointTrajectoryConstPtr;

ros::Duration period;

// For printing out matlab code for testing
void printCoefs(std::stringstream &s, const std::string &name, const std::vector<swerve_point_generator::Coefs> &coefs)
{
	for (size_t i = 0; i < coefs.size(); i++)
	{
		s << "p" << name << i << " = [";
		for (size_t j = 0; j < coefs[i].spline.size(); j++)
		{
			s << coefs[i].spline[j];
			if (j < coefs[i].spline.size() - 1)
				s << ", ";
		}
		s << "];" << std::endl;
	}
}

// For printing out matlab code for testing
void printPolyval(std::stringstream &s, const std::string &name, size_t size, const std::vector<double> &end_points)
{
	s << "p" << name << "_y = [";
	for (size_t i = 0; i < size; i++)
	{
		double x_offset;
		if (i == 0)
		{
			x_offset = 0;
		}
		else
		{
			x_offset = end_points[i - 1];
		}
		s << "polyval(p" << name << i << ", x" << i << " - " << x_offset << ")";
		if (i < size - 1)
			s << ", ";
	}
	s << "];" << std::endl;
}

void writeMatlabCode(const base_trajectory::GenerateSpline::Response &msg)
{
	// Generate matlab / octave code for displaying generated splines
	std::stringstream s;
	s << std::endl;
	for (size_t i = 0; i < msg.end_points.size(); i++)
	{
		double range;
		double prev_x;
		if (i == 0)
		{
			range = msg.end_points[0];
			prev_x = 0;
		}
		else
		{
			range = msg.end_points[i] - msg.end_points[i-1];
			prev_x = msg.end_points[i-1];
		}
		s << "x" << i << " = " << prev_x << ":" << range / 100.
		  << ":" << msg.end_points[i] << ";" << std::endl;
	}
	s << "x = [";
	for (size_t i = 0; i < msg.end_points.size(); i++)
	{
		s << "x" << i;
		if (i < msg.end_points.size() - 1)
			s << ", ";
	}
	s << "];" << std::endl;
	s << std::endl;
	printCoefs(s, "x", msg.x_coefs);
	printCoefs(s, "y", msg.y_coefs);
	printCoefs(s, "orient", msg.orient_coefs);
	for (size_t i = 0; i < msg.x_coefs.size(); i++)
	{
		s << "pdx" << i << " = polyder(px" << i << ");" << std::endl;
		s << "pddx" << i << " = polyder(pdx" << i << ");" << std::endl;
		s << "pdddx" << i << " = polyder(pddx" << i << ");" << std::endl;
		s << "pdy" << i << " = polyder(py" << i << ");" << std::endl;
		s << "pddy" << i << " = polyder(pdy" << i << ");" << std::endl;
		s << "pdddy" << i << " = polyder(pddy" << i << ");" << std::endl;
		s << "pdorient" << i << " = polyder(porient" << i << ");" << std::endl;
		s << "pddorient" << i << " = polyder(pdorient" << i << ");" << std::endl;
		s << "pdddorient" << i << " = polyder(pddorient" << i << ");" << std::endl;
	}
	printPolyval(s, "x", msg.x_coefs.size(), msg.end_points);
	printPolyval(s, "dx", msg.x_coefs.size(), msg.end_points);
	printPolyval(s, "ddx", msg.x_coefs.size(), msg.end_points);
	printPolyval(s, "dddx", msg.x_coefs.size(), msg.end_points);
	printPolyval(s, "y", msg.y_coefs.size(), msg.end_points);
	printPolyval(s, "dy", msg.y_coefs.size(), msg.end_points);
	printPolyval(s, "ddy", msg.y_coefs.size(), msg.end_points);
	printPolyval(s, "dddy", msg.y_coefs.size(), msg.end_points);
	printPolyval(s, "orient", msg.orient_coefs.size(), msg.end_points);
	printPolyval(s, "dorient", msg.orient_coefs.size(), msg.end_points);
	printPolyval(s, "ddorient", msg.orient_coefs.size(), msg.end_points);
	printPolyval(s, "dddorient", msg.orient_coefs.size(), msg.end_points);
	s << "subplot(1,1,1)" << std::endl;
	s << "subplot(3,2,1)" << std::endl;
	s << "plot(px_y, py_y)" << std::endl;
	s << "subplot(3,2,3)" << std::endl;
	s << "plot (x,px_y, x, py_y)" << std::endl;
	s << "subplot(3,2,4)" << std::endl;
	s << "plot (x,pdx_y, x, pdy_y)" << std::endl;
	s << "subplot(3,2,5)" << std::endl;
	s << "plot (x,pddx_y, x, pddy_y)" << std::endl;
	s << "subplot(3,2,6)" << std::endl;
	s << "plot (x,pdddx_y, x, pdddy_y)" << std::endl;
	ROS_INFO_STREAM("Matlab_splines : " << s.str());
}

// Find the angle that line p1p2 is pointing at
double getLineAngle(const std::vector<double> &p1, const std::vector<double> &p2)
{
	return angles::normalize_angle_positive(atan2(p2[1] - p1[1], p2[0] - p1[0]));
}

// Optimization parameters - these are deltas added to
// the original guess for the spline generation used
// to improve the overall cost of following the spline
struct OptParams
{
	double posX_;
	double posY_;
	double length_;

	OptParams(void)
		: posX_(0.)
		, posY_(0.)
		, length_(0.)
	{
	}
};

// Helper function for finding 2nd derivative
// (acceleration) term of start and end point
void setFirstLastPointAcceleration(const std::vector<double> &p1, const std::vector<double> &p2,
								   const std::vector<double> &v1, const std::vector<double> &v2,
								   std::vector<double> &accelerations)
{
	// Rename vars to match notation in the paper
	const double Ax  = p1[0];
	const double Ay  = p1[1];
	const double tAx = v1[0];
	const double tAy = v1[1];
	const double Bx  = p2[0];
	const double By  = p2[1];
	const double tBx = v2[0];
	const double tBy = v2[1];

	const double xaccel = 6.0 * Ax + 2.0 * tAx + 4.0 * tBx - 6.0 * Bx;
	const double yaccel = 6.0 * Ay + 2.0 * tAy + 4.0 * tBy - 6.0 * By;

	accelerations.push_back(xaccel); // x
	accelerations.push_back(yaccel); // y
	accelerations.push_back(0.); // theta
}

bool generateSpline(      std::vector<trajectory_msgs::JointTrajectoryPoint> points,
					const std::vector<OptParams> &optParams,
					Trajectory &trajectory)
{
	// Hard code 3 dimensions for paths to
	// follow - x&y translation and z rotation
	std::vector<std::string> joint_names = {"x_linear_joint", "y_linear_joint", "z_rotation_joint"};
	const size_t n_joints = joint_names.size();
	std::vector<bool> angle_wraparound;

	// Assume the path starts at time 0
	ros::Time next_update_time = ros::Time(0);
	ros::Time next_update_uptime = next_update_time;

	// Set this to false to prevent the code
	// from thinking we're driving rotation joints
	// rather than running linear motion
	for (size_t i = 0; i < n_joints; i++)
		angle_wraparound.push_back(false);

	// Allocate memory to hold an initial trajectory
	Trajectory hold_trajectory;
	typename Segment::State current_joint_state = typename Segment::State(1);
	for (size_t i = 0; i < n_joints; ++i)
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
	//
	// TODO : make the starting position and
	// velocity a variable passed in to the
	// path generation request.


	//TODO: WHAT BE THIS BRACKET
	// Adding scope for these var names - they're repeated
	// in other bits of code taken from various functions
	// in other source files.  Easier than renaming them
	{
		typename Segment::State hold_start_state = typename Segment::State(1);
		typename Segment::State hold_end_state = typename Segment::State(1);

		double stop_trajectory_duration = period.toSec() / 2.;
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

	// Auto - generate velocities and accelerations for splines
	// based on a simple heuristic. This is becoming less simple
	// by the moment.
	if (points[0].velocities.size() == 0)
	{
		// Offset x and y by the amount determined by the optimizer
		// Paper says to make the coord system based off the
		// tangent direction (perp & parallel to tangent)
		// We'll see if that makes a difference
		for (size_t i = 1; i < (points.size() - 1); i++)
		{
			points[i].positions[0] += optParams[i].posX_;
			points[i].positions[1] += optParams[i].posY_;
		}

		// Starting with 0 velocity for the initial condition
		// TODO - handle arbitrary starting velocity
		for(size_t i = 0; i < n_joints; i++)
		{
			points[0].velocities.push_back(0.);
		}
		// Velocities of intermediate points are tangent
		// to the bisection of the angle between the incoming
		// and outgoing line segments.
		// Length of the tangent is min distance to either
		// previous or next waypoint
		// See http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf
		// sectopm 4.1.1 and
		// http://ais.informatik.uni-freiburg.de/teaching/ws09/robotics2/projects/mr2-p6-paper.pdf
		// section 3.2 (equation 2 & 3) for details
		//
		double prev_angle = getLineAngle(points[0].positions, points[1].positions);
		double prev_length = hypot(points[1].positions[0] - points[0].positions[0],
								   points[1].positions[1] - points[0].positions[1]);

		for (size_t i = 1; i < (points.size() - 1); i++)
		{
			const auto &mi   = points[i].positions;
			const auto &mip1 = points[i + 1].positions;

			const double curr_angle = getLineAngle(mi, mip1);
			const double angle = angles::normalize_angle_positive(prev_angle + (curr_angle - prev_angle) / 2.0);

			const double curr_length = hypot(mip1[0] - mi[0], mip1[1] - mi[1]);

			// Adding a scaling factor here controls the velocity
			// at the waypoints.  Bigger than 1 ==> curvier path with
			// higher speeds.  Less than 1 ==> tigher turns to stay
			// closer to straight paths.
			const double length = std::min(prev_length, curr_length) * .75 + optParams[i].length_;

			points[i].velocities.push_back(length * cos(angle)); // x
			points[i].velocities.push_back(length * sin(angle)); // y
			points[i].velocities.push_back(0); // theta

#if 0
			ROS_INFO_STREAM("prev_angle " << prev_angle << " prev_length " << prev_length);
			ROS_INFO_STREAM("curr_angle " << curr_angle << " curr_length " << curr_length);
			ROS_INFO_STREAM("angle " << angle << " length " << length);
#endif
			// Update for next step
			prev_angle = curr_angle;
			prev_length = curr_length;
		}

		// End position is also 0 velocity
		// TODO - handle input end velocity
		for(size_t i = 0; i < n_joints; i++)
		{
			points.back().velocities.push_back(0.);
		}

		// Guess for acceleration term is explained in
		// http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf
		// section 4.2.1.  Basically, pretend there are cubic splines
		// (bezier splines in this case) between two points.  These are splines
		// which just connect the two points, and use the velocity at
		// those points calculated above.  This gives reasonable curvature along
		// the path between the two points. Taking the
		// weighted average of the 2nd derivatives of the pretend cubic
		// splines at the given point, and using that to generate
		// a quintic spline does a reasonable job of preserving
		// smoothness while also making the curve continuous.
		//
		// First and last point don't have a prev / next point
		// to connect to, so just grab the 2nd derivitave off
		// the point they are adjacent to
		setFirstLastPointAcceleration(points[0].positions, points[1].positions,
				points[0].velocities, points[1].velocities,
				points[0].accelerations);

		const size_t last = points.size() - 1;
		setFirstLastPointAcceleration(points[last-1].positions, points[last].positions,
				points[last-1].velocities, points[last].velocities,
				points[last].accelerations);
		// For interior points, weight the average of the
		// 2nd derivative of each of the pretend cubic
		// splines and use them as the acceleration for
		// the to-be-generated quintic spline.
		for (size_t i = 1; i < (points.size() - 1); i++)
		{
			const double Ax = points[i-1].positions[0];
			const double Ay = points[i-1].positions[1];
			const double tAx = points[i-1].velocities[0];
			const double tAy = points[i-1].velocities[1];

			const double Bx = points[i].positions[0];
			const double By = points[i].positions[1];
			const double tBx = points[i].velocities[0];
			const double tBy = points[i].velocities[1];

			const double Cx = points[i+1].positions[0];
			const double Cy = points[i+1].positions[1];
			const double tCx = points[i+1].velocities[0];
			const double tCy = points[i+1].velocities[1];

			// L2 distance between A and B
			const double dab = hypot(Bx - Ax, By - Ay);
			// L2 distance between B and C
			const double dbc = hypot(Cx - Bx, Cy - By);

			// Weighting factors
			const double alpha = dbc / (dab + dbc);
			const double beta  = dab / (dab + dbc);

			const double xaccel = alpha * ( 6.0 * Ax + 2.0 * tAx + 4.0 * tBx - 6.0 * Bx) +
								  beta  * (-6.0 * Bx - 4.0 * tBx - 2.0 * tCx + 6.0 * Cx);
			const double yaccel = alpha * ( 6.0 * Ay + 2.0 * tAy + 4.0 * tBy - 6.0 * By) +
								  beta  * (-6.0 * By - 4.0 * tBy - 2.0 * tCy + 6.0 * Cy);

			points[i].accelerations.push_back(xaccel); // x
			points[i].accelerations.push_back(yaccel); // y
			points[i].accelerations.push_back(0.); // theta

#if 0
			ROS_INFO_STREAM("dab = " << dab << " dbc = " << dbc);
			ROS_INFO_STREAM("Ax = " << Ax << " tAx = " << tAx <<
			                " Bx = " << Bx << " tBx = " << tBx <<
			                " Cx = " << Cx << " tCx = " << tCx);
			ROS_INFO_STREAM("Ay = " << Ay << " tAy = " << tAy <<
			                " By = " << By << " tBy = " << tBy <<
			                " Cy = " << Cy << " tCy = " << tCy);
#endif
		}
	}

	for (size_t i = 0; i < points.size(); i++)
	{
		ROS_INFO_STREAM("points[" << i << "]");
		ros::message_operations::Printer<::trajectory_msgs::JointTrajectoryPoint_<std::allocator<void>>>::stream(std::cout, "  ", points[i]);
	}

	// Actually generate the new trajectory
	// This will create spline coefficents for
	// each of the x,y,z paths
	try
	{
		trajectory_msgs::JointTrajectory jtm;
		jtm.joint_names = joint_names;
		jtm.points = points;
		trajectory = joint_trajectory_controller::initJointTrajectory<Trajectory>(jtm, next_update_time, options);
		if (trajectory.empty())
		{
			ROS_WARN("Not publishing empty trajectory");
			return false;
		}
	}
	catch(const std::invalid_argument& ex)
	{
		ROS_ERROR_STREAM(ex.what());
		return false;
	}
	catch(...)
	{
		ROS_ERROR("Unexpected exception caught when initializing trajectory from ROS message data.");
		return false;
	}
	return true;
}

// input should be JointTrajectory[] custom message
// Output wil be array of spline coefficents swerve_point_generator/Coefs[] for x, y, orientation
bool callback(base_trajectory::GenerateSpline::Request &msg,
			  base_trajectory::GenerateSpline::Response &out_msg)
{
	// Hold current position if trajectory is empty
	if (msg.points.empty())
	{
		ROS_DEBUG("Empty trajectory command, stopping.");
		return false;
	}
	// TODO - operate in two modes
	// 1 - if velocity and accelerations are empty, run a full optimization
	// 2 - if both are set, just generate a spline based on them, then
	//     convert that into an optimal path to follow
	for (size_t i = 0; i < msg.points.size(); ++i)
	{
		if (msg.points[i].positions.size() != 3)
		{
			ROS_ERROR_STREAM("Input point " << i << " must have 3 positions (x, y, orientation)");
			return false;
		}
		if (msg.points[i].velocities.size())
		{
			if (msg.points[i].velocities.size() != 3)
			{
				ROS_ERROR_STREAM("Input point " << i << " must have 0 or 3 velocities (x, y, orientation)");
				return false;
			}
			if ((msg.points[i].accelerations.size() != 0) &&
				(msg.points[i].accelerations.size() != 3))
			{
				ROS_ERROR_STREAM("Input point " << i << " must have 0 or 3 accelerations (x, y, orientation)");
				return false;
			}
		}
		else if (msg.points[i].accelerations.size())
		{
			ROS_ERROR_STREAM("Input point " << i << " must have 0 accelerations since there are also 0 velocities for that point)");
			return false;
		}
	}

	std::vector<OptParams> optParams(msg.points.size());
	Trajectory trajectory;
	if (!generateSpline(msg.points, optParams, trajectory))
		return false;

	out_msg.orient_coefs.resize(trajectory[0].size());
	out_msg.x_coefs.resize(trajectory[0].size());
	out_msg.y_coefs.resize(trajectory[0].size());

	const std::vector<std::string> joint_names = {"x_linear_joint", "y_linear_joint", "z_rotation_joint"};
	const size_t n_joints = joint_names.size();
	for (size_t seg = 0; seg < trajectory[0].size(); seg++)
	{
		for (size_t joint = 0; joint < n_joints; joint++)
		{
			ROS_INFO_STREAM("joint = " << joint_names[joint] << " seg = " << seg <<
			                " start_time = " << trajectory[joint][seg].startTime() <<
			                " end_time = " << trajectory[joint][seg].endTime());
			auto coefs = trajectory[joint][seg].getCoefs();

			std::stringstream s;
			s << "coefs ";
			for (size_t i = 0; i < coefs[i].size(); ++i)
				s << coefs[0][i] << " ";
			ROS_INFO_STREAM(s.str());

			std::vector<double> *m;

			if (joint == 0)
				m = &out_msg.x_coefs[seg].spline;
			else if (joint == 1)
				m = &out_msg.y_coefs[seg].spline;
			else if (joint == 2)
				m = &out_msg.orient_coefs[seg].spline;
			else
			{
				ROS_WARN("Unexpected joint number constructing out_msg in base_trajectory");
				continue;
			}

			// Push in reverse order to match expectations
			// of point_gen code?
			m->clear();
			for (int i = coefs[0].size() - 1; i >= 0; i--)
				m->push_back(coefs[0][i]);
		}

		// All splines in a waypoint end at the same time?
		out_msg.end_points.push_back(trajectory[0][seg].endTime());
	}

#if 0
	// Test using middle switch auto spline
	out_msg.x_coefs[1].spline[0] = 6.0649999999999995;
	out_msg.x_coefs[1].spline[1] = -15.510000000000002;
	out_msg.x_coefs[1].spline[2] = 10.205;
	out_msg.x_coefs[1].spline[3] = 0.42;
	out_msg.x_coefs[1].spline[4] = 0.10999999999999999;
	out_msg.x_coefs[1].spline[5] = 0.18;

	out_msg.y_coefs[1].spline[0] = 1.9250000000000025;
	out_msg.y_coefs[1].spline[1] = -5.375;
	out_msg.y_coefs[1].spline[2] = 4.505000000000003;
	out_msg.y_coefs[1].spline[3] = -0.8149999999999998;
	out_msg.y_coefs[1].spline[4] = 2.4299999999999997;
	out_msg.y_coefs[1].spline[5] = 0.45;

	out_msg.orient_coefs[1].spline[0] = 0.0;
	out_msg.orient_coefs[1].spline[1] = 0.0;
	out_msg.orient_coefs[1].spline[2] = 0.0;
	out_msg.orient_coefs[1].spline[3] = 0.0;
	out_msg.orient_coefs[1].spline[4] = 0.0;
	out_msg.orient_coefs[1].spline[5] = -3.14159;
	out_msg.end_points[1] = 1.0; // change me to 4 to match end time in yaml and break point_gen
#endif

	writeMatlabCode(out_msg);
	return true;
}

// From http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
// equation 14
double perpDistance(const std::vector<double> &l1, const std::vector<double> &l2,
		double px, double py)
{
	return fabs((l2[0] - l1[0]) * (l1[1] - py) - (l1[0] - px) * (l2[1] - l1[1])) /
		   hypot(l2[0] - l1[0], l2[1] - l1[1]);

}
// evaluate spline - inputs are spline coeffs (Trajectory), waypoints, dmax, vmax, amax, acentmax
//                   returns time taken to drive spline, cost, possibly waypoints
//
//
struct ArclengthAndTime
{
	ArclengthAndTime(const double arclength, const double time) :
		arclength_(arclength),
		time_(time)
	{
	}
	double arclength_;
	double time_;
};

// Real way to do this is return magnitude of max error term for
// the approximation :
// 1/90 * (periodT/2)^5 * max(f''''(startT), f''''(endT))
// f'''' is weird because is is hypot(fx''''(), fy''''())
// f'''' is the 4th derivative of the spline function =
// 120 * acoeff + 24* bcoeff.
// Complicating matters - start and end can come from different
// spline segments, so we need an aStart, bStart, aend, bEnd
// as well.
// If this is done, then return the error and if it is below
// a threshold, use that value. Otherwise, split segment
// in half and recursively calculate both.
void simpsonsRule(double &estimate, double &error,
				  const double startT, const double endT,
				  const SplineCoefs &xStartCoefs,
				  const SplineCoefs &yStartCoefs,
				  const SplineCoefs &xEndCoefs,
				  const SplineCoefs &yEndCoefs,
				  const double startXdot, const double startYdot,
				  const double midXdot, const double midYdot,
				  const double endXdot, const double endYdot)
{
	const double periodT = endT - startT;
	estimate = periodT / 6.0 * (hypot(startXdot, startYdot) + 4.0 * hypot(midXdot, midYdot) + hypot(endXdot, endYdot));

	const double commonErrorTerm = (1.0/90.0) * pow(periodT / 2.0, 5.0);
	const double startError = hypot(120.0 * xStartCoefs[0][5] * startT + 24.0 * xStartCoefs[0][4],
									120.0 * yStartCoefs[0][5] * startT + 24.0 * yStartCoefs[0][4]);
	const double endError = hypot(120.0 * xEndCoefs[0][5] * endT + 24.0 * xEndCoefs[0][4],
								  120.0 * yEndCoefs[0][5] * endT + 24.0 * yEndCoefs[0][4]);

	error = commonErrorTerm * std::max(startError, endError);
}

// Recursive function to get arc length of x/y path using
// subdivision.
// Basic algorithm breaks the segment into two halves. If the
// estimated length of the two halves is close enough to the length
// estimated for the full length of the segment, assume the algorithm
// has converged and keep that result (use the two halves since they're
// likely more accurate).
// If the sum of the halves are too far off from the total length
// recursively call the function on the first and second halves.
// Returns two things - arcLengthAndTime is a vector of
// <cumulative arclength at that time, time> tuples, hopefully increasing in time
// along with totalLength, which is the total length calculated so far.
bool getPathSegLength(std::vector<ArclengthAndTime> &arclengthAndTime,
		const Trajectory &trajectory,
		double &totalLength,
		TrajectoryPerJoint::const_iterator startXIt,
		TrajectoryPerJoint::const_iterator startYIt,
		TrajectoryPerJoint::const_iterator endXIt,
		TrajectoryPerJoint::const_iterator endYIt,
		const double startTime,
		const double endTime,
		const double startXdot,
		const double startYdot,
		const double endXdot,
		const double endYdot)
{
	constexpr double epsilon = 1.0e-5;
	const double midTime = (startTime + endTime) / 2.0;

	// TODO : Perhaps a check on dT being too small?

	Segment::State xState;
	Segment::State yState;

	// Simpson's rule needs start and end values and
	// also one at the midpoint.
	TrajectoryPerJoint::const_iterator midXIt = sample(trajectory[0], midTime, xState);
	if (midXIt == trajectory[0].cend())
	{
		ROS_ERROR_STREAM("base_trajectory : could not sample mid xState at time " << midTime);
		return false;
	}

	TrajectoryPerJoint::const_iterator midYIt = sample(trajectory[1], midTime, yState);
	if (midYIt == trajectory[1].cend())
	{
		ROS_ERROR_STREAM("base_trajectory : could not sample mid yState at time " << midTime);
		return false;
	}
	const double midXdot = xState.velocity[0];
	const double midYdot = yState.velocity[0];

	double estimate;
	double error;

	simpsonsRule(estimate, error,
				 startTime, endTime,
				 startXIt->getCoefs(), startYIt->getCoefs(),
				 endXIt->getCoefs(), endYIt->getCoefs(),
				 startXdot, startYdot, midXdot, midYdot, endXdot, endYdot);

	// If the error magnitude is less than epsilon,
	// use this approximation for the arclength from
	// start to end
	if (fabs(error) < epsilon)
	{
		totalLength += estimate;
		arclengthAndTime.push_back(ArclengthAndTime(totalLength, endTime));
		return true;
	}

	// Otherwise, split segment in half
	// and recursively calculate the length of each half.
	if (!getPathSegLength(arclengthAndTime, trajectory, totalLength,
						  startXIt, startYIt,
						  midXIt, midYIt,
						  startTime, midTime,
						  startXdot, startYdot,
						  midXdot, midYdot))
		return false;
	if (!getPathSegLength(arclengthAndTime, trajectory, totalLength,
						  midXIt, midYIt,
						  endXIt, endYIt,
						  midTime, endTime,
						  midXdot, midYdot,
						  endXdot, endYdot))
		return false;
	return true;
}

bool getPathLength(std::vector<ArclengthAndTime> &arclengthAndTime,
				   double &totalLength,
				   const Trajectory &trajectory)
{
	Segment::State xState;
	Segment::State yState;

	// Get initial conditions for getPathSegLength call
	// for this particular segment
	TrajectoryPerJoint::const_iterator startXIt = sample(trajectory[0], 0, xState);
	if (startXIt == trajectory[0].cend())
	{
		ROS_ERROR("base_trajectory : could not sample initial xState 0");
		return false;
	}

	TrajectoryPerJoint::const_iterator startYIt = sample(trajectory[1], 0, yState);
	if (startYIt == trajectory[1].cend())
	{
		ROS_ERROR("base_trajectory : could not sample initial yState 0");
		return false;
	}
	const double startXdot = xState.velocity[0];
	const double startYdot = yState.velocity[0];

	const double endTime = trajectory[0].back().endTime();
	TrajectoryPerJoint::const_iterator endXIt = sample(trajectory[0], endTime, xState);
	if (endXIt == trajectory[0].cend())
	{
		ROS_ERROR("base_trajectory : could not sample initial xState end");
		return false;
	}
	TrajectoryPerJoint::const_iterator endYIt = sample(trajectory[1], endTime, yState);
	if (endYIt == trajectory[1].cend())
	{
		ROS_ERROR("base_trajectory : could not sample initial yState end");
		return false;
	}

	const double endXdot = xState.velocity[0];
	const double endYdot = yState.velocity[0];

	totalLength = 0.0;
	arclengthAndTime.clear();

	getPathSegLength(arclengthAndTime, trajectory, totalLength,
			startXIt, startYIt,
			endXIt, endYIt,
			0, endTime, // start, end time
			startXdot, startYdot,
			endXdot, endYdot);

	// Build piecewise quintic bezier spline connecting each point in arcLengthAndTime
	// that gives function from time->arclength.
	// Then construct the inverse, a vector of times which correspond to roughly equal
	// sized arc-length intervals along the entire path
	return true;
}

bool evaluateSpline(const Trajectory &trajectory,
					const std::vector<trajectory_msgs::JointTrajectoryPoint> &points,
					double dMax, // limit of path excursion from straight line b/t waypoints
					double vMax, // max overall velocity
					double aMax, // max allowed acceleration
					double aCentMax, // max allowed centripetal acceleration
					double &time,    // time taken to drive the path
					double &cost)    // cost of the path
{
	time = 0.0;
	cost = 0.0;
	const double dt = 0.05; // 20 samples per segment

	trajectory_interface::PosVelAccState<double> prevXState;
	trajectory[0][0].sample(0.0, prevXState);
	trajectory_interface::PosVelAccState<double> prevYState;
	trajectory[0][0].sample(0.0, prevYState);

	for (size_t seg = 0; seg < trajectory[0].size(); seg++)
	{
		// get velocity from start point
		double currV = hypot(prevXState.velocity[0], prevYState.velocity[0]);
		std::vector<double> sampleT;        // time for each sample in the forward pass - reused for the back pass
		std::vector<double> sampleDeltaS;   // change in position along spline for each step
		std::vector<double> sampleVForward; // allowed forward V at step
		for (double t = dt; t < 1.0; t += dt)
		{
			sampleT.push_back(t);
			trajectory_interface::PosVelAccState<double> xState;
			trajectory[0][seg].sample(t, xState);
			trajectory_interface::PosVelAccState<double> yState;
			trajectory[1][seg].sample(t, yState);
			trajectory_interface::PosVelAccState<double> thetaState;
			trajectory[2][seg].sample(t, thetaState);

			// Save distance between prev and current position for this timestep
			sampleDeltaS.push_back(hypot(xState.position[0] - prevXState.position[0],
										 yState.position[0] - prevYState.position[0]));

			// Get curvature for this sample
			const double pXdot = xState.velocity[0];
			const double pXdotdot = xState.acceleration[0];
			const double pYdot = yState.velocity[0];
			const double pYdotdot = yState.acceleration[0];
			const double curvature = (pXdot * pYdotdot - pYdot * pXdotdot) / pow(pXdot * pXdot + pYdot * pYdot, 3./2.);
			// Get orthogonal distance between spline position and
			// line segment connecting the corresponding waypoints
			const double distToLine = perpDistance(points[seg].positions, points[seg+1].positions, xState.position[0], yState.position[0]);
		}
	}
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "base_trajectory");
	ros::NodeHandle nh;

	double loop_hz;

	nh.param<double>("loop_hz", loop_hz, 100.);
	period = ros::Duration(1.0 / loop_hz);

	ros::ServiceServer service = nh.advertiseService("base_trajectory/spline_gen", callback);

	ros::spin();
}
