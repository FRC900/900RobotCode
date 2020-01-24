// base_trajectory.cpp
// Generates a spline-based path connecting input waypoints
// Implements an algorithm similar to that found in
// http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf
// http://www2.informatik.uni-freiburg.de/~lau/paper/lau09iros.pdf
// http://ais.informatik.uni-freiburg.de/teaching/ws09/robotics2/projects/mr2-p6-paper.pdf
//
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
#include <tf2/LinearMath/Quaternion.h>

#include <angles/angles.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

// Various tuning paramters - will be read as params
// and exposed as dynamic reconfigure options
//
// Used for determining arclength vs. time function.  This is the
// maximum allowed error reported from the simpson's rule approximation
// of the segment. Segments with errors greater than this are subdivided
// and each subsegment's length is evaluated separetely, leading to a
// more accurate approximation
double segLengthEpsilon;

// Used to generate equally-spaced points along the entire
// arclength of the x-y spline.  Points are spaced
// distBetweenArcLengths apart, +/- distBetweenArcLengthEpsilon
// midTimeInflation is used to bias the binary search value - since
// the search is for monotonically increasing arglengths, the code assumes
// that the "mid"point of the next search should be the location of the
// last point found plus the previous distance between points.  The "mid"point
// is moved a bit futher out to account for cases where the curvature changes
//  - it is more efficent to be a little bit beyond the expected value rather than
//  closer to it.  midTimeInflation is the multipler for that distance.
double distBetweenArcLengths; // 3 cm
double distBetweenArcLengthEpsilon; // 2.5 mm
double midTimeInflation;

double pathDistBetweenArcLengths; // 30 cm
double pathDistBetweenArcLengthsEpsilon; // 1 cm

// RPROP optimization parameters.  The code skips to the next optimization
// variable if the deltaCost for a step is less than deltaCost.  This value is
// gradually decreased as the result is optimized. This holds the initial and
// minimum value of deltaCost
double initialDeltaCostEpsilon;
double minDeltaCostEpsilon;

// Initial change added to optimization parameter in the RPROP loop
double initialDParam;

// Robot limits for evaluating path cost
double pathLimitDistance;
double maxVel;
double maxLinearAcc;
double wheelRadius;
double maxCentAcc;

// Simple class to turn debugging output on and off
class MessageFilter : public ros::console::FilterBase
{
	public:
		MessageFilter(bool enabled)
		{
			enabled_ = enabled;
		}
		void enable(void)
		{
			enabled_ = true;
		}
		void disable(void)
		{
			enabled_ = false;
		}
		bool isEnabled(void) override
		{
			return enabled_;
		}
		bool isEnabled(ros::console::FilterParams &) override
		{
			return isEnabled();
		}
	private:
		bool enabled_;
};

MessageFilter messageFilter(true);


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
void printCoefs(std::stringstream &s, const std::string &name, const std::vector<base_trajectory::Coefs> &coefs)
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

// Generate matlab / octave code for displaying generated splines
void writeMatlabCode(const base_trajectory::GenerateSpline::Response &msg)
{
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
	s << "subplot(3,2,2)" << std::endl;
	s << "plot(x, porient_y, x, pdorient_y)" << std::endl;
	s << "subplot(3,2,3)" << std::endl;
	s << "plot (x,px_y, x, py_y)" << std::endl;
	s << "subplot(3,2,4)" << std::endl;
	s << "plot (x,pdx_y, x, pdy_y)" << std::endl;
	s << "subplot(3,2,5)" << std::endl;
	s << "plot (x,pddx_y, x, pddy_y)" << std::endl;
	s << "subplot(3,2,6)" << std::endl;
	s << "plot (x,pdddx_y, x, pdddy_y)" << std::endl;
	ROS_INFO_STREAM_FILTER(&messageFilter, "Matlab_splines : " << s.str());
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

	// Syntax to let callers use the values in this
	// object as if they were an array. Since the optimizer
	// loops over all member vars, this turns the code
	// in there into a simple for() loop
	const size_t size(void) const
	{
		return 3;
	}

	double& operator[](size_t i)
	{
		if (i == 0)
			return posX_;
		if (i == 1)
			return posY_;
		if (i == 2)
			return length_;
		throw std::out_of_range ("out of range in OptParams operator[]");
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

bool initSpline(Trajectory &trajectory,
				std::vector<std::string> jointNames,
				const std::vector<trajectory_msgs::JointTrajectoryPoint> &points)
{
	const size_t nJoints = jointNames.size();
	std::vector<bool> angle_wraparound;

	// Assume the path starts at time 0
	ros::Time next_update_time = ros::Time(0);
	ros::Time next_update_uptime = next_update_time;

	// Set this to false to prevent the code
	// from thinking we're driving rotation joints
	// rather than running linear motion
	for (size_t i = 0; i < nJoints; i++)
		angle_wraparound.push_back(false);

	// Allocate memory to hold an initial trajectory
	Trajectory hold_trajectory;
	static typename Segment::State current_joint_state = typename Segment::State(1);
	for (size_t i = 0; i < nJoints; ++i)
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
		static typename Segment::State hold_start_state = typename Segment::State(1);
		static typename Segment::State hold_end_state = typename Segment::State(1);

		const double stop_trajectory_duration = period.toSec() / 2.;
		const typename Segment::Time start_time  = 0;
		const typename Segment::Time end_time    = stop_trajectory_duration;
		const typename Segment::Time end_time_2x = 2.0 * stop_trajectory_duration;

		// Create segment that goes from current (pos,vel) to (pos,-vel)
		for (size_t i = 0; i < nJoints; ++i)
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
	options.joint_names               = &jointNames;
	options.angle_wraparound          = &angle_wraparound;
	options.rt_goal_handle            = NULL;
	options.default_tolerances        = NULL;
	options.allow_partial_joints_goal = true;
	for (size_t i = 0; (i < points.size()) && (messageFilter.isEnabled()); i++)
	{
		ROS_INFO_STREAM_FILTER(&messageFilter, "points[" << i << "]");
		ros::message_operations::Printer<::trajectory_msgs::JointTrajectoryPoint_<std::allocator<void>>>::stream(std::cout, "  ", points[i]);
	}

	// Actually generate the new trajectory
	// This will create spline coefficents for
	// each of the x,y,z paths
	try
	{
		trajectory_msgs::JointTrajectory jtm;
		jtm.joint_names = jointNames;
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

bool generateSpline(      std::vector<trajectory_msgs::JointTrajectoryPoint> points,
					const std::vector<OptParams> &optParams,
					Trajectory &trajectory)
{
	// Hard code 3 dimensions for paths to
	// follow - x&y translation and z rotation
	static std::vector<std::string> jointNames = {"x_linear_joint", "y_linear_joint", "z_rotation_joint"};
	const size_t nJoints = jointNames.size();

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
		for(size_t i = 0; i < nJoints; i++)
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
		double prevAngle = getLineAngle(points[0].positions, points[1].positions);
		double prevLength = hypot(points[1].positions[0] - points[0].positions[0],
								   points[1].positions[1] - points[0].positions[1]);

		for (size_t i = 1; i < (points.size() - 1); i++)
		{
			const auto &mi   = points[i].positions;
			const auto &mip1 = points[i + 1].positions;

			const double currAngle = getLineAngle(mi, mip1);
			double deltaAngle = currAngle - prevAngle;
			if (deltaAngle < -M_PI)
				deltaAngle += 2.0 * M_PI;
			const double angle = angles::normalize_angle_positive(prevAngle + deltaAngle / 2.0);

			const double currLength = hypot(mip1[0] - mi[0], mip1[1] - mi[1]);

			// Adding a scaling factor here controls the velocity
			// at the waypoints.  Bigger than 1 ==> curvier path with
			// higher speeds.  Less than 1 ==> tigher turns to stay
			// closer to straight paths.
			const double length = std::min(prevLength, currLength) * .75 + optParams[i].length_;

			points[i].velocities.push_back(length * cos(angle)); // x
			points[i].velocities.push_back(length * sin(angle)); // y
			points[i].velocities.push_back(0.0); // theta TODO : what if there is rotation both before and after this waypoint?

#if 0
			ROS_INFO_STREAM_FILTER(&messageFilter, "prevAngle " << prevAngle << " prevLength " << prevLength);
			ROS_INFO_STREAM_FILTER(&messageFilter, "currAngle " << currAngle << " currLength " << currLength);
			ROS_INFO_STREAM_FILTER(&messageFilter, "currAngle - prevAngle = " << currAngle - prevAngle << " angles::normalize_angle_positive(currAngle - prevAngle) = " << angles::normalize_angle_positive(currAngle - prevAngle));
			ROS_INFO_STREAM_FILTER(&messageFilter, "angle " << angle << " length " << length);
#endif
			// Update for next step
			prevAngle = currAngle;
			prevLength = currLength;
		}

		// End position is also 0 velocity
		// TODO - handle input end velocity
		for(size_t i = 0; i < nJoints; i++)
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
			ROS_INFO_STREAM_FILTER(&messageFilter, "dab = " << dab << " dbc = " << dbc);
			ROS_INFO_STREAM_FILTER(&messageFilter, "Ax = " << Ax << " tAx = " << tAx <<
			                " Bx = " << Bx << " tBx = " << tBx <<
			                " Cx = " << Cx << " tCx = " << tCx);
			ROS_INFO_STREAM_FILTER(&messageFilter, "Ay = " << Ay << " tAy = " << tAy <<
			                " By = " << By << " tBy = " << tBy <<
			                " Cy = " << Cy << " tCy = " << tCy);
#endif
		}
	}

	return initSpline(trajectory, jointNames, points);
}

double distSquared(const std::vector<double> &v, const std::vector<double> &w)
{
	return ((v[0] - w[0]) * (v[0] - w[0])) + ((v[1] - w[1]) * (v[1] - w[1]));
}

double distSquared(const double px, const double py, const std::vector<double> &w)
{
	return ((px - w[0]) * (px - w[0])) + ((py - w[1]) * (py - w[1]));
}

// Minimum distance between segment vw and point (p1, p2)
double pointToLineSegmentDistance(const std::vector<double> &v, const std::vector<double> &w,
		double px, double py)
{
	const float l2 = distSquared(v, w);
	if (l2 == 0.0)
		return sqrt(distSquared(px, py, v));   // v == w case, distance to single point
	// Consider the line extending the segment, parameterized as v + t (w - v).
	// We find projection of point p onto the line.
	// It falls where t = [(p-v) . (w-v)] / |w-v|^2
	// We clamp t from [0,1] to handle points outside the segment vw.
	const double t = std::max(0.0, std::min(1.0, ((px - v[0]) * (w[0] - v[0]) + (py - v[1]) * (w[1] - v[1])) / l2));
	const double projectionX = v[0] + t * (w[0] - v[0]);
	const double projectionY = v[1] + t * (w[1] - v[1]);
	return hypot(px - projectionX, py - projectionY);
}

struct ArclengthAndTime
{
	ArclengthAndTime(const double arcLength, const double time) :
		arcLength_(arcLength),
		time_(time)
	{
	}
	double arcLength_;
	double time_;
};

// Use Simpson's rule to estimate arc length between start and
// and time.  Function is combining dx and dy into a length,
// so use hypot(dx,dy) as the function being evaluated.
// Get the uppwer bound of the error as well - this lets
// us know if the estimate is within the bounds specified
// for calculating the estimate.
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
	// Error term is a line (mx+b) so min/max is going to be at one end
	// or the other.
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
// <cumulative arcLength at that time, time> tuples, hopefully increasing in time
// along with totalLength, which is the total length calculated so far.
bool getPathSegLength(std::vector<ArclengthAndTime> &arcLengthAndTime,
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
	const double midTime = (startTime + endTime) / 2.0;

	// TODO : Perhaps a check on dT being too small?

	// Simpson's rule needs the provided start and end values and
	// also one at the midpoint. Grab the midpoint values here
	static Segment::State xState;
	TrajectoryPerJoint::const_iterator midXIt = sample(trajectory[0], midTime, xState);
	if (midXIt == trajectory[0].cend())
	{
		ROS_ERROR_STREAM("base_trajectory : could not sample mid xState at time " << midTime);
		return false;
	}

	static Segment::State yState;
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
	//ROS_INFO_STREAM_FILTER(&messageFilter, "simpsonsRule : startTime = " << startTime << " endTime = " << endTime << " error = " << error);

	// If the error magnitude is less than epsilon,
	// use this approximation for the arcLength from
	// start to end
	if (fabs(error) < segLengthEpsilon)
	{
		totalLength += estimate;
		arcLengthAndTime.push_back(ArclengthAndTime(totalLength, endTime));
		return true;
	}

	// Otherwise, split segment in half
	// and recursively calculate the length of each half.
	if (!getPathSegLength(arcLengthAndTime, trajectory, totalLength,
						  startXIt, startYIt,
						  midXIt, midYIt,
						  startTime, midTime,
						  startXdot, startYdot,
						  midXdot, midYdot))
		return false;
	if (!getPathSegLength(arcLengthAndTime, trajectory, totalLength,
						  midXIt, midYIt,
						  endXIt, endYIt,
						  midTime, endTime,
						  midXdot, midYdot,
						  endXdot, endYdot))
		return false;
	return true;
}

// Given a trajectory, find time values which generate
// increasing, equally spaced lengths along
// that trajectory
bool subdivideLength(std::vector<double> &equalLengthTimes,
		const Trajectory &trajectory,
		const double distanceBetweenLengths,
		const double distanceEpsilon)
{
	equalLengthTimes.clear();
	equalLengthTimes.push_back(0); // start at t==0

	// For each cumulative distance
	// start = prev found time, end = last time
	// Binary seh to get sample[] within tolerance of desired cumulative disance
	// Push that result onto equalLengthTimes
	double start = 0.0;

	// since we're looking for more or less monotonincally increasing values
	// keep track of the jump to get from one location to another. Use
	// this as a starting guess for the next length increment
	double prevStart = 0.0;
	const double endTime = trajectory[0].back().endTime();
	double prevTimeDelta = endTime / 2.0;

	static Segment::State state;
	sample(trajectory[0], endTime, state);
	const double totalLength = state.position[0];

	size_t iterCount = 0;
	for (double currDistance = distanceBetweenLengths; currDistance <= totalLength; currDistance += distanceBetweenLengths)
	{
		double end = endTime;
		double mid = start + prevTimeDelta;
		while((end - mid) > 0.001) // Quit if time delta gets too small
		{
			TrajectoryPerJoint::const_iterator trajIt = sample(trajectory[0], mid, state);
			if (trajIt == trajectory[0].cend())
			{
				ROS_ERROR_STREAM("base_trajectory : could not sample mid state at time " << mid);
				return false;
			}
			iterCount += 1;
			const double delta = currDistance - state.position[0];
			//ROS_INFO_STREAM_FILTER(&messageFilter, "currDistance = " << currDistance << " start=" << start << " mid=" << mid << " end=" << end << " position[0]=" << state.position[0] << " delta=" << delta);
			if (fabs(delta) < distanceEpsilon)
			{
				break;
			}
			else if (delta > 0)
			{
				start = mid;
			}
			else
			{
				end = mid;
			}
			mid = (start + end) / 2.0;
		}

		if (mid > endTime)
		{
			equalLengthTimes.push_back(endTime);
			break;
		}

		equalLengthTimes.push_back(mid);
		start = mid;
		// Use starting "midpoint" guess of of start time plus
		// the previous time jump, plus a little bit extra to
		// make sure we don't undershoot. Undershooting would require a
		// binary search of basically the entire distance between
		// mid and end, and since mid is very close to the start,
		// it is basically a binary search of the entire range.
		prevTimeDelta = (start - prevStart) * midTimeInflation;
		prevStart = start;
	}
	if (equalLengthTimes.back() != endTime)
	{
		ROS_WARN_STREAM("Addding endTime to equalLengthsTime array");
		equalLengthTimes.push_back(endTime);
	}
	ROS_INFO_STREAM_FILTER(&messageFilter, "iterCount = " << iterCount);
	return true;
}

bool getPathLength(Trajectory &arcLengthTrajectory,
				   const Trajectory &trajectory)
{
	static Segment::State xState;
	static Segment::State yState;

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

	double totalLength = 0.0;

	// Generate a list of time -> arclength pairs stored
	// in arcLengthAndTime
	std::vector<ArclengthAndTime> arcLengthAndTime;
	if (!getPathSegLength(arcLengthAndTime, trajectory, totalLength,
			startXIt, startYIt,
			endXIt, endYIt,
			0, endTime, // start, end time
			startXdot, startYdot,
			endXdot, endYdot))
		return false;

	// Create a path between each of these length/time
	// coordinates using cubic splines to interpolate
	// between each point
	static const std::vector<std::string> jointNames = { "arcLength" };
	std::vector<trajectory_msgs::JointTrajectoryPoint> points;
	for (const auto &alt : arcLengthAndTime)
	{
		points.push_back(trajectory_msgs::JointTrajectoryPoint());
		points.back().positions.push_back(alt.arcLength_);
		points.back().time_from_start = ros::Duration(alt.time_);
	}

	if (!initSpline(arcLengthTrajectory, jointNames, points))
		return false;

	return true;
}

// evaluate spline - inputs are spline coeffs (Trajectory), waypoints, dmax, vmax, amax, acentmax
//                   returns time taken to drive spline, cost, possibly waypoints
bool evaluateSpline(double &cost,
					const Trajectory &trajectory,
					const std::vector<trajectory_msgs::JointTrajectoryPoint> &points,
					const double dMax, // limit of path excursion from straight line b/t waypoints
					const double vMax, // max overall velocity
					const double aMax, // max allowed acceleration
					const double wheelRadius, // radius from center to wheels
					const double aCentMax) // max allowed centripetal acceleration
{

	// arcLengthTrajectory takes in a time and returns the x-y distance
	// traveled up to that time.
	Trajectory arcLengthTrajectory;
	if (!getPathLength(arcLengthTrajectory, trajectory))
	{
		ROS_ERROR("base_trajectory_node : getPathLength() failed");
		return false;
	}
	// equalArcLengthTimes will contain times that are
	// spaced equidistant along the arc length passed in
	std::vector<double> equalArcLengthTimes;
	if (!subdivideLength(equalArcLengthTimes, arcLengthTrajectory,
						 distBetweenArcLengths, distBetweenArcLengthEpsilon))
		return false;

	// equalArcLengthTimes is a vector of times. Each entry is the time
	// of an equally-spaced sample from arcLengthTrajectory. That is
	// sample i is arclength (approx) d away from sample i-1 and i+1 for all i.

	// Starting at arclength 0
	double prevArcLength = 0;

	std::vector<double> deltaS;   // change in position along spline for each step
	std::vector<double> distanceToPathMidpoint;
	std::vector<double> vTrans; // max velocity allowed at each step
	std::vector<double> vRot;

	// Add 0th entries for arrays so indices line up with equalArcLengthTimes
	deltaS.push_back(0);
	distanceToPathMidpoint.push_back(0);
	vTrans.push_back(0);
	vRot.push_back(0);
	size_t seg = 0;
	// Declare these static so they're not constantly created
	// and destroyed on every call to the function
	static Segment::State arcLengthState;
	static Segment::State xState;
	static Segment::State yState;
	static Segment::State thetaState;
	for (size_t i = 1; i < equalArcLengthTimes.size(); i++)
	{
		const double t = equalArcLengthTimes[i];
		while (points[seg+1].time_from_start < ros::Duration(t))
			seg += 1;
		//ROS_INFO_STREAM_FILTER(&messageFilter, "processing index " << i << " t=" << t <<" seg=" << seg);
		// Save distance between prev and current position for this timestep
		// This should be nearly constant between points, but
		// get the exact value for each to be more precise
		TrajectoryPerJoint::const_iterator arcLengthIt = sample(arcLengthTrajectory[0], t, arcLengthState);
		if (arcLengthIt == arcLengthTrajectory[0].cend())
		{
			ROS_ERROR_STREAM("base_trajectory : evaluateSpline could not sample arcLengthState at time " << t);
			return false;
		}
		deltaS.push_back(arcLengthState.position[0] - prevArcLength);
		prevArcLength = arcLengthState.position[0];

		// Since seg is set to the current spline segment, use this to
		// index into each trajectory. This saves time compared to searching
		// through the range of times in each trajectory array
		// Add 1 to account for the dummy starting segment
		TrajectoryPerJoint::const_iterator xIt = trajectory[0].cbegin() + seg + 1;
		if (xIt >= trajectory[0].cend())
		{
			ROS_ERROR_STREAM("base_trajectory : evaluateSpline could not sample xState at time " << t);
			return false;
		}
		xIt->sample(t, xState);

		TrajectoryPerJoint::const_iterator yIt = trajectory[1].cbegin() + (xIt - trajectory[0].cbegin());
		if (yIt >= trajectory[1].cend())
		{
			ROS_ERROR_STREAM("base_trajectory : evaluateSpline could not sample yState at time " << t);
			return false;
		}
		yIt->sample(t, yState);

		TrajectoryPerJoint::const_iterator thetaIt = trajectory[2].cbegin() + (xIt - trajectory[0].cbegin());
		if (thetaIt >= trajectory[2].cend())
		{
			ROS_ERROR_STREAM("base_trajectory : evaluateSpline could not sample thetaState at time " << t);
			return false;
		}
		thetaIt->sample(t, thetaState);

		// Get orthogonal distance between spline position and
		// line segment connecting the corresponding waypoints
		distanceToPathMidpoint.push_back(pointToLineSegmentDistance(points[seg].positions, points[seg+1].positions, xState.position[0], yState.position[0]));
#if 0
		ROS_INFO_STREAM_FILTER(&messageFilter, "pointToLineSegmentDistance = " << distanceToPathMidpoint.back() <<
				" p1: " << points[seg].positions[0] << "," << points[seg].positions[1] <<
				" p2: " << points[seg+1].positions[0] << "," << points[seg+1].positions[1] <<
				" x: " << xState.position[0] << " y: " << yState.position[0]);
#endif

		// Get curvature for this sample
		const double pXdot = xState.velocity[0];
		const double pXdotdot = xState.acceleration[0];
		const double pYdot = yState.velocity[0];
		const double pYdotdot = yState.acceleration[0];
		const double curvature = (pXdot * pYdotdot - pYdot * pXdotdot) / pow(pXdot * pXdot + pYdot * pYdot, 3.0/2.0);
		//ROS_INFO_STREAM_FILTER(&messageFilter, "curvature = " << curvature);

		// First pass of vTrans limits by absolute max velocity
		// and also velocity limited by max centripetal acceleration
		// Assume rotational velocity is a hard constraint we have to hit
		vTrans.push_back(std::min(vMax - fabs(thetaState.velocity[0]) * wheelRadius, sqrt(aCentMax / fabs(curvature))));
		//ROS_INFO_STREAM_FILTER(&messageFilter, "vTrans[" << i << "==" << equalArcLengthTimes[i] <<"]=" << vTrans[i]);
	}

	// Forward pass
	for (size_t i = 1; i < vTrans.size(); i++)
	{
		vTrans[i] = std::min(vTrans[i], sqrt(vTrans[i - 1] * vTrans[i - 1] + 2.0 * aMax * deltaS[i]));
		//ROS_INFO_STREAM_FILTER(&messageFilter, "vTrans[" << i << "==" << equalArcLengthTimes[i] <<"]=" << vTrans[i]);
	}

	// Backwards pass
	vTrans.back() = 0; // Hard-code end velocity for now. TODO - make it grab from last point or spline segment
	for (size_t i = vTrans.size() - 2; i > 0; i--)
	{
		vTrans[i] = std::min(vTrans[i], sqrt(vTrans[i + 1] * vTrans[i + 1] + 2.0 * aMax * deltaS[i - 1]));
		//ROS_INFO_STREAM_FILTER(&messageFilter, "vTrans[" << i << "==" << equalArcLengthTimes[i] <<"]=" << vTrans[i]);
	}

	// Calculate arrival time at each of the equalLengthArcTimes distances
	std::vector<double> remappedTimes;
	remappedTimes.push_back(0);
	for (size_t i = 1; i < vTrans.size(); i++)
	{
		remappedTimes.push_back(remappedTimes.back() + (2.0 * deltaS[i]) / (vTrans[i - 1] + vTrans[i]));
	}

	// Cost is total time to traverse the path plus a large
	// penalty for moving more that dMax past the mipoint of the
	// straight line segment connecting each waypoint. The latter
	// imposes a constraint that the path can't be too curvy - and
	// keeping close to the straight-line path should prevent it from running
	// into obstacles too far off that path.
	cost = remappedTimes.back();
	for (const auto d: distanceToPathMidpoint)
	{
		cost += exp(25.0 * ((fabs(d) / dMax) - 0.9));
	}
	ROS_INFO_STREAM_FILTER(&messageFilter, "time = " << remappedTimes.back() << " cost = " << cost);

	return true;
}

void trajectoryToSplineResponseMsg(base_trajectory::GenerateSpline::Response &out_msg,
		const Trajectory &trajectory,
		const std::vector<std::string> &jointNames)
{
	// Convert from Trajectory type into the correct output
	// message type
	out_msg.orient_coefs.resize(trajectory[0].size());
	out_msg.x_coefs.resize(trajectory[0].size());
	out_msg.y_coefs.resize(trajectory[0].size());
	out_msg.end_points.clear();

	const size_t n_joints = jointNames.size();
	for (size_t seg = 0; seg < trajectory[0].size(); seg++)
	{
		for (size_t joint = 0; joint < n_joints; joint++)
		{
			ROS_INFO_STREAM_FILTER(&messageFilter, "joint = " << jointNames[joint] << " seg = " << seg <<
			                " start_time = " << trajectory[joint][seg].startTime() <<
			                " end_time = " << trajectory[joint][seg].endTime());
			auto coefs = trajectory[joint][seg].getCoefs();

			std::stringstream s;
			s << "coefs ";
			for (size_t i = 0; i < coefs[i].size(); ++i)
				s << coefs[0][i] << " ";
			ROS_INFO_STREAM_FILTER(&messageFilter, s.str());

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
	Trajectory arcLengthTrajectory;
	if (!getPathLength(arcLengthTrajectory, trajectory))
	{
		ROS_ERROR("base_trajectory_node trajectoryToSplineResponseMsg : getPathLength() failed");
		return;
	}
	// --- Generate waypoints for final path ---
	// equalArcLengthTimes will contain times that are
	// spaced equidistant along the arc length passed in
	std::vector<double> equalArcLengthTimes;
	if (!subdivideLength(equalArcLengthTimes, arcLengthTrajectory,
						 pathDistBetweenArcLengths, pathDistBetweenArcLengthsEpsilon))
	{
		ROS_ERROR("base_trajectory_node trajectoryToSplineResponseMsg : subdivideLength() failed");
		return;
	}
	out_msg.path.poses.clear();
	const auto current_time = ros::Time::now();
	out_msg.path.header.stamp = current_time;
	out_msg.path.header.frame_id = "initial_pose";
	static Segment::State state;
	for (const auto t: equalArcLengthTimes)
	{
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = current_time + ros::Duration(t);
		pose.header.frame_id = "initial_pose";

		TrajectoryPerJoint::const_iterator xIt = sample(trajectory[0], t, state);
		if (xIt == trajectory[0].cend())
		{
			ROS_ERROR_STREAM("base_trajectory trajectoryToSplineResponseMsg : could not sample xState at time " << t);
			return;
		}
		pose.pose.position.x = state.position[0];

		TrajectoryPerJoint::const_iterator yIt = sample(trajectory[1], t, state);
		if (yIt == trajectory[1].cend())
		{
			ROS_ERROR_STREAM("base_trajectory trajectoryToSplineResponseMsg : could not sample yState at time " << t);
			return;
		}
		pose.pose.position.y = state.position[0];
		pose.pose.position.z = 0;

		TrajectoryPerJoint::const_iterator orientationIt = sample(trajectory[2], t, state);
		if (orientationIt == trajectory[2].cend())
		{
			ROS_ERROR_STREAM("base_trajectory trajectoryToSplineResponseMsg : could not sample orientationState at time " << t);
			return;
		}
		geometry_msgs::Quaternion orientation;
		tf2::Quaternion tf_orientation;
		tf_orientation.setRPY(0, 0, state.position[0]);
		orientation.x = tf_orientation.getX();
		orientation.y = tf_orientation.getY();
		orientation.z = tf_orientation.getZ();
		orientation.w = tf_orientation.getW();

		pose.pose.orientation = orientation;
		out_msg.path.poses.push_back(pose);
	}
}

// Algorithm to optimize parameters using the sign
// of the change in cost function.
// The parameters in this case are an offset to the X&Y positions
// of each point the spline passes through along with the length of the
// tangent to the path (the velocity) at that point.  Each point
// can have all 3 altered to improve the overall cost of the path (where
// cost is time to drive plus a sum of penalties for the path straying too
// far from the straight line between waypoints).
// The parameters are looped through one by one.  A small offset is added
// to the parameter and a new spline is generated and scored.  The cost
// compared to the previous cost is used to generate a new offset value
// for the next iteration.
bool RPROP(
		Trajectory &bestTrajectory,
		const std::vector<trajectory_msgs::JointTrajectoryPoint> &points,
		double dMax, // limit of path excursion from straight line b/t waypoints
		double vMax, // max overall velocity
		double aMax, // max allowed acceleration
		double wheelRadius, // radius from center to wheels
		double aCentMax) // max allowed centripetal acceleration
{
	// initialize params to 0 offset in x, y and tangent
	// length for all spline points
	std::vector<OptParams> bestOptParams(points.size());
	if (!generateSpline(points, bestOptParams, bestTrajectory))
	{
		ROS_ERROR("base_trajectory_node : RPROP initial generateSpline() falied");
		return false;
	}

	// Generate initial trajectory, evaluate to get cost
	double bestCost;
	if (!evaluateSpline(bestCost,
				bestTrajectory, points,
				dMax,    // limit of path excursion from straight line b/t waypoints
				vMax,    // max overall velocity
				aMax,    // max allowed acceleration
				wheelRadius, // wheel radius
				aCentMax))   // max allowed centripetal acceleration
	{
		ROS_ERROR("base_trajectory_node : RPROP initial evaluateSpline() falied");
		return false;
	}

	double deltaCostEpsilon = initialDeltaCostEpsilon;

	while (deltaCostEpsilon >= minDeltaCostEpsilon)
	{
		bool bestCostChanged = false;

		// Start with the previous best optimization parameters,
		// then loop through each and try to improve them
		// Ignore the first and last point - can't
		// optimize the starting and end position since
		// those need to be hit exactly.
		for (size_t i = 1; i < (bestOptParams.size() - 1); i++) // index of param optimized
		{
			// OptParams is overloaded to act like an array
			// for accessing individual params for each point
			for (size_t j = 0; j < bestOptParams[i].size(); j++)
			{
				auto optParams = bestOptParams;
				double deltaCost = std::numeric_limits<double>::max();
				double currCost = bestCost;
				double dparam = initialDParam;
				// One exit criteria for the inner loop is if the cost
				// stops improving by an appreciable amount while changing
				// this one parameter. Track that here
				while (deltaCost > deltaCostEpsilon)
				{
					// Alter one optimization parameter
					// and see how it changes the cost compared
					// to the last iteration
					Trajectory thisTrajectory;
					optParams[i][j] += dparam;
					if (!generateSpline(points, optParams, thisTrajectory))
					{
						ROS_ERROR("base_trajectory_node : RPROP generateSpline() falied");
						return false;
					}

					double thisCost;
					if (!evaluateSpline(thisCost,
								thisTrajectory, points,
								0.2, // limit of path excursion from straight line b/t waypoints
								4.5, // max overall velocity
								2.5, // max allowed acceleration
								0.3682, // wheel radius
								3.5)) // max allowed centripetal acceleration
					{
						ROS_ERROR("base_trajectory_node : RPROP evaluateSpline() failed");
						return false;
					}

					// If cost is better than the best cost, record it and
					// move on to optimizing the next parameter. It is possible
					// to loop back and return to this one again, but the paper
					// says that hyper-optimizing one parameter before moving
					// to others can lead to getting stuck in local minima.
					if (thisCost < bestCost)
					{
						bestTrajectory = thisTrajectory;
						bestCost = thisCost;
						bestOptParams = optParams;
						bestCostChanged = true;
						ROS_INFO_STREAM_FILTER(&messageFilter, "+++++++++ New best cost : " <<  bestCost);
						for (const auto &it: bestOptParams)
							ROS_INFO_STREAM_FILTER(&messageFilter, "     posX_:" << it.posX_ << " posY_:" << it.posY_ << " length_:" << it.length_);
						break;
					}
					// Use sign of difference between this cost and the
					// previous one to adjust the dparam value added to
					// the parameter being optimized.  1.2 and -0.5 are
					// intentionally not factors of each other to prevent
					// oscillating between the same set of values
					if (thisCost < currCost)
					{
						dparam *= 1.2;
					}
					else
					{
						dparam *= -0.5;
					}
					// Record values for next iteration
					deltaCost = fabs(thisCost - currCost);
					ROS_INFO_STREAM_FILTER(&messageFilter, "RPROP : i=" << i << " j=" << j << " bestCost=" << bestCost << " thisCost=" << thisCost << " currCost=" << currCost << " deltaCost=" << deltaCost << " deltaCostEpsilon=" << deltaCostEpsilon);
					currCost = thisCost;
				}
			}
		}
		if (!bestCostChanged)
			deltaCostEpsilon /= 1.75;
	}
	return true;
}


// input should be JointTrajectory[] custom message
// Output wil be array of spline coefficents base_trajectory/Coefs[] for x, y, orientation,
// along with a path consisting of waypoints evenly spaced along the spline
bool callback(base_trajectory::GenerateSpline::Request &msg,
			  base_trajectory::GenerateSpline::Response &out_msg)
{
	const auto startTime = ros::Time::now();
	// Hold current position if trajectory is empty
	if (msg.points.empty())
	{
		ROS_ERROR("Empty trajectory command, nothing to do.");
		return false;
	}
	if (msg.points.size() < 2)
	{
		ROS_WARN("Only one point passed into base_trajectory - adding a starting position of 0,0,0");
		msg.points.push_back(msg.points[0]);
		msg.points[0].positions.clear();
		for (size_t i = 0; i < 3; i++)
			msg.points[i].positions.push_back(0);
		msg.points[0].velocities.clear();
		msg.points[0].accelerations.clear();
	}

	// Splines segments are each 1 arbitrary unit long.
	// This later gets mapped to actual wall-clock times
	for (size_t i = 0; i < msg.points.size(); ++i)
		msg.points[i].time_from_start = ros::Duration(static_cast<double>(i));

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

	double cost;
	if (!evaluateSpline(cost,
				trajectory, msg.points,
				0.2, // limit of path excursion from straight line b/t waypoints
				4.5, // max overall velocity
				2.5, // max allowed acceleration
				0.3682, // wheel radius
				3.5)) // max allowed centripetal acceleration
	{
		ROS_ERROR("base_trajectory_node : evaluateSpline() returned false");
		return false;
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
	base_trajectory::GenerateSpline::Response tmp_msg;
	const std::vector<std::string> jointNames = {"x_linear_joint", "y_linear_joint", "z_rotation_joint"};
	trajectoryToSplineResponseMsg(tmp_msg, trajectory, jointNames);
	writeMatlabCode(tmp_msg);
	messageFilter.disable();
	fflush(stdout);

	if (!RPROP(trajectory,
				msg.points,
				pathLimitDistance, // limit of path excursion from straight line b/t waypoints
				maxVel, // max overall velocity
				maxLinearAcc, // max allowed acceleration
				wheelRadius, // wheel radius
				maxCentAcc)) // max allowed centripetal acceleration
	{
		ROS_ERROR("base_trajectory_node : RPROP() returned false");
		return false;
	}

	messageFilter.enable();
	trajectoryToSplineResponseMsg(out_msg, trajectory, jointNames);
	writeMatlabCode(out_msg);
	fflush(stdout);
	ROS_INFO_STREAM("base_trajectory_callback took " <<
			(ros::Time::now() - startTime).toSec() <<
			" seconds");
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "base_trajectory");
	ros::NodeHandle nh;

	double loop_hz;

	nh.param<double>("loop_hz", loop_hz, 100.);
	period = ros::Duration(1.0 / loop_hz);

	ddynamic_reconfigure::DDynamicReconfigure ddr;
	nh.param("seg_length_epsilon", segLengthEpsilon, 1.0e-4);
    ddr.registerVariable<double>("seg_length_epsilon", &segLengthEpsilon, "maximum error for each segment when parameterizing spline arclength", 0, .1);

	nh.param("dist_between_arc_lengths", distBetweenArcLengths, 0.03); // 3 cm
	nh.param("dist_between_arc_lengths_epsilon", distBetweenArcLengthEpsilon, 0.0025); // 2.5 mm
	nh.param("mid_time_inflation", midTimeInflation, 1.25);
	ddr.registerVariable<double>("dist_between_arc_lengths", &distBetweenArcLengths, "equal-spaced arc length distance", 0, 0.5);
	ddr.registerVariable<double>("dist_between_arc_lengths_epsilon", &distBetweenArcLengthEpsilon, "error tolerance for dist_between_arc_length", 0, 0.5);
	ddr.registerVariable<double>("mid_time_inflation", &midTimeInflation, "multiplier to prev distance for picking midpoint of next distance searched in arc length subdivision", 0, 3);
	nh.param("path_dist_between_arc_lengths", pathDistBetweenArcLengths, 0.30);
	nh.param("path_dist_between_arc_lengths_epsilon", pathDistBetweenArcLengthsEpsilon, 0.01);
	ddr.registerVariable<double>("path_dist_between_arc_lengths", &pathDistBetweenArcLengths, "spacing of waypoints along final generated path", 0, 2);
	ddr.registerVariable<double>("path_dist_between_arc_lengths_epsilon", &pathDistBetweenArcLengthsEpsilon, "error tolerance for final path waypoint spacing", 0, 2);

	nh.param("initial_delta_cost_epsilon", initialDeltaCostEpsilon, 0.05);
	nh.param("min_delta_cost_epsilon", minDeltaCostEpsilon, 0.005);
	ddr.registerVariable<double>("initial_delta_cost_epsilon", &initialDeltaCostEpsilon, "RPROP initial deltaCost value", 0, 1);
	ddr.registerVariable<double>("min_delta_cost_epsilon", &minDeltaCostEpsilon, "RPROP minimum deltaCost value", 0, 1);
	nh.param("initial_dparam", initialDParam, 0.05);
	ddr.registerVariable<double>("initial_dparam", &initialDParam, "RPROP initial optimization value change", 0, 2);

	nh.param("path_distance_limit", pathLimitDistance, 0.2);
	nh.param("max_vel", maxVel, 4.5);
	nh.param("max_linear_acc", maxLinearAcc, 2.5);
	nh.param("wheel_radius", wheelRadius, 0.3682);
	nh.param("max_cent_acc", maxCentAcc, 3.5);

	ddr.registerVariable<double>("path_distance_limit", &pathLimitDistance, "how far robot can diverge from straight-line path between waypoints", 0, 2);
	ddr.registerVariable<double>("max_vel", &maxVel, "max translational velocity", 0, 20);
	ddr.registerVariable<double>("max_linear_acc", &maxLinearAcc, "max linear acceleration", 0, 10);
	ddr.registerVariable<double>("wheel_radius", &wheelRadius, "robot's wheel radius", 0, 2);
	ddr.registerVariable<double>("max_cent_acc", &maxCentAcc, "max centrepital acceleration", 0, 10);
    ddr.publishServicesTopics();
	ros::ServiceServer service = nh.advertiseService("base_trajectory/spline_gen", callback);

	ros::spin();
}
