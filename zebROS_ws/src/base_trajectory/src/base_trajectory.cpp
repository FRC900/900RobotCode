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

// Define typedefs - we're generating Quinitc (5-th order polynomial) splines
// which have doubles as their datatype
typedef joint_trajectory_controller::JointTrajectorySegment<trajectory_interface::MyQuinticSplineSegment<double>> Segment;

typedef std::vector<Segment> TrajectoryPerJoint;
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

// Find the angle that line p1p2 is pointing at
double getLineAngle(const std::vector<double> &p1, const std::vector<double> &p2)
{
	return angles::normalize_angle_positive(atan2(p2[1] - p1[1], p2[0] - p1[0]));
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
		s << "x" << i << " = " << prev_x << ":" << range / 100. << ":" << msg.end_points[i] << ";" << std::endl;;
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

// input should be JointTrajectory[] custom message
// Output wil be array of spline coefficents swerve_point_generator/Coefs[] for x, y, orientation
bool generate(base_trajectory::GenerateSpline::Request &msg,
			  base_trajectory::GenerateSpline::Response &out_msg)
{
	// Hold current position if trajectory is empty
	if (msg.points.empty())
	{
		ROS_DEBUG("Empty trajectory command, stopping.");
		return false;
	}
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
	if (msg.points[0].velocities.size() == 0)
	{
		// Starting with 0 velocity
		// TODO - handle arbitrary starting velocity
		for(size_t i = 0; i < n_joints; i++)
		{
			msg.points[0].velocities.push_back(0.);
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
		double prev_angle = getLineAngle(msg.points[0].positions, msg.points[1].positions);
		double prev_length = hypot(msg.points[1].positions[0] - msg.points[0].positions[0],
								   msg.points[1].positions[1] - msg.points[0].positions[1]);

		for (size_t i = 1; i < (msg.points.size() - 1); i++)
		{
			const auto &mi   = msg.points[i].positions;
			const auto &mip1 = msg.points[i + 1].positions;

			const double curr_angle = getLineAngle(mi, mip1);

			const double angle = angles::normalize_angle_positive(prev_angle + (curr_angle - prev_angle) / 2.0);

			const double curr_length = hypot(mip1[0] - mi[0], mip1[1] - mi[1]);

			// Adding a scaling factor here controls the velocity
			// at the waypoints.  Bigger than 1 ==> curvier path with
			// higher speeds.  Less than 1 ==> tigher turns to stay
			// closer to straight paths.
			const double length = std::min(prev_length, curr_length) * .75;

			msg.points[i].velocities.push_back(length * cos(angle)); // x
			msg.points[i].velocities.push_back(length * sin(angle)); // y
			msg.points[i].velocities.push_back(0);                   // theta

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
			msg.points.back().velocities.push_back(0.);
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
		{
			// First point is a special case - just
			// find the 2nd derivative of the pretend cubic between
			// points 0 & 1
			const double Ax = msg.points[0].positions[0];
			const double Ay = msg.points[0].positions[1];
			const double tAx = msg.points[0].velocities[0];
			const double tAy = msg.points[0].velocities[1];
			const double Bx = msg.points[1].positions[0];
			const double By = msg.points[1].positions[1];
			const double tBx = msg.points[1].velocities[0];
			const double tBy = msg.points[1].velocities[1];

			const double xaccel = 6.0 * Ax + 2.0 * tAx + 4.0 * tBx - 6.0 * Bx;
			const double yaccel = 6.0 * Ay + 2.0 * tAy + 4.0 * tBy - 6.0 * By;

			msg.points[0].accelerations.push_back(xaccel); // x
			msg.points[0].accelerations.push_back(yaccel); // y
			msg.points[0].accelerations.push_back(0.); // theta
		}

		// For interior points, weight the average of the
		// 2nd derivative of each of the pretend cubic
		// splines and use them as the acceleration for
		// the to-be-generated quintic spline.
		for (size_t i = 1; i < (msg.points.size() - 1); i++)
		{
			const double Ax = msg.points[i-1].positions[0];
			const double Ay = msg.points[i-1].positions[1];
			const double tAx = msg.points[i-1].velocities[0];
			const double tAy = msg.points[i-1].velocities[1];

			const double Bx = msg.points[i].positions[0];
			const double By = msg.points[i].positions[1];
			const double tBx = msg.points[i].velocities[0];
			const double tBy = msg.points[i].velocities[1];

			const double Cx = msg.points[i+1].positions[0];
			const double Cy = msg.points[i+1].positions[1];
			const double tCx = msg.points[i+1].velocities[0];
			const double tCy = msg.points[i+1].velocities[1];

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

			msg.points[i].accelerations.push_back(xaccel); // x
			msg.points[i].accelerations.push_back(yaccel); // y
			msg.points[i].accelerations.push_back(0.); // theta

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
		{
			// Last point is also a special case since
			// there's just one pretend cubic spline to consider
			const size_t last = msg.points.size() - 1;
			const double Ax = msg.points[last-1].positions[0];
			const double Ay = msg.points[last-1].positions[1];
			const double tAx = msg.points[last-1].velocities[0];
			const double tAy = msg.points[last-1].velocities[1];
			const double Bx = msg.points[last].positions[0];
			const double By = msg.points[last].positions[1];
			const double tBx = msg.points[last].velocities[0];
			const double tBy = msg.points[last].velocities[1];

			const double xaccel = 6.0 * Ax + 2.0 * tAx + 4.0 * tBx - 6.0 * Bx;
			const double yaccel = 6.0 * Ay + 2.0 * tAy + 4.0 * tBy - 6.0 * By;

			msg.points[last].accelerations.push_back(xaccel); // x
			msg.points[last].accelerations.push_back(yaccel); // y
			msg.points[last].accelerations.push_back(0.); // theta
		}
	}

	ros::message_operations::Printer<::base_trajectory::GenerateSplineRequest_<std::allocator<void>>>::stream(std::cout, "", msg);

	// Actually generate the new trajectory
	// This will create spline coefficents for
	// each of the x,y,z paths
	Trajectory trajectory;
	try
	{
		trajectory_msgs::JointTrajectory jtm;
		jtm.joint_names = joint_names;
		jtm.points = msg.points;
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

	out_msg.orient_coefs.resize(trajectory[0].size());
	out_msg.x_coefs.resize(trajectory[0].size());
	out_msg.y_coefs.resize(trajectory[0].size());

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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "base_trajectory");
	ros::NodeHandle nh;

	double loop_hz;

	nh.param<double>("loop_hz", loop_hz, 100.);
	period = ros::Duration(1.0 / loop_hz);

	ros::ServiceServer service = nh.advertiseService("base_trajectory/spline_gen", generate);

	ros::spin();
}
