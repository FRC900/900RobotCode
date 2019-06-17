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

typedef joint_trajectory_controller::JointTrajectorySegment<trajectory_interface::MyQuinticSplineSegment<double>> Segment;

typedef std::vector<Segment> TrajectoryPerJoint;
typedef std::vector<TrajectoryPerJoint> Trajectory;

typedef trajectory_msgs::JointTrajectory::ConstPtr JointTrajectoryConstPtr;

ros::Duration period;

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

	constexpr double t = 0.1;
	if (msg.points[0].velocities.size() == 0)
	{
		// from http://scaledinnovation.com/analytics/splines/aboutSplines.html
		for (size_t i = 0; i < msg.points.size() - 1; i++)
		{
			double x0;
			double y0;
			if (i == 0)
			{
				x0 = 0;
				y0 = 0;
			}
			else
			{
				x0 = msg.points[i-1].positions[0];
				y0 = msg.points[i-1].positions[1];
			}
			const double x1 = msg.points[i].positions[0];
			const double y1 = msg.points[i].positions[1];
			const double x2 = msg.points[i+1].positions[0];
			const double y2 = msg.points[i+1].positions[1];
			const double d01 = hypot(x1 - x0, y1 - y0);
			const double d12 = hypot(x2 - x1, y2 - y1);
			const double fa = t * d01 / (d01 + d12);
			const double fb = t * d12 / (d01 + d12);
			const double p1x = x1 - fa * (x2 - x0);    // x2-x0 is the width of triangle T
			const double p1y = y1 - fa * (y2 - y0);    // y2-y0 is the height of T
			const double p2x = x1 + fb * (x2 - x0);
			const double p2y = y1 + fb * (y2 - y0);
			msg.points[i].velocities.push_back(p2x - p1x);
			msg.points[i].velocities.push_back(p2y - p1y);
			msg.points[i].velocities.push_back(0.);
			msg.points[i].accelerations.push_back(0.); // x
			msg.points[i].accelerations.push_back(0.); // y
			msg.points[i].accelerations.push_back(0.);  // theta
		}
		for(size_t j = 0; j < n_joints; j++)
		{
			msg.points.back().velocities.push_back(0.);
			msg.points.back().accelerations.push_back(0.);
		}
	}
#if 0
	// Generate a rough estimate of velocity magnitude and
	// vector at each waypoint, use that to populate the x&y velocity
	// for each waypoint before generating the spline
	if (msg.points[0].velocities.size() == 0)
	{
		// Velocity vector angle for a given point is the direction
		// from it to the next point. This has nothing to do with
		// the robot's orientation. Instead, it is the direction
		// the robot will be moving to get to the next waypoint
		std::vector<double> vel_vec;
		vel_vec.push_back(std::atan2(msg.points[0].positions[1], msg.points[0].positions[0]));
			ROS_INFO_STREAM("vel_vec : added " << vel_vec.back());
		for (size_t i = 0; i < msg.points.size() - 1; i++)
		{
			const auto &mi   = msg.points[i].positions;
			const auto &mip1 = msg.points[i + 1].positions;
			vel_vec.push_back(std::atan2(mip1[1] - mi[1], mip1[0] - mi[0]));
			ROS_INFO_STREAM("vel_vec : added " << vel_vec.back());
		}
		// Direction for last point is kinda undefined. Set it here
		// to be the same direction as between it and the previous
		// point and see how that works
		vel_vec.push_back(vel_vec.back());
			ROS_INFO_STREAM("vel_vec : added " << vel_vec.back());

		const double accel_max = .25;
		const double decel_max = .4;
		const double vel_max   = 1;

		// Regardless of any other constraints, the robot only
		// has to move so fast to cover a certain distance in
		// a given amount of time.  USe that as another constraint
		// on velocity here
		double path_vmax = hypot(msg.points[0].positions[0], msg.points[0].positions[1]);
		for (size_t i = 0; i < msg.points.size() - 1; i++)
		{
			path_vmax += hypot(msg.points[i+1].positions[0] - msg.points[i].positions[0],
			                   msg.points[i+1].positions[1] - msg.points[i].positions[1]);
		}
		std::vector<double> vel_mag(vel_vec.size(), 0.0);
		const double end_secs = msg.points.back().time_from_start.toSec();
		path_vmax /= end_secs;
		ROS_INFO_STREAM("Path vmax = " << path_vmax);
		// Backwards pass
		for (size_t i = vel_vec.size() - 1; i > 0; --i)
		{
			vel_mag[i] = std::min(std::min(vel_max, decel_max * (end_secs - msg.points[i - 1].time_from_start.toSec())), path_vmax);
		}
		for (size_t i = 1; i < vel_mag.size(); i++)
		{
			vel_mag[i] = std::min(vel_mag[i], accel_max * msg.points[i - 1].time_from_start.toSec());
		}
		for (size_t i = 0; i < vel_mag.size(); i++)
			ROS_INFO_STREAM("vel_mag[" << i << "] = " << vel_mag[i]);
		// Really basic motion modeling here - later steps should refine it?
		// Starting at 0 velocity
		for (size_t i = 0; i < (vel_vec.size() - 2); i++)
		{
			double angle;

			// assume hading at midpoint is kinda half-way
			// between the heading getting to that point and the heading
			// leading out of the point
			angle = (vel_vec[i] + vel_vec[i + 1]) / 2.0; // TODO - scale by segment length?
			msg.points[i].velocities.push_back(vel_mag[i+1] * cos(angle)); // x
			msg.points[i].velocities.push_back(vel_mag[i+1] * sin(angle)); // y
			msg.points[i].velocities.push_back(0);                         // theta
			msg.points[i].accelerations.push_back(0.); // x
			msg.points[i].accelerations.push_back(0.); // y
			msg.points[i].accelerations.push_back(0);  // theta
		}
		for(size_t j = 0; j < n_joints; j++)
		{
			msg.points.back().velocities.push_back(0);
			msg.points.back().accelerations.push_back(0);
		}
	}
#endif
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

	// Generate matlab / octave code for displaying generated splines
	std::stringstream s;
	s << std::endl;
	for (size_t i = 0; i < out_msg.end_points.size(); i++)
	{
		double range;
		double prev_x;
		if (i == 0)
		{
			range = out_msg.end_points[0];
			prev_x = 0;
		}
		else
		{
			range = out_msg.end_points[i] - out_msg.end_points[i-1];
			prev_x = out_msg.end_points[i-1];
		}
		s << "x" << i << " = " << prev_x << ":" << range / 100. << ":" << out_msg.end_points[i] << ";" << std::endl;;
	}
	s << "x = [";
	for (size_t i = 0; i < out_msg.end_points.size(); i++)
	{
		s << "x" << i;
		if (i < out_msg.end_points.size() - 1)
			s << ", ";
	}
	s << "];" << std::endl;
	s << std::endl;
	printCoefs(s, "x", out_msg.x_coefs);
	printCoefs(s, "y", out_msg.y_coefs);
	printCoefs(s, "orient", out_msg.orient_coefs);
	for (size_t i = 0; i < out_msg.x_coefs.size(); i++)
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
	printPolyval(s, "x", out_msg.x_coefs.size(), out_msg.end_points);
	printPolyval(s, "dx", out_msg.x_coefs.size(), out_msg.end_points);
	printPolyval(s, "ddx", out_msg.x_coefs.size(), out_msg.end_points);
	printPolyval(s, "dddx", out_msg.x_coefs.size(), out_msg.end_points);
	printPolyval(s, "y", out_msg.y_coefs.size(), out_msg.end_points);
	printPolyval(s, "dy", out_msg.y_coefs.size(), out_msg.end_points);
	printPolyval(s, "ddy", out_msg.y_coefs.size(), out_msg.end_points);
	printPolyval(s, "dddy", out_msg.y_coefs.size(), out_msg.end_points);
	printPolyval(s, "orient", out_msg.orient_coefs.size(), out_msg.end_points);
	printPolyval(s, "dorient", out_msg.orient_coefs.size(), out_msg.end_points);
	printPolyval(s, "ddorient", out_msg.orient_coefs.size(), out_msg.end_points);
	printPolyval(s, "dddorient", out_msg.orient_coefs.size(), out_msg.end_points);
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
