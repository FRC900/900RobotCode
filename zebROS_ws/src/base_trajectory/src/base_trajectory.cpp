// base_trajectory.cpp
// Generates a spline-based path connecting input waypoints
// Implements an algorithm similar to that found in
// http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf
// http://www2.informatik.uni-freiburg.de/~lau/paper/lau09iros.pdf
// http://ais.informatik.uni-freiburg.de/teaching/ws09/robotics2/projects/mr2-p6-paper.pdf
//
#include <chrono>
#include <cmath>
#include <cstdio>
#include <angles/angles.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <nlopt.hpp>

#include "base_trajectory_msgs/GenerateSpline.h"
#include "base_trajectory/arclengthtime_to_segtime.h"
#include "base_trajectory/kinematic_constraints.h"
#include "base_trajectory/matlab_printer.h"
#include "base_trajectory/message_filter.h"
#include "base_trajectory/obstacle_calculator.h"
#include "base_trajectory/opt_params_list.h"
#include "base_trajectory/sample_trajectory_impl_cpu.h"
#include "ddynamic_reconfigure/ddynamic_reconfigure.h"
#include "spline_util/spline_util.h"
#include "nav_msgs/Path.h"

// Uncomment to generate Matlab scripts which create a movie of the
// optimization process - one frame per improved path cost
//#define WRITE_OPTIMIZATION_MOVIE

using TrajectoryPointType = double;

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
// closer to it.  midTimeInflation is the multipler for that distance.
double distBetweenArcLengths; // 3 cm
double distBetweenArcLengthEpsilon; // 2.5 mm
double midTimeInflation;

// TODO - these aren't used. They should be similar to the above vars,
// but for when publishing the final path results rather than for internal
// calcs. Currently the above vars are used in both cases
double pathDistBetweenArcLengths; // 30 cm
double pathDistBetweenArcLengthsEpsilon; // 1 cm

// RPROP optimization parameters.  The code skips to the next optimization
// variable if the deltaCost for a step is less than deltaCost.  This value is
// gradually decreased as the result is optimized. This holds the initial and
// minimum value of deltaCost
double initialDeltaCostEpsilon;
double minDeltaCostEpsilon;

// Threshold for counting a new best cost as significant. Additional best
// costs below this threshold will be used, but they won't count towards
// checks to repeat another loop of the same detlta cost epsilon value.
// This prevents the code from microoptimizing for results that can't
// be seen on the robot, e.g. improving a path by 0.01 seconds using
// thousands of iterations of optimization attempts
double initialBestCostEpsilon;

// RPROP - how many improvements in cost before moving on to the next variable
int momentumCounter;

// RPROP - multipler for param regularization cost - the higher this is,
// the more the optimizer will prefer paths with opt params closer to
// their starting values rather than paths with lower times
double regularizationLambda;

// How many consecutive attemps at optimization the resulting cost is
// change is less than the minimum before moving on to the next variable
int maxDeltaCostExitCounter;

// Initial change added to optimization parameter in the RPROP loop
double initialDParam;

// Robot limits for evaluating path cost
double driveBaseRadius;
KinematicConstraints<TrajectoryPointType> kinematicConstraints;

MessageFilter messageFilter(true);
bool writeMatlabFiles{false};

int optimizationCounterMax;

std::shared_ptr<costmap_2d::Costmap2DROS> costmap{nullptr};
std::string pathFrameID;
std::unique_ptr<tf2_ros::Buffer>            tfBuffer;
std::unique_ptr<tf2_ros::TransformListener> tfListener;

std::unique_ptr<ObstacleCalculator<TrajectoryPointType>> obstacleCalculator;

template <class T, size_t O>
void printTrajectory(const Trajectory<T, O> &trajectory, const std::vector<std::string> &jointNames)
{
	// Prevent spending time generating messages which are never actually printed
	if (!messageFilter.isEnabled())
	{
		return;
	}
	const size_t n_joints = jointNames.size();
	for (size_t seg = 0; seg < trajectory[0].size(); seg++)
	{
		for (size_t joint = 0; joint < n_joints; joint++)
		{
			ROS_INFO_STREAM("joint = " << jointNames[joint] << " seg = " << seg <<
			                " start_time = " << trajectory[joint][seg].startTime() <<
			                " end_time = " << trajectory[joint][seg].endTime());
			const auto &coefs = trajectory[joint][seg].getCoefs();

			std::stringstream s;
			s << "coefs ";
			for (size_t i = 0; i < coefs.size(); ++i)
			{
				s << coefs[i] << " ";
			}
			ROS_INFO_STREAM(s.str());
		}
	}
}

// Figure out how far a wheel has to move linearly
// to produce the requested rotation. This is to
// make sure all 3 dimensions of motion are using
// the same units.
// This also works for velocities
template <class T>
T rotationToLinear(const T radians)
{
	// A wheel facing diagonally will produce 2pi radians of
	// robot rotation after moving linearly in a circle of
	// 2*wheelbase_radius * pi.
	// Canceling out 2 pi :
	// 1 radian rotation = 1 wheelbase_radius linear meters
	return driveBaseRadius * radians;
}

// Transform the x,y, theta values encoded in positions using
// the transformStamped passed in as the second argument
bool transformTrajectoryPoint(std::vector<double> &positions,
                              const geometry_msgs::TransformStamped &transformStamped)
{

	geometry_msgs::Pose pose;

	ROS_INFO_STREAM("From pose " << positions[0] << ", " << positions[1] << ", " << positions[2] << " in source frame");
	pose.position.x = positions[0];
	pose.position.y = positions[1];
	pose.position.z = 0;
	tf2::Quaternion quaternion;
	quaternion.setRPY(0,0,positions[2]);
	pose.orientation = tf2::toMsg(quaternion);

	tf2::doTransform(pose, pose, transformStamped);
	positions[0] = pose.position.x;
	positions[1] = pose.position.y;
	const tf2::Quaternion poseQuat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	double roll;
	double pitch;
	tf2::Matrix3x3(poseQuat).getRPY(roll, pitch, positions[2]);

	ROS_INFO_STREAM("  To pose " << positions[0] << ", " << positions[1] << ", " << positions[2] << " in target frame ");
	return true;
}

// Transform the x,y, theta values encoded in positions from
// a source reference frame (included in fromHeader) to a
// different destination frame (in toFrame)
bool transformTrajectoryPoint(std::vector<double> &positions,
							  const std_msgs::Header &fromHeader,
							  const std::string &toFrame)
{
	if (fromHeader.frame_id.empty() || toFrame.empty())
	{
		return true;
	}

	geometry_msgs::PoseStamped poseStamped;

	ROS_INFO_STREAM("From pose " << positions[0] << ", " << positions[1] << ", " << positions[2] << " in frame " << fromHeader.frame_id);
	poseStamped.header = fromHeader;
	poseStamped.pose.position.x = positions[0];
	poseStamped.pose.position.y = positions[1];
	poseStamped.pose.position.z = 0;
	tf2::Quaternion quaternion;
	quaternion.setRPY(0,0,positions[2]);
	poseStamped.pose.orientation = tf2::toMsg(quaternion);

	try
	{
		poseStamped = tfBuffer->transform(poseStamped, toFrame);
	}
	catch(tf2::TransformException &ex)
	{
		ROS_ERROR_STREAM("base_trajectory : Error transforming frame from " << fromHeader.frame_id << " to " << toFrame << " : " << ex.what());
		return false;
	}
	positions[0] = poseStamped.pose.position.x;
	positions[1] = poseStamped.pose.position.y;
	const tf2::Quaternion poseQuat(poseStamped.pose.orientation.x, poseStamped.pose.orientation.y, poseStamped.pose.orientation.z, poseStamped.pose.orientation.w);
	double roll;
	double pitch;
	tf2::Matrix3x3(poseQuat).getRPY(roll, pitch, positions[2]);

	ROS_INFO_STREAM("  To pose " << positions[0] << ", " << positions[1] << ", " << positions[2] << " in frame " << toFrame);
	return true;
}

bool transformConstraintPoint(geometry_msgs::Point &point,
							  const std_msgs::Header &fromHeader,
							  const std::string &toFrame)
{

	std::vector<double> position;
	position.push_back(point.x);
	position.push_back(point.y);
	position.push_back(0);
	if (!transformTrajectoryPoint(position, fromHeader, toFrame))
	{
		return false;
	}
	point.x = position[0];
	point.y = position[1];
	return true;
}

bool transformConstraintPoint(geometry_msgs::Point &point,
                              const geometry_msgs::TransformStamped &transformStamped)
{

	std::vector<double> position;
	position.push_back(point.x);
	position.push_back(point.y);
	position.push_back(0);
	if (!transformTrajectoryPoint(position, transformStamped))
	{
		return false;
	}
	point.x = position[0];
	point.y = position[1];
	return true;
}


// Helper function for finding 2nd derivative
// (acceleration) term of start and end point
template <class T, size_t Joints>
void setFirstLastPointAcceleration(const std::vector<double> &accelerations,
								   const std::array<trajectory_interface::SegmentState<T>, Joints> &state1,
								   const std::array<trajectory_interface::SegmentState<T>, Joints> &state2,
								   std::array<trajectory_interface::SegmentState<T>, Joints> &state)
{
	if (accelerations.size() >= Joints)
	{
		for (size_t i = 0; i < Joints; i++)
		{
			state[i].acceleration = accelerations[i];
		}
		return;
	}
	// Rename vars to match notation in the paper
	const T Ax  = state1[0].position;
	const T Ay  = state1[1].position;
	const T At  = state1[2].position;
	const T tAx = state1[0].velocity;
	const T tAy = state1[1].velocity;
	const T tAt = state1[2].velocity;
	const T Bx  = state2[0].position;
	const T By  = state2[1].position;
	const T Bt  = state2[2].position;
	const T tBx = state2[0].velocity;
	const T tBy = state2[1].velocity;
	const T tBt = state2[2].velocity;

	const T xaccel = 6.0 * Ax + 2.0 * tAx + 4.0 * tBx - 6.0 * Bx;
	const T yaccel = 6.0 * Ay + 2.0 * tAy + 4.0 * tBy - 6.0 * By;
	const T taccel = 6.0 * At + 2.0 * tAt + 4.0 * tBt - 6.0 * Bt;

	// Don't overwrite requested input accelerations
	if (accelerations.empty())
	{
		state[0].acceleration = xaccel;
	}
	else
	{
		state[0].acceleration = accelerations[0];
	}
	if (accelerations.size() <= 1)
	{
		state[1].acceleration = yaccel;
	}
	else
	{
		state[1].acceleration = accelerations[1];
	}
	if (accelerations.size() <= 2)
	{
		state[2].acceleration = taccel;
	}
	else
	{
		state[2].acceleration = accelerations[2];
	}
}

// Find the angle that line p1p2 is pointing at
template <class T>
T getLineAngle(const std::vector<T> &p1, const std::vector<T> &p2)
{
	return angles::normalize_angle_positive(atan2(p2[1] - p1[1], p2[0] - p1[0]));
}


// Generate a spline which hits the positions / velocities / accelerations
// defined in points + optParams, and return the trajectory in
// trajectory
template <class T>
bool generateSpline(const std::vector<trajectory_msgs::JointTrajectoryPoint> &points,
					const OptParamsList<T> &optParams,
					const std::vector<std::string> &jointNames,
					XYTTrajectory<T> &trajectory)
{
	std::vector<InitSplinePoints<T, 3>> initSplinePoints(points.size());

	// x, y, theta position/velocity/acceleration
	const size_t nJoints = jointNames.size();

	// Offset x and y by the amount determined by the optimizer
	// Paper says to make the coord system based off the
	// tangent direction (perp & parallel to tangent)
	// We'll see if that makes a difference
	for (size_t i = 0; i < points.size(); i++)
	{
		initSplinePoints[i].time_from_start = points[i].time_from_start.toSec();
		initSplinePoints[i].state[0].position = points[i].positions[0] + optParams[i].posX_;
		initSplinePoints[i].state[1].position = points[i].positions[1] + optParams[i].posY_;
		initSplinePoints[i].state[2].position = points[i].positions[2];
		//points[i].positions[0] += optParams[i].posX_;
		//points[i].positions[1] += optParams[i].posY_;
	}

	initSplinePoints[0].state[0].velocity = points[0].velocities[0];
	initSplinePoints[0].state[1].velocity = points[0].velocities[1];
	initSplinePoints[0].state[2].velocity = points[0].velocities[2];

	initSplinePoints.back().state[0].velocity = points.back().velocities[0];
	initSplinePoints.back().state[1].velocity = points.back().velocities[1];
	initSplinePoints.back().state[2].velocity = points.back().velocities[2];
	// Auto - generate velocities and accelerations for splines
	// based on a simple heuristic. This is becoming less simple
	// by the moment.
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
	T prevAngle = getLineAngle(points[0].positions, points[1].positions);
	T prevLength = hypot(points[1].positions[0] - points[0].positions[0],
						 points[1].positions[1] - points[0].positions[1]);

	T prevRotLength = points[1].positions[2] - points[0].positions[2];

	for (size_t i = 1; i < (points.size() - 1); i++)
	{
		const auto &mi   = points[i].positions;
		const auto &mip1 = points[i + 1].positions;

		const T currAngle = getLineAngle(mi, mip1);
		T deltaAngle = currAngle - prevAngle;
		if (deltaAngle < -M_PI)
		{
			deltaAngle += 2.0 * M_PI;
		}
		else if (deltaAngle > M_PI)
		{
			deltaAngle -= 2.0 * M_PI;
		}
		const T angle = angles::normalize_angle_positive(prevAngle + deltaAngle / 2.0);

		const T currLength = hypot(mip1[0] - mi[0], mip1[1] - mi[1]);
		const T currRotLength = mip1[2] - mi[2];

		// Adding a scaling factor here controlling the velocity
		// at the waypoints.  Bigger than 1 ==> curvier path with
		// higher speeds.  Less than 1 ==> tigher turns to stay
		// closer to straight paths.
		const T length = std::min(prevLength, currLength) * optParams[i].lengthScale_ + optParams[i].length_;
		const T rotLength = (prevRotLength + currRotLength) / 2.0 * optParams[i].rotLengthScale_ + optParams[i].rotLength_;

		// Don't overwrite requested input velocities
		if (points[i].velocities.empty())
		{
			initSplinePoints[i].state[0].velocity = length * cos(angle);
		}
		else
		{
			initSplinePoints[i].state[0].velocity = points[i].velocities[0];
		}

		if (points[i].velocities.size() <= 1)
		{
			initSplinePoints[i].state[1].velocity = length * sin(angle);
		}
		else
		{
			initSplinePoints[i].state[1].velocity = points[i].velocities[1];
		}

		if (points[i].velocities.size() <= 2)
		{
			initSplinePoints[i].state[2].velocity = rotLength;
		}
		else
		{
			initSplinePoints[i].state[2].velocity = points[i].velocities[2];
		}

		if (optParams[i].deltaVMagnitude_)
		{
			initSplinePoints[i].state[0].velocity += optParams[i].deltaVMagnitude_ * cos(angle + optParams[i].deltaVDirection_);
			initSplinePoints[i].state[1].velocity += optParams[i].deltaVMagnitude_ * sin(angle + optParams[i].deltaVDirection_);
		}
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
	// Save some typing
	auto &pb = initSplinePoints.back();
	const auto opb = optParams.back();
	// prevAngle after exiting the loop is the direction from the 2nd to last to the
	// last point.  Decompose the optParms dv magnitude into x and y components
	// and set it here.
	// TODO - this overwrites the previous values, which might be weird if users specified
	// a final velocity?
	pb.state[0].velocity = opb.deltaVMagnitude_ * cos(prevAngle + opb.deltaVDirection_); // x
	pb.state[1].velocity = opb.deltaVMagnitude_ * sin(prevAngle + opb.deltaVDirection_); // y
	pb.state[2].velocity = 0; // theta TODO : anything to do here?
	//ROS_INFO_STREAM("pb.velocities = " << pb.state[0].velocity << ", " << pb.state[1].velocity << " opb.deltaVMagnitude_= " << opb.deltaVMagnitude_
			//<< " opb.deltaVDirection_=" << opb.deltaVDirection_);

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
	setFirstLastPointAcceleration(points[0].accelerations,
			initSplinePoints[1].state,
			initSplinePoints[0].state,
			initSplinePoints[0].state);

	const size_t last = points.size() - 1;
	setFirstLastPointAcceleration(points[last].accelerations,
			initSplinePoints[last-1].state,
			initSplinePoints[last].state,
			initSplinePoints[last].state);

	// For interior points, weight the average of the
	// 2nd derivative of each of the pretend cubic
	// splines and use them as the acceleration for
	// the to-be-generated quintic spline.
	for (size_t i = 1; i < (points.size() - 1); i++)
	{
		if (points[i].accelerations.size() >= nJoints)
		{
			for (size_t j = 0; j < nJoints; j++)
			{
				initSplinePoints[i].state[j].acceleration = points[i].accelerations[j];
			}
			continue;
		}

		const T Ax = initSplinePoints[i-1].state[0].position;
		const T Ay = initSplinePoints[i-1].state[1].position;
		const T At = initSplinePoints[i-1].state[2].position;
		const T tAx = initSplinePoints[i-1].state[0].velocity;
		const T tAy = initSplinePoints[i-1].state[1].velocity;
		const T tAt = initSplinePoints[i-1].state[2].velocity;

		const T Bx = initSplinePoints[i].state[0].position;
		const T By = initSplinePoints[i].state[1].position;
		const T Bt = initSplinePoints[i].state[2].position;
		const T tBx = initSplinePoints[i].state[0].velocity;
		const T tBy = initSplinePoints[i].state[1].velocity;
		const T tBt = initSplinePoints[i].state[2].velocity;

		const T Cx = initSplinePoints[i+1].state[0].position;
		const T Cy = initSplinePoints[i+1].state[1].position;
		const T Ct = initSplinePoints[i+1].state[2].position;
		const T tCx = initSplinePoints[i+1].state[0].velocity;
		const T tCy = initSplinePoints[i+1].state[1].velocity;
		const T tCt = initSplinePoints[i+1].state[2].velocity;

		// L2 distance between A and B
		const T dab = hypot(Bx - Ax, By - Ay, rotationToLinear(Bt - At));
		// L2 distance between B and C
		const T dbc = hypot(Cx - Bx, Cy - By, rotationToLinear(Ct - Bt));

		// Weighting factors
		const T alpha = dbc / (dab + dbc);
		const T beta  = dab / (dab + dbc);

		const T xaccel = alpha * ( 6.0 * Ax + 2.0 * tAx + 4.0 * tBx - 6.0 * Bx) +
						 beta  * (-6.0 * Bx - 4.0 * tBx - 2.0 * tCx + 6.0 * Cx);
		const T yaccel = alpha * ( 6.0 * Ay + 2.0 * tAy + 4.0 * tBy - 6.0 * By) +
						 beta  * (-6.0 * By - 4.0 * tBy - 2.0 * tCy + 6.0 * Cy);
		const T taccel = alpha * ( 6.0 * At + 2.0 * tAt + 4.0 * tBt - 6.0 * Bt) +
						 beta  * (-6.0 * Bt - 4.0 * tBt - 2.0 * tCt + 6.0 * Ct);

		// Don't overwrite requested input accelerations
		if (points[i].accelerations.empty())
		{
			initSplinePoints[i].state[0].acceleration = xaccel;
		}
		else
		{
			initSplinePoints[i].state[0].acceleration = points[i].accelerations[0];
		}
		if (points[i].accelerations.size() <= 1)
		{
			initSplinePoints[i].state[1].acceleration = yaccel;
		}
		else
		{
			initSplinePoints[i].state[1].acceleration = points[i].accelerations[1];
		}
		if (points[i].accelerations.size() <= 2)
		{
			initSplinePoints[i].state[2].acceleration = taccel;
		}
		else
		{
			initSplinePoints[i].state[2].acceleration = points[i].accelerations[2];
		}

#if 0
		ROS_INFO_STREAM("dab = " << dab << " dbc = " << dbc);
		ROS_INFO_STREAM("xaccel = " << xaccel << " yaccel = " << yaccel << " taccel = " << taccel);
		ROS_INFO_STREAM("Ax = " << Ax << " tAx = " << tAx <<
						" Bx = " << Bx << " tBx = " << tBx <<
						" Cx = " << Cx << " tCx = " << tCx);
		ROS_INFO_STREAM("Ay = " << Ay << " tAy = " << tAy <<
						" By = " << By << " tBy = " << tBy <<
						" Cy = " << Cy << " tCy = " << tCy);
		ROS_INFO_STREAM("At = " << At << " tAt = " << tAt <<
						" Bt = " << Bt << " tBt = " << tBt <<
						" Ct = " << Ct << " tCt = " << tCt);
#endif
	}

	//ROS_WARN_STREAM("----  trajectory---- ");
	if (!initSpline(trajectory, initSplinePoints))
	{
		return false;
	}

	printTrajectory(trajectory, jointNames);
	//ROS_WARN_STREAM("----  trajectory---- ");
	return true;
}

template <class T>
T distSquared(const std::array<T, 2> &v, const std::array<T, 2> &w)
{
	return ((v[0] - w[0]) * (v[0] - w[0])) + ((v[1] - w[1]) * (v[1] - w[1]));
}

template <class T>
T distSquared(const T px, const T py, const std::array<T, 2> &w)
{
	return ((px - w[0]) * (px - w[0])) + ((py - w[1]) * (py - w[1]));
}

template <class T>
// Minimum distance between segment vw and point (p1, p2)
T pointToLineSegmentDistance(const std::array<T, 2> &v, const std::array<T, 2> &w,
		T px, T py)
{
	const auto l2 = distSquared(v, w);
	if (l2 == static_cast<T>(0.0))
	{
		return sqrt(distSquared(px, py, v));   // v == w case, distance to single point
	}
	// Consider the line extending the segment, parameterized as v + t (w - v).
	// We find projection of point p onto the line.
	// It falls where t = [(p-v) . (w-v)] / |w-v|^2
	// We clamp t from [0,1] to handle points outside the segment vw.
	const auto t = std::max(static_cast<T>(0.0),
			std::min(static_cast<T>(1.0), ((px - v[0]) * (w[0] - v[0]) + (py - v[1]) * (w[1] - v[1])) / static_cast<T>(l2)));
	const auto projectionX = v[0] + t * (w[0] - v[0]);
	const auto projectionY = v[1] + t * (w[1] - v[1]);
	return hypot(px - projectionX, py - projectionY);
}

// Use Simpson's rule to estimate arc length between start and
// and time.  Function is combining dx and dy into a length,
// so use hypot(dx,dy) as the function being evaluated.
// Get the upper bound of the error as well - this lets
// us know if the estimate is within the bounds specified
// for calculating the estimate.
template <class T, size_t O>
void simpsonsRule(T &estimate, T &error,
				  const T startT, const T endT,
				  const typename Segment<T, O>::SplineCoefficients &xStartCoefs,
				  const typename Segment<T, O>::SplineCoefficients &yStartCoefs,
				  const typename Segment<T, O>::SplineCoefficients &oStartCoefs,
				  const typename Segment<T, O>::SplineCoefficients &xEndCoefs,
				  const typename Segment<T, O>::SplineCoefficients &yEndCoefs,
				  const typename Segment<T, O>::SplineCoefficients &oEndCoefs,
				  const T startXdot, const T startYdot, const T startOdot,
				  const T midXdot, const T midYdot, const T midOdot,
				  const T endXdot, const T endYdot, const T endOdot)
{
	const T periodT = endT - startT;
	estimate = periodT / 6.0 * (hypot(startXdot, startYdot, startOdot) + 4.0 * hypot(midXdot, midYdot, midOdot) + hypot(endXdot, endYdot, endOdot));

	const T periodTOver2 = periodT / 2.0;
	const T periodTOver2Squared = periodTOver2 * periodTOver2;
	const T periodTOver2Fifth = periodTOver2Squared * periodTOver2Squared * periodTOver2;
	//const T commonErrorTerm = (1.0/90.0) * pow(periodT / 2.0, 5.0);
	const T commonErrorTerm = (1.0/90.0) * periodTOver2Fifth;
	// Error term is a line (mx+b) so min/max is going to be at one end
	// or the other.
	const T startError = hypot(120.0 * xStartCoefs[5] * startT + 24.0 * xStartCoefs[4],
							   120.0 * yStartCoefs[5] * startT + 24.0 * yStartCoefs[4],
							   120.0 * oStartCoefs[5] * startT + 24.0 * oStartCoefs[4]);
	const T endError = hypot(120.0 * xEndCoefs[5] * endT + 24.0 * xEndCoefs[4],
							 120.0 * yEndCoefs[5] * endT + 24.0 * yEndCoefs[4],
							 120.0 * oEndCoefs[5] * endT + 24.0 * oEndCoefs[4]);

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
// Returns points, a vector of points defining the
// arclength curve
template <class T, size_t O>
bool getPathSegLength(std::vector<InitSplinePoints<T, 1>> &points,
		const Trajectory<T, O> &trajectory,
		T &totalLength,
		typename TrajectoryPerJoint<T, O>::const_iterator startXIt,
		typename TrajectoryPerJoint<T, O>::const_iterator startYIt,
		typename TrajectoryPerJoint<T, O>::const_iterator startOIt,
		typename TrajectoryPerJoint<T, O>::const_iterator endXIt,
		typename TrajectoryPerJoint<T, O>::const_iterator endYIt,
		typename TrajectoryPerJoint<T, O>::const_iterator endOIt,
		const T startTime,
		const T endTime,
		const T startXdot,
		const T startYdot,
		const T startOdot,
		const T endXdot,
		const T endYdot,
		const T endOdot)
{
	const T midTime = (startTime + endTime) / 2.0;

	// TODO : Perhaps a check on dT being too small?

	// Simpson's rule needs the provided start and end values and
	// also one at the midpoint. Grab the midpoint values here
	T midXdot;
	const auto midXIt = sampleVelocity(trajectory[0], midTime, midXdot);
	if (midXIt == trajectory[0].cend())
	{
		ROS_ERROR_STREAM("base_trajectory : could not sample mid xState at time " << midTime);
		return false;
	}
	const auto itOffset = midXIt - trajectory[0].cbegin();

	T midYdot;
	const auto midYIt = trajectory[1].cbegin() + itOffset;
	midYIt->sampleVelocity(midTime, midYdot);

	T midOdot;
	const auto midOIt = trajectory[2].cbegin() + itOffset;
	midOIt->sampleVelocity(midTime, midOdot);
	midOdot = 0; // TODO - this breaks linear path distance, not sure why rotationToLinear(midOdot);

	T estimate;
	T error;

	simpsonsRule<T, O>(estimate, error,
				 startTime, endTime,
				 startXIt->getCoefs(), startYIt->getCoefs(), startOIt->getCoefs(),
				 endXIt->getCoefs(), endYIt->getCoefs(), endOIt->getCoefs(),
				 startXdot, startYdot, startOdot,
				 midXdot, midYdot, midOdot,
				 endXdot, endYdot, endOdot);
	//ROS_INFO_STREAM_FILTER(&messageFilter, "simpsonsRule : startTime = " << startTime << " endTime = " << endTime << " error = " << error);

	// If the error magnitude is less than epsilon,
	// use this approximation for the arcLength from
	// start to end
	if (fabs(error) < static_cast<T>(segLengthEpsilon))
	{
		totalLength += estimate;
		points.push_back(InitSplinePoints<T,1>());
		points.back().state[0].position = totalLength;
		points.back().time_from_start = endTime;
		return true;
	}

	// Otherwise, split segment in half
	// and recursively calculate the length of each half.
	if (!getPathSegLength(points, trajectory, totalLength,
						  startXIt, startYIt, startOIt,
						  midXIt, midYIt, midOIt,
						  startTime, midTime,
						  startXdot, startYdot, startOdot,
						  midXdot, midYdot, midOdot))
	{
		return false;
	}
	if (!getPathSegLength(points, trajectory, totalLength,
						  midXIt, midYIt, midOIt,
						  endXIt, endYIt, endOIt,
						  midTime, endTime,
						  midXdot, midYdot, midOdot,
						  endXdot, endYdot, endOdot))
	{
		return false;
	}
	return true;
}

template <class T>
bool getPathLength(ArcLengthTrajectory<T> &arcLengthTrajectory,
				   const XYTTrajectory<T> &trajectory)
{
	// Get initial conditions for getPathSegLength call
	// for this particular segment
	T startXdot;
	const auto startXIt = sampleVelocity(trajectory[0], 0, startXdot);
	if (startXIt == trajectory[0].cend())
	{
		ROS_ERROR_STREAM("base_trajectory : could not sample initial xState 0");
		return false;
	}

	T startYdot;
	const auto startYIt = sampleVelocity(trajectory[1], 0, startYdot);
	if (startYIt == trajectory[1].cend())
	{
		ROS_ERROR_STREAM("base_trajectory : could not sample initial yState 0");
		return false;
	}

	T startOdot;
	const auto startOIt = sampleVelocity(trajectory[2], 0, startOdot);
	if (startOIt == trajectory[2].cend())
	{
		ROS_ERROR_STREAM("base_trajectory : could not sample initial oState 0");
		return false;
	}
	startOdot = 0; // TODO - this breaks linear path distance, not sure why rotationToLinear(startOdot);

	const T endTime = trajectory[0].back().endTime();
	T endXdot;
	const auto endXIt = sampleVelocity(trajectory[0], endTime, endXdot);
	if (endXIt == trajectory[0].cend())
	{
		ROS_ERROR_STREAM("base_trajectory : could not sample initial xState end");
		return false;
	}
	T endYdot;
	const auto endYIt = sampleVelocity(trajectory[1], endTime, endYdot);
	if (endYIt == trajectory[1].cend())
	{
		ROS_ERROR_STREAM("base_trajectory : could not sample initial yState end");
		return false;
	}
	T endOdot;
	const auto endOIt = sampleVelocity(trajectory[2], endTime, endOdot);
	if (endOIt == trajectory[2].cend())
	{
		ROS_ERROR_STREAM("base_trajectory : could not sample initial oState end");
		return false;
	}
	endOdot = 0; // TODO - this breaks linear path distance, not sure why rotationToLinear(endOdot);

	T totalLength = 0.0;

	// Start with initial position 0 at time 0
	std::vector<InitSplinePoints<T, 1>> points;
	points.push_back(InitSplinePoints<T,1>());
	points[0].state[0].position = 0.;
	points[0].time_from_start = 0.;

	if (!getPathSegLength(points, trajectory, totalLength,
			startXIt, startYIt, startOIt,
			endXIt, endYIt, endOIt,
			static_cast<T>(0), endTime, // start, end time
			startXdot, startYdot, startOdot,
			endXdot, endYdot, endOdot))
	{
		return false;
	}

	// Create a path between each of these length/time
	// coordinates using cubic splines to interpolate
	// between each point
	if constexpr (ARCLENGTH_TRAJECTORY_ORDER > 1)
	{
		points[0].state[0].velocity = 0;
		T prevLength = fabs(points[1].state[0].position - points[0].state[0].position);
		for (size_t i = 1; i < (points.size() - 1); i++)
		{
			const T currLength = fabs(points[i+1].state[0].position - points[i].state[0].position);

			points[i].state[0].velocity = std::min(prevLength, currLength);
			prevLength = currLength;
		}
		points.back().state[0].velocity = 0;
	}

	//ROS_WARN_STREAM("----  arcLengthTrajectory ---- ");
	if (!initSpline(arcLengthTrajectory, points))
	{
		return false;
	}

	static const std::vector<std::string> jointNames = { "arcLength" };
	printTrajectory(arcLengthTrajectory, jointNames);
	//ROS_WARN_STREAM("----  arcLengthTrajectory ---- ");

	return true;
}

template <class T>
void pathDistanceWorker(std::vector<T> &arcSegCostsPathDistance,
						const std::vector<Kinematics<T>> &kinematics,
						const std::vector<T> &equalArcLengthTimes,
						const XYTTrajectory<T> &trajectory,
						const std::vector<SegmentState<T>> &xStates,
						const std::vector<SegmentState<T>> &yStates)
{
	// Grab positions from each of the control points on the trajectory
	// Since the trajectory is still in arbitrary-time that means every integer
	//   value of time from 0 to the last time in equalArcLengthTimes
	static std::vector<std::array<T, 2>> controlPointPositions;
	controlPointPositions.clear();
	for (size_t i = 0; i <= static_cast<size_t>(equalArcLengthTimes.back()); i++)
	{
		controlPointPositions.emplace_back(std::array<T, 2>());
		for (size_t j = 0; j <= 1; j++)
		{
			const auto it = samplePosition(trajectory[j], i, controlPointPositions[i][j]);
			if (it >= trajectory[j].cend())
			{
				ROS_ERROR_STREAM("base_trajectory : evaluateTrajectory could not sample control point state " << i);
			}
		}
	}
	arcSegCostsPathDistance.resize(equalArcLengthTimes.size());
	arcSegCostsPathDistance[0] = 0;
	for (size_t i = 1; i < equalArcLengthTimes.size(); i++)
	{
		const auto seg = arcLengthTimeToSegTime(equalArcLengthTimes, i);
		const auto xPos = xStates[i].position;
		const auto yPos = yStates[i].position;
		// Get orthogonal distance between spline position and
		// line segment connecting the corresponding waypoints
		const auto distanceToPathMidpoint =
			pointToLineSegmentDistance(controlPointPositions[seg], controlPointPositions[seg + 1], xPos, yPos);
		//ROS_INFO_STREAM_FILTER(&messageFilter, "pointToLineSegmentDistance() = " << distanceToPathMidpoint <<
		//		" p1: " << controlPointPositions[seg][0] << "," << controlPointPositions[seg][1] <<
		//		" p2: " << controlPointPositions[seg+1][0] << "," << controlPointPositions[seg+1][1] <<
		//		" x: " << xPos << " y: " << yPos);
		//
		// Add a large penalty for exceeding the distance allowed
		// for this particular part of the path
		arcSegCostsPathDistance[i] = expf(static_cast<T>(25.0) * ((distanceToPathMidpoint / kinematics[i].getPathLimitDistance()) - static_cast<T>(0.9)));
	}
}

template <class T>
struct SegCosts
{
	std::vector<T> time;
	std::vector<T> obstacles;
	std::vector<T> pathDistance;
	std::vector<T> regularization;
};

// evaluate trajectory - inputs are spline coeffs (Trajectory), plus various kinematic constraints
//                   Creates a velocity profile bound by those constraints
//                   returns cost for the path in cost, true if the evaluation succeeds and false
//                   if it fails
//
// equalArcLengthTimes is a vector of times. Each entry is the time
// of an equally-spaced sample from arcLengthTrajectory. That is
// sample i is arclength d away from both samples i-1 and i+1 for all i.
//#define VERBOSE_EVALUATE_TRAJECTORY
template <class T>
bool evaluateTrajectory(T &cost,
						SegCosts<T> &segCosts,
						std::vector<T> &equalArcLengthTimes,
						std::vector<T> &remappedTimes,
						std::vector<T> &vTrans,
						const XYTTrajectory<T> &trajectory,
						const geometry_msgs::TransformStamped &pathToMapTransform,
						SampleTrajectoryImplCpu<T> &sampleTrajectory,
						const OptParamsList<T> &optParams)
{
	// arcLengthTrajectory takes in a time and returns the x-y-theta distance
	// traveled up to that time.
	static ArcLengthTrajectory<T> arcLengthTrajectory;
	if (!getPathLength(arcLengthTrajectory, trajectory))
	{
		ROS_ERROR_STREAM("base_trajectory_node : evaluateTrajectory -> getPathLength() failed");
		return false;
	}
	// equalArcLengthTimes will contain times that are
	// spaced equidistant along the arc length passed in
	static std::vector<T> equalArcLengthPositions;
	static std::vector<SegmentState<T>> xStates;
	static std::vector<SegmentState<T>> yStates;
	static std::vector<SegmentState<T>> tStates;
	if (!sampleTrajectory.sample(equalArcLengthTimes, equalArcLengthPositions,
								 xStates, yStates, tStates,
								 trajectory, arcLengthTrajectory))
	{
		ROS_ERROR_STREAM("evaluateTrajectory :: sample() failed");
		return false;
	}

#ifdef VERBOSE_EVALUATE_TRAJECTORY
	for (size_t i = 0; i < equalArcLengthTimes.size(); i++)
		ROS_INFO_STREAM("equalArcLength[" << i << "] : time = " << equalArcLengthTimes[i]
				<< ", position = " << equalArcLengthPositions[i]);
#endif

	// Init translational velocity from requested starting x&y&rotational velocities
	vTrans.clear();
	vTrans.push_back(hypot(xStates[0].velocity, yStates[0].velocity, rotationToLinear(tStates[0].velocity)));

	// Add 0th entries for arrays so indices line up with equalArcLengthTimes
	// Starting at arclength 0
	static std::vector<T> deltaS;   // change in position along spline for each step
	deltaS.resize(equalArcLengthTimes.size());
	deltaS[0] = 0;

	std::vector<Kinematics<T>> kinematics;
	kinematics.emplace_back(kinematicConstraints.getKinematics(0,0));

	static std::vector<T> arcSegCostsObstacles;
	obstacleCalculator->calculateCosts(arcSegCostsObstacles, pathToMapTransform, xStates, yStates);

	for (size_t i = 1; i < equalArcLengthTimes.size(); i++)
	{
		const auto seg = arcLengthTimeToSegTime(equalArcLengthTimes, i);
		//ROS_INFO_STREAM_FILTER(&messageFilter, "processing index " << i << " t=" << equalArcLengthTimes[i] <<" seg=" << seg);

		// Save distance between prev and current position for this timestep
		// This should be nearly constant between points, but
		// get the exact value for each to be more precise
		deltaS[i] = equalArcLengthPositions[i] - equalArcLengthPositions[i-1];
#ifdef VERBOSE_EVALUATE_TRAJECTORY
		//ROS_INFO_STREAM("deltaS[" << i << "]=" << deltaS[i]);
#endif

		const auto xPos = xStates[i].position;
		const auto yPos = yStates[i].position;
#ifdef VERBOSE_EVALUATE_TRAJECTORY
		ROS_INFO_STREAM("[" << i << "] = x: " << xPos << " y: " << yPos);
#endif
		kinematics.emplace_back(kinematicConstraints.getKinematics(xPos, yPos));

		// Get curvature for this sample
		// https://en.wikipedia.org/wiki/Curvature#General_expressions
		const auto pXdot    = xStates[i].velocity;
		const auto pXdotdot = xStates[i].acceleration;
		const auto pYdot    = yStates[i].velocity;
		const auto pYdotdot = yStates[i].acceleration;
		const auto pTdot    = rotationToLinear(tStates[i].velocity);
		const auto pTdotdot = rotationToLinear(tStates[i].acceleration);
		const auto num =
				(pTdotdot * pYdot - pYdotdot * pTdot) * (pTdotdot * pYdot - pYdotdot * pTdot) +
				(pXdotdot * pTdot - pTdotdot * pXdot) * (pXdotdot * pTdot - pTdotdot * pXdot) +
				(pYdotdot * pXdot - pXdotdot * pYdot) * (pYdotdot * pXdot - pXdotdot * pYdot);
		const auto denom = pXdot * pXdot + pYdot * pYdot + pTdot * pTdot;
		T curvature;
		if (num && denom) // avoid divide by zero
		{
			curvature = sqrt(num  / (denom * denom * denom));
		}
		else
		{
			curvature = 0;
		}
#if 0
#ifdef VERBOSE_EVALUATE_TRAJECTORY
		ROS_INFO_STREAM_FILTER(&messageFilter, "pXdot = " << pXdot << ", pXdotdot = " << pXdotdot << ", pYdot = " << pYdot << ", pYdotdot = " << pYdotdot
				<< ", pTdot = " << pTdot << ", pTdotdot = " << pTdotdot << ", num = " << num << ", denom = " << denom);
#endif
#endif

		// First pass of vTrans limits by absolute max velocity
		// and also velocity limited by max centripetal acceleration
		// Convert rotational velocity to linear velocity needed to
		// achieve that rotation and include it as part of "translating" through
		// a 3-d path
		const auto &k = kinematics.back();
		// There's lots of sqrt(vel) followed by equations which use vel^2
		// so use an optimization where, until then end of the process,
		// vTrans is actually vTrans^2.
		vTrans.push_back(k.getMaxVel() * k.getMaxVel());
		if (curvature != 0) // avoid divide by 0 again
		{
			vTrans.back() = std::min(vTrans.back(), k.getMaxCentAccel() / curvature);
		}
#ifdef VERBOSE_EVALUATE_TRAJECTORY
		ROS_INFO_STREAM_FILTER(&messageFilter, "vTrans0[" << i << "]=" << equalArcLengthTimes[i] << "," << vTrans[i] << " maxCentAccel = " << k.getMaxCentAccel() << " curvature = " << curvature);
#endif
	}

	static std::vector<T> arcSegCostsPathDistance;
	pathDistanceWorker(arcSegCostsPathDistance, kinematics, equalArcLengthTimes, trajectory, xStates, yStates);

	// Forward pass
	// Start with initial velocity at point 0. Make sure subsequent points'
	// velocities don't exceed the acceleration limit defined by the kinematic constraints
	for (size_t i = 1; i < vTrans.size(); i++)
	{
		vTrans[i] = std::min(vTrans[i], vTrans[i - 1] + static_cast<T>(2.0) * kinematics[i].getMaxAccel() * deltaS[i]);
#ifdef VERBOSE_EVALUATE_TRAJECTORY
		ROS_INFO_STREAM_FILTER(&messageFilter, "vTrans1[" << i << "]=" << equalArcLengthTimes[i] << "," << vTrans[i]);
#endif
	}

	// Backwards pass
	// Grab lateral velocity from end point as starting condition for backwards iteration
	// Then iterate backwards through the velocities, limiting them to ones which can
	// don't exceed the deceleration limit imposed by the kinematic constraints
	const auto xEndVel = xStates.back().velocity;
	const auto yEndVel = yStates.back().velocity;
	const auto tEndVel = rotationToLinear(tStates.back().velocity);
	vTrans.back() = xEndVel * xEndVel + yEndVel * yEndVel + tEndVel * tEndVel;

#ifdef VERBOSE_EVALUATE_TRAJECTORY
	ROS_INFO_STREAM_FILTER(&messageFilter, "vTrans2[" << vTrans.size()-1 << "]=" << equalArcLengthTimes[vTrans.size()-1] << "," << vTrans[vTrans.size()-1]);
#endif
	for (size_t i = vTrans.size() - 2; i > 0; i--)
	{
		vTrans[i] = std::min(vTrans[i], vTrans[i + 1] + static_cast<T>(2.0) * kinematics[i].getMaxDecel() * deltaS[i + 1]);
#ifdef VERBOSE_EVALUATE_TRAJECTORY
		ROS_INFO_STREAM_FILTER(&messageFilter, "vTrans2[" << i << "]=" << equalArcLengthTimes[i] << "," << vTrans[i]);
#endif
	}

	// After all the passes over vTrans are finished, convert from vTrans^2 to vTrans
	for (auto &v : vTrans)
		v = sqrt(v);

	// Initialze cost function vars
	std::vector<T> arcSegCostsTime(equalArcLengthTimes.size());

	// Calculate arrival time at each of the equalLengthArcTimes distances
	remappedTimes.clear();
	remappedTimes.push_back(0);
	for (size_t i = 1; i < vTrans.size(); i++)
	{
		remappedTimes.push_back(remappedTimes.back() + (static_cast<T>(2.0) * deltaS[i]) / (vTrans[i - 1] + vTrans[i]));
		arcSegCostsTime[i] = remappedTimes[i] - remappedTimes[i-1];
#ifdef VERBOSE_EVALUATE_TRAJECTORY
		ROS_INFO_STREAM_FILTER(&messageFilter, "deltaS[i] = " << deltaS[i] << " vTrans[i-1] = " << vTrans[i-1] << " vTrans[i] = " << vTrans[i]);
		ROS_INFO_STREAM_FILTER(&messageFilter, "remappedTimes[" << i << "]=" << remappedTimes[i]);
		ROS_INFO_STREAM_FILTER(&messageFilter, "arcSegCostsTime[" << i << "]=" << arcSegCostsTime.back());
#endif
	}

	//ROS_INFO_STREAM_FILTER(&messageFilter, " total obstacle cost = " << cost);

	// Cost is total time to traverse the path plus a large
	// penalty for moving more than distMax away from the midpoint of the
	// straight line segment connecting each waypoint. The latter
	// imposes a constraint that the path can't be too curvy - and
	// keeping close to the straight-line path should prevent it from running
	// into obstacles too far off that path.

	// Generate a cost-per-spline-segment vector
	cost = remappedTimes.back();
	segCosts.regularization.clear();
	for (const auto &op : optParams)
	{
		segCosts.regularization.push_back(op.regularizationCost(regularizationLambda));
		cost += segCosts.regularization.back();
	}

	const size_t segCostsLength = std::floor(equalArcLengthTimes.back());
	segCosts.time           = std::vector<T>(segCostsLength, 0.0);
	segCosts.obstacles      = std::vector<T>(segCostsLength, 0.0);
	segCosts.pathDistance   = std::vector<T>(segCostsLength, 0.0);
	obstacleCalculator->waitForCompletion();
	for (size_t i = 0; i < equalArcLengthTimes.size(); i++)
	{
		// Cost already has time - it was added up to get the final
		// remappedTime value
		cost += arcSegCostsObstacles[i] + arcSegCostsPathDistance[i];
		const auto seg = arcLengthTimeToSegTime(equalArcLengthTimes, i);
		if (i < (equalArcLengthTimes.size() - 1))
		{
			const auto segp1 = arcLengthTimeToSegTime(equalArcLengthTimes, i+1);
			if (seg != segp1)
			{
				// split this arcSegCost proportionally between two segments
				const auto deltaT = equalArcLengthTimes[i+1] - equalArcLengthTimes[i];
				const auto segPercent = (static_cast<T>(segp1) - equalArcLengthTimes[i]) / deltaT;
				segCosts.time[seg]           += arcSegCostsTime[i] * segPercent;
				segCosts.time[seg+1]         += arcSegCostsTime[i] * (static_cast<T>(1.0) - segPercent);
				segCosts.obstacles[seg]      += arcSegCostsObstacles[i] * segPercent;
				segCosts.obstacles[seg+1]    += arcSegCostsObstacles[i] * (static_cast<T>(1.0) - segPercent);
				segCosts.pathDistance[seg]   += arcSegCostsPathDistance[i] * segPercent;
				segCosts.pathDistance[seg+1] += arcSegCostsPathDistance[i] * (static_cast<T>(1.0) - segPercent);
				continue;
			}
		}
		segCosts.time[seg] += arcSegCostsTime[i];
		segCosts.obstacles[seg] += arcSegCostsObstacles[i];
		segCosts.pathDistance[seg] += arcSegCostsPathDistance[i];
	}
	if (messageFilter.isEnabled())
	{
		ROS_INFO_STREAM("time = " << remappedTimes.back() << " cost = " << cost);
	}

#ifdef VERBOSE_EVALUATE_TRAJECTORY
	for (size_t i = 0; i < optParams.size(); i++)
	{
		ROS_INFO_STREAM("segCosts.regularization[" << i << "] = " <<  segCosts.regularization[i]);
	}
	for (size_t i = 0; i < equalArcLengthTimes.size(); i++)
	{
		ROS_INFO_STREAM("arcSegCostsObstacles[" << i << "] = " << arcSegCostsObstacles[i]);
	}
	for (size_t i = 0; i < equalArcLengthTimes.size(); i++)
	{
		ROS_INFO_STREAM("arcSegCostsPathDistance[" << i << "] = " << arcSegCostsPathDistance[i]);
	}
#endif

	return true;
}


// Convert from Trajectory type into the correct output
// message type
ros::Publisher local_plan_pub;
template <class T>
void trajectoryToSplineResponseMsg(base_trajectory_msgs::GenerateSpline::Response &out_msg,
								   const nav_msgs::Path &input_position_waypoints_msg,
								   const nav_msgs::Path &input_velocity_waypoints_msg,
								   const XYTTrajectory<T> &trajectory,
								   const std::vector<std::string> &jointNames,
								   const geometry_msgs::TransformStamped &pathToMapTransform,
								   const OptParamsList<T> &optParams)
{
	SampleTrajectoryImplCpu<TrajectoryPointType> sampleTrajectory(pathDistBetweenArcLengths, pathDistBetweenArcLengthsEpsilon, midTimeInflation);
	out_msg.orient_coefs.resize(trajectory[0].size());
	out_msg.x_coefs.resize(trajectory[0].size());
	out_msg.y_coefs.resize(trajectory[0].size());
	out_msg.end_points.clear();

	printTrajectory(trajectory, jointNames);
	const size_t n_joints = jointNames.size();
	for (size_t seg = 0; seg < trajectory[0].size(); seg++)
	{
		for (size_t joint = 0; joint < n_joints; joint++)
		{
			const auto &coefs = trajectory[joint][seg].getCoefs();
			std::vector<double> *m;

			if (joint == 0)
			{
				m = &out_msg.x_coefs[seg].spline;
			}
			else if (joint == 1)
			{
				m = &out_msg.y_coefs[seg].spline;
			}
			else if (joint == 2)
			{
				m = &out_msg.orient_coefs[seg].spline;
			}
			else
			{
				ROS_WARN_STREAM("Unexpected joint number constructing out_msg in base_trajectory");
				continue;
			}

			// Push in reverse order to match expectations
			// of point_gen code?
			m->clear();
			for (int i = coefs.size() - 1; i >= 0; i--)
			{
				m->push_back(coefs[i]);
			}
		}

		// All splines in a waypoint end at the same time?
		out_msg.end_points.push_back(trajectory[0][seg].endTime());
	}
	// Grab velocity profile, use that to construct a set of position
	// waypoints for the robot in wall-clock time
	T cost;
	SegCosts<T> segCosts;
	std::vector<T> equalArcLengthTimes;
	std::vector<T> remappedTimes;
	std::vector<T> vTrans;
	if (!evaluateTrajectory(cost, segCosts, equalArcLengthTimes, remappedTimes, vTrans, trajectory, pathToMapTransform, sampleTrajectory, optParams))
	{
		ROS_ERROR_STREAM("base_trajectory_node trajectoryToSplineResponseMsg : evaluateTrajectory() returned false");
		return;
	}
	out_msg.path.poses.clear();
	out_msg.path.header.stamp = ros::Time::now();
	out_msg.path.header.frame_id = pathFrameID;
	out_msg.path_velocity.header = out_msg.path.header;
	out_msg.path_acceleration.header = out_msg.path.header;
	double prevRemappedTime = 0;
	double prevVTrans = 0;

	SegmentState<T> xState;
	auto xIt = sample(trajectory[0], 0, xState);
	if (xIt == trajectory[0].cend())
	{
		ROS_ERROR_STREAM("base_trajectory trajectoryToSplineResponseMsg : could not sample initial X velocity");
		return;
	}
	double prevX = xState.position;
	double prevXVel = xState.velocity;

	SegmentState<T> yState;
	auto yIt = sample(trajectory[1], 0, yState);
	if (yIt == trajectory[1].cend())
	{
		ROS_ERROR_STREAM("base_trajectory trajectoryToSplineResponseMsg : could not sample initial Y velocity");
		return;
	}
    double prevY = yState.position;
	double prevYVel = yState.velocity;

	SegmentState<T> rotState;
#if 0
	auto rotIt = sample(trajectory[2], 0, rotState);
	if (rotIt == trajectory[2].cend())
	{
		ROS_ERROR_STREAM("base_trajectory trajectoryToSplineResponseMsg : could not sample initial rotational velocity");
		return;
	}
    double prevRot = angles::normalize_angle(rotState.position);
	double prevRotVel = rotState.velocity;
#endif

	for (size_t i = 0; i < equalArcLengthTimes.size(); i++)
	{
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = pathFrameID;
		// Remapped times is wall-clock time
		pose.header.stamp = out_msg.path.header.stamp + ros::Duration(remappedTimes[i]);
		pose.header.seq = i;

		// Finds the index of the inital point that the current segment corresponds to
		auto xIt = sample(trajectory[0], equalArcLengthTimes[i], xState);
		if (xIt == trajectory[0].cend())
		{
			ROS_ERROR_STREAM("base_trajectory trajectoryToSplineResponseMsg : could not sample xState at time " << equalArcLengthTimes[i]);
			return;
		}

		auto yIt = sample(trajectory[1], equalArcLengthTimes[i], yState);
		if (yIt == trajectory[1].cend())
		{
			ROS_ERROR_STREAM("base_trajectory trajectoryToSplineResponseMsg : could not sample yState at time " << equalArcLengthTimes[i]);
			return;
		}

		int waypointIndex = xIt - trajectory[0].cbegin();
		out_msg.waypointsIdx.push_back(waypointIndex);

		// Get the angle of the velocity vector at spline time t, add the
		// average vTrans velocity times dt times the x & y components of the velocity vector
		// to calculate the new displacement
		const auto dt = remappedTimes[i] - prevRemappedTime;
		prevRemappedTime = remappedTimes[i];
		T dx = 0;
		T dy = 0;
		const auto avgXVel = (prevXVel + xState.velocity) / 2.0;
		const auto avgYVel = (prevYVel + yState.velocity) / 2.0;
		if ((avgXVel != 0) || (avgYVel != 0))
		{
			const auto velocityVectorAngle = atan2(avgYVel, avgXVel);
			dx = ((vTrans[i] + prevVTrans) / 2.0) * cos(velocityVectorAngle) * dt;
			dy = ((vTrans[i] + prevVTrans) / 2.0) * sin(velocityVectorAngle) * dt;
		}

		prevX = pose.pose.position.x = prevX + dx;
		prevY = pose.pose.position.y = prevY + dy;
		prevXVel = xState.velocity;
		prevYVel = yState.velocity;

		pose.pose.position.z = 0;

		prevVTrans = vTrans[i];

		auto rotIt = sample(trajectory[2], equalArcLengthTimes[i], rotState);
		if (rotIt == trajectory[2].cend())
		{
			ROS_ERROR_STREAM("base_trajectory trajectoryToSplineResponseMsg : could not sample rotState at time " << equalArcLengthTimes[i]);
			return;
		}

#if 0
		const auto dRot = ((rotState.velocity + prevRotVel) / 2.0) * dt;

		ROS_INFO_STREAM("time = " << remappedTimes[i] << " prevRotVel = " << prevRotVel << " rotState = " << rotState.position << ", " << rotState.velocity << ", " << rotState.acceleration << " prevRot = " << prevRot << " dt = " << dt << " dRot = " << dRot);
		prevRotVel = rotState.velocity;
		const auto thisRotSample =  prevRot + dRot;
		prevRot += dRot;
#else
		const auto thisRotSample = rotState.position;
#endif
	
		tf2::Quaternion tfOrientation;
		tfOrientation.setRPY(0, 0, angles::normalize_angle(thisRotSample));
		pose.pose.orientation = tf2::toMsg(tfOrientation);

		out_msg.path.poses.emplace_back(pose);

		if (i == 0)
		{
			pose.pose.position.x = xState.velocity;
			pose.pose.position.y = yState.velocity;
			tfOrientation.setRPY(0, 0, angles::normalize_angle(rotState.velocity));
			pose.pose.orientation = tf2::toMsg(tfOrientation);
			out_msg.path_velocity.poses.emplace_back(pose);

			pose.pose.position.x = xState.acceleration;
			pose.pose.position.y = yState.acceleration;
			tfOrientation.setRPY(0, 0, angles::normalize_angle(rotState.acceleration));
			pose.pose.orientation = tf2::toMsg(tfOrientation);
			out_msg.path_acceleration.poses.emplace_back(pose);
		}
		else
		{
			const auto &positionPoseI = out_msg.path.poses[i];
			const auto &positionPoseIm1 = out_msg.path.poses[i -1];
			pose.pose.position.x = (positionPoseI.pose.position.x - positionPoseIm1.pose.position.x) / dt;
			pose.pose.position.y = (positionPoseI.pose.position.y - positionPoseIm1.pose.position.y) / dt;

			auto yaw_from_orientiation = [](const geometry_msgs::PoseStamped &pose)
			{
				double roll;
				double pitch;
				double yaw;
				const tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

				tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
				return yaw;
			};
			tfOrientation.setRPY(0, 0, angles::shortest_angular_distance(yaw_from_orientiation(positionPoseIm1), yaw_from_orientiation(positionPoseI)) / dt);
			pose.pose.orientation = tf2::toMsg(tfOrientation);
			out_msg.path_velocity.poses.emplace_back(pose);

			const auto &velocityPoseI = out_msg.path_velocity.poses[i];
			const auto &velocityPoseIm1 = out_msg.path_velocity.poses[i -1];
			pose.pose.position.x = (velocityPoseI.pose.position.x - velocityPoseIm1.pose.position.x) / dt;
			pose.pose.position.y = (velocityPoseI.pose.position.y - velocityPoseIm1.pose.position.y) / dt;

			tfOrientation.setRPY(0, 0, angles::shortest_angular_distance(yaw_from_orientiation(velocityPoseIm1), yaw_from_orientiation(velocityPoseI)) / dt);
			pose.pose.orientation = tf2::toMsg(tfOrientation);
			out_msg.path_acceleration.poses.emplace_back(pose);
		}
	}
	writeMatlabPath(out_msg.path.poses, 3, "Optimized paths vs real time");
	writeMatlabPathNew(out_msg.path.poses, out_msg.path_velocity.poses, out_msg.path_acceleration.poses, 4, "New optimized paths vs real time");
	local_plan_pub.publish(out_msg.path);
}

template <class T>
bool addWayPointAfter(size_t index,
		std::vector<trajectory_msgs::JointTrajectoryPoint> &points,
		OptParamsList<T> &optParams)
{
	if (index >= (points.size() - 1))
	{
		ROS_ERROR_STREAM("Trying to add point after end of trajectory");
		return false;
	}
	double xp       = points[index].positions[0];
	double xpp1     = points[index + 1].positions[0];
	double yp       = points[index].positions[1];
	double ypp1     = points[index + 1].positions[1];
	double thetap   = points[index].positions[2];
	double thetapp1 = points[index+1].positions[2];

	trajectory_msgs::JointTrajectoryPoint point;
	point.positions.push_back((xp + xpp1) / 2.0);
	point.positions.push_back((yp + ypp1) / 2.0);
	point.positions.push_back((thetap + thetapp1) / 2.0);

#if 0 // let optimizer figure these values out
	for (size_t i = 0; i < 3; i++)
	{
		point.velocities.push_back(0.0);
		point.accelerations.push_back(0.0);
	}
#endif

	points.insert(points.begin() + index, point);
	for (size_t i = 0; i < points.size(); i++)
		points[i].time_from_start = ros::Duration(static_cast<double>(i));

	const auto dx = fabs((xpp1 - xp) / 2.0) * 1.2;
	const auto dy = fabs((ypp1 - yp) / 2.0) * 1.2;
	OptParams<T> optParam(-dx, dx, -dy, dy);

	optParams.insert(optParams.begin() + index, optParam);

	return true;
}

// Struct holding data needed for an nlopt optimization iteration
template <class T>
struct NLOPTParams
{
	NLOPTParams(XYTTrajectory<T> &trajectory,
			const std::vector<trajectory_msgs::JointTrajectoryPoint> &points,
			OptParamsList<T> &optParams,
			const std::vector<std::string> &jointNames,
			const geometry_msgs::TransformStamped &pathToMapTransform,
			SampleTrajectoryImplCpu<T> &sampleTrajectory,
			SegCosts<T> &segCosts,
			size_t &optimizationAttempts)
		: trajectory_(trajectory)
		, points_(points)
		, optParams_(optParams)
		, jointNames_(jointNames)
		, pathToMapTransform_(pathToMapTransform)
		, sampleTrajectory_(sampleTrajectory)
		, segCosts_(segCosts)
		, optimizationAttempts_(optimizationAttempts)
		, frameCounter_(1)
		, bestCost_(std::numeric_limits<T>::max())
	{
	}

	XYTTrajectory<T> &trajectory_;
	const std::vector<trajectory_msgs::JointTrajectoryPoint> &points_;
	OptParamsList<T> &optParams_;
	const std::vector<std::string> &jointNames_;
	const geometry_msgs::TransformStamped &pathToMapTransform_;
	SampleTrajectoryImplCpu<T> &sampleTrajectory_;
	SegCosts<T> &segCosts_;
	size_t &optimizationAttempts_;
	size_t frameCounter_;
	T bestCost_;
};


template <class T>
double myfunc(const std::vector<double> &x, std::vector<double> &grad, void *f_data)
{
	if (!grad.empty())
	{
		ROS_ERROR_STREAM("myfunc() : grad not empty");
		return std::numeric_limits<double>::quiet_NaN();
	}
	auto nloptParams = static_cast<NLOPTParams<T>*>(f_data);
	nloptParams->optParams_.fromVector(x);

	if (!generateSpline(nloptParams->points_,
						nloptParams->optParams_,
						nloptParams->jointNames_,
						nloptParams->trajectory_))
	{
		ROS_ERROR_STREAM("base_trajectory_node : RPROP initial generateSpline() falied");
		return std::numeric_limits<double>::quiet_NaN();
	}

	// Generate initial trajectory, evaluate to get cost
	T cost;
	static std::vector<T> equalArcLengthTimes;
	static std::vector<T> remappedTimes;
	static std::vector<T> vTrans;
	if (!evaluateTrajectory(cost,
							nloptParams->segCosts_,
							equalArcLengthTimes,
							remappedTimes,
							vTrans,
							nloptParams->trajectory_,
							nloptParams->pathToMapTransform_,
							nloptParams->sampleTrajectory_,
							nloptParams->optParams_))
	{
		ROS_ERROR_STREAM("base_trajectory_node : RPROP initial evaluateTrajectory() falied");
		return std::numeric_limits<double>::quiet_NaN();
	}
	nloptParams->optimizationAttempts_ += 1;
#ifdef WRITE_OPTIMIZATION_MOVIE
	if (nloptParams->bestCost_ > cost)
	{
		nloptParams->bestCost_ = cost;
		std::stringstream s;
		s << "OptimizationMovie" << nloptParams->frameCounter_;
		writeMatlabSplines(nloptParams->trajectory_, nloptParams->frameCounter_++, s.str());
	}
#endif
	return cost;
}

template <class T>
bool NLOPT(
		XYTTrajectory<T> &trajectory,
		const std::vector<trajectory_msgs::JointTrajectoryPoint> &points,
		OptParamsList<T> &optParams,
		const std::vector<std::string> &jointNames,
		const geometry_msgs::TransformStamped &pathToMapTransform,
		SampleTrajectoryImplCpu<T> &sampleTrajectory)
{
	for (const auto &it: optParams)
	{
		ROS_INFO_STREAM("     " << it);
	}
	std::vector<T> lowerBounds;
	std::vector<T> upperBounds;
	optParams.getLimits(lowerBounds, upperBounds);

	nlopt::opt opt(nlopt::LN_SBPLX, lowerBounds.size());

	opt.set_lower_bounds(lowerBounds);
	opt.set_upper_bounds(upperBounds);

	SegCosts<T> segCosts;
	size_t optimizationAttempts = 0;
	NLOPTParams nloptParams(trajectory,
							points,
							optParams,
							jointNames,
							pathToMapTransform,
							sampleTrajectory,
							segCosts,
							optimizationAttempts);

	opt.set_min_objective(myfunc<T>, &nloptParams);
	opt.set_xtol_abs(0.05);
	opt.set_ftol_abs(0.05);
	std::vector<double> x;
	optParams.toVector(x);
	if (!x.size())
	{
		ROS_WARN("base_trajectory : no parameters to optimize");
		return true;
	}

	double cost;
	try
	{
		opt.optimize(x, cost);
	}
	catch(std::exception &e)
	{
		ROS_INFO_STREAM("nlopt failed: " << e.what() << std::endl);
		return false;
	}
	optParams.fromVector(x);

	for (size_t i = 0; i < segCosts.time.size(); i++)
	{
		ROS_INFO_STREAM("segCosts[" << i << "] : time = " << segCosts.time[i] << ", obstacles = " << segCosts.obstacles[i] << ", pathDistance = " << segCosts.pathDistance[i] << ", regularization = " << segCosts.regularization[i]);
	}
	ROS_INFO_STREAM("NLOPT optimizationAttempts=" << optimizationAttempts);
	ROS_INFO_STREAM("NLOPT best params: ");
	for (const auto &it: optParams)
	{
		ROS_INFO_STREAM("     " << it);
	}
#ifdef WRITE_OPTIMIZATION_MOVIE
	writeMatlabMovieScript(nloptParams.frameCounter_, "OptimizationMovie");
#endif
	return true;
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
template <class T>
bool RPROP(
		XYTTrajectory<T> &bestTrajectory,
		const std::vector<trajectory_msgs::JointTrajectoryPoint> &points,
		OptParamsList<T> &bestOptParams,
		const std::vector<std::string> &jointNames,
		const geometry_msgs::TransformStamped &pathToMapTransform,
		SampleTrajectoryImplCpu<T> &sampleTrajectory)
{
	ROS_INFO_STREAM("RPROP initial params: ");
	for (const auto &it: bestOptParams)
	{
		ROS_INFO_STREAM("     " << it);
	}

	if (!generateSpline(points, bestOptParams, jointNames, bestTrajectory))
	{
		ROS_ERROR_STREAM("base_trajectory_node : RPROP initial generateSpline() falied");
		return false;
	}
#ifdef WRITE_OPTIMIZATION_MOVIE
	size_t frameCounter = 1;
	std::stringstream s;
	s << "OptimizationMovie" << frameCounter;
	writeMatlabSplines(bestTrajectory, frameCounter++, s.str());
#endif

	// Generate initial trajectory, evaluate to get cost
	T bestCost;
	SegCosts<T> segCosts;
	std::vector<T> equalArcLengthTimes;
	std::vector<T> remappedTimes;
	std::vector<T> vTrans;
	if (!evaluateTrajectory(bestCost, segCosts, equalArcLengthTimes, remappedTimes, vTrans, bestTrajectory, pathToMapTransform, sampleTrajectory, bestOptParams))
	{
		ROS_ERROR_STREAM("base_trajectory_node : RPROP initial evaluateTrajectory() falied");
		return false;
	}

	unsigned int optimizationAttempts = 0;
	auto deltaCostEpsilon = static_cast<T>(initialDeltaCostEpsilon);
	auto bestCostEpsilon = static_cast<T>(initialBestCostEpsilon);


	while (deltaCostEpsilon >= static_cast<T>(minDeltaCostEpsilon))
	{
		// Keep track of how much this pass improved the best cost
		// Once improvements get small enough, stop optimizing
		const auto outerLoopPrevBestCost = bestCost;

		// Start with the previous best optimization parameters,
		// then loop through each and try to improve them
		// Ignore the first and last point - can't
		// optimize the starting and end position since
		// those need to be hit exactly.
		for (size_t i = 0; i < bestOptParams.size(); i++) // index of point being optimized
		{
			// OptParams is overloaded to act like an array
			// for accessing individual params for each point
			for (size_t j = 0; j < bestOptParams[i].size(); j++)
			{
				if (bestOptParams[i].doNotOptimize(j))
				{
					continue;
				}
				int optimizationCounter = 0;
				int momentumCount = 0;
				auto optParams = bestOptParams;
				auto deltaCost = std::numeric_limits<T>::max();
				auto currCost = bestCost;
				auto dparam = optParams[i].initialDParamGuess(j, initialDParam);

				// One exit criteria for the inner loop is if the cost
				// stops improving by an appreciable amount while changing
				// this one parameter. Track that here
				// Break after several consecutive attempts stop improving
				// the results by enough - this will let the code check both
				// positive and negative directions
				int deltaCostExitCounter = 0;
				while (deltaCostExitCounter < maxDeltaCostExitCounter)
				{
					// Alter one optimization parameter
					// and see how it changes the cost compared
					// to the last iteration
					if (!optParams[i].incrementVariable(j, dparam))
					{
						// If a parameter is already maxed out, start this pass
						// of the optimization by moving away from that maximum
						// Without this, IncrementVariable will continue to return
						// false and the code will never move values off their maximum
						dparam *= static_cast<T>(-0.1);
						continue;
					}

					XYTTrajectory<T> thisTrajectory;
					if (!generateSpline(points, optParams, jointNames, thisTrajectory))
					{
						ROS_ERROR_STREAM("base_trajectory_node : RPROP generateSpline() falied");
						return false;
					}

					T thisCost;
					if (!evaluateTrajectory(thisCost, segCosts, equalArcLengthTimes, remappedTimes, vTrans, thisTrajectory, pathToMapTransform, sampleTrajectory, optParams))
					{
						ROS_ERROR_STREAM("base_trajectory_node : RPROP evaluateTrajectory() failed");
						return false;
					}
					optimizationAttempts += 1;
					//messageFilter.enable();

					if (messageFilter.isEnabled())
					{
						ROS_INFO_STREAM("RPROP : optimizationAttempts=" << optimizationAttempts << " i=" << i << " j=" << j << " dparam = " << dparam << " bestCost=" << bestCost << " thisCost=" << thisCost << " currCost=" << currCost << " deltaCost=" << fabs(thisCost- currCost)<< " deltaCostEpsilon=" << deltaCostEpsilon);
					}
					// If cost is better than the best cost, record it.
					// After a certaint number of improvmenets, break out of the
					// inner loop and move on to optimizing the next parameter. It
					// is possible to loop back and return to this one again, but
					// the paper says that hyper-optimizing one parameter before
					// moving to others can lead to getting stuck in local minima.
					if (thisCost < bestCost)
					{
						bestTrajectory = thisTrajectory;
						bestCost = thisCost;
						bestOptParams = optParams;
						if (messageFilter.isEnabled())
						{
							ROS_INFO_STREAM("+++++++++ New best cost : " <<  bestCost);
							for (const auto &it: bestOptParams)
							{
								ROS_INFO_STREAM("     " << it);
							}
						}
#ifdef WRITE_OPTIMIZATION_MOVIE
						s = std::stringstream();
						s << "OptimizationMovie" << frameCounter;
						writeMatlabSplines(bestTrajectory, frameCounter++, s.str());
#endif
						//messageFilter.disable();
						if (++momentumCount > momentumCounter)
						{
							break;
						}
					}
					// Record values for next iteration
					deltaCost = fabs(thisCost - currCost);
					//ROS_INFO_STREAM("\t" << optParams[i]);
					//ROS_INFO_STREAM_FILTER(&messageFilter, "pos_ = (" << optParams[i].posX_ << " ," <<  optParams[i].posY_ << ")");
					//messageFilter.disable();
					// Use sign of difference between this cost and the
					// previous one to adjust the dparam value added to
					// the parameter being optimized.  1.2 and -0.5 are
					// intentionally not factors of each other to prevent
					// oscillating between the same set of values
					if (thisCost < currCost)
					{
						dparam *= static_cast<T>(1.2);
					}
					else
					{
						dparam *= static_cast<T>(-0.5);
					}
					if (deltaCost < deltaCostEpsilon)
					{
						deltaCostExitCounter += 1;
					}
					else
					{
						deltaCostExitCounter = 0;
					}
					currCost = thisCost;

					if (thisCost > 150)
					{
						optimizationCounter++;
					}
					else
					{
						optimizationCounter = 0;
					}
					if (optimizationCounter > optimizationCounterMax)
					{
						// ROS_INFO_STREAM("broken out of the optimization loop");
						break;
					}
				}
			}
		}
		ROS_INFO_STREAM("optimizationAttempts = " << optimizationAttempts << " deltaCostEpsilon = " << deltaCostEpsilon << " bestCost = " << bestCost);
		if ((outerLoopPrevBestCost - bestCost) < bestCostEpsilon)
		{
			deltaCostEpsilon /= 2.75;
			bestCostEpsilon  /= 10.0;
		}
	}
	for (size_t i = 0; i < segCosts.time.size(); i++)
	{
		ROS_INFO_STREAM("segCosts[" << i << "] : time = " << segCosts.time[i] << ", obstacles = " << segCosts.obstacles[i] << ", pathDistance = " << segCosts.pathDistance[i] << ", regularization = " << segCosts.regularization[i]);
	}
	ROS_INFO_STREAM("RPROP optimizationAttempts=" << optimizationAttempts);
	ROS_INFO_STREAM("RPROP best params: ");
	for (const auto &it: bestOptParams)
	{
		ROS_INFO_STREAM("     " << it);
	}
#ifdef WRITE_OPTIMIZATION_MOVIE
	writeMatlabMovieScript(frameCounter, "OptimizationMovie");
#endif
	return true;
}


// input should be JointTrajectory[] custom message
// Output wil be array of spline coefficents base_trajectory/Coefs[] for x, y, orientation,
// along with a path consisting of waypoints evenly spaced along the spline
bool callback(base_trajectory_msgs::GenerateSpline::Request &msg,
			  base_trajectory_msgs::GenerateSpline::Response &out_msg)
{
	using std::chrono::high_resolution_clock;
	using std::chrono::duration_cast;
	using std::chrono::duration;
	using std::chrono::milliseconds;
	const auto t1 = high_resolution_clock::now();
	const std::vector<std::string> jointNames = {"x_linear_joint", "y_linear_joint", "z_rotation_joint"};
	const size_t nJoints = jointNames.size();
	// Hold current position if trajectory is empty
	if (msg.points.empty())
	{
		ROS_ERROR_STREAM("Empty trajectory command, nothing to do.");
		return false;
	}
	if (msg.points.size() < 2)
	{
		ROS_WARN_STREAM("Only one point passed into base_trajectory - adding a starting position of 0,0,0");
		msg.points.push_back(msg.points[0]);
		msg.points[0].positions.clear();
		for (size_t i = 0; i < nJoints; i++)
		{
			msg.points[0].positions.push_back(0);
		}
		msg.points[0].velocities.clear();
		msg.points[0].accelerations.clear();
	}

	// Splines segments are each 1 arbitrary unit long.
	// This later gets mapped to actual wall-clock times
	for (size_t i = 0; i < msg.points.size(); ++i) 
	{
		msg.points[i].time_from_start = ros::Duration(static_cast<double>(i));
	}

	// Operate in two modes
	// 1 - if velocity and accelerations are empty, run a full optimization
	// 2 - if both are set, just generate a spline based on them, then
	//     convert that into an optimal path to follow
	bool runOptimization = false;
	for (size_t i = 0; i < msg.points.size(); ++i)
	{
		if (msg.points[i].positions.size() != nJoints)
		{
			ROS_ERROR_STREAM("Input point " << i << " must have 3 positions (x, y, orientation)");
			return false;
		}
		if (!msg.points[i].velocities.empty())
		{
			if (msg.points[i].velocities.size() != nJoints)
			{
				ROS_ERROR_STREAM("Input point " << i << " must have 0 or 3 velocities (x, y, orientation)");
				return false;
			}
			if (!msg.points[i].accelerations.empty() && (msg.points[i].accelerations.size() != nJoints))
			{
				ROS_ERROR_STREAM("Input point " << i << " must have 0 or 3 accelerations (x, y, orientation)");
				return false;
			}
		}
		else if (!msg.points[i].accelerations.empty())
		{
			ROS_ERROR_STREAM("Input point " << i << " must have 0 accelerations since there are also 0 velocities for that point)");
			return false;
		}

		// Optimization will be run to find the best velocity and acceleration
		// values for each intermediate point. This should only happen
		// if the velocities aren't specified for those points.
		if ((i > 0) && (i < msg.points.size() - 1))
		{
			// For intermediate points, if a velocity
			// isn't defined run the optimization
			if (msg.points[i].velocities.empty())
			{
				runOptimization = true;
			}
		}
		else
		{
			// For start and end points, velocities can be set to specify initial
			// conditions.  In that case, if the accelerations are empty
			// run optimization.  This works since due to the check above
			// if the velocities are empty the accelerations will also be.
			if (msg.points[i].accelerations.empty())
			{
				runOptimization = true;
			}
		}
	}
	ROS_WARN_STREAM(__PRETTY_FUNCTION__ << " : runOptimization = " << runOptimization);
	// Give a default starting velocity if not specified
	while (msg.points[0].velocities.size() < msg.points[0].positions.size())
	{
		msg.points[0].velocities.push_back(0.);
	}

	// Same for end velocity
	while (msg.points.back().velocities.size() < msg.points.back().positions.size())
	{
		msg.points.back().velocities.push_back(0.);
	}

	// cases where points are generated relative to other frames, e.g. from camera
	// data or relative to a fixed map
	// The second pass optionally applies a transform from a given frame back to base
	// link. This would be useful if we want a particular point to be relative to a
	// different part of the robot rather than the center - e.g. moving the intake
	// over a game piece rather than running it over with the center of the robot
	// Note first point should be 0,0,0, don't transform this since the starting
	// point has to be robot-relative and at the robot's current position
	geometry_msgs::TransformStamped pathToMapTransform;
	if (msg.header.frame_id == "map")
	{
		try
		{
			pathToMapTransform = tfBuffer->lookupTransform("map", pathFrameID, msg.header.stamp, ros::Duration(0.25));
		}
		catch (const tf2::TransformException &ex)
		{
			ROS_ERROR_STREAM("base_trajectory : Error getting transfom from map to " << pathFrameID << " : " << ex.what());
			return false;
		}
	}

#if 0
	// Use this hard-coded transform if path is defined in terms
	// of a fixed map.  This allows for a repeatable test using
	// the exact same starting conditions rather that relying on
	// the output from localization.
	pathToMapTransform.header.stamp = ros::Time(0.795);
	pathToMapTransform.transform.translation.x = 1.08885;
	pathToMapTransform.transform.translation.y = 2.24674;
	pathToMapTransform.transform.translation.z = 0;

	pathToMapTransform.transform.rotation.x = 0;
	pathToMapTransform.transform.rotation.y = 0;
	pathToMapTransform.transform.rotation.z = 6.14337e-05;
	pathToMapTransform.transform.rotation.w = 1;
#endif

	std_msgs::Header header = msg.header;
	header.frame_id = pathFrameID;

	// Calculate inverse of path to map transform to get the map to path frame
	// transform.
	// TODO - this assignment might not be necessary since tf2::convert
	// sets it in the if block below?
	geometry_msgs::TransformStamped inversePathToMapTransform = pathToMapTransform;
	if (msg.header.frame_id == "map")
	{
		tf2::Transform invtf;
		fromMsg(pathToMapTransform.transform, invtf);
		tf2::convert(invtf.inverse(), inversePathToMapTransform.transform);
		ROS_INFO_STREAM("pathToMapTransform = " << pathToMapTransform);
	}
	nav_msgs::Path input_position_waypoints;
	nav_msgs::Path input_velocity_waypoints;
	// might not need to add header at all
	input_position_waypoints.header = header;
	input_velocity_waypoints.header = header;
	for (size_t i = 1; i < msg.points.size(); i++)
	{
		geometry_msgs::PoseStamped input_position_pose;
		input_position_pose.header = header;
		input_position_pose.pose.position.x = msg.points[i].positions[0];
		input_position_pose.pose.position.y = msg.points[i].positions[1];
		tf2::Quaternion quaternion;
		quaternion.setRPY(0,0,msg.points[i].positions[2]);
		input_position_pose.pose.orientation = tf2::toMsg(quaternion);
		input_position_waypoints.poses.push_back(input_position_pose);

		geometry_msgs::PoseStamped input_velocity_pose;
		input_velocity_pose.header = header;
		if (msg.points[i].velocities.size() >= 3)
		{
			input_velocity_pose.pose.position.x = msg.points[i].velocities[0];
			input_velocity_pose.pose.position.y = msg.points[i].velocities[1];
			tf2::Quaternion velocity_quaternion;
			velocity_quaternion.setRPY(0, 0, msg.points[i].velocities[2]);
			input_velocity_pose.pose.orientation = tf2::toMsg(velocity_quaternion);
		}
		else
		{
			input_velocity_pose.pose.position.x = 0;
			input_velocity_pose.pose.position.y = 0;
			tf2::Quaternion velocity_quaternion;
			velocity_quaternion.setRPY(0, 0, 0);
			input_velocity_pose.pose.orientation = tf2::toMsg(velocity_quaternion);
		}
		input_velocity_waypoints.poses.push_back(input_velocity_pose);

		if (msg.header.frame_id == "map")
		{	
			// If the path is specified in terms of the map, use the transform read above
			// This is mostly useful for reusing the hard-coded pathToMapTransform when
			// we want to test the code (the #if 0 block above)
			// It also works for real paths, and might be nice to make sure the same
			// transform is applied to each waypoint rather than possibly having the transform
			// be updated in the middle of this loop (unlikely, but who knows)
			if (!transformTrajectoryPoint(msg.points[i].positions, inversePathToMapTransform))
			{
				return false;
			}
		}
		else
		{
			// Otherwise run through a code path which looks up the correct transform
			// between msg.header.frame_id and the pathFrameID
			if (!transformTrajectoryPoint(msg.points[i].positions, msg.header, pathFrameID))
			{
				return false;
			}
		}
		// Apply the transform from each individual waypoint as well. This is useful
		// for e.g. specifying that the intake should be driven through a waypoint
		// to pick up an object as opposed to driving the center of the robot there

		if (i < msg.point_frame_id.size())
		{
			if (!transformTrajectoryPoint(msg.points[i].positions, header, msg.point_frame_id[i]))
				return false;
		}
	}

	out_msg.position_waypoints = input_position_waypoints;
	out_msg.velocity_waypoints = input_velocity_waypoints;
	// Need to also transform kinematic constraints from whatever frame they're
	// specified in into the path frame.
	kinematicConstraints.resetConstraints();
	for (auto &constraint : msg.constraints)
	{
		constraint.header.stamp = msg.header.stamp;
		if (constraint.header.frame_id == "map")
		{
			if (!transformConstraintPoint(constraint.corner1, inversePathToMapTransform) ||
				!transformConstraintPoint(constraint.corner2, inversePathToMapTransform))
			{
				return false;
			}
		}
		else
		{
			if (!transformConstraintPoint(constraint.corner1, constraint.header, pathFrameID) ||
				!transformConstraintPoint(constraint.corner2, constraint.header, pathFrameID))
			{
				return false;
			}
		}
	}
	kinematicConstraints.addConstraints(msg.constraints);

	OptParamsList<TrajectoryPointType> optParams;
	for (const auto &pol : msg.path_offset_limit)
	{
		optParams.push_back(OptParams<TrajectoryPointType>(pol.min_x, pol.max_x,
															  pol.min_y, pol.max_y));
	}
	while (optParams.size() < msg.points.size())
	{
		optParams.push_back(OptParams<TrajectoryPointType>());
	}
	if (msg.optimize_final_velocity)
	{
		optParams.back().setDeltaV(0, kinematicConstraints.globalKinematics().getMaxVel(), -M_PI, M_PI);
	}
	// Length and length scale aren't used for first and last point
	// so disable those params here to speed up RPROP a bit - no point
	// in trying to change variables which don't actually change the path
	optParams[0].clearLengthLimits();
	optParams.back().clearLengthLimits();

	// If the robot doesn't rotate at a given waypoint, don't bother trying
	// to optimize the rotational velocity and accel at each waypoint,
	// everything should be 0
	for (size_t i = 1; i < (msg.points.size() - 1); i++)
	{
		if ((fabs(msg.points[i-1].positions[2] - msg.points[i].positions[2]) < 0.015) &&
		    (fabs(msg.points[i].positions[2] - msg.points[i+1].positions[2]) < 0.015))
		{
			optParams[i].clearRotationLengthLimits();
		}
	}

	XYTTrajectory<TrajectoryPointType> trajectory;
	if (!generateSpline(msg.points, optParams, jointNames, trajectory))
		return false;

	// TODO - do these create enough overhead that we'd need to make them
	// global rather than created for each call?
	SampleTrajectoryImplCpu<TrajectoryPointType> sampleTrajectory(distBetweenArcLengths, distBetweenArcLengthEpsilon, midTimeInflation);

	obstacleCalculator->calculatePotentials(); // initialze cost map potentials based on current costmap values
	if (runOptimization)
	{
		// Display starting spline cost
		TrajectoryPointType cost;
		SegCosts<TrajectoryPointType> segCosts;
		std::vector<TrajectoryPointType> equalArcLengthTimes;
		std::vector<TrajectoryPointType> remappedTimes;
		std::vector<TrajectoryPointType> vTrans;
		if (!evaluateTrajectory(cost, segCosts, equalArcLengthTimes, remappedTimes, vTrans, trajectory, pathToMapTransform, sampleTrajectory, optParams))
		{
			ROS_ERROR_STREAM("base_trajectory_node : evaluateTrajectory() returned false");
			return false;
		}

		base_trajectory_msgs::GenerateSpline::Response tmp_msg;
		trajectoryToSplineResponseMsg(tmp_msg, input_position_waypoints, input_velocity_waypoints, trajectory, jointNames, pathToMapTransform, optParams);
		writeMatlabSplines(trajectory, 1, "Initial Splines");
		messageFilter.disable();
		fflush(stdout);

		// Call optimization, get optimizated result in trajectory
#if 0
		if (!RPROP(trajectory, msg.points, optParams, jointNames, pathToMapTransform, sampleTrajectory))
		{
			ROS_ERROR_STREAM("base_trajectory_node : RPROP() returned false");
			return false;
		}
#else
		if (!NLOPT(trajectory, msg.points, optParams, jointNames, pathToMapTransform, sampleTrajectory))
		{
			ROS_ERROR_STREAM("base_trajectory_node : NLOPT() returned false");
			return false;
		}
#endif
		messageFilter.enable();
	}

	trajectoryToSplineResponseMsg(out_msg, input_position_waypoints, input_velocity_waypoints, trajectory, jointNames, pathToMapTransform, optParams);
	writeMatlabSplines(trajectory, 2, "Optimized Splines");
	fflush(stdout);
	const auto t2 = high_resolution_clock::now();
	const duration<double, std::milli> ms_double = t2 - t1;
	ROS_INFO_STREAM("callback() took " << ms_double.count() << " mSec");

	return true;
}


// Callbacks for ddynamic reconfigure code
void pathLimitDistanceSetCB(double pathLimitDistance)
{
	auto k = kinematicConstraints.globalKinematics();
	k.setPathLimitDistance(pathLimitDistance);
	kinematicConstraints.globalKinematics(k);
}
void maxVelSetCB(double maxVel)
{
	auto k = kinematicConstraints.globalKinematics();
	k.setMaxVel(maxVel);
	kinematicConstraints.globalKinematics(k);
}
void maxLinearAccSetCB(double maxLinearAcc)
{
	auto k = kinematicConstraints.globalKinematics();
	k.setMaxAccel(maxLinearAcc);
	kinematicConstraints.globalKinematics(k);
}
void maxLinearDecSetCB(double maxLinearDec)
{
	auto k = kinematicConstraints.globalKinematics();
	k.setMaxDecel(maxLinearDec);
	kinematicConstraints.globalKinematics(k);
}
void maxCentAccSetCB(double maxCentAcc)
{
	auto k = kinematicConstraints.globalKinematics();
	k.setMaxCentAccel(maxCentAcc);
	kinematicConstraints.globalKinematics(k);
}

double pathLimitDistanceGetCB(void)
{
	return kinematicConstraints.globalKinematics().getPathLimitDistance();
}
double maxVelGetCB(void)
{
	return kinematicConstraints.globalKinematics().getMaxVel();
}
double maxLinearAccGetCB(void)
{
	return kinematicConstraints.globalKinematics().getMaxAccel();
}
double maxLinearDecGetCB(void)
{
	return kinematicConstraints.globalKinematics().getMaxDecel();
}
double maxCentAccGetCB(void)
{
	return kinematicConstraints.globalKinematics().getMaxCentAccel();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "base_trajectory");
	ros::NodeHandle nh;

	tfBuffer = std::make_unique<tf2_ros::Buffer>(ros::Duration(1000.));
	tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);;

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
	nh.param("write_matlab_files", writeMatlabFiles, writeMatlabFiles);
	ddr.registerVariable<bool>("write_matlab_files", &writeMatlabFiles, "write matlab files for debugging", false, true);

#if 0  // RPROP not used
	nh.param("initial_delta_cost_epsilon", initialDeltaCostEpsilon, 0.05);
	nh.param("min_delta_cost_epsilon", minDeltaCostEpsilon, 0.005);
	nh.param("initial_best_cost_epsilon", initialBestCostEpsilon, 0.02);
	nh.param("momentum_counter", momentumCounter, 4);
	nh.param("max_delta_cost_exit_counter", maxDeltaCostExitCounter, 3);
	nh.param("initial_dparam", initialDParam, 0.25);
	nh.param("regularization_lambda", regularizationLambda, 0.01);
	nh.param("optimization_counter_max", optimizationCounterMax, 500);

	ddr.registerVariable<double>("initial_delta_cost_epsilon", &initialDeltaCostEpsilon, "RPROP initial deltaCostEpsilon value", 0, 1);
	ddr.registerVariable<double>("min_delta_cost_epsilon", &minDeltaCostEpsilon, "RPROP minimum deltaCost value", 0, 1);
	ddr.registerVariable<double>("initial_best_cost_epsilon", &initialBestCostEpsilon, "RPROP initial bestCostEpsilon value", 0, 1);
	ddr.registerVariable<int>("momentum_counter", &momentumCounter, "RPROP - how many cost improvements before moving to the next variable", 0, 10);
	ddr.registerVariable<int>("max_delta_cost_exit_counter", &maxDeltaCostExitCounter, "RPROP inner iteration with low change in code exit criteria", 0, 10);
	ddr.registerVariable<double>("initial_dparam", &initialDParam, "RPROP initial optimization value change", 0, 2);
	ddr.registerVariable<double>("regularization_lambda", &regularizationLambda, "RPROP cost multiplier to optparam changes", 0, 2);
	ddr.registerVariable<int>("optimization_counter_max", &optimizationCounterMax, "Iteration count for breaking out of optimization loop", 0, 500000);
#endif

	double pathLimitDistance;
	double maxVel;
	double maxLinearAcc;
	double maxLinearDec;
	double maxCentAcc;

	nh.param("path_distance_limit", pathLimitDistance, 2.0);
	nh.param("max_vel", maxVel, 4.5);
	nh.param("max_linear_acc", maxLinearAcc, 2.5);
	nh.param("max_linear_dec", maxLinearDec, 2.5);
	nh.param("max_cent_acc", maxCentAcc, 8.0);
	kinematicConstraints.globalKinematics(Kinematics<TrajectoryPointType>(maxLinearAcc, maxLinearDec, maxVel, maxCentAcc, pathLimitDistance));

	ddr.registerVariable<double>("path_distance_limit", pathLimitDistanceGetCB, pathLimitDistanceSetCB, "how far robot can diverge from straight-line path between waypoints", 0, 20);
	ddr.registerVariable<double>("max_vel", maxVelGetCB, maxVelSetCB, "max translational velocity", 0, 20);
	ddr.registerVariable<double>("max_linear_acc", maxLinearAccGetCB, maxLinearAccSetCB, "max linear acceleration", 0, 50);
	ddr.registerVariable<double>("max_linear_dec", maxLinearDecGetCB, maxLinearDecSetCB, "max linear deceleration", 0, 50);
	ddr.registerVariable<double>("max_cent_acc", maxCentAccGetCB, maxCentAccSetCB, "max centrepital acceleration", 0, 20);

	nh.param("drive_base_radius", driveBaseRadius, 0.38615);
	ddr.registerVariable<double>("drive_base_radius", &driveBaseRadius, "robot's drive base radius - half the distance of the diagonal between two opposite wheels", 0, .75);

	nh.param("path_frame_id", pathFrameID, std::string("base_link"));
	ros::ServiceServer service = nh.advertiseService("base_trajectory/spline_gen", callback);

	bool useObstacleCost;
	nh.param("use_obstacle_cost", useObstacleCost, true);
	if (useObstacleCost)
	{
		costmap = std::make_shared<costmap_2d::Costmap2DROS>("/costmap", *tfBuffer);
	}

	int obstacleCostThreshold;
	nh.param("obstacle_cost_threshold", obstacleCostThreshold, 50);

	obstacleCalculator = std::make_unique<ObstacleCalculator<TrajectoryPointType>>(costmap, obstacleCostThreshold);
	ddr.registerVariable<int>("obstacle_cost_threshold",
			boost::bind(&ObstacleCalculator<TrajectoryPointType>::getThreshold, obstacleCalculator.get()),
			boost::bind(&ObstacleCalculator<TrajectoryPointType>::setThreshold, obstacleCalculator.get(), _1),
			"Costs lower than this are ignored in obstacle gradient calculation", 0, 255);

	local_plan_pub = nh.advertise<nav_msgs::Path>("local_plan", 1, true);
	ddr.publishServicesTopics();

	ros::spin();
}
