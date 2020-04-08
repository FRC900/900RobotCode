#include <limits>
#include "base_trajectory/kinematic_constraints.h"

Kinematics::Kinematics(void)
	: maxAccel_(std::numeric_limits<double>::max())
	, maxDecel_(std::numeric_limits<double>::max())
	, maxVel_(std::numeric_limits<double>::max())
	, maxCentAccel_(std::numeric_limits<double>::max())
	, pathLimitDistance_(std::numeric_limits<double>::max())
{
}

Kinematics::Kinematics(double maxAccel, double maxDecel, double maxVel, double maxCentAccel, double pathLimitDistance)
	: maxAccel_(maxAccel)
	, maxDecel_(maxDecel)
	, maxVel_(maxVel)
	, maxCentAccel_(maxCentAccel)
	, pathLimitDistance_(pathLimitDistance)
{
}

// Limit the kinematics to the min of the current values
// and the one in other
void Kinematics::limit(const Kinematics &other)
{
	maxAccel_          = std::min(maxAccel_, other.maxAccel_);
	maxDecel_          = std::min(maxDecel_, other.maxDecel_);
	if(other.maxVel_ > 0)
	{
		maxVel_        = std::min(maxVel_, other.maxVel_);
	}
	else
	{
		ROS_WARN_STREAM("Tried to set maxVel to <= 0. Retaining previous value.");
	}
	maxCentAccel_      = std::min(maxCentAccel_, other.maxCentAccel_);
	pathLimitDistance_ = std::min(pathLimitDistance_, other.pathLimitDistance_);
}

double Kinematics::getMaxAccel(void) const
{
	return maxAccel_;
}
double Kinematics::getMaxDecel(void) const
{
	return maxDecel_;
}
double Kinematics::getMaxVel(void) const
{
	return maxVel_;
}
double Kinematics::getMaxCentAccel(void) const
{
	return maxCentAccel_;
}
double Kinematics::getPathLimitDistance(void) const
{
	return pathLimitDistance_;
}
void   Kinematics::setMaxAccel(double max_accel)
{
	maxAccel_ = max_accel;
}
void   Kinematics::setMaxDecel(double max_decel)
{
	maxDecel_ = max_decel;
}
void   Kinematics::setMaxVel(double max_vel)
{
	maxVel_ = max_vel;
}
void   Kinematics::setMaxCentAccel(double max_cent_accel)
{
	maxCentAccel_ = max_cent_accel;
}
void   Kinematics::setPathLimitDistance(double path_limit_distance)
{
	pathLimitDistance_ = path_limit_distance;
}

Constraint::Constraint(const base_trajectory_msgs::Constraint &msg)
	: minX_(std::min(msg.corner1.x, msg.corner2.x))
	, maxX_(std::max(msg.corner1.x, msg.corner2.x))
	, minY_(std::min(msg.corner1.y, msg.corner2.y))
	, maxY_(std::max(msg.corner1.y, msg.corner2.y))
	, kinematics_(msg.max_accel, msg.max_decel, msg.max_vel, msg.max_cent_accel, msg.path_limit_distance)
{
}

// Check to see if the x,y are in range of this Constraint definition
bool Constraint::inRange(double robotX, double robotY) const
{
	return ((robotX >= minX_) && (robotX <= maxX_) &&
	        (robotY >= minY_) && (robotY <= maxY_));
}

// Get the kinematic limits for the current x, y position
Kinematics Constraint::getKinematics(double robotX, double robotY) const
{
	// They are either the limits specified by this set of Constraints
	// if the x,y are in range
	if (inRange(robotX, robotY))
		return kinematics_;

	// Or return a set of max values for the limits so the previous
	// constraints are used instead (either global or from another
	// coord-matching set of Constraints)
	return Kinematics();
}

KinematicConstraints::KinematicConstraints(void)
{
}

KinematicConstraints::KinematicConstraints(const Kinematics &kinematics)
	: globalKinematics_(kinematics)
{
}

void KinematicConstraints::addConstraints(const std::vector<base_trajectory_msgs::Constraint> &msg)
{
	for (const auto m : msg)
		addConstraint(m);
}

void KinematicConstraints::addConstraint(const base_trajectory_msgs::Constraint &msg)
{
	constraints_.push_back(Constraint(msg));
}

void KinematicConstraints::resetConstraints(void)
{
	constraints_.clear();
}

Kinematics KinematicConstraints::globalKinematics(void) const
{
	return globalKinematics_;
}

void KinematicConstraints::globalKinematics(const Kinematics globalKinematics)
{
	globalKinematics_ = globalKinematics;
}

// The kinemaic limits for this x,y position are the min of all the applicable
// limits for that coord - global or any constraints in which the x and y are in range
Kinematics KinematicConstraints::getKinematics(double robotX, double robotY) const
{
	Kinematics ret(globalKinematics_);

	for (const auto &c : constraints_)
	{
		ret.limit(c.getKinematics(robotX, robotY));
	}
	return ret;
}
