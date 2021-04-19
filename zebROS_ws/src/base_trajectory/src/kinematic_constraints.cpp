#include <limits>
#include "base_trajectory/kinematic_constraints.h"

template <class T>
Kinematics<T>::Kinematics(void)
	: maxAccel_(std::numeric_limits<T>::max())
	, maxDecel_(std::numeric_limits<T>::max())
	, maxVel_(std::numeric_limits<T>::max())
	, maxCentAccel_(std::numeric_limits<T>::max())
	, pathLimitDistance_(std::numeric_limits<T>::max())
{
}

template <class T>
Kinematics<T>::Kinematics(T maxAccel, T maxDecel, T maxVel, T maxCentAccel, T pathLimitDistance)
	: maxAccel_(maxAccel)
	, maxDecel_(maxDecel)
	, maxVel_(maxVel)
	, maxCentAccel_(maxCentAccel)
	, pathLimitDistance_(pathLimitDistance)
{
}

// Limit the kinematics to the min of the current values
// and the one in other
template <class T>
void Kinematics<T>::limit(const Kinematics &other)
{
	maxAccel_          = std::min(maxAccel_, other.maxAccel_);
	maxDecel_          = std::min(maxDecel_, other.maxDecel_);
	// If velocity is limited to 0, the robot will get stuck
	if(other.maxVel_ > 0)
	{
		maxVel_        = std::min(maxVel_, other.maxVel_);
	}
	else
	{
		ROS_WARN_STREAM("Tried to set Kinematics() maxVel to <= 0. Retaining previous value.");
	}
	maxCentAccel_      = std::min(maxCentAccel_, other.maxCentAccel_);

	// The cost function divides by path limit distance, so avoid NaN
	// costs by disallowing path limit distance of 0
	if(other.pathLimitDistance_ > 0)
	{
		pathLimitDistance_ = std::min(pathLimitDistance_, other.pathLimitDistance_);
	}
	else
	{
		ROS_WARN_STREAM("Tried to set Kinematics() pathLimitDistance to <= 0. Retaining previous value.");
	}
}

template <class T>
T Kinematics<T>::getMaxAccel(void) const
{
	return maxAccel_;
}
template <class T>
T Kinematics<T>::getMaxDecel(void) const
{
	return maxDecel_;
}
template <class T>
T Kinematics<T>::getMaxVel(void) const
{
	return maxVel_;
}
template <class T>
T Kinematics<T>::getMaxCentAccel(void) const
{
	return maxCentAccel_;
}
template <class T>
T Kinematics<T>::getPathLimitDistance(void) const
{
	return pathLimitDistance_;
}
template <class T>
void Kinematics<T>::setMaxAccel(T max_accel)
{
	maxAccel_ = max_accel;
}
template <class T>
void Kinematics<T>::setMaxDecel(T max_decel)
{
	maxDecel_ = max_decel;
}
template <class T>
void Kinematics<T>::setMaxVel(T max_vel)
{
	maxVel_ = max_vel;
}
template <class T>
void Kinematics<T>::setMaxCentAccel(T max_cent_accel)
{
	maxCentAccel_ = max_cent_accel;
}
template <class T>
void Kinematics<T>::setPathLimitDistance(T path_limit_distance)
{
	pathLimitDistance_ = path_limit_distance;
}

template <class T>
Constraint<T>::Constraint(const base_trajectory_msgs::Constraint &msg)
	: minX_(std::min(msg.corner1.x, msg.corner2.x))
	, maxX_(std::max(msg.corner1.x, msg.corner2.x))
	, minY_(std::min(msg.corner1.y, msg.corner2.y))
	, maxY_(std::max(msg.corner1.y, msg.corner2.y))
	, kinematics_(msg.max_accel, msg.max_decel, msg.max_vel, msg.max_cent_accel, msg.path_limit_distance)
{
}

// Check to see if the x,y are in range of this Constraint definition
template <class T>
bool Constraint<T>::inRange(T robotX, T robotY) const
{
	return ((robotX >= minX_) && (robotX <= maxX_) &&
	        (robotY >= minY_) && (robotY <= maxY_));
}

// Get the kinematic limits for the current x, y position
template <class T>
Kinematics<T> Constraint<T>::getKinematics(T robotX, T robotY) const
{
	// They are either the limits specified by this set of Constraints
	// if the x,y are in range
	if (inRange(robotX, robotY))
		return kinematics_;

	// Or return a set of max values for the limits so the previous
	// constraints are used instead (either global or from another
	// coord-matching set of Constraints)
	return Kinematics<T>();
}

template <class T>
KinematicConstraints<T>::KinematicConstraints(void)
{
}

template <class T>
KinematicConstraints<T>::KinematicConstraints(const Kinematics<T> &kinematics)
	: globalKinematics_(kinematics)
{
}

template <class T>
void KinematicConstraints<T>::addConstraints(const std::vector<base_trajectory_msgs::Constraint> &msg)
{
	for (const auto &m : msg)
		addConstraint(m);
}

template <class T>
void KinematicConstraints<T>::addConstraint(const base_trajectory_msgs::Constraint &msg)
{
	constraints_.emplace_back(Constraint<T>(msg));
}

template <class T>
void KinematicConstraints<T>::resetConstraints(void)
{
	constraints_.clear();
}

template <class T>
Kinematics<T> KinematicConstraints<T>::globalKinematics(void) const
{
	return globalKinematics_;
}

template <class T>
void KinematicConstraints<T>::globalKinematics(const Kinematics<T> &globalKinematics)
{
	globalKinematics_ = globalKinematics;
}

// The kinemaic limits for this x,y position are the min of all the applicable
// limits for that coord - global or any constraints in which the x and y are in range
template <class T>
Kinematics<T> KinematicConstraints<T>::getKinematics(T robotX, T robotY) const
{
	Kinematics<T> ret(globalKinematics_);

	for (const auto &c : constraints_)
	{
		ret.limit(c.getKinematics(robotX, robotY));
	}
	return ret;
}

template class Constraint<double>;
template class Kinematics<double>;
template class KinematicConstraints<double>;
//template class Constraint<float>;
//template class Kinematics<float>;
//template class KinematicConstraints<float>;
