// Classes used to track constraints on robot velocity & acceleraion
// over a trajectory.  Has a global set of constraints based on the robot itself
// plus the option for local constraints adding additional limits at various locations
// throughout the trajectory
//
#pragma once

#include "base_trajectory_msgs/Constraint.h"
#include "ros/ros.h"

template <class T>
class Kinematics
{
	public:
		Kinematics(void);
		Kinematics(T maxAccel, T maxDecel, T maxVel, T maxCentAccel, T pathLimitDistance);
		void limit(const Kinematics &other);
		T getMaxAccel(void) const;
		T getMaxDecel(void) const;
		T getMaxVel(void) const;
		T getMaxCentAccel(void) const;
		T getPathLimitDistance(void) const;
		void   setMaxAccel(T max_accel);
		void   setMaxDecel(T max_decel);
		void   setMaxVel(T max_vel);
		void   setMaxCentAccel(T max_cent_accel);
		void   setPathLimitDistance(T path_limit_distance);

	private:
		T maxAccel_;
		T maxDecel_;
		T maxVel_;
		T maxCentAccel_;
		T pathLimitDistance_;
};

template <class T>
class Constraint
{
	public:
		Constraint(const base_trajectory_msgs::Constraint &msg);

		Kinematics<T> getKinematics(T robotX, T robotY) const;

	private:
		bool inRange(T robotX, T robotY) const;
		T minX_;
		T maxX_;
		T minY_;
		T maxY_;
		Kinematics<T> kinematics_;
};

template <class T>
class KinematicConstraints
{
	public:
		KinematicConstraints(void);
		KinematicConstraints(const Kinematics<T> &kinematics);

		void addConstraint(const base_trajectory_msgs::Constraint &msg);
		void addConstraints(const std::vector<base_trajectory_msgs::Constraint> &msg);
		void resetConstraints(void);

		Kinematics<T> globalKinematics(void) const;
		void          globalKinematics(const Kinematics<T> &globalKinematics);
		Kinematics<T> getKinematics(T robotX, T robotY) const;

	private:
		std::vector<Constraint<T>> constraints_;
		Kinematics<T>              globalKinematics_;
};
