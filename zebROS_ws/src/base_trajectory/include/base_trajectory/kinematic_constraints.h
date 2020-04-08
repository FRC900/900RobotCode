// Classes used to track constraints on robot velocity & acceleraion
// over a trajectory.  Has a global set of constraints based on the robot itself
// plus the option for local constraints adding additional limits at various locations
// throughout the trajectory
//
#pragma once

#include "base_trajectory_msgs/Constraint.h"
#include "ros/ros.h"

class Kinematics
{
	public:
		Kinematics(void);
		Kinematics(double maxAccel, double maxDecel, double maxVel, double maxCentAccel, double pathLimitDistance);
		void limit(const Kinematics &other);
		double getMaxAccel(void) const;
		double getMaxDecel(void) const;
		double getMaxVel(void) const;
		double getMaxCentAccel(void) const;
		double getPathLimitDistance(void) const;
		void   setMaxAccel(double max_accel);
		void   setMaxDecel(double max_decel);
		void   setMaxVel(double max_vel);
		void   setMaxCentAccel(double max_cent_accel);
		void   setPathLimitDistance(double path_limit_distance);

	private:
		double maxAccel_;
		double maxDecel_;
		double maxVel_;
		double maxCentAccel_;
		double pathLimitDistance_;
};

class Constraint
{
	public:
		Constraint(const base_trajectory_msgs::Constraint &msg);

		Kinematics getKinematics(double robotX, double robotY) const;

	private:
		bool inRange(double robotX, double robotY) const;
		double minX_;
		double maxX_;
		double minY_;
		double maxY_;
		Kinematics kinematics_;
};

class KinematicConstraints
{
	public:
		KinematicConstraints(void);
		KinematicConstraints(const Kinematics &kinematics);

		void addConstraint(const base_trajectory_msgs::Constraint &msg);
		void addConstraints(const std::vector<base_trajectory_msgs::Constraint> &msg);
		void resetConstraints(void);

		Kinematics globalKinematics(void) const;
		void       globalKinematics(const Kinematics globalKinematics);
		Kinematics getKinematics(double robotX, double robotY) const;

	private:
		std::vector<Constraint> constraints_;
		Kinematics              globalKinematics_;
};
