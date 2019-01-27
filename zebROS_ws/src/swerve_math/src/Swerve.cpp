#include <swerve_math/Swerve.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <functional>
#include <cmath>
#include <ros/console.h>
using namespace std;
using namespace Eigen;

// TODO : use initializer list rather than assignment.
// string should be a const & var, as should the swerveVar args

swerve::swerve(array<Vector2d, WHEELCOUNT> wheelCoordinates, std::vector<double> offsets, bool /*wheelAngleInvert*/, swerveVar::ratios ratio, swerveVar::encoderUnits units, swerveVar::driveModel drive)
{
	wheelCoordinates_ = wheelCoordinates;

	swerveMath_ = swerveDriveMath(wheelCoordinates_);

	ratio_ = ratio;
	units_ = units;
	drive_ = drive;
	//wheelAngleInvert_ = wheelAngleInvert ? -1 : 1;

	// TODO : Error checking in case offsets.size() != WHEELCOUNT
	for (size_t i = 0; i < offsets.size() && i < WHEELCOUNT; i++)
		offsets_[i] = offsets[i];
}

// TODO : split into motorOutputsDrive and motorOutputsPark
array<Vector2d, WHEELCOUNT> swerve::motorOutputs(Vector2d velocityVector, double rotation, double angle, bool /*forceRead*/, array<bool, WHEELCOUNT> &reverses, bool park, const array<double, WHEELCOUNT> &positionsNew, bool norm, const Eigen::Vector2d &centerOfRotation)
{
	array<Vector2d, WHEELCOUNT> speedsAndAngles;
	if (!park)
	{
		// See if the current centerOfRotation coords have been used before
		// If not, calculate the multiplers and matRotRate for them
		// If so, just reuse previously saved values
		auto mult_it = multiplierSets_.find(centerOfRotation);
		if (mult_it == multiplierSets_.end())
		{
			multiplierSet newSet;
			newSet.multipliers_ = swerveMath_.wheelMultipliersXY(centerOfRotation);
			newSet.maxRotRate_ = drive_.maxSpeed / furthestWheel(centerOfRotation);
			multiplierSets_[centerOfRotation] = newSet;
			mult_it = multiplierSets_.find(centerOfRotation);
			ROS_INFO_STREAM("Added new swerve center of rotation: " << centerOfRotation[0] << "," << centerOfRotation[1]);
		}

		velocityVector /= drive_.maxSpeed;
		rotation       /= mult_it->second.maxRotRate_;

		//ROS_WARN_STREAM("max rate r/s: " <<  multiplierSets_[rotationCenterID].maxRotRate_);
		//ROS_INFO_STREAM("vel: " << velocityVector[0] << " " << velocityVector[1] << " rot: " << rotation);
		speedsAndAngles = swerveMath_.wheelSpeedsAngles(mult_it->second.multipliers_, velocityVector, rotation, angle, norm);
		for (int i = 0; i < WHEELCOUNT; i++)
		{
			//ROS_INFO_STREAM("PRE NORMalIZE pos/vel in direc: " << speedsAndAngles[i][0] << " rot: " <<speedsAndAngles[i][1] );
			const double currpos = getWheelAngle(i, positionsNew[i]);
			const double nearestangle = leastDistantAngleWithinHalfPi(currpos, speedsAndAngles[i][1], reverses[i]);

			speedsAndAngles[i][0] *= ((drive_.maxSpeed / (drive_.wheelRadius)) / ratio_.encodertoRotations) * units_.rotationSetV * (reverses[i] ? -1 : 1);
			//ROS_INFO_STREAM(" id: " << i <<" speed: " << speedsAndAngles[i][0] << " reverse: " << reverse);
			speedsAndAngles[i][1] = nearestangle * units_.steeringSet;
			speedsAndAngles[i][1] += offsets_[i];
			//ROS_INFO_STREAM("pos/vel in direc: " << speedsAndAngles[i][0] << " rot: " << speedsAndAngles[i][1] );
		}
	}
	else
	{
		for (int i = 0; i < WHEELCOUNT; i++)
		{
			speedsAndAngles[i][1] = swerveMath_.parkingAngle_[i]; // TODO : find a way not to access member of swervemath here
			speedsAndAngles[i][0] = 0;

			const double currpos = getWheelAngle(i, positionsNew[i]);
			const double nearestanglep = leastDistantAngleWithinHalfPi(currpos, speedsAndAngles[i][1], reverses[i]);
			//ROS_INFO_STREAM(" id: " << i << " currpos: " << currpos << "target" <<nearestanglep);
			speedsAndAngles[i][1] = nearestanglep * units_.steeringSet;
			speedsAndAngles[i][1] += offsets_[i];
		}
	}
	return speedsAndAngles;
}
void swerve::saveNewOffsets(bool /*useVals*/, array<double, WHEELCOUNT> /*newOffsets*/, array<double, WHEELCOUNT> /*newPosition*/)
{
#if 0
	encoderPosition_ = newPosition;
	if (!useVals)
	{
		for (int i = 0; i < WHEELCOUNT; i++)
		{
			newOffsets[i] = encoderPosition_[i];
		}
	}
	offsets_ = newOffsets;

	// TODO : Uncondtionally open in out|trunc mode?
	ofstream offsetFile(fileName_);
	if (offsetFile)
	{
		offsetFile.close();
		offsetFile.open(fileName_, ios::out | ios::trunc);

	}
	for (int i = 0; i < WHEELCOUNT; i++)
	{
		offsetFile << offsets_[i] << endl;
	}
#endif
}
/*
Vector2d calculateOdom()
{

//Steal code from steered wheel base

}
*/

double swerve::getWheelAngle(int index, double pos) const
{
	return (pos - offsets_[index]) * units_.steeringGet;
}

double swerve::furthestWheel(const Vector2d &centerOfRotation) const
{
	double maxD = 0;
	for (int i = 0; i < WHEELCOUNT; i++)
		maxD = std::max(maxD, hypot(wheelCoordinates_[i][0] - centerOfRotation[0], wheelCoordinates_[i][1] - centerOfRotation[1]));
	return maxD;
}
