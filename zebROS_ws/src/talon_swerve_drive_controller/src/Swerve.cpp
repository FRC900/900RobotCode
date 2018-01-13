#include <talon_swerve_drive_controller/Swerve.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <functional>
#include <cmath>
using namespace std;
using namespace Eigen;

// TODO : use initializaer list rather than assignment.
// string should be a const & var, as should the swerveVar args

swerve::swerve(array<Vector2d, WHEELCOUNT> wheelCoordinates, string fileName, bool wheelAngleInvert, swerveVar::ratios ratio, swerveVar::encoderUnits units, swerveVar::driveModel drive)
{
	wheelCoordinates_ = wheelCoordinates;

	swerveMath_ = swerveDriveMath(wheelCoordinates_);

	ratio_ = ratio;
	units_ = units;
	drive_ = drive;
	fileName_ = fileName;
	wheelAngleInvert_ = wheelAngleInvert ? -1 : 1;
	ifstream offsetRead;
	offsetRead.open(fileName);
	if (!offsetRead)
	{
		cout << "No Offset File!!!" << endl;
		for (int i = 0; i < WHEELCOUNT; i++)
		{
			offsets_[i] = 0;
		}
	}
	else
	{
		double offset;
		int i = 0;
		while (offsetRead >> offset)
		{
			offsets_[i] = offset;
			i++;
		}
	}

	// TODO : this shouldn't be hard-coded
	setCenterOfRotation(0, {0,0});
}

void swerve::setCenterOfRotation(int id, const Vector2d &centerOfRotation)
{
	if (id < multiplierSets_.size())
	{
		multiplierSet newSet;
		newSet.multipliers_ = swerveMath_.wheelMultipliersXY(centerOfRotation);
		newSet.maxRotRate_ = furthestWheel(centerOfRotation) / drive_.maxSpeed;
		multiplierSets_[id] = newSet;
	}
}

// TODO : split into motorOutputsDrive and motorOutputsPark
// Make positionsNew and all Vector2ds const & arguments
array<Vector2d, WHEELCOUNT> swerve::motorOutputs(Vector2d velocityVector, double rotation, double angle, bool forceRead, array<bool, WHEELCOUNT> &reverses, bool park, array<double, WHEELCOUNT> positionsNew, int rotationCenterID)
{
	if (rotationCenterID >= multiplierSets_.size())
	{
		cerr << "Tell Ryan to stop using fixed-sized arrays for dynamically growable stuff" << endl;
		return array<Vector2d, WHEELCOUNT>();
	}
	encoderPosition_ = positionsNew;
	array<Vector2d, WHEELCOUNT> speedsAndAngles;
	if (!park)
	{
		velocityVector /= drive_.maxSpeed;
		rotation /= multiplierSets_[rotationCenterID].maxRotRate_;
		speedsAndAngles = swerveMath_.wheelSpeedsAngles(multiplierSets_[rotationCenterID].multipliers_, velocityVector, rotation, angle);
		for (int i = 0; i < WHEELCOUNT; i++)
		{
			double nearestangle;
			bool reverse;
			getWheelAngle(i, encoderPosition_[i]);
			nearestangle = leastDistantAngleWithinHalfPi(encoderPosition_[i], speedsAndAngles[i][1], reverse);
			reverses[i] = reverse;
			speedsAndAngles[i][0] *= ((drive_.maxSpeed / (drive_.wheelRadius * 2.0 * M_PI)) / ratio_.encodertoRotations) * units_.rotationSetV * (reverse ? -1 : 1);
			speedsAndAngles[i][1] = (nearestangle) * units_.steeringSet - offsets_[i];
		}
	}
	else
	{
		for (int i = 0; i < WHEELCOUNT; i++)
		{
			speedsAndAngles[i][1] = swerveMath_.parkingAngle_[i]; // TODO : find a way not to access member of swervemath here
			speedsAndAngles[i][0] = 0;

			double nearestangle;
			bool reverse;
			getWheelAngle(i, encoderPosition_[i]);
			nearestangle = leastDistantAngleWithinHalfPi(encoderPosition_[i], speedsAndAngles[i][1], reverse);
			speedsAndAngles[i][1] = (nearestangle / (2 * M_PI)) * units_.steeringSet - offsets_[i];
		}

	}
	return speedsAndAngles;
}
void swerve::saveNewOffsets(bool useVals, array<double, WHEELCOUNT> newOffsets, array<double, WHEELCOUNT> newPosition)
{
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
}
/*
Vector2d calculateOdom()
{

//Steal code from steered wheel base

}
*/

double swerve::getWheelAngle(int index, double pos) const
{
	return (pos + offsets_[index]) * units_.steeringGet * 2. * M_PI * wheelAngleInvert_;
}

double swerve::furthestWheel(Vector2d centerOfRotation) const
{
	double maxD = 0;
	for (int i = 0; i < WHEELCOUNT; i++)
	{
		// TODO : rewrite using hypto() function
		double dist = sqrt(pow(wheelCoordinates_[i][0] - centerOfRotation[0], 2) + pow(wheelCoordinates_[i][1] - centerOfRotation[1], 2));
		if (dist > maxD)
		{
			maxD = dist;
		}
	}
	return maxD;
}
