#ifndef INC_SWERVE_H__
#define INC_SWERVE_H__

#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <Eigen/Dense>
#include "SwerveMath.h"

//meters, radians, newtons, kg
//Gets should be rotations/encoder unit
//Sets should be encoder unit/rotation (or rotations/second)
namespace swerveVar
{
struct ratios
{
	double encodertoRotations;
	//double motortoRotations;
	//double motortoSteering;
};

struct encoderUnits
{
	double steeringGet;
	double steeringSet;
	double rotationGetV;
	double rotationGetP;
	double rotationSetV;
	double rotationSetP;
};
struct driveModel
{
	double maxSpeed;
	double wheelRadius;
	//double mass;
	//double motorFreeSpeed;
	//double motorStallTorque;
	//int motorQuantity;
	//double speedLossConstant = .81; // Don't set this here
}; //more info should be added to this struct
}

template <size_t WHEELCOUNT>
class swerve
{
	public:
		swerve() = delete;
		swerve(const swerve &) = delete;
		swerve(swerve &&) noexcept = delete;
		swerve &operator=(const swerve &) = delete;
		swerve &operator=(swerve &&) noexcept = delete;
		virtual ~swerve() = default;

		swerve(const std::array<swervemath::Point2d, WHEELCOUNT> &wheelCoordinates,
			   const swerveVar::ratios &ratio,
			   const swerveVar::encoderUnits &units,
			   const swerveVar::driveModel &drive);

		std::array<swervemath::SpeedAndAngle, WHEELCOUNT> motorOutputs(swervemath::Point2d linearVelocity,
																	   double angularVelocity,
																	   const std::array<double, WHEELCOUNT> &positionsNew,
																	   const bool norm,
																	   const swervemath::Point2d &centerOfRotation = swervemath::Point2d{0, 0},
																	   const bool useCosScaling = false);
		std::array<double, WHEELCOUNT> parkingAngles(const std::array<double, WHEELCOUNT> &positionsNew) const;

		double getWheelAngle(const double pos) const;
	private:
		swervemath::SwerveDriveMath<WHEELCOUNT> swerveMath_;

		struct multiplierSet
		{
			std::array<swervemath::Point2d, WHEELCOUNT> multipliers_;
			double maxRotRate_;
		};
		std::map<swervemath::Point2d, multiplierSet> multiplierSets_;
		swerveVar::ratios ratio_;
		swerveVar::encoderUnits units_;
		swerveVar::driveModel drive_;
};

#endif