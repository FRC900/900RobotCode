#include <angles/angles.h>
#include <talon_swerve_drive_controller/Swerve.h>

#include <ros/console.h>

// Struct for returning optimized wheel motion.
// The angle is the direction to point the wheel, while speed
// multiplier is +/- 1 to run the corresponding drive motor in
// the forward or reverse direction.
struct WheelDirection
{
	WheelDirection(const double angle, const double speed_multiplier)
		: angle(angle)
		, speed_multiplier(speed_multiplier)
	{
	}
    double angle;
    double speed_multiplier;
};

// returns the closest angle to the current angle = to x*.5*M_PI + target angle where x is any integer
// used for turning wheels to the target angle
static WheelDirection optimizeWheelDirection(const double currentAngle, const double targetAngle)
{
	const double withinPi = angles::shortest_angular_distance(currentAngle, targetAngle);

	// If within +/- 90 degress, turn to that angle and move in the requested direction
	if (fabs(withinPi) <= M_PI_2)
	{
		return WheelDirection(withinPi + currentAngle, 1.0);
	}
	// If outside +/- 90 degress, turn to the opposite angle and move in the opposite direction
	// to minimize the turning needed to move in the requested direction
	return WheelDirection(withinPi - copysign(M_PI, withinPi) + currentAngle, -1.0);
}

template <size_t WHEELCOUNT>
swerve<WHEELCOUNT>::swerve(const std::array<swervemath::Point2d, WHEELCOUNT> &wheelCoordinates,
						   const swerveVar::ratios &ratio,
						   const swerveVar::encoderUnits &units,
						   const swerveVar::driveModel &drive)
	: swerveMath_(swervemath::SwerveDriveMath<WHEELCOUNT>(wheelCoordinates))
	, ratio_(ratio)
	, units_(units)
	, drive_(drive)
{
}

template <size_t WHEELCOUNT>
std::array<swervemath::SpeedAndAngle, WHEELCOUNT> swerve<WHEELCOUNT>::motorOutputs(swervemath::Point2d linearVelocity,
																				   double angularVelocity,
																				   const std::array<double, WHEELCOUNT> &positionsNew,
																				   const bool norm,
																				   const swervemath::Point2d &centerOfRotation,
																				   const bool useCosScaling)
{
	// See if the current centerOfRotation coords have been used before
	// If so, just reuse previously saved values
	// If not, calculate the multiplers and matRotRate for them
	auto mult_it = multiplierSets_.find(centerOfRotation);
	if (mult_it == multiplierSets_.end())
	{
		multiplierSet newSet;
		newSet.multipliers_ = swerveMath_.wheelMultipliersXY(centerOfRotation);
		newSet.maxRotRate_ = drive_.maxSpeed / swerveMath_.furthestWheel(centerOfRotation);
		mult_it = multiplierSets_.try_emplace(centerOfRotation, newSet).first;
		ROS_INFO_STREAM("Added new swerve center of rotation: " << centerOfRotation.x << "," << centerOfRotation.y);
	}

	linearVelocity  /= drive_.maxSpeed;
	angularVelocity /= mult_it->second.maxRotRate_;

	//ROS_WARN_STREAM("max rate r/s: " <<  multiplierSets_[rotationCenterID].maxRotRate_);
	//ROS_INFO_STREAM("vel: " << linearVelocity[0] << " " << linearVelocity[1] << " rot: " << angularVelocity);
	auto speedsAndAngles = swerveMath_.wheelSpeedsAngles(mult_it->second.multipliers_, linearVelocity, angularVelocity, norm);
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		const double currpos = getWheelAngle(positionsNew[i]);
		const auto wheel_direction = optimizeWheelDirection(currpos, speedsAndAngles[i].angle);

		//ROS_INFO_STREAM("id: " << i << " PRE NORMalIZE pos/vel in direc: " << speedsAndAngles[i][0] << " rot: " <<speedsAndAngles[i][1] );
		//ROS_INFO_STREAM("wheel " << i << " currpos: " << currpos << " wheel_direction : " << wheel_direction.angle << " " << wheel_direction.speed_multiplier");
		// Slow down wheels the further they are from their target
		// angle. This will help to prevent wheels which are in the process
		// of getting to the correct orientation from dragging the robot
		// in random directions while turning to the expected direction
		// cos() shouldn't care about +/-, so don't worry about fabs()
		const double cosScaling = useCosScaling ? cos(currpos - wheel_direction.angle) : 1.0;
		// ROS_INFO_STREAM("i=" << i << " cosScaling: " << cosScaling << " positionsNew: " << positionsNew[i] << " currpos: " << currpos << " wheel_direction.angle: " << wheel_direction.angle << " wheel_direction.speed_multiplier: " << wheel_direction.speed_multiplier << " speedsAndAngles: " << speedsAndAngles[i][0] << " " << speedsAndAngles[i][1]);	

		speedsAndAngles[i].speed *= ((drive_.maxSpeed / drive_.wheelRadius) / ratio_.encodertoRotations) * units_.rotationSetV * wheel_direction.speed_multiplier * cosScaling;
		speedsAndAngles[i].angle = wheel_direction.angle * units_.steeringSet;
		//ROS_INFO_STREAM("pos/vel in direc: " << speedsAndAngles[i][0] << " rot: " << speedsAndAngles[i][1] << " steeringSet: " << units_.steeringSet);
	}
	return speedsAndAngles;
}

template<size_t WHEELCOUNT>
std::array<double, WHEELCOUNT> swerve<WHEELCOUNT>::parkingAngles(const std::array<double, WHEELCOUNT> &positionsNew) const
{
	std::array<double, WHEELCOUNT> retAngles;
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		const double currpos = getWheelAngle(positionsNew[i]);
		const auto wheel_direction = optimizeWheelDirection(currpos, swerveMath_.getParkingAngle(i));

		retAngles[i] = wheel_direction.angle * units_.steeringSet;
		//ROS_INFO_STREAM(" id: " << i << " currpos: " << currpos << " target: " << nearestanglep);
		//ROS_INFO_STREAM("park[i]: " << swerveMath_.getParkingAngle(i) << " " << retAngles[i]);
	}
	return retAngles;
}

// Apply steering ratio to calculate desired
// measured wheel angle from a wheel angle setpoint
template<size_t WHEELCOUNT>
double swerve<WHEELCOUNT>::getWheelAngle(const double pos) const
{
	return pos * units_.steeringGet;
}

template class swerve<4>;
