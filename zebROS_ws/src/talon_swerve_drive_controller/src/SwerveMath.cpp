#include <algorithm>
#include <array>
#include <numeric>
#include <cmath>
#include <talon_swerve_drive_controller/SwerveMath.h>
//#include <ros/console.h> // for debugging ROS_*_STREAM functions when needed

namespace swervemath
{
template<size_t WHEELCOUNT>
SwerveDriveMath<WHEELCOUNT>::SwerveDriveMath(const std::array<Point2d, WHEELCOUNT> &wheelCoordinates)
	: wheelCoordinates_(wheelCoordinates)
	, parkingAngle_(parkingAngles()) // Has to be run after wheelCoordinates are set
{
}

// Calculate the X&Y components of the vector from the center of rotation to each wheel
template<size_t WHEELCOUNT>
std::array<Point2d, WHEELCOUNT> SwerveDriveMath<WHEELCOUNT>::wheelMultipliersXY(const Point2d &rotationCenter) const
{
	// Used to scale the wheel speeds by the distance of each
	// from the center of the robot
	std::array<double, WHEELCOUNT> wheelMultipliers;
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		wheelMultipliers[i] = hypot(wheelCoordinates_[i].x - rotationCenter.x,
									wheelCoordinates_[i].y - rotationCenter.y);
	}
	normalize(wheelMultipliers, true);
	std::array<Point2d, WHEELCOUNT> multipliersXY;
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		// Add M_PI_2 to get the angle normal to the corner rather than
		// the angle pointing out from the center of the robot
		const double wheelAngle = atan2(wheelCoordinates_[i].y - rotationCenter.y,
										wheelCoordinates_[i].x - rotationCenter.x) +
								  M_PI_2;
		multipliersXY[i].x = wheelMultipliers[i] * cos(wheelAngle);
		multipliersXY[i].y = wheelMultipliers[i] * sin(wheelAngle);
	}
	return multipliersXY;
}

//Below function calculates wheel speeds and angles for some target linear and angular velocity
//Angular velocity is positive counter clockwise
template <size_t WHEELCOUNT>
std::array<SpeedAndAngle, WHEELCOUNT> SwerveDriveMath<WHEELCOUNT>::wheelSpeedsAngles(const std::array<Point2d, WHEELCOUNT> &wheelMultipliersXY,
																					   const Point2d &linearVelocity,
																					   const double angularVelocity,
																					   const bool norm) const
{
	std::array<SpeedAndAngle, WHEELCOUNT> speedsAngles;
	// Speeds are put into one array here because they might need to be normalized,
	// and the function to do that expects a single array of speeds
	std::array<double, WHEELCOUNT> speeds;

	//Sum cartisian velocity for each wheel and then convert to polar coordinates
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		//Only the rotation of the robot differently effects each wheel
		const double x = wheelMultipliersXY[i].x * angularVelocity + linearVelocity.x;
		const double y = wheelMultipliersXY[i].y * angularVelocity + linearVelocity.y;
		speeds[i] = hypot(x, y);
		speedsAngles[i].angle = atan2(y, x);
	}
	if(norm)
	{
		normalize(speeds);
	}
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		speedsAngles[i].speed = speeds[i];
	}
	return speedsAngles;
}

template<size_t WHEELCOUNT>
std::array<double, WHEELCOUNT> SwerveDriveMath<WHEELCOUNT>::parkingAngles(void) const
{
	//only must be run once to determine the angles of the wheels in parking config
	std::array<double, WHEELCOUNT> angles;
	std::transform(wheelCoordinates_.cbegin(), wheelCoordinates_.cend(), angles.begin(),
				   [](const Point2d &wheelCoordinate)
				   {
					   return atan2(wheelCoordinate.y, wheelCoordinate.x);
				   });
	return angles;
}

template<size_t WHEELCOUNT>
double SwerveDriveMath<WHEELCOUNT>::getParkingAngle(const size_t wheel) const
{
	return parkingAngle_[wheel];
}

// input is an array of wheel speeds
// find the absolute value of the min and max speeds for all wheels
// If this absolute max is greater than 1, scale all speeds down so
// the fastest wheel is moving at 1
// If force_norm is set, do the same normalization
template<size_t WHEELCOUNT>
void SwerveDriveMath<WHEELCOUNT>::normalize(std::array<double, WHEELCOUNT> &input, const bool force_norm) const
{
	const auto [minv, maxv] = std::minmax_element(input.cbegin(), input.cend());
	const double absoluteMax = std::max(fabs(*minv), fabs(*maxv));
	if ((absoluteMax > 1.0) || force_norm)
	{
		for (auto &i : input)
		{
			i /= absoluteMax;
		}
	}
}

// Return the larges of the distances between the wheels and the 
// provided center of rotation
template<size_t WHEELCOUNT>
double SwerveDriveMath<WHEELCOUNT>::furthestWheel(const Point2d &centerOfRotation) const
{
	return std::accumulate(wheelCoordinates_.cbegin(), wheelCoordinates_.cend(), 0.0,
						   [centerOfRotation](const double maxD, const Point2d &wheelCoordinate)
						   {
							   return std::max(maxD, hypot(wheelCoordinate.x - centerOfRotation.x, wheelCoordinate.y - centerOfRotation.y));
						   });
}

template class SwerveDriveMath<4>;

};