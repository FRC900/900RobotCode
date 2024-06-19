#include <algorithm>
#include <array>
#include <numeric>
#include <cmath>
#include <talon_swerve_drive_controller/SwerveMath.h>
//#include <ros/console.h> // for debugging ROS_*_STREAM functions when needed

template<size_t WHEELCOUNT>
swerveDriveMath<WHEELCOUNT>::swerveDriveMath(const std::array<Eigen::Vector2d, WHEELCOUNT> &wheelCoordinates)
	: wheelCoordinates_(wheelCoordinates)
	, parkingAngle_(parkingAngles()) // Has to be run after wheelCoordinates are set
{
}

//used for varying center of rotation and must be run once for initialization
template<size_t WHEELCOUNT>
std::array<Eigen::Vector2d, WHEELCOUNT> swerveDriveMath<WHEELCOUNT>::wheelMultipliersXY(const Eigen::Vector2d &rotationCenter) const
{
	std::array<double, WHEELCOUNT> wheelMultipliers;
	for (size_t i = 0; i < WHEELCOUNT; i++) //increment for each wheel
	{
		wheelMultipliers[i] = hypot(wheelCoordinates_[i][0] - rotationCenter[0],
									wheelCoordinates_[i][1] - rotationCenter[1]);
	}
	normalize(wheelMultipliers, true);
	std::array<Eigen::Vector2d, WHEELCOUNT> multipliersXY;
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		const double wheelAngle = atan2(wheelCoordinates_[i][1], wheelCoordinates_[i][0]) + M_PI_2;
		multipliersXY[i][0] = wheelMultipliers[i] * cos(wheelAngle);
		multipliersXY[i][1] = wheelMultipliers[i] * sin(wheelAngle);
	}
	return multipliersXY;
}

//Below function calculates wheel speeds and angles for some target linear and angular velocity
//Angular velocity is positive counter clockwise
template<size_t WHEELCOUNT>
std::array<Eigen::Vector2d, WHEELCOUNT> swerveDriveMath<WHEELCOUNT>::wheelSpeedsAngles(const std::array<Eigen::Vector2d, WHEELCOUNT> &wheelMultipliersXY, const Eigen::Vector2d &linearVelocity, double angularVelocity, const bool norm) const
{
	std::array<double, WHEELCOUNT> speeds;
	std::array<double, WHEELCOUNT> angles;

	//Sum cartisian velocity for each wheel and then convert to polar coordinates
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		//Only the rotation of the robot differently effects each wheel
		const double x = wheelMultipliersXY[i][0] * angularVelocity + linearVelocity[0];
		const double y = wheelMultipliersXY[i][1] * angularVelocity + linearVelocity[1];
		//ROS_INFO_STREAM("angularVelocity: " << angularVelocity << " wheel_multipliers_x: " << wheelMultipliersXY[i][0] << " wheel_multipliers_y: " << wheelMultipliersXY[i][1]);
		speeds[i] = hypot(x, y);
		angles[i] = atan2(y, x);
        //ROS_INFO_STREAM("angles at " << i << " = " << angles[i] << " speeds at " << i << " = " << speeds[i]);
	}
	if(norm)
	{
		normalize(speeds);
	}
	//Speed and angles are put into one array here because speeds needed to be normalized
	std::array<Eigen::Vector2d, WHEELCOUNT> speedsAngles;
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		speedsAngles[i][0] = speeds[i];
		speedsAngles[i][1] = angles[i];
	}
	return speedsAngles;
}

template<size_t WHEELCOUNT>
std::array<double, WHEELCOUNT> swerveDriveMath<WHEELCOUNT>::parkingAngles(void) const
{
	//only must be run once to determine the angles of the wheels in parking config
	std::array<double, WHEELCOUNT> angles;
	std::transform(wheelCoordinates_.cbegin(), wheelCoordinates_.cend(), angles.begin(),
				   [](const Eigen::Vector2d &wheelCoordinate)
				   {
					   return atan2(wheelCoordinate[1], wheelCoordinate[0]);
				   });
	return angles;
}

template<size_t WHEELCOUNT>
double swerveDriveMath<WHEELCOUNT>::getParkingAngle(const size_t wheel) const
{
	return parkingAngle_[wheel];
}

template<size_t WHEELCOUNT>
void swerveDriveMath<WHEELCOUNT>::normalize(std::array<double, WHEELCOUNT> &input, const bool force_norm) const
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

template<size_t WHEELCOUNT>
double swerveDriveMath<WHEELCOUNT>::furthestWheel(const Eigen::Vector2d &centerOfRotation) const
{
	return std::accumulate(wheelCoordinates_.cbegin(), wheelCoordinates_.cend(), 0.0,
						   [centerOfRotation](const double maxD, const Eigen::Vector2d &wheelCoordinate)
						   {
							   return std::max(maxD, hypot(wheelCoordinate[0] - centerOfRotation[0], wheelCoordinate[1] - centerOfRotation[1]));
						   });
}

template class swerveDriveMath<4>;
