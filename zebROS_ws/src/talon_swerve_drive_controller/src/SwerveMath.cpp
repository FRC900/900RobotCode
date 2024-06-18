#include <array>
#include <cmath>
#include <talon_swerve_drive_controller/SwerveMath.h>
//#include <ros/console.h> // for debugging ROS_*_STREAM functions when needed

template<size_t WHEELCOUNT>
swerveDriveMath<WHEELCOUNT>::swerveDriveMath(const std::array<Eigen::Vector2d, WHEELCOUNT> &wheelCoordinate)
	: wheelCoordinate_(wheelCoordinate)
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
		const double x = wheelCoordinate_[i][0] - rotationCenter[0];
		const double y = wheelCoordinate_[i][1] - rotationCenter[1];
		wheelMultipliers[i] = hypot(x, y);
	}
	normalize(wheelMultipliers, true);
	std::array<Eigen::Vector2d, WHEELCOUNT> multipliersXY;
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		const double wheelAngle = atan2(wheelCoordinate_[i][1], wheelCoordinate_[i][0]);
		multipliersXY[i][0] = wheelMultipliers[i] * cos(wheelAngle);
		multipliersXY[i][1] = wheelMultipliers[i] * sin(wheelAngle);
	}
	return multipliersXY;
}

//Below function calculates wheel speeds and angles for some target rotation and translation velocity
//Rotation is positive counter clockwise
//Angle is the angle of the gyro for field centric driving
//In radians, 0 is horizontal, increases counterclockwise
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
		//ROS_INFO_STREAM("rot: " << angularVelocity << " wheel_multipliers_x: " << wheelMultipliersXY[i][0]<< " wheel_multipliers_y " << wheelMultipliersXY[i][1]);
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
	//ROS_WARN_STREAM("######## " << __PRETTY_FUNCTION__ << " wheelCoordinate_.size() = " << wheelCoordinate_.size());
	for (size_t i = 0; i < wheelCoordinate_.size(); i++)
	{
		angles[i] = atan2(wheelCoordinate_[i][1], wheelCoordinate_[i][0]);
	}
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
	//Note that this function only works on arrays of size WHEELCOUNT
	const double maxi = fabs(*std::max_element(input.begin(), input.end()));
	const double mini = fabs(*std::min_element(input.begin(), input.end()));
	const double absoluteMax = std::max(maxi, mini);
	if ((absoluteMax > 1.0) || force_norm)
	{
		for (auto &i : input)
		{
			i /= absoluteMax;
		}
	}
}

template class swerveDriveMath<4>;
