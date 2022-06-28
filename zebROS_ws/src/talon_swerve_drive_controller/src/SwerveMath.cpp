#include <array>
#include <cmath>
#include <talon_swerve_drive_controller/SwerveMath.h>
//#include <ros/console.h> // for debugging ROS_*_STREAM functions when needed

using namespace std;

template<size_t WHEELCOUNT>
swerveDriveMath<WHEELCOUNT>::swerveDriveMath(const array<Eigen::Vector2d, WHEELCOUNT> &wheelCoordinate)
	: wheelCoordinate_(wheelCoordinate)
	, parkingAngle_(parkingAngles()) // Has to be run after wheelCoordinates are set
{
}

//used for varying center of rotation and must be run once for initialization
template<size_t WHEELCOUNT>
array<Eigen::Vector2d, WHEELCOUNT> swerveDriveMath<WHEELCOUNT>::wheelMultipliersXY(const Eigen::Vector2d &rotationCenter) const
{
	array<double, WHEELCOUNT> wheelAngles;
	array<double, WHEELCOUNT> wheelMultipliers;
	for (size_t i = 0; i < WHEELCOUNT; i++) //increment for each wheel
	{
		const double x = wheelCoordinate_[i][0] - rotationCenter[0];
		const double y = wheelCoordinate_[i][1] - rotationCenter[1];
		wheelMultipliers[i] = -hypot(x, y);
		wheelAngles[i] = atan2(x, y) + .5 * M_PI;
	}
	normalize(wheelMultipliers, true);
	array<Eigen::Vector2d, WHEELCOUNT> multipliersXY;
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		multipliersXY[i][0] = wheelMultipliers[i] * cos(wheelAngles[i]);
		multipliersXY[i][1] = wheelMultipliers[i] * sin(wheelAngles[i]);
	}
	return multipliersXY;
}

//Below function calculates wheel speeds and angles for some target rotation and translation velocity
//Rotation is positive counter clockwise
//Angle is the angle of the gyro for field centric driving
//In radians, 0 is horizontal, increases counterclockwise
//For non field centric set angle to pi/2
template<size_t WHEELCOUNT>
array<Eigen::Vector2d, WHEELCOUNT> swerveDriveMath<WHEELCOUNT>::wheelSpeedsAngles(const array<Eigen::Vector2d, WHEELCOUNT> &wheelMultipliersXY, const Eigen::Vector2d &velocityVector, double rotation, double angle, bool norm) const
{
	//Rotate the target velocity by the robots angle to make it field centric
	const Eigen::Rotation2Dd r(M_PI / 2 - angle);
	const Eigen::Vector2d rotatedVelocity = r.toRotationMatrix() * velocityVector;

	//Should this instead be a function in 900Math of the form: rotate(vector, angle) rather than 2 lines of eigen stuff?
	array<double, WHEELCOUNT> speeds;
	array<double, WHEELCOUNT> angles;
	//Sum cartisian velocity for each wheel and then convert to polar coordinates

	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		//Only the rotation of the robot differently effects each wheel
		const double x = wheelMultipliersXY[i][0] * rotation + rotatedVelocity[0];
		const double y = wheelMultipliersXY[i][1] * rotation - rotatedVelocity[1];
		//ROS_INFO_STREAM("rot: " << rotation << " wheel_multipliers_x: " << wheelMultipliersXY[i][0]<< " wheel_multipliers_y " << wheelMultipliersXY[i][1]);
		angles[i] = atan2(x, y);
		speeds[i] = hypot(x, y);
        //ROS_INFO_STREAM("angles at " << i << " = " << angles[i] << " speeds at " << i << " = " << speeds[i]);
	}
	if(norm)
	{
		normalize(speeds);
	}
	//Speed and angles are put into one array here because speeds needed to be normalized
	array<Eigen::Vector2d, WHEELCOUNT> speedsAngles;
	for (size_t i = 0; i < WHEELCOUNT; i++)
	{
		speedsAngles[i][0] = speeds[i];
		speedsAngles[i][1] = angles[i];
	}
	return speedsAngles;
}

template<size_t WHEELCOUNT>
array<double, WHEELCOUNT> swerveDriveMath<WHEELCOUNT>::parkingAngles(void) const
{
	//only must be run once to determine the angles of the wheels in parking config
	array<double, WHEELCOUNT> angles;
	//ROS_WARN_STREAM("######## " << __PRETTY_FUNCTION__ << " wheelCoordinate_.size() = " << wheelCoordinate_.size());
	for (size_t i = 0; i < wheelCoordinate_.size(); i++)
	{
		angles[i] = atan2(wheelCoordinate_[i][0], wheelCoordinate_[i][1]);
	}
	return angles;
}

template<size_t WHEELCOUNT>
double swerveDriveMath<WHEELCOUNT>::getParkingAngle(size_t wheel) const
{
	return parkingAngle_[wheel];
}

template<size_t WHEELCOUNT>
void swerveDriveMath<WHEELCOUNT>::normalize(array<double, WHEELCOUNT> &input, const bool force_norm) const
{
	//Note that this function only works on arrays of size WHEELCOUNT
	const double maxi = fabs(*max_element(input.begin(), input.end()));
	const double mini = fabs(*min_element(input.begin(), input.end()));
	const double absoluteMax = std::max(maxi, mini);
	if (absoluteMax > 1 || force_norm)
		for (size_t i = 0; i < input.size(); i++)
			input[i] /= absoluteMax;
}

template class swerveDriveMath<4>;
