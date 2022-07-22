#include <ros/console.h>
#include <cmath>
#include "talon_swerve_drive_controller/Swerve.h"

constexpr size_t WHEELCOUNT = 4;
void print(const std::array<double, WHEELCOUNT> &positions, const std::array<Eigen::Vector2d, WHEELCOUNT> &speedsAngles)
{
	ROS_INFO_STREAM("===============================");
	for (size_t i = 0; i < WHEELCOUNT; i++)
		ROS_INFO_STREAM("\ti=" << i << " position=" << positions[i] << " speed=" << speedsAngles[i][0] << " angle=" << speedsAngles[i][1]);
}


int main(void)
{
	std::array<Eigen::Vector2d, WHEELCOUNT> wheel_coords_;
	std::array<double, WHEELCOUNT> offsets{0};
	for (auto &w: wheel_coords_)
	{
		w[0] = 1;
		w[1] = 1;
	}
	swerveVar::ratios ratios;
	ratios.encodertoRotations = 1;
	ratios.motortoRotations = 1;
	ratios.motortoSteering = 1;
	swerveVar::encoderUnits encoderUnits;
	encoderUnits.rotationGetP = 1;
	encoderUnits.rotationGetV = 1;
	encoderUnits.rotationSetP = 1;
	encoderUnits.rotationSetV = 1;
	encoderUnits.steeringGet = 1;
	encoderUnits.steeringSet = 1;
	swerveVar::driveModel driveModel;
	driveModel.maxSpeed = 1;
	driveModel.wheelRadius = 1;
	swerve<WHEELCOUNT> swerveC(wheel_coords_, offsets, ratios, encoderUnits, driveModel);

	std::array<double, WHEELCOUNT> positions;

	positions[0] = 0.01;
	positions[1] = -0.01;
	positions[2] = 0.01;
	positions[3] = -0.01;
	Eigen::Vector2d linearV;
	double rotation;

	constexpr double angle = M_PI / 2.0;


	linearV[0] = 1;
	linearV[1] = 0;
	rotation = 0;
	auto speedsAngles = swerveC.motorOutputs(linearV, rotation, angle, positions, true);
	print(positions, speedsAngles);
	positions[0] = speedsAngles[0][1] - 0.01;
	positions[1] = speedsAngles[1][1] + 0.01;
	positions[2] = speedsAngles[2][1] - 0.01;
	positions[3] = speedsAngles[3][1] + 0.01;
	linearV[0] = 0;
	linearV[1] = 1;
	speedsAngles = swerveC.motorOutputs(linearV, rotation, angle, positions, true);
	print(positions, speedsAngles);
#if 0
	auto angles = swerveC.parkingAngles(positions);
	speedsAngles = swerveC.motorOutputs(linearV, rotation, angle, positions, true);

	angles = swerveC.parkingAngles(positions);
	angles[0] += M_PI / 2 + 0.01;
	angles = swerveC.parkingAngles(angles);
#endif
}
