#include <ros/ros.h>
#include <cmath>
#include "talon_swerve_drive_controller/Swerve.h"
#include "talon_swerve_drive_controller/get_wheel_names.h"

constexpr size_t WHEELCOUNT = 4;
void print(const std::array<double, WHEELCOUNT> &positions, const std::array<swervemath::SpeedAndAngle, WHEELCOUNT> &speedsAngles)
{
	ROS_INFO_STREAM("===============================");
	for (size_t i = 0; i < WHEELCOUNT; i++)
		ROS_INFO_STREAM("\ti=" << i << " position=" << positions[i] << " speed=" << speedsAngles[i].speed << " angle=" << speedsAngles[i].angle);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "swerve_test");
	ros::NodeHandle nh;

	XmlRpc::XmlRpcValue wheel_coords_param;
	std::array<swervemath::Point2d, WHEELCOUNT> wheel_coords;
	if(!nh.getParam("wheel_coords", wheel_coords_param))
	{
		ROS_ERROR("talon_swerve_drive_controller : could not read wheel_coords");
		return false;
	}
	if(wheel_coords_param.getType() != XmlRpc::XmlRpcValue::TypeArray )
	{
	    ROS_ERROR("talon_swerve_drive_controller : param 'wheel_coords' is not a list");
		return false;
	}
	if (wheel_coords_param.size() != WHEELCOUNT)
	{
	    ROS_ERROR_STREAM("talon_swerve_drive_controller : param 'wheel_coords' is not correct length (expecting WHEELCOUNT = " << WHEELCOUNT << ")");
		return false;
	}
	for(int i=0; i < wheel_coords_param.size(); ++i)
	{
		if(wheel_coords_param[i].getType() != XmlRpc::XmlRpcValue::TypeArray )
		{
			ROS_ERROR("talon_swerve_drive_controller : param wheel_coords[%d] is not a list", i);
			return false;
		}
		if(wheel_coords_param[i].size() != 2)
		{
			ROS_ERROR("talon_swerve_drive_controller: param wheel_coords[%d] is not a pair", i);
			return false;
		}
		if(	wheel_coords_param[i][0].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
			wheel_coords_param[i][1].getType() != XmlRpc::XmlRpcValue::TypeDouble)
		{
			ROS_ERROR("talon_swerve_drive_controller : param wheel_coords[%d] is not a pair of doubles", i);
			return false;
		}
		wheel_coords[i] = swervemath::Point2d{wheel_coords_param[i][0], wheel_coords_param[i][1]};
	}

	ROS_INFO_STREAM("Coords: " << wheel_coords[0] << "\t" << wheel_coords[1] << "\t" << wheel_coords[2] << "\t" << wheel_coords[3]);
	swervemath::Point2d center_of_rotation{0, 0};

	swerveVar::ratios ratios;
	ratios.encodertoRotations = 0.186666666666666666666666;
	swerveVar::encoderUnits encoderUnits;
	encoderUnits.rotationGetP = 1;
	encoderUnits.rotationGetV = 1;
	encoderUnits.rotationSetP = 1;
	encoderUnits.rotationSetV = 1;
	encoderUnits.steeringGet = 1;
	encoderUnits.steeringSet = 1;
	swerveVar::driveModel driveModel;
	driveModel.maxSpeed = 5;
	driveModel.wheelRadius = 0.0508;
	swerve<WHEELCOUNT> swerveC(wheel_coords, ratios, encoderUnits, driveModel);

	std::array<double, WHEELCOUNT> positions;

	positions[0] = 0.01;
	positions[1] = -0.01;
	positions[2] = 0.01;
	positions[3] = -0.01;
	Eigen::Vector2d linearV;
	double rotationV;

	rotationV = 0;
	auto speedsAngles = swerveC.motorOutputs({1, 0}, rotationV, positions, true);
	print(positions, speedsAngles);
#if 0
	auto angles = swerveC.parkingAngles(positions);
	speedsAngles = swerveC.motorOutputs(linearV, rotation, angle, positions, true);

	angles = swerveC.parkingAngles(positions);
	angles[0] += M_PI / 2 + 0.01;
	angles = swerveC.parkingAngles(angles);
#endif
}
