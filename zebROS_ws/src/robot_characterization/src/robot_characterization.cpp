#include <ros/ros.h>
#include <networktables/NetworkTableInstance.h>
#include <talon_state_msgs/TalonState.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>

// Current battery voltage - read from talon bus voltage
double battery = 0;

// Motor output voltages - read from TalonState via callback
double leftMotorVolts = 0;
double rightMotorVolts = 0;

// Motor positions - also read from TalonState
double leftPosition = 0;
double rightPosition = 0;

// Motor speeds - also read from TalonState
double leftRate = 0;
double rightRate = 0;

// Find the index in the array of talon data for the given talon_name
void getTalonIndex(const talon_state_msgs::TalonState &talon_state,
		const std::string &talon_name,
		size_t &talon_idx)
{
	if (talon_idx >= talon_state.name.size())
	{
		for (size_t i = 0; i < talon_state.name.size(); i++)
		{
			if (talon_state.name[i] == talon_name)
			{
				talon_idx = i;
				return;
			}
		}
	}
}

bool getTalonData(const talon_state_msgs::TalonState &talon_state, const size_t talon_idx,
		double &volts, double &position, double &rate, double &busVolts)
{
	if (talon_idx < talon_state.name.size())
	{
		volts    = talon_state.output_voltage[talon_idx];
		position = talon_state.position[talon_idx];
		rate     = talon_state.speed[talon_idx];
		busVolts = talon_state.bus_voltage[talon_idx];
		return true;
	}
	volts = 0;
	position = 0;
	rate = 0;
	busVolts = 0;
	return false;
}

void talonStateCallback(const talon_state_msgs::TalonState &talon_state)
{
	static size_t bl_talon_idx = std::numeric_limits<size_t>::max();
	static size_t br_talon_idx = std::numeric_limits<size_t>::max();
	static size_t fl_talon_idx = std::numeric_limits<size_t>::max();
	static size_t fr_talon_idx = std::numeric_limits<size_t>::max();
	getTalonIndex(talon_state, "speed_joint_bl", fl_talon_idx);
	getTalonIndex(talon_state, "speed_joint_br", fl_talon_idx);
	getTalonIndex(talon_state, "speed_joint_fl", fl_talon_idx);
	getTalonIndex(talon_state, "speed_joint_fr", fl_talon_idx);

	double volts;
	double position;
	double rate;
	double busVolts;

	size_t lCount = 0;
	size_t rCount = 0;
	battery = 0;

	leftMotorVolts = 0;
	rightMotorVolts = 0;
	leftPosition = 0;
	rightPosition = 0;
	leftRate = 0;
	rightRate = 0;
	if (getTalonData(talon_state, bl_talon_idx, volts, position, rate, busVolts))
	{
		lCount += 1;
		leftMotorVolts += volts;
		leftPosition += position;
		leftRate += rate;
		battery += busVolts;
	}
	if (getTalonData(talon_state, br_talon_idx, volts, position, rate, busVolts))
	{
		rCount += 1;
		rightMotorVolts += volts;
		rightPosition += position;
		rightRate += rate;
		battery += busVolts;
	}
	if (getTalonData(talon_state, fl_talon_idx, volts, position, rate, busVolts))
	{
		lCount += 1;
		leftMotorVolts += volts;
		leftPosition += position;
		leftRate += rate;
		battery += busVolts;
	}
	if (getTalonData(talon_state, fr_talon_idx, volts, position, rate, busVolts))
	{
		rCount += 1;
		rightMotorVolts += volts;
		rightPosition += position;
		rightRate += rate;
		battery += busVolts;
	}

	if (lCount)
	{
		leftMotorVolts /= lCount;
		leftPosition /= lCount;
		leftRate /= lCount;
	}
	if (rCount)
	{
		rightMotorVolts /= rCount;
		rightPosition /= rCount;
		rightRate /= rCount;
	}
	if (lCount || rCount)
	{
		battery /= lCount + rCount;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_characterization_node");
	ros::NodeHandle nh;
	auto talon_states_sub = nh.subscribe("/frcrobot_jetson/talon_states", 1, talonStateCallback);

	// Set up a network tables server
	auto ntInst = nt::NetworkTableInstance::GetDefault();
	ntInst.SetNetworkIdentity("Robot");
	ntInst.StartServer("/home/ubuntu/networktables.ini");
	ntInst.SetUpdateRate(0.01); // 100hz
	// Get the entries for the input and output tables
	auto autoSpeedEntry = ntInst.GetEntry("/robot/autospeed");
	auto telemetryEntry = ntInst.GetEntry("/robot/telemetry");

	std::vector<double> telemetry(9);

	ros::Rate r(90);

	double autospeed = 0;
	auto modeClient = nh.serviceClient<std_srvs::SetBool>("/frcrobot_jetson/swerve_drive_controller/percent_out_drive_mode");

	ROS_INFO("Waiting for percent_out_drive_mode service");
#if 0
	if (!modeClient.waitForExistence(ros::Duration(30)))
	{
		ROS_ERROR("Timeout waiting for percent_out_drive_mode service");
		return -1;
	}
#endif
	ROS_INFO("Calling percent_out_drive_mode service");
	std_srvs::SetBool modeData;
	modeData.request.data = true;
#if 0
	while (!modeClient.call(modeData) || (modeData.response.message != "SUCCESS!"))
	{
		ROS_ERROR("percent_out_drive_mode service returned error");
		ros::Duration(1).sleep();
	}
#endif
	ROS_INFO("   Success");

	auto joystickRobotVel = nh.advertise<geometry_msgs::Twist>("/frcrobot_jetson/swerve_drive_controller/cmd_vel", 1);

	ros::Time start_time = ros::Time::now();
	while (ros::ok())
	{
		// Example of reading a double
		autospeed = autoSpeedEntry.GetDouble(0);
		//ROS_INFO_STREAM("/robot/autospeed is " << autoSpeedEntry.GetDouble(0));
		geometry_msgs::Twist vel;
		vel.linear.x = autospeed;
		vel.linear.y = 0;
		vel.linear.z = 0;

		vel.angular.x = 0;
		vel.angular.y = 0;
		vel.angular.z = 0;

		joystickRobotVel.publish(vel);

		telemetry[0] = (ros::Time::now() - start_time).toSec();
		ROS_INFO_STREAM("t[0] = " << telemetry[0]);
        telemetry[1] = battery;
        telemetry[2] = autospeed;
        telemetry[3] = leftMotorVolts;
        telemetry[4] = rightMotorVolts;
        telemetry[5] = leftPosition;
        telemetry[6] = rightPosition;
        telemetry[7] = leftRate;
        telemetry[8] = rightRate;
		telemetryEntry.SetDoubleArray(telemetry);

		ntInst.Flush();

		// Loop at 100hz
		ros::spinOnce();
		r.sleep();
	}

	modeData.request.data = false;
	modeClient.call(modeData);
	return 0;
}
