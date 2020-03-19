#include <ros/ros.h>
#include <networktables/NetworkTableInstance.h>
#include <talon_state_msgs/TalonState.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float64.h>

// Current battery voltage - read from talon bus voltage
double battery = 0;

// Motor output voltages - read from TalonState via callback
double motorVolts = 0;

// Motor positions - also read from TalonState
double position = 0;

// Motor speeds - also read from TalonState
double rate = 0;

std::string talon_name;

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
	static size_t talon_idx = std::numeric_limits<size_t>::max();
	getTalonIndex(talon_state, talon_name, talon_idx);

	if (!getTalonData(talon_state, talon_idx, motorVolts, position, rate, battery))
	{
		ROS_ERROR_STREAM("Could not get talon data for joint " << talon_name);
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

	std::string controller_topic;
	if (!nh.getParam("controller_topic", controller_topic))
	{
		ROS_ERROR("Fatal : could not read \"controller_topic\" parameter");
		return -1;
	}

	std::vector<double> telemetry(6);

	ros::Rate r(90);

	double autospeed = 0;

	auto command_pub = nh.advertise<std_msgs::Float64>(controller_topic, 1);

	ros::Time start_time = ros::Time::now();
	while (ros::ok())
	{
		if (battery > 0)
		{
			// Example of reading a double
			autospeed = autoSpeedEntry.GetDouble(0);
			//ROS_INFO_STREAM("/robot/autospeed is " << autoSpeedEntry.GetDouble(0));
			std_msgs::Float64 command;
			command.data = autospeed;

			command_pub.publish(command);

			telemetry[0] = (ros::Time::now() - start_time).toSec();
			ROS_INFO_STREAM("t[0] = " << telemetry[0]);
			telemetry[1] = battery;
			telemetry[2] = autospeed;
			telemetry[3] = motorVolts;
			telemetry[4] = position;
			telemetry[5] = rate;
			telemetryEntry.SetDoubleArray(telemetry);

			ntInst.Flush();
		}

		// Loop at ~100hz
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
