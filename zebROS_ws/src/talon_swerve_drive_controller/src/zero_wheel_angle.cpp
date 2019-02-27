#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float64.h>
#include <vector>

ros::Publisher fr_angle_position;
ros::Publisher fl_angle_position;
ros::Publisher br_angle_position;
ros::Publisher bl_angle_position;
std::vector<double> offsets;

bool zero_wheel(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& /*res*/)
{
	ROS_INFO_STREAM("running zero_wheel_service");
	std_msgs::Float64 fl_msg;
	fl_msg.data = offsets[0] + M_PI/2;

	std_msgs::Float64 fr_msg;
	fr_msg.data = offsets[1] + M_PI/2;

	std_msgs::Float64 bl_msg;
	bl_msg.data = offsets[2] + M_PI/2;

	std_msgs::Float64 br_msg;
	br_msg.data = offsets[3] + M_PI/2;

	fr_angle_position.publish(fr_msg);
	fl_angle_position.publish(fl_msg);
	br_angle_position.publish(br_msg);
	bl_angle_position.publish(bl_msg);

	ROS_INFO_STREAM("finished zero_wheel_service");
	return 0;
}

bool keep_wheels_zeroed(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& /*res*/)
{
	ROS_INFO_STREAM("running zero_wheel_service");
	std_msgs::Float64 fl_msg;
	fl_msg.data = offsets[0] + M_PI/2;

	std_msgs::Float64 fr_msg;
	fr_msg.data = offsets[1] + M_PI/2;

	std_msgs::Float64 bl_msg;
	bl_msg.data = offsets[2] + M_PI/2;

	std_msgs::Float64 br_msg;
	br_msg.data = offsets[3] + M_PI/2;

	ros::Rate r(10);
	while(ros::ok())
	{
		fr_angle_position.publish(fr_msg);
		fl_angle_position.publish(fl_msg);
		br_angle_position.publish(br_msg);
		bl_angle_position.publish(bl_msg);
		r.sleep();
	}

	ROS_INFO_STREAM("finished zero_wheel_service");
	return 0;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "zero_wheel_service");
	ros::NodeHandle nh;

	std::vector<std::string> steering_names;

	steering_names.push_back("swerve_drive_controller/steering_joint_fl");
	steering_names.push_back("swerve_drive_controller/steering_joint_fr");
	steering_names.push_back("swerve_drive_controller/steering_joint_bl");
	steering_names.push_back("swerve_drive_controller/steering_joint_br");

	for(size_t i = 0; i < steering_names.size(); i++)
		ROS_INFO_STREAM("steering_name = " << steering_names[i]);

	for (auto it = steering_names.cbegin(); it != steering_names.cend(); ++it)
	{
		ros::NodeHandle offsets_nh(nh, *it);
		double dbl_val = 0;
		if (!offsets_nh.getParam("offset", dbl_val))
			ROS_ERROR_STREAM("Can not read offset for " << *it);
		offsets.push_back(dbl_val);
	}

	for(size_t i = 0; i < offsets.size(); i++)
		ROS_INFO_STREAM("offsets = " << offsets[i]);

	ros::ServiceServer zero_wheel_srv = nh.advertiseService("zero_wheel", zero_wheel);
	ros::ServiceServer keep_wheels_zeroed_srv = nh.advertiseService("keep_wheels_zeroed", keep_wheels_zeroed);
	fr_angle_position = nh.advertise<std_msgs::Float64>("fr_angle_motionmagic_controller/command", 1000);
	fl_angle_position = nh.advertise<std_msgs::Float64>("fl_angle_motionmagic_controller/command", 1000);
	br_angle_position = nh.advertise<std_msgs::Float64>("br_angle_motionmagic_controller/command", 1000);
	bl_angle_position = nh.advertise<std_msgs::Float64>("bl_angle_motionmagic_controller/command", 1000);
	ros::spin();

	return 0;
}
