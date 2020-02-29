/*
NODE NAME: imu_zero_node

SUBSCRIBES TO:
sub_topic

PUBLISHES TO:
pub_topic

SERVICES:
set_imu_zero

FUNCTIONALITY:
Zeros z-axis orientation from IMU.
Other IMU data is passed through.
Zero point in degrees is set using service call.
*/


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include <imu_zero/ImuZeroAngle.h>
#include <std_srvs/Trigger.h>
#include <robot_localization/SetPose.h>

constexpr double pi = 3.14159;
const std::string sub_topic = "imu/data";
const std::string pub_topic = "zeroed_imu";
const std::string service_name = "set_imu_zero";
ros::Publisher pub;
tf2::Quaternion zero_rot;
tf2::Quaternion last_raw;
ros::ServiceClient bias_estimate;
ros::ServiceClient ukf_zero_pos;


double degToRad(double deg) {
  const double rad = (deg / 180) * pi;
  return rad;
}

void zeroCallback(const sensor_msgs::Imu::ConstPtr& raw_msg) {
  tf2::convert(raw_msg -> orientation, last_raw);
  tf2::Quaternion zeroed = zero_rot * last_raw;
  zeroed.normalize();
  sensor_msgs::Imu zeroed_imu = *raw_msg;
  zeroed_imu.orientation = tf2::toMsg(zeroed);
  pub.publish(zeroed_imu);
}

bool zeroSet(imu_zero::ImuZeroAngle::Request& req,
             imu_zero::ImuZeroAngle::Response& res) {
  double roll, pitch, yaw;
  tf2::Matrix3x3(last_raw).getRPY(roll, pitch, yaw);

  // takes zero angle in degrees, converts to radians
  const double a = degToRad(req.angle);

  zero_rot.setRPY(0.0, 0.0, a - yaw);

  if(bias_estimate.exists())
  {
	std_srvs::Trigger biasCall;
	if (!bias_estimate.call(biasCall))
		ROS_ERROR("imu_zero : bias_estimate call failed");
	ROS_INFO("Bias estimate: %s", biasCall.response.message.c_str());
  }
  if (ukf_zero_pos.exists())
  {
	  robot_localization::SetPose zeroPose;
	  zeroPose.request.pose.header.stamp = ros::Time::now();
	  zeroPose.request.pose.header.frame_id = "odom";
	  // Needs more pose.pose.pose
	  zeroPose.request.pose.pose.pose.position.x	= 0;
	  zeroPose.request.pose.pose.pose.position.y	= 0;
	  zeroPose.request.pose.pose.pose.position.z	= 0;
	  tf2::Quaternion tf2_quat;
	  tf2_quat.setRPY(0,0,0);
	  tf2_quat.normalize();
	  zeroPose.request.pose.pose.pose.orientation = tf2::toMsg(tf2_quat);
	  if (!ukf_zero_pos.call(zeroPose))
		  ROS_ERROR("imu_zero : ukf_zero_pos call failed");
	  //ROS_INFO("Zero pose: %s", zeroPose.response.message.c_str());
  }
  return true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "imu_zero_node");
  ros::NodeHandle node;
  pub = node.advertise<sensor_msgs::Imu>(pub_topic, 1);
  ros::Subscriber sub = node.subscribe(sub_topic, 1, zeroCallback);
  ros::ServiceServer svc = node.advertiseService(service_name, zeroSet);
  zero_rot.setRPY(0,0,0);
  bias_estimate = node.serviceClient<std_srvs::Trigger>("imu/bias_estimate");
  ukf_zero_pos = node.serviceClient<robot_localization::SetPose>("/swerve_imu_ukf/set_pose");

  ros::spin();
  return 0;
}
