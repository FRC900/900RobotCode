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

constexpr double pi = 3.14159;
const std::string sub_topic = "imu/data";
const std::string pub_topic = "zeroed_imu";
const std::string service_name = "set_imu_zero";
ros::Publisher pub;
tf2::Quaternion zero_rot;
tf2::Quaternion last_raw;
ros::ServiceClient bias_estimate;

double degToRad(double deg) {
  double rad = (deg / 180) * pi;
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
  double a = degToRad(req.angle);

  zero_rot.setRPY(0.0, 0.0, a - yaw);

  if(ros::service::exists("imu/bias_estimate", false))
  {
	std_srvs::Trigger biasCall;
	bias_estimate.call(biasCall);
	ROS_INFO("Bias estimate: %s", biasCall.response.message.c_str());
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
  if(ros::service::exists("imu/bias_estimate",false))
    bias_estimate = node.serviceClient<std_srvs::Trigger>("imu/bias_estimate");

  ros::spin();
  return 0;
}
