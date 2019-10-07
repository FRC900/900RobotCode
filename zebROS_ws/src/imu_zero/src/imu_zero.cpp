#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include <imu_zero/imu_zero_angle.h>


// NAME THESE
const double pi = 3.14159;
const std::string sub_topic = "raw_imu";
const std::string pub_topic = "zeroed_imu";
const std::string service_name = "set_imu_zero";
ros::Publisher pub;
tf2::Quaternion zero_rot;
tf2::Quaternion last_raw;

double degToRad(double deg) {
  double rad = (deg / 180) * pi;
  return rad;
}

void zeroCallback(const geometry_msgs::Quaternion raw_msg) {
  tf2::convert(raw_msg, last_raw);
  tf2::Quaternion zeroed = zero_rot * last_raw;
  zeroed.normalize();
  pub.publish(tf2::toMsg(zeroed));
}

bool zeroSet(imu_zero::imu_zero_angle::Request& req,
             imu_zero::imu_zero_angle::Response& res) {
  // takes zero angle in degrees, converts to radians
  double a = degToRad(req.angle);
  zero_rot.setRPY(0.0, 0.0, - a);
  return true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "imu_zero_node");
  ros::NodeHandle node;
  pub = node.advertise<geometry_msgs::Quaternion>(pub_topic, 1);
  ros::Subscriber sub = node.subscribe(sub_topic, 1, zeroCallback);
  ros::ServiceServer svc = node.advertiseService(service_name, zeroSet);
  zero_rot.normalize();

  ros::spin();
  return 0;
}
